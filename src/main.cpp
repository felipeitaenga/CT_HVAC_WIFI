#include <WiFi.h>
#include <MQTT.h>
#include <OneWire.h>
#include <PID_v1.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <time.h>
#include <math.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>

// ─── Controle da VAG — parâmetros de sintonia ─────────────────────────────────
// PID_v1 em REVERSE (temperatura acima do setpoint → abre a VAG), saída em % (0–100),
// proporcional na medição (P_ON_M): o Kp freia a variação da temperatura em vez de
// reagir a degrau de setpoint; quem leva a temperatura até o setpoint é a integral.
//
// Sintonia em campo (mude UM parâmetro por vez e observe 1–2 h no Serial):
//  - VAG ainda vai de um extremo ao outro, temperatura cruza o setpoint em ondas → aumente VAG_TI_S
//    (720 → 1080 → 1440 s); se as ondas forem rápidas (< 10 min), reduza VAG_KP.
//  - Temperatura fica parada fora da zona morta, VAG quase não se mexe → reduza VAG_TI_S (720 → 480 s).
//  - Reação lenta a mudança de carga (porta aberta, sol) → aumente VAG_KP (30 → 40).
//  - Válvula se mexendo à toa com temperatura estável → aumente VAG_ZONA_MORTA (0.2 → 0.3 °C)
//    ou reduza VAG_ALFA_EMA (0.3 → 0.2, filtra mais porém atrasa).
//  - Equilíbrio bom: temperatura dentro da zona morta, VAG parada num valor intermediário.
const double        VAG_KP         = 30.0;                 // %/°C
const double        VAG_TI_S       = 720.0;                // s — tempo integral
const double        VAG_KI         = VAG_KP / VAG_TI_S;    // %/(°C·s) ≈ 0.042
const double        VAG_KD         = 0.0;                  // D desligado: só amplificaria ruído do sensor
const unsigned long CICLO_PID_MS   = 30000UL;              // período de leitura + Compute() (= SetSampleTime)
const double        VAG_ALFA_EMA   = 0.3;                  // filtro da temperatura, por leitura (0–1; menor = mais filtrado)
const double        VAG_ZONA_MORTA = 0.2;                  // °C — |setpoint − temp filtrada| <= isto: mantém a saída
const double        VAG_TAXA_MAX   = 5.0;                  // %  — variação máxima da saída por ciclo
const double        VAG_PASSO_MIN  = 1.0;                  // %  — só atualiza o atuador se mudar >= isto
const double        PUB_VAG_DELTA  = 2.0;                  // %  — publica VAG se variar >= isto…
const unsigned long PUB_VAG_MS     = 60000UL;              //      …ou a cada este intervalo

// Pressão (inversor do ventilador): com o fancoil LIGADO a AO2 nunca fica abaixo deste valor,
// para o inversor não rodar abaixo do mínimo seguro. Ajuste conforme o motor/inversor.
const float         PRESSAO_MIN_LIGADO = 20.0f;            // %

const unsigned long DEBOUNCE_MS    = 200UL;                // entradas MODO / STATUS

// ─── Recuperação de rede (degraus, sem bloquear o loop) ──────────────────────
const unsigned long MQTT_RETRY_MS        = 5000UL;      // a) MQTT caiu com Wi-Fi OK → tenta a cada 5 s
const unsigned long WIFI_RECONNECT_MS    = 30000UL;     // b) Wi-Fi fora > 30 s → WiFi.reconnect()
const unsigned long WIFI_RESET_PILHA_MS  = 180000UL;    // c) Wi-Fi fora > 3 min → reinicia a pilha Wi-Fi (repete a cada 3 min)
const unsigned long MQTT_RESET_PILHA_MS  = 600000UL;    //    MQTT fora > 10 min com Wi-Fi OK → também reinicia a pilha (degrau c)
const unsigned long MQTT_REINICIO_MS     = 1800000UL;   // d) MQTT fora > 30 min com Wi-Fi OK e c já tentado → reinício suave
const unsigned long PING_MS              = 60000UL;     // ping só para LED/diagnóstico
const unsigned long DIAG_MS              = 300000UL;    // RSSI e UPTIME a cada 5 min
const uint32_t      WDT_TIMEOUT_S        = 60;          // watchdog do loop principal

// ─── NTP ──────────────────────────────────────────────────────────────────────
const char* ntpServer          = "pool.ntp.org";
const long  gmtOffset_sec      = -3 * 3600;
const int   daylightOffset_sec = 0;

// ─── MQTT ─────────────────────────────────────────────────────────────────────
String getShortUniqueID() {
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  return mac.substring(mac.length() - 4);
}

const char* mqtt_server   = "156.67.82.199";
const char* mqtt_username = "";
const char* mqtt_password = "";

String mqtt_id;
String topic_temperatura;
String topic_vag;
String topic_modo;
String topic_status;
String topic_cmd;
String topic_estado;
String topic_setpoint;
String topic_online;
String topic_schedule;
String topic_schedule_state;
String topic_cmd_estado;      // TJS/<id>/CMD     — estado real do relé (retido)
String topic_estado_real;     // TJS/<id>/ESTADO  — modo real AUTOMÁTICO/FORÇADO (retido)
String topic_pressao;         // TJS/<id>/PRESSAO/SET
String topic_versao;          // TJS/<id>/VERSAO
String topic_pressao_estado;  // TJS/<id>/PRESSAO — valor aplicado na AO2 (retido)
String topic_reset;           // TJS/<id>/RESET   — motivo do último reinício (retido)
String topic_rssi;            // TJS/<id>/RSSI    — dBm (retido, a cada 5 min)
String topic_uptime;          // TJS/<id>/UPTIME  — minutos (retido, a cada 5 min)

// ─── Hardware ─────────────────────────────────────────────────────────────────
#define ONE_WIRE_BUS    13
// Saídas 0–10 V da KC868-A4: DAC do ESP32 (0–3,3 V) + amplificador ~3x. Só dacWrite, nunca PWM/LEDC.
#define AO_VAG          25   // AO1 / DAC1 → atuador da VAG       (0–10 V = 0–100 %)
#define AO_PRESSAO      26   // AO2 / DAC2 → inversor do ventilador (0–10 V = 0–100 %)
#define LED_PIN         22
#define RELE1_PIN        2
#define RELE2_PIN       15
#define RELE3_PIN        5
#define RELE4_PIN        4
#define BTN_STATUS      27
#define BTN_AUTO_MANUAL 14
#define BTN_WIFI         0

#define RELE_LIGADO    HIGH
#define RELE_DESLIGADO LOW

#define EEPROM_SIZE       16
#define ADDR_RELE1        0
// endereços 1–3: livres (antiga janela horária local — agenda agora vem só do supervisório, no NVS)
#define ADDR_ESTADO_CTRL  4   // estadoControladora (0=FORÇADO, 1=AUTOMÁTICO)
#define ADDR_SETPOINT     8   // float (4 bytes) — setpoint persistido
#define ADDR_PRESSAO     12   // float (4 bytes) — setpoint de pressão (%) persistido

// ─── Versão do firmware ───────────────────────────────────────────────────────
#define FIRMWARE_VERSION "1.3.0"

// ─── OTA ──────────────────────────────────────────────────────────────────────
// Senha do AP OTA e do ElegantOTA (usuário "admin"). Mínimo 8 caracteres (WPA2).
// Pode ser sobrescrita sem mexer no código: build_flags = -D OTA_SECRET=\"...\"
#ifndef OTA_SECRET
#define OTA_SECRET "@pilares1"
#endif
static_assert(sizeof(OTA_SECRET) - 1 >= 8, "OTA_SECRET precisa ter pelo menos 8 caracteres (WPA2)");

WebServer otaServer(80);
#define OTA_TIMEOUT_MS 300000UL

// ─── Variáveis ────────────────────────────────────────────────────────────────
unsigned long buttonPressStartTime = 0;
bool          btnOtaArmed          = false;
const unsigned long buttonPressThreshold = 5000;

double temperatura = NAN;   // temperatura FILTRADA (EMA) — entrada do PID; NAN até a 1ª leitura válida
double vagPid      = 0;     // saída do PID (%) — memória do controlador
double setpoint    = 23.0;  // default seguro
PID pid1(&temperatura, &vagPid, &setpoint, VAG_KP, VAG_KI, VAG_KD, P_ON_M, REVERSE);

double vagAtuador   = 0;     // % efetivamente aplicado na AO1
bool   naZonaMorta  = false;
bool   sensorEmFalha = false;
double vagPublicada = -1;    // -1 = força publicação
unsigned long tsPubVag = 0;

float pressaoSetpoint = 0;   // % (0–100) — PRESSAO/SET, persistido na EEPROM
float pressaoAplicada = -1;  // % efetivamente na AO2 (-1 = ainda não aplicada/publicada)

int  estadoControladora = 1;
bool releAtivado        = false;
volatile bool portalAtivo = false;   // escrito no loop/callbacks, lido na taskLED

int modoAnterior   = -1;   // último valor PUBLICADO (-1 = força publicação)
int statusAnterior = -1;

// Debounce das entradas digitais
struct Debounce { int bruto; int estavel; unsigned long ts; };
Debounce dbModo   = {-1, -1, 0};
Debounce dbStatus = {-1, -1, 0};

unsigned long ts1          = 0;
unsigned long tsPing       = 0;
unsigned long ultimoPingOK = 0;
volatile bool pingOK = false;   // escrito no loop, lido na taskLED

// ─── Agenda semanal (recebida do supervisório via MQTT, persistida no NVS) ────
// Payload: uma linha por slot  IDX;HH:MM;DIAS;ACAO;VALOR;EN  (vazio = apagar tudo)
enum AcaoAgenda : uint8_t { ACAO_ESTADO, ACAO_CMD, ACAO_SETPOINT };

struct SlotAgenda {
  uint8_t hora;
  uint8_t minuto;
  uint8_t dias;    // bitmap: bit0=Dom ... bit6=Sáb
  uint8_t acao;    // AcaoAgenda
  float   valor;   // ESTADO/CMD: 1=true 0=false | SETPOINT: °C
};

#define MAX_SLOTS 10
SlotAgenda  slots[MAX_SLOTS];
int         nSlots            = 0;
String      agendaPayload     = "";    // última agenda aplicada (texto bruto, é o que vai pro NVS)
long        agendaUltimaChave = -1;    // ano/dia/minuto já processado — evita re-disparo no mesmo minuto
bool        agendaCatchupPendente = false;   // aplicar o estado esperado assim que houver hora válida (boot / agenda nova)
Preferences prefs;

WiFiClient        net;
MQTTClient        client(1024);   // padrão da lib é 128 bytes — 10 slots passam de 250 e seriam descartados
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
WiFiManager       wifiManager;

// ─── Estado que sobrevive a reinício por software/watchdog (não a falta de energia) ───
// Gravado a cada mudança de relé / VAG / pressão / modo, para que mesmo um reinício por
// watchdog (que não passa por reinicioSuave) encontre o estado atual.
struct EstadoRTC {
  uint32_t magic;
  uint8_t  rele;              // releAtivado
  uint8_t  estado;            // estadoControladora
  float    vagAtuador;
  float    pressaoAplicada;
  float    vagPid;
  char     motivo[24];        // motivo próprio do reinício suave ("" = nenhum)
  uint32_t checksum;
};
RTC_NOINIT_ATTR EstadoRTC rtcEstado;
const uint32_t RTC_MAGIC = 0x48564143UL ^ (uint32_t)sizeof(EstadoRTC);   // "HVAC" + tamanho do layout

static uint32_t checksumRTC() {   // FNV-1a sobre tudo menos o próprio checksum
  const uint8_t* p = (const uint8_t*)&rtcEstado;
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < offsetof(EstadoRTC, checksum); i++) { h ^= p[i]; h *= 16777619UL; }
  return h;
}

void salvarRTC() {
  rtcEstado.magic           = RTC_MAGIC;
  rtcEstado.rele            = releAtivado ? 1 : 0;
  rtcEstado.estado          = (uint8_t)estadoControladora;
  rtcEstado.vagAtuador      = (float)vagAtuador;
  rtcEstado.pressaoAplicada = pressaoAplicada;
  rtcEstado.vagPid          = (float)vagPid;
  rtcEstado.motivo[sizeof(rtcEstado.motivo) - 1] = '\0';
  rtcEstado.checksum        = checksumRTC();
}

String motivoReset;   // publicado em TJS/<id>/RESET — ex.: "SW / MQTT_30MIN", "TASK_WDT", "POWERON"

static const char* nomeResetReason(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

// ─── Relés com gpio_hold ──────────────────────────────────────────────────────
// Os relés ficam SEMPRE travados (gpio_hold_en) e só são liberados durante a escrita.
// Assim um reinício por software OU watchdog encontra o pino preso no último nível.
const gpio_num_t PINOS_RELE[] = {(gpio_num_t)RELE1_PIN, (gpio_num_t)RELE2_PIN,
                                 (gpio_num_t)RELE3_PIN, (gpio_num_t)RELE4_PIN};

static void travarReles(bool travar) {
  for (gpio_num_t p : PINOS_RELE) { if (travar) gpio_hold_en(p); else gpio_hold_dis(p); }
}

void escreverReles(bool ligado) {
  travarReles(false);
  digitalWrite(RELE1_PIN, ligado ? RELE_LIGADO : RELE_DESLIGADO);
  digitalWrite(RELE2_PIN, ligado ? RELE_LIGADO : RELE_DESLIGADO);   // exaustor acompanha o fancoil
  travarReles(true);
}

// Reinício proposital sem mexer no fancoil: grava o estado na RTC, trava relés e AO1/AO2
// no nível atual e reinicia. O setup() restaura tudo antes de Serial/Wi-Fi/NTP/MQTT.
void reinicioSuave(const char* motivo) {
  Serial.printf("♻️ REINÍCIO SUAVE | motivo: %s | relé=%s VAG=%.1f%% PRESSÃO=%.1f%%\n",
                motivo, releAtivado ? "LIGADO" : "DESLIGADO", vagAtuador, pressaoAplicada);
  strncpy(rtcEstado.motivo, motivo, sizeof(rtcEstado.motivo) - 1);
  salvarRTC();
  travarReles(true);
  gpio_hold_en((gpio_num_t)AO_VAG);        // efeito nas saídas DAC: validar na bancada
  gpio_hold_en((gpio_num_t)AO_PRESSAO);
  Serial.flush();
  ESP.restart();
}

// ─── Helpers de payload ───────────────────────────────────────────────────────
bool payloadVerdadeiro(const String& p) {
  return (p == "true" || p == "1" || p.equalsIgnoreCase("LIGAR"));
}
bool payloadFalso(const String& p) {
  return (p == "false" || p == "0" || p.equalsIgnoreCase("DESLIGAR"));
}

// ─── EEPROM ───────────────────────────────────────────────────────────────────
void salvarRele(int estado) {
  EEPROM.write(ADDR_RELE1, estado);
  EEPROM.commit();
}

// ─── EEPROM: estado da controladora + setpoint ───────────────────────────────
void salvarEstadoCtrl() {
  EEPROM.write(ADDR_ESTADO_CTRL, (uint8_t)estadoControladora);
  EEPROM.commit();
  salvarRTC();
}

void salvarSetpoint() {
  float sp = (float)setpoint;
  EEPROM.put(ADDR_SETPOINT, sp);
  EEPROM.commit();
}

void carregarEstadoCtrl() {
  uint8_t e = EEPROM.read(ADDR_ESTADO_CTRL);
  estadoControladora = (e == 0xFF) ? 1 : (e ? 1 : 0);   // EEPROM virgem → AUTOMÁTICO

  float sp;
  EEPROM.get(ADDR_SETPOINT, sp);
  setpoint = (isnan(sp) || sp < 10.0 || sp > 35.0) ? 23.0 : sp;

  float pr;
  EEPROM.get(ADDR_PRESSAO, pr);
  pressaoSetpoint = (isnan(pr) || pr < 0.0f || pr > 100.0f) ? 0.0f : pr;   // EEPROM virgem → 0%
  // log fica no setup(): esta função roda antes do Serial.begin()
}

void salvarPressao() {
  EEPROM.put(ADDR_PRESSAO, pressaoSetpoint);
  EEPROM.commit();
}

// ─── MQTT publish com log ─────────────────────────────────────────────────────
// retained=true -> broker guarda ultima mensagem e entrega imediatamente a novos subscribers
void mqttPublish(const String& topic, const String& payload, bool retained = false) {
  bool ok = client.publish(topic, payload, retained, 0);
  Serial.printf("📤 ENVIADO   [%s] → \"%s\" %s%s\n",
                topic.c_str(), payload.c_str(),
                ok ? "✅" : "❌ FALHOU",
                retained ? " 📌" : "");
}

// ─── Saídas analógicas 0–10 V (DAC) ──────────────────────────────────────────
void saidaAnalogica(uint8_t pino, float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  dacWrite(pino, (uint8_t)lround(pct * 2.55f));
}

// AO2 = pressão salva (com piso PRESSAO_MIN_LIGADO) só com o relé LIGADO; senão 0 V.
// Publica o valor aplicado em TJS/<id>/PRESSAO (retido) quando muda.
void aplicarPressao() {
  float alvo = releAtivado ? max(pressaoSetpoint, PRESSAO_MIN_LIGADO) : 0.0f;
  saidaAnalogica(AO_PRESSAO, alvo);
  if (fabsf(alvo - pressaoAplicada) > 0.01f) {
    pressaoAplicada = alvo;
    salvarRTC();
    Serial.printf("💨 AO2 PRESSÃO → %.1f%%\n", alvo);
    mqttPublish(topic_pressao_estado, String(alvo, 1), true);
  }
}


// ─── MODO OTA ─────────────────────────────────────────────────────────────────
void entrarModoOTA() {
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("🔄 ENTRANDO EM MODO OTA");
  escreverReles(false);                      // fancoil + exaustor desligados durante o OTA
  saidaAnalogica(AO_VAG, 0);                 // relé desligado → VAG e inversor em 0 V
  saidaAnalogica(AO_PRESSAO, 0);
  client.publish(topic_online.c_str(), "0", true, 1); // saída deliberada (OTA) também marca offline
  client.disconnect();
  WiFi.mode(WIFI_AP_STA);  // mantém STA conectado + sobe AP
  delay(300);

  String apNome  = "HVAC-" + mqtt_id + "-OTA";
  WiFi.softAP(apNome.c_str(), OTA_SECRET);
  delay(500);

  IPAddress ip = WiFi.softAPIP();
  Serial.println("📶 AP OTA ATIVO");
  Serial.println("   SSID  : " + apNome);
  Serial.println("   Senha : OTA_SECRET (definida no firmware)");
  Serial.print(  "   Acesse: http://"); Serial.print(ip); Serial.println("/update");
  Serial.println("   Timeout: 5 minutos");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

  // ── Página inicial ──────────────────────────────────────────────────────────
  otaServer.on("/", []() {
    String html =
      "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>body{font-family:sans-serif;text-align:center;padding:30px}"
      "h2{color:#333}.btn{display:inline-block;padding:12px 24px;margin:8px;"
      "background:#0078d4;color:white;text-decoration:none;"
      "border-radius:6px;font-size:16px}"
      ".btn2{background:#28a745}.btn3{background:#dc3545}.btn4{background:#6f42c1}</style></head><body>"
      "<h2>🔧 HVAC Controladora</h2>"
      "<p><b>ID:</b> " + mqtt_id + " &nbsp;|&nbsp; <b>v" FIRMWARE_VERSION "</b></p>"
      "<p><b>IP AP:</b> " + WiFi.softAPIP().toString() + "</p>"
      "<br>"
      "<a class='btn' href='/update'>📦 Atualizar Firmware</a><br>"
      "<a class='btn btn4' href='/mqtt-test'>🔌 Testar Conexão MQTT</a><br>"
      "<a class='btn btn3' href='/restart' onclick=\"return confirm('Reiniciar a controladora?')\">🔄 Reiniciar Controladora</a><br>"
      "<a class='btn btn2' href='/exit' onclick=\"return confirm('Sair do modo OTA e voltar à operação normal?')\">✅ Sair do modo OTA</a>"
      "</body></html>";
    otaServer.send(200, "text/html", html);
  });

  // ── Reiniciar controladora ──────────────────────────────────────────────────
  otaServer.on("/restart", HTTP_GET, []() {
    String html =
      "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
      "<meta http-equiv='refresh' content='3;url=/'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>body{font-family:sans-serif;text-align:center;padding:40px}"
      ".ok{background:#f8d7da;border-radius:8px;padding:20px;font-size:20px}"
      "</style></head><body>"
      "<div class='ok'>🔄 Reiniciando...<br><small>Aguarde alguns segundos</small></div>"
      "</body></html>";
    otaServer.send(200, "text/html", html);
    delay(500);
    reinicioSuave("OTA_RESTART");
  });

  // ── Teste de conexão MQTT ────────────────────────────────────────────────────
  otaServer.on("/mqtt-test", HTTP_GET, []() {
    bool wifiOK = (WiFi.status() == WL_CONNECTED);
    bool mqttOK = false;
    unsigned long t0 = millis();

    if (wifiOK) {
      WiFiClient testNet;
      testNet.setTimeout(3);           // core 2.0.x: SEGUNDOS (o antigo 3000 eram 50 min)
      MQTTClient testClient(256);
      testClient.setTimeout(2000);
      testClient.begin(mqtt_server, testNet);
      String testId = mqtt_id + "_test";
      mqttOK = testClient.connect(testId.c_str(), mqtt_username, mqtt_password);
      if (mqttOK) testClient.disconnect();
      testNet.stop();
    }

    unsigned long elapsed = millis() - t0;
    String cor  = mqttOK ? "#d4edda" : "#f8d7da";
    String icon = mqttOK ? "✅" : "❌";
    String msg  = mqttOK ? "Conectado com sucesso!" : "Falha na conexao";

    String html =
      "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>body{font-family:sans-serif;text-align:center;padding:20px}"
      "h2{color:#333}.card{background:" + cor + ";border-radius:8px;"
      "padding:20px;margin:12px auto;max-width:400px}"
      ".lbl{color:#555;font-size:13px}.val{font-size:18px;font-weight:bold}"
      ".btn{display:inline-block;padding:10px 24px;margin:6px;"
      "color:white;text-decoration:none;border-radius:6px}"
      ".b1{background:#0078d4}.b2{background:#6f42c1}"
      "</style></head><body>"
      "<h2>🔌 Teste MQTT</h2>"
      "<div class='card'>"
      "<p style='font-size:48px;margin:0'>" + icon + "</p>"
      "<p class='val'>" + msg + "</p><hr>"
      "<p><span class='lbl'>Wi-Fi: </span>"
        + (wifiOK ? "✅ " + WiFi.localIP().toString() : "❌ Desconectado") + "</p>"
      "<p><span class='lbl'>Servidor: </span>" + String(mqtt_server) + ":1883</p>"
      "<p><span class='lbl'>ID testado: </span>" + mqtt_id + "_test</p>"
      "<p><span class='lbl'>Tempo resposta: </span>" + String(elapsed) + " ms</p>"
      "</div>"
      "<a class='btn b1' href='/'>&#8592; Voltar</a>"
      "<a class='btn b2' href='/mqtt-test'>&#x1F504; Testar de novo</a>"
      "</body></html>";

    otaServer.send(200, "text/html", html);
  });

  otaServer.on("/exit", HTTP_GET, []() {
    otaServer.send(200, "text/html",
      "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
      "<style>body{font-family:sans-serif;text-align:center;padding:40px}</style></head><body>"
      "<h2>✅ Saindo do modo OTA...</h2><small>A controladora vai reiniciar.</small>"
      "</body></html>");
    delay(300);
    reinicioSuave("OTA_EXIT");
  });

  ElegantOTA.setAuth("admin", OTA_SECRET);
  ElegantOTA.begin(&otaServer);
  otaServer.begin();

  unsigned long tOTA    = millis();
  unsigned long tPiscar = millis();

  while (millis() - tOTA < OTA_TIMEOUT_MS) {
    esp_task_wdt_reset();
    otaServer.handleClient();
    ElegantOTA.loop();
    if (millis() - tPiscar >= 50) {
      tPiscar = millis();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }

  Serial.println("⏱️ Timeout OTA. Reiniciando...");
  reinicioSuave("OTA_TIMEOUT");
}

// ─── VAG: atuador ─────────────────────────────────────────────────────────────
// O controle trabalha em %; a conversão para 0–255 do DAC acontece só em saidaAnalogica().
void escreverVAG(double pct) {
  vagAtuador = constrain(pct, 0.0, 100.0);
  saidaAnalogica(AO_VAG, vagAtuador);
  salvarRTC();
}

void publicarVAG(bool forcar) {
  if (!forcar && vagPublicada >= 0 && fabs(vagAtuador - vagPublicada) < PUB_VAG_DELTA &&
      millis() - tsPubVag < PUB_VAG_MS) return;
  mqttPublish(topic_vag, releAtivado ? String(vagAtuador, 1) : String("0"), true);
  vagPublicada = vagAtuador;
  tsPubVag     = millis();
}

// ─── Liga relé ────────────────────────────────────────────────────────────────
void ligarRele(const char* motivo) {
  Serial.printf("🔌 ligarRele() | motivo: %s | releAtivado: %s\n",
                motivo, releAtivado ? "já ligado" : "false");
  if (releAtivado) return;   // idempotente: nada a fazer, nada a publicar
  escreverReles(true);                    // fancoil + exaustor
  salvarRele(RELE_LIGADO);
  releAtivado = true;
  salvarRTC();
  naZonaMorta = false;
  // Parte de vagPid (0 após desligar). Sem temperatura válida ainda (boot/falha),
  // o vagPID() passa para AUTOMATIC na primeira leitura boa.
  if (!sensorEmFalha && !isnan(temperatura)) pid1.SetMode(AUTOMATIC);
  Serial.println("✅ RELÉ LIGADO (fancoil + exaustor)");
  aplicarPressao();                          // inversor na pressão salva
  mqttPublish(topic_cmd_estado, "true", true);
}

// ─── Desliga relé ─────────────────────────────────────────────────────────────
void desligarRele(const char* motivo) {
  Serial.printf("🔌 desligarRele() | motivo: %s | releAtivado: %s\n",
                motivo, releAtivado ? "true" : "já desligado");
  if (!releAtivado) return;   // idempotente: nada a fazer, nada a publicar
  pid1.SetMode(MANUAL);
  vagPid = 0;
  escreverVAG(0);
  escreverReles(false);                      // fancoil + exaustor
  salvarRele(RELE_DESLIGADO);
  releAtivado = false;
  salvarRTC();
  Serial.println("⛔ RELÉ DESLIGADO (fancoil + exaustor)");
  aplicarPressao();                          // inversor em 0 V
  publicarVAG(true);
  mqttPublish(topic_cmd_estado, "false", true);
}

// ─── Agenda: parser ───────────────────────────────────────────────────────────
// Divide `s` por `sep` em até `max` campos; devolve quantos foram encontrados
static int dividir(const String& s, char sep, String* out, int max) {
  int n = 0, ini = 0;
  while (n < max) {
    int fim = s.indexOf(sep, ini);
    if (fim < 0) { out[n++] = s.substring(ini); break; }
    out[n++] = s.substring(ini, fim);
    ini = fim + 1;
  }
  return n;
}

static bool apenasDigitos(const String& s) {
  if (s.length() == 0) return false;
  for (unsigned i = 0; i < s.length(); i++) if (!isDigit(s[i])) return false;
  return true;
}

// 1 = slot válido e ativo | 0 = válido mas desativado (EN=0) | -1 = malformado
static int parseLinhaAgenda(const String& linha, SlotAgenda& s) {
  String c[7];
  if (dividir(linha, ';', c, 7) != 6) return -1;
  for (int i = 0; i < 6; i++) c[i].trim();

  if (c[5] != "0" && c[5] != "1") return -1;                       // EN

  int dp = c[1].indexOf(':');                                      // HH:MM
  if (dp < 0) return -1;
  String hh = c[1].substring(0, dp), mm = c[1].substring(dp + 1);
  if (!apenasDigitos(hh) || !apenasDigitos(mm)) return -1;
  int h = hh.toInt(), m = mm.toInt();
  if (h > 23 || m > 59) return -1;

  uint8_t dias = 0;                                                // DIAS
  if (c[2] == "*") {
    dias = 0x7F;
  } else {
    String d[8];
    int nd = dividir(c[2], ',', d, 8);
    for (int i = 0; i < nd; i++) {
      d[i].trim();
      if (!apenasDigitos(d[i]) || d[i].toInt() > 6) return -1;
      dias |= (1 << d[i].toInt());
    }
  }

  c[3].trim(); c[3].toUpperCase();                                 // ACAO
  if      (c[3] == "ESTADO")   s.acao = ACAO_ESTADO;
  else if (c[3] == "CMD")      s.acao = ACAO_CMD;
  else if (c[3] == "SETPOINT") s.acao = ACAO_SETPOINT;
  else return -1;

  if (s.acao == ACAO_SETPOINT) {                                   // VALOR
    char* fim;
    float v = strtof(c[4].c_str(), &fim);
    if (c[4].length() == 0 || *fim != '\0' || isnan(v)) return -1;
    s.valor = constrain(v, 10.0f, 35.0f);
  } else {
    c[4].toLowerCase();
    if      (payloadVerdadeiro(c[4])) s.valor = 1;
    else if (payloadFalso(c[4]))      s.valor = 0;
    else return -1;
  }

  s.hora = h; s.minuto = m; s.dias = dias;
  return (c[5] == "1") ? 1 : 0;
}

// Substitui a lista inteira (sem merge). Devolve a quantidade de slots ativos.
static int interpretarAgenda(const String& payload) {
  nSlots = 0;
  int ini = 0;
  while (ini <= (int)payload.length()) {
    int fim = payload.indexOf('\n', ini);
    if (fim < 0) fim = payload.length();
    String linha = payload.substring(ini, fim);
    ini = fim + 1;
    linha.trim();
    if (linha.length() == 0) continue;

    SlotAgenda s;
    int r = parseLinhaAgenda(linha, s);
    if (r < 0)  { Serial.printf("⚠️ Agenda: linha malformada ignorada: \"%s\"\n", linha.c_str()); continue; }
    if (r == 0) { Serial.printf("ℹ️ Agenda: slot desativado (EN=0) ignorado: \"%s\"\n", linha.c_str()); continue; }
    if (nSlots >= MAX_SLOTS) { Serial.printf("⚠️ Agenda: acima de %d slots, ignorado: \"%s\"\n", MAX_SLOTS, linha.c_str()); continue; }
    slots[nSlots++] = s;
  }
  return nSlots;
}

// Lista aplicada, no mesmo formato recebido (IDX renumerado de 0)
static String descreverAgenda() {
  String out;
  for (int i = 0; i < nSlots; i++) {
    const SlotAgenda& s = slots[i];
    String dias;
    if (s.dias == 0x7F) dias = "*";
    else for (int d = 0; d < 7; d++) if ((s.dias >> d) & 1) { if (dias.length()) dias += ','; dias += String(d); }
    const char* acao = (s.acao == ACAO_ESTADO) ? "ESTADO" : (s.acao == ACAO_CMD) ? "CMD" : "SETPOINT";
    String valor = (s.acao == ACAO_SETPOINT) ? String(s.valor, 1) : String(s.valor > 0 ? "true" : "false");
    char hm[6];
    snprintf(hm, sizeof(hm), "%02d:%02d", s.hora, s.minuto);
    if (i) out += '\n';
    out += String(i) + ";" + hm + ";" + dias + ";" + acao + ";" + valor + ";1";
  }
  return out;
}

static void publicarEstadoAgenda() {
  mqttPublish(topic_schedule_state, descreverAgenda(), true);   // vazio → limpa o retained
}

// ─── Agenda: persistência (NVS) ───────────────────────────────────────────────
static void salvarAgendaNVS() {
  prefs.begin("hvac", false);
  prefs.putString("agenda", agendaPayload);
  prefs.end();
}

static void carregarAgendaNVS() {
  prefs.begin("hvac", false);
  agendaPayload = prefs.getString("agenda", "");
  prefs.end();
  int n = interpretarAgenda(agendaPayload);
  Serial.printf("📂 Agenda carregada do NVS | %d slot(s)\n", n);
  if (n) Serial.println(descreverAgenda());
  agendaCatchupPendente = (n > 0);
}

// ─── Estado da controladora (AUTOMÁTICO / FORÇADO) ───────────────────────────
// Publica o estado real em TJS/<id>/ESTADO só quando muda (idempotente).
// porAgenda=true: regrava também o retido de ESTADO/SET (1 vez por disparo), para o
// retido antigo do broker não reverter o modo no próximo boot.
static void definirEstado(bool automatico, bool porAgenda) {
  int novo = automatico ? 1 : 0;
  if (estadoControladora != novo) {
    estadoControladora = novo;
    salvarEstadoCtrl();
    Serial.printf("   🔄 Controladora → %s\n", automatico ? "AUTOMÁTICO" : "FORÇADO");
    mqttPublish(topic_estado_real, automatico ? "true" : "false", true);
  }
  if (porAgenda) mqttPublish(topic_estado, automatico ? "true" : "false", true);
}

// ─── Agenda: execução ─────────────────────────────────────────────────────────
// ESTADO e SETPOINT valem sempre. CMD só vale em AUTOMÁTICO (não altera o ESTADO);
// em FORÇADO quem manda no relé é o CMD/SET manual.
static void executarSlot(const SlotAgenda& s) {
  static const char* nomes[] = {"ESTADO", "CMD", "SETPOINT"};
  Serial.printf("⏰ AGENDA %02d:%02d → %s = %.1f\n", s.hora, s.minuto, nomes[s.acao], s.valor);
  switch (s.acao) {
    case ACAO_ESTADO:
      definirEstado(s.valor > 0, true);
      break;
    case ACAO_CMD:
      if (estadoControladora == 0) { Serial.println("   ℹ️ CMD da agenda ignorado: FORÇADO"); break; }
      if (s.valor > 0) ligarRele("agenda");
      else             desligarRele("agenda");
      break;
    case ACAO_SETPOINT:
      if (fabs(s.valor - setpoint) > 0.01) { setpoint = s.valor; salvarSetpoint(); }
      mqttPublish(topic_setpoint, String(setpoint, 1), true);
      break;
  }
}

// Aplica o último evento já passado de cada ação (até 7 dias atrás).
// Cobre queda de energia (boot às 07:05 com slot das 07:00) e agenda recém-recebida.
// Ordem fixa ESTADO → SETPOINT → CMD: o CMD é avaliado já com o ESTADO esperado para agora
// (mesma regra do disparo normal: ignorado se o resultado for FORÇADO).
static void recuperarEstadoAgenda(const struct tm& t) {
  int  agora = t.tm_hour * 60 + t.tm_min;
  long ago[3] = {-1, -1, -1};   // minutos desde o último evento de cada ação
  int  idx[3] = {-1, -1, -1};   // slot correspondente

  for (int i = 0; i < nSlots; i++) {
    const SlotAgenda& s = slots[i];
    for (int volta = 0; volta <= 7; volta++) {
      long min = (long)volta * 1440 + agora - (s.hora * 60 + s.minuto);
      if (min < 0) continue;                                            // ainda não chegou hoje
      if (!((s.dias >> (((t.tm_wday - volta) % 7 + 7) % 7)) & 1)) continue;
      if (ago[s.acao] < 0 || min < ago[s.acao]) { ago[s.acao] = min; idx[s.acao] = i; }
      break;                                                            // ocorrência mais recente deste slot
    }
  }

  static const uint8_t ordem[3] = {ACAO_ESTADO, ACAO_SETPOINT, ACAO_CMD};
  for (uint8_t a : ordem)
    if (idx[a] >= 0) executarSlot(slots[idx[a]]);
}

// Chamar a cada iteração do loop(): checa 1x por segundo
void verificarAgenda() {
  static unsigned long tCheck = 0, tAviso = 0;
  if (millis() - tCheck < 1000UL) return;
  tCheck = millis();
  if (nSlots == 0) return;

  struct tm t;
  if (!getLocalTime(&t, 0)) {   // timeout 0: sem NTP não pode travar o loop
    if (millis() - tAviso >= 60000UL) {
      tAviso = millis();
      Serial.println("⚠️ Agenda suspensa — relógio NTP ainda não sincronizado");
    }
    return;
  }

  long chave = ((long)t.tm_year * 366L + t.tm_yday) * 1440L + t.tm_hour * 60 + t.tm_min;
  if (agendaCatchupPendente) {
    agendaCatchupPendente = false;
    agendaUltimaChave = chave;              // slots deste minuto já entram na recuperação
    Serial.println("🔁 Agenda: aplicando o estado esperado para o horário atual");
    recuperarEstadoAgenda(t);
    return;
  }
  if (chave == agendaUltimaChave) return;   // este minuto já foi processado
  agendaUltimaChave = chave;

  for (int i = 0; i < nSlots; i++) {
    const SlotAgenda& s = slots[i];
    if (s.hora == t.tm_hour && s.minuto == t.tm_min && ((s.dias >> t.tm_wday) & 1)) executarSlot(s);
  }
}

// ─── Entradas digitais → publica (retido) se mudou ───────────────────────────
// Devolve o valor estável (1 = ativo/LOW) ou -1 enquanto ainda não estabilizou desde o boot
static int lerComDebounce(uint8_t pino, Debounce& d) {
  int r = (digitalRead(pino) == LOW) ? 1 : 0;
  if (r != d.bruto) { d.bruto = r; d.ts = millis(); }
  else if (millis() - d.ts >= DEBOUNCE_MS) d.estavel = r;
  return d.estavel;
}

void lerEPublicarEntradas() {
  int modoAtual = lerComDebounce(BTN_AUTO_MANUAL, dbModo);
  if (modoAtual >= 0 && modoAtual != modoAnterior) {
    modoAnterior = modoAtual;
    Serial.printf("🔁 MODO (GPIO%d) → %s\n", BTN_AUTO_MANUAL, modoAtual ? "AUTO" : "MANUAL");
    mqttPublish(topic_modo, String(modoAtual), true);
  }

  int statusAtual = lerComDebounce(BTN_STATUS, dbStatus);
  if (statusAtual >= 0 && statusAtual != statusAnterior) {
    statusAnterior = statusAtual;
    Serial.printf("🔁 STATUS (GPIO%d) → %s\n", BTN_STATUS, statusAtual ? "LIGADO" : "DESLIGADO");
    mqttPublish(topic_status, String(statusAtual), true);
  }
}

// ─── Portal Wi-Fi (WiFiManager não bloqueante) ───────────────────────────────
// Abre o AP de configuração sem parar o controle; wifiManager.process() roda no loop.
// As credenciais atuais só são substituídas se o usuário salvar novas no portal.
void abrirPortalWifi() {
  if (wifiManager.getConfigPortalActive()) return;
  String apNome = "AutoConnect_" + mqtt_id;
  Serial.println("📶 Abrindo portal Wi-Fi (não bloqueante): " + apNome);
  wifiManager.startConfigPortal(apNome.c_str());
}

// ─── ✅ Botão WiFi — leitura não bloqueante ───────────────────────────────────
void verificarBotaoWifi() {
  if (digitalRead(BTN_WIFI) == LOW) {
    if (buttonPressStartTime == 0) {
      buttonPressStartTime = millis();
      btnOtaArmed = true;
      Serial.println("🔘 BTN_WIFI pressionado — aguardando...");
    } else {
      unsigned long held = millis() - buttonPressStartTime;

      // Feedback progressivo no Serial
      if (held >= 1000 && held < 1100) Serial.println("   1s...");
      if (held >= 2000 && held < 2100) Serial.println("   2s...");
      if (held >= 3000 && held < 3100) Serial.println("   3s...");
      if (held >= 4000 && held < 4100) Serial.println("   4s...");

      // ≥ 5s → abre o portal Wi-Fi (uma vez por pressão; o fancoil continua rodando)
      if (held >= buttonPressThreshold && btnOtaArmed) {
        btnOtaArmed = false;
        Serial.println("📶 5s atingido → PORTAL WI-FI");
        abrirPortalWifi();
      }
    }
  } else {
    // Botão solto
    if (buttonPressStartTime > 0 && btnOtaArmed) {
      unsigned long pressTime = millis() - buttonPressStartTime;
      Serial.printf("🔘 BTN_WIFI solto | tempo pressionado: %lums\n", pressTime);
      // 0.5s – 5s → modo OTA
      if (pressTime >= 500 && pressTime < buttonPressThreshold) {
        Serial.println("   → Modo OTA ativado");
        entrarModoOTA();
      } else if (pressTime < 500) {
        Serial.println("   → Pressão muito curta (< 0.5s), ignorada");
      }
    }
    buttonPressStartTime = 0;
    btnOtaArmed = false;
  }
}

// ─── Callbacks WiFiManager ────────────────────────────────────────────────────
void cbPortalIniciado(WiFiManager* wm) {
  portalAtivo = true;
  Serial.println("📶 Portal Wi-Fi ATIVO");
  Serial.printf( "   SSID: AutoConnect_%s\n", mqtt_id.c_str());
  Serial.println("   IP  : 192.168.4.1");
}
void cbWifiSalvo() {
  portalAtivo = false;
  Serial.println("✅ Credenciais Wi-Fi salvas!");
}

// ─── Task FreeRTOS: LED independente do loop() ────────────────────────────────
// Roda em paralelo — pisca mesmo durante autoConnect (bloqueante)
void taskLED(void* param) {
  while (true) {
    if (portalAtivo) {
      // Portal WiFi aberto → pisca lento (1x a cada 3s)
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      vTaskDelay(3000 / portTICK_PERIOD_MS);
    } else if (pingOK) {
      // Online → pisca rápido (1x a cada 300ms)
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      vTaskDelay(300 / portTICK_PERIOD_MS);
    } else {
      // Offline → LED fixo aceso
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

// ─── Ping (só LED / diagnóstico — não reinicia mais nada) ────────────────────
// IPs fixos (sem DNS, que podia bloquear até 15 s) e timeout de connect em ms explícito:
// no core 2.0.x WiFiClient::setTimeout() é em SEGUNDOS — o antigo setTimeout(800) virava 800 s.
bool pingInternet() {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClient pingClient;
  if (pingClient.connect(IPAddress(1, 1, 1, 1), 80, 1500))        { pingClient.stop(); return true; }
  if (pingClient.connect(IPAddress(208, 67, 222, 222), 80, 1500)) { pingClient.stop(); return true; }
  return false;
}

// ─── Diagnóstico (retidos) ────────────────────────────────────────────────────
void publicarDiagnostico() {
  mqttPublish(topic_rssi,   String(WiFi.RSSI()), true);
  mqttPublish(topic_uptime, String((uint32_t)(esp_timer_get_time() / 60000000ULL)), true);
}

// ─── Reconexão MQTT ───────────────────────────────────────────────────────────
void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.println("🔄 Reconectando MQTT...");
  client.disconnect();   // descarta sessão/socket meio-aberto antes de tentar de novo
  net.stop();
  if (client.connect(mqtt_id.c_str(), mqtt_username, mqtt_password)) {
    Serial.println("✅ MQTT conectado!");
    client.subscribe(topic_cmd,      1); Serial.println("📋 Subscrito: " + topic_cmd);
    client.subscribe(topic_estado,   1); Serial.println("📋 Subscrito: " + topic_estado);
    client.subscribe(topic_setpoint, 1); Serial.println("📋 Subscrito: " + topic_setpoint);
    client.subscribe(topic_schedule, 1); Serial.println("📋 Subscrito: " + topic_schedule);
    client.subscribe(topic_pressao,  1); Serial.println("📋 Subscrito: " + topic_pressao);
    modoAnterior   = -1;
    statusAnterior = -1;
    lerEPublicarEntradas();
    // Estado REAL em tópicos próprios. CMD/SET e ESTADO/SET são comandos de entrada —
    // a controladora não os republica (exceto ESTADO/SET no disparo da agenda).
    mqttPublish(topic_cmd_estado,  releAtivado ? "true" : "false", true);
    mqttPublish(topic_estado_real, estadoControladora == 1 ? "true" : "false", true);
    mqttPublish(topic_versao,      FIRMWARE_VERSION, true);
    if (pressaoAplicada >= 0) mqttPublish(topic_pressao_estado, String(pressaoAplicada, 1), true);
    mqttPublish(topic_reset, motivoReset, true);
    publicarDiagnostico();
    client.publish(topic_online.c_str(), "1", true, 1); // confirma presença (retido, QoS 1)
    publicarEstadoAgenda();
    vagPublicada = -1;   // VAG republicada no próximo ciclo do PID
  } else {
    Serial.printf("⚠️ MQTT falhou | erro: %d\n", client.lastError());
  }
}

// ─── Recuperação de rede em degraus (máquina de estados com millis) ──────────
void configurarWifi() {
  WiFi.setSleep(false);          // modem-sleep atrasa/perde pacotes e derruba o keepalive MQTT
  WiFi.setAutoReconnect(true);
}

unsigned long tsWifiCaiu     = 0;      // 0 = Wi-Fi conectado
unsigned long tsDegrauWifi   = 0;      // último degrau b/c executado
uint8_t       degrauWifi     = 0;      // 0 = nenhum | 1 = reconnect (b) | 2 = pilha reiniciada (c)
unsigned long tsMqttOK       = 0;      // última vez que o MQTT estava conectado
unsigned long tsResetPilha   = 0;      // último degrau c
bool          pilhaResetadaSemMqtt = false;   // houve degrau c desde a última conexão MQTT

// c) Wi-Fi off/on e begin() com as credenciais salvas (NVS). Bloqueia ~0,3 s.
static void reiniciarPilhaWifi(const char* porque) {
  Serial.printf("🛠️ [c] Reiniciando a pilha Wi-Fi (%s)\n", porque);
  client.disconnect();
  net.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(200);
  WiFi.mode(WIFI_STA);
  configurarWifi();
  WiFi.begin();
  tsResetPilha         = millis();
  pilhaResetadaSemMqtt = true;
}

void verificarRede() {
  unsigned long agora = millis();
  if (portalAtivo) return;   // portal de configuração no ar: ele cuida do Wi-Fi

  if (WiFi.status() == WL_CONNECTED) {
    if (tsWifiCaiu) {
      Serial.printf("✅ Wi-Fi voltou após %lus | IP %s | RSSI %d dBm\n",
                    (agora - tsWifiCaiu) / 1000UL, WiFi.localIP().toString().c_str(), WiFi.RSSI());
      tsWifiCaiu = 0;
      degrauWifi = 0;
    }
    if (client.connected()) {
      tsMqttOK = agora;
      pilhaResetadaSemMqtt = false;
      return;
    }

    // a) MQTT fora com Wi-Fi OK → tenta a cada 5 s
    static unsigned long tReconnect = 0;
    if (agora - tReconnect >= MQTT_RETRY_MS) {
      tReconnect = agora;
      Serial.printf("🔄 [a] MQTT fora há %lus com Wi-Fi OK\n", (agora - tsMqttOK) / 1000UL);
      reconnectMQTT();
      if (client.connected()) return;
    }

    unsigned long semMqtt = agora - tsMqttOK;
    if (semMqtt > MQTT_RESET_PILHA_MS && !pilhaResetadaSemMqtt) {
      reiniciarPilhaWifi("MQTT fora > 10 min com Wi-Fi OK");
    } else if (semMqtt > MQTT_REINICIO_MS && pilhaResetadaSemMqtt &&
               agora - tsResetPilha > WIFI_RESET_PILHA_MS) {
      Serial.println("❌ [d] MQTT fora > 30 min com Wi-Fi OK e degraus a–c esgotados");
      reinicioSuave("MQTT_30MIN");
    }
    return;
  }

  // Wi-Fi desconectado
  if (tsWifiCaiu == 0) {
    tsWifiCaiu = tsDegrauWifi = agora;
    degrauWifi = 0;
    Serial.printf("⚠️ Wi-Fi caiu (status %d)\n", WiFi.status());
  }
  unsigned long fora = agora - tsWifiCaiu;
  if (degrauWifi == 0 && fora > WIFI_RECONNECT_MS) {
    degrauWifi = 1;
    Serial.printf("🔄 [b] Wi-Fi fora há %lus → WiFi.reconnect()\n", fora / 1000UL);
    WiFi.reconnect();
  } else if (fora > WIFI_RESET_PILHA_MS && agora - tsDegrauWifi > WIFI_RESET_PILHA_MS) {
    degrauWifi   = 2;
    tsDegrauWifi = agora;
    reiniciarPilhaWifi("Wi-Fi fora > 3 min");   // repete a cada 3 min enquanto não voltar
  }
}

// ─── Callback MQTT ────────────────────────────────────────────────────────────
void messageReceived(String &topic, String &payload) {
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.printf("📩 RECEBIDO  [%s] → \"%s\"\n", topic.c_str(), payload.c_str());

  if (topic == topic_setpoint) {
    float sp = payload.toFloat();
    if (sp >= 10.0 && sp <= 35.0) {
      if (fabs(sp - setpoint) > 0.01) { setpoint = sp; salvarSetpoint(); }
      Serial.printf("   🌡️  Setpoint → %.1f°C\n", setpoint);
    } else {
      Serial.printf("   ⚠️ Setpoint fora da faixa (10–35°C): \"%s\" — ignorado\n", payload.c_str());
    }
  }
  else if (topic == topic_estado) {
    if (payloadVerdadeiro(payload) || payloadFalso(payload)) {
      bool automatico = payloadVerdadeiro(payload);
      if ((estadoControladora == 1) == automatico)
        Serial.println("   ℹ️ ESTADO igual ao atual — nada a fazer");
      else
        definirEstado(automatico, false);
    } else {
      Serial.printf("   ⚠️ ESTADO payload desconhecido: \"%s\"\n", payload.c_str());
    }
  }
  else if (topic == topic_pressao) {
    char* fim;
    float pr = strtof(payload.c_str(), &fim);
    if (payload.length() == 0 || *fim != '\0' || isnan(pr) || pr < 0.0f || pr > 100.0f) {
      Serial.printf("   ⚠️ PRESSAO fora da faixa (0–100%%) ou inválida: \"%s\" — ignorado\n", payload.c_str());
    } else if (fabsf(pr - pressaoSetpoint) <= 0.01f) {
      Serial.println("   ℹ️ PRESSAO igual à atual — nada a fazer");
    } else {
      pressaoSetpoint = pr;
      salvarPressao();
      Serial.printf("   💨 Pressão salva → %.1f%%\n", pressaoSetpoint);
      aplicarPressao();                      // só chega na AO2 com o relé LIGADO
    }
  }
  else if (topic == topic_schedule) {
    if (payload == agendaPayload) {
      Serial.println("   ℹ️ Agenda idêntica à salva — nada a gravar");
    } else {
      agendaPayload = payload;
      int n = interpretarAgenda(agendaPayload);
      salvarAgendaNVS();
      Serial.printf("   📅 Agenda substituída | %d slot(s) ativo(s)\n", n);
      if (n) Serial.println(descreverAgenda());
      agendaCatchupPendente = (n > 0);
    }
    publicarEstadoAgenda();
  }
  else if (topic == topic_cmd) {
    Serial.printf("   🔘 CMD | estadoControladora=%d (%s)\n",
                  estadoControladora, estadoControladora == 1 ? "AUTOMÁTICO" : "FORÇADO");
    if (!payloadVerdadeiro(payload) && !payloadFalso(payload)) {
      Serial.printf("   ⚠️ CMD payload desconhecido: \"%s\"\n", payload.c_str());
    } else if (estadoControladora == 1) {
      Serial.println("   ℹ️ CMD manual ignorado: AUTOMÁTICO (relé segue a agenda)");
    } else if (payloadVerdadeiro(payload) == releAtivado) {
      Serial.println("   ℹ️ CMD igual ao estado atual — nada a fazer");
    } else if (payloadVerdadeiro(payload)) {
      ligarRele("CMD/SET = LIGAR");
    } else {
      desligarRele("CMD/SET = DESLIGAR");
    }
  }
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

// ─── PID + VAG (chamado a cada CICLO_PID_MS) ──────────────────────────────────
void vagPID() {
  sensors.requestTemperatures();               // bloqueante, ~750 ms em 12 bits
  float leitura = sensors.getTempCByIndex(0);

  // DS18B20 retorna -127 em falha de comunicação e 85.0 no power-on reset
  if (leitura <= -100.0 || leitura >= 84.0) {
    Serial.printf("⚠️ Sensor DS18B20 FALHOU (leitura: %.1f°C) — VAG fechada, PID em MANUAL\n", leitura);
    if (!sensorEmFalha) {
      sensorEmFalha = true;
      pid1.SetMode(MANUAL);       // vagPid guarda o último valor para retomar sem salto
      escreverVAG(0);
      mqttPublish(topic_vag, "0", true);
      vagPublicada = 0;
      tsPubVag     = millis();
    }
    mqttPublish(topic_temperatura, "ERRO", true);
    return;
  }
  if (sensorEmFalha) {
    sensorEmFalha = false;
    temperatura   = NAN;          // filtro reinicia com a leitura atual
    Serial.println("✅ Sensor DS18B20 recuperado");
  }

  // Filtro EMA antes do PID
  temperatura = isnan(temperatura) ? leitura : temperatura + VAG_ALFA_EMA * (leitura - temperatura);
  mqttPublish(topic_temperatura, String(temperatura, 1), true);

  if (!releAtivado) {
    Serial.printf("   🌡️  Temp: %.2f°C (bruta %.2f) | Relé desligado\n", temperatura, leitura);
    publicarVAG(false);
    return;
  }

  // Volta da falha de sensor: Initialize() parte de vagPid (último valor) e da temperatura atual
  if (pid1.GetMode() == MANUAL) pid1.SetMode(AUTOMATIC);

  double erro = setpoint - temperatura;
  if (fabs(erro) <= VAG_ZONA_MORTA) {
    naZonaMorta = true;           // Compute() não roda: saída mantida e sem integrar
  } else {
    if (naZonaMorta) {            // saída da zona morta sem salto: PID retoma do valor aplicado
      naZonaMorta = false;
      vagPid = vagAtuador;
      pid1.SetMode(MANUAL);
      pid1.SetMode(AUTOMATIC);
    }
    pid1.Compute();

    // Limite de taxa + passo mínimo no atuador
    double alvo    = vagAtuador + constrain(vagPid - vagAtuador, -VAG_TAXA_MAX, VAG_TAXA_MAX);
    bool   extremo = (alvo <= 0.0 || alvo >= 100.0) && alvo != vagAtuador;   // fechar/abrir totalmente
    if (fabs(alvo - vagAtuador) >= VAG_PASSO_MIN || extremo) escreverVAG(alvo);
  }

  Serial.printf("   🌡️  Temp: %.2f°C (bruta %.2f) | SP: %.1f°C | PID: %.1f%% | VAG: %.1f%%%s\n",
                temperatura, leitura, setpoint, vagPid, vagAtuador, naZonaMorta ? " [zona morta]" : "");
  publicarVAG(false);
}

// ─── Restauro das saídas no boot ──────────────────────────────────────────────
// Roda no INÍCIO do setup(), antes de Serial/Wi-Fi/NTP/MQTT. Não passa por ligarRele/
// desligarRele e não publica nada: o relé não pisca e o supervisório não vê "false".
//  - Reset "quente" (software, watchdog, panic) com RTC válida → estado exato de antes.
//  - Senão (falta de energia, brownout, RTC inválida) → EEPROM: relé e pressão salvos, VAG de 0.
// Devolve true se veio da RTC.
static bool restaurarSaidas(esp_reset_reason_t rr) {
  bool quente = (rr == ESP_RST_SW || rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT ||
                 rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT);
  bool rtcOk = quente && rtcEstado.magic == RTC_MAGIC && rtcEstado.checksum == checksumRTC() &&
               rtcEstado.rele <= 1 && rtcEstado.estado <= 1 &&
               rtcEstado.vagAtuador >= 0.0f && rtcEstado.vagAtuador <= 100.0f &&
               rtcEstado.vagPid >= 0.0f && rtcEstado.vagPid <= 100.0f &&
               rtcEstado.pressaoAplicada >= -1.0f && rtcEstado.pressaoAplicada <= 100.0f;   // NaN reprova

  motivoReset = nomeResetReason(rr);
  if (rtcOk) {
    releAtivado        = rtcEstado.rele;
    estadoControladora = rtcEstado.estado;
    vagAtuador         = rtcEstado.vagAtuador;
    vagPid             = rtcEstado.vagPid;   // PID retoma daqui (AUTOMATIC na 1ª leitura de temperatura)
    pressaoAplicada    = rtcEstado.pressaoAplicada;
    if (rtcEstado.motivo[0]) motivoReset += String(" / ") + rtcEstado.motivo;
  } else {
    releAtivado     = (EEPROM.read(ADDR_RELE1) == RELE_LIGADO);
    vagAtuador      = 0;
    vagPid          = 0;
    pressaoAplicada = releAtivado ? max(pressaoSetpoint, PRESSAO_MIN_LIGADO) : 0.0f;
  }

  // Nível escrito ANTES de habilitar a saída: o pino já nasce no estado certo.
  // Se o hold do reinício anterior ainda estiver ativo, isto só vale quando ele for liberado.
  uint8_t nivel = releAtivado ? RELE_LIGADO : RELE_DESLIGADO;
  digitalWrite(RELE1_PIN, nivel);
  digitalWrite(RELE2_PIN, nivel);
  digitalWrite(RELE3_PIN, RELE_DESLIGADO);
  digitalWrite(RELE4_PIN, RELE_DESLIGADO);
  for (gpio_num_t p : PINOS_RELE) pinMode(p, OUTPUT);
  saidaAnalogica(AO_VAG, vagAtuador);
  saidaAnalogica(AO_PRESSAO, max(pressaoAplicada, 0.0f));

  // Só agora solta o hold: o pad passa a seguir a configuração acima (mesmo nível)
  travarReles(false);
  gpio_hold_dis((gpio_num_t)AO_VAG);
  gpio_hold_dis((gpio_num_t)AO_PRESSAO);
  travarReles(true);                      // relés voltam a ficar travados entre escritas

  rtcEstado.motivo[0] = '\0';             // motivo próprio vale só para este boot
  salvarRTC();
  return rtcOk;
}

// ─── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
  // 1) Fancoil primeiro: relés + AO1/AO2 no estado certo antes de qualquer coisa lenta
  esp_reset_reason_t rr = esp_reset_reason();
  EEPROM.begin(EEPROM_SIZE);
  carregarEstadoCtrl();
  bool restauroRTC = restaurarSaidas(rr);

  Serial.begin(115200);
  Serial.printf("\n♻️ Reset: %s | restauro: %s | relé=%s VAG=%.1f%% PRESSÃO=%.1f%%\n",
                motivoReset.c_str(), restauroRTC ? "RTC (reinício suave)" : "EEPROM",
                releAtivado ? "LIGADO" : "DESLIGADO", vagAtuador, pressaoAplicada);
  Serial.printf("📂 Estado | modo=%s | setpoint=%.1f°C | pressão salva=%.1f%%\n",
                estadoControladora == 1 ? "AUTOMÁTICO" : "FORÇADO", setpoint, pressaoSetpoint);

  carregarAgendaNVS();
  sensors.begin();
  sensors.setResolution(12);

  client.onMessage(messageReceived);
  pid1.SetOutputLimits(0, 100);          // saída em %; conversão para o DAC só em saidaAnalogica()
  pid1.SetSampleTime(CICLO_PID_MS);      // = intervalo real entre Compute()
  pid1.SetMode(MANUAL);                  // vai para AUTOMATIC (a partir de vagPid) na 1ª leitura válida

  pinMode(BTN_WIFI,         INPUT_PULLUP);
  pinMode(BTN_AUTO_MANUAL,  INPUT_PULLUP);
  pinMode(BTN_STATUS,       INPUT_PULLUP);
  pinMode(LED_PIN,          OUTPUT);
  digitalWrite(LED_PIN,   LOW);

  // ✅ ID gerado antes do Wi-Fi para usar no nome do AP
  mqtt_id = getShortUniqueID();

  String prefix = "TJS/" + mqtt_id;
  topic_temperatura = prefix + "/TEMPERATURA";
  topic_vag         = prefix + "/VAG";
  topic_modo        = prefix + "/MODO";
  topic_status      = prefix + "/STATUS";
  topic_cmd         = prefix + "/CMD/SET";
  topic_estado      = prefix + "/ESTADO/SET";
  topic_setpoint    = prefix + "/SETPOINT/SET";
  topic_online      = prefix + "/ONLINE";
  topic_schedule       = prefix + "/SCHEDULE/SET";
  topic_schedule_state = prefix + "/SCHEDULE/STATE";
  topic_cmd_estado     = prefix + "/CMD";
  topic_estado_real    = prefix + "/ESTADO";
  topic_pressao        = prefix + "/PRESSAO/SET";
  topic_versao         = prefix + "/VERSAO";
  topic_pressao_estado = prefix + "/PRESSAO";
  topic_reset          = prefix + "/RESET";
  topic_rssi           = prefix + "/RSSI";
  topic_uptime         = prefix + "/UPTIME";

  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("🔧 CONTROLADORA HVAC | ID: " + mqtt_id + " | v" FIRMWARE_VERSION);
  Serial.printf( "   RELE1   GPIO%-2d  NIVEL=%s\n", RELE1_PIN, RELE_LIGADO == HIGH ? "HIGH" : "LOW");
  Serial.printf( "   DS18B20 GPIO%-2d\n", ONE_WIRE_BUS);
  Serial.printf( "   AO1 VAG GPIO%-2d (DAC1)\n", AO_VAG);
  Serial.printf( "   AO2 PRS GPIO%-2d (DAC2)\n", AO_PRESSAO);
  Serial.printf( "   STATUS  GPIO%-2d\n", BTN_STATUS);
  Serial.printf( "   AUTO/MN GPIO%-2d\n", BTN_AUTO_MANUAL);
  Serial.printf( "   BTN_WIFI GPIO%-2d | curto=OTA | longo(5s)=portal Wi-Fi\n", BTN_WIFI);
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

  // ✅ Task LED independente do loop()
  xTaskCreate(taskLED, "LED", 2048, NULL, 1, NULL);

  // 2) Wi-Fi: tenta as credenciais salvas por até 20 s; sem rede, segue para o loop
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  configurarWifi();
  wifiManager.setAPCallback(cbPortalIniciado);
  wifiManager.setSaveConfigCallback(cbWifiSalvo);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConfigPortalTimeout(300);
  wifiManager.setConnectTimeout(20);
  wifiManager.setEnableConfigPortal(false);   // portal só pelo botão (ou sem credenciais)

  String apNome = "AutoConnect_" + mqtt_id;
  if (!wifiManager.getWiFiIsSaved()) {
    Serial.println("📶 Sem credenciais Wi-Fi salvas → portal aberto");
    abrirPortalWifi();
  } else {
    Serial.println("🔌 Conectando Wi-Fi (até 20 s)...");
    if (wifiManager.autoConnect(apNome.c_str())) {
      Serial.print("✅ Wi-Fi OK | IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("⚠️ Wi-Fi não conectou — seguindo sem rede (recuperação automática no loop)");
    }
  }
  configurarWifi();   // WiFiManager mexe no modo; reaplica sleep/auto-reconnect

  // 3) NTP sem esperar: a agenda aguarda a hora válida sozinha
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // 4) MQTT com timeouts curtos para nada travar o loop por mais de ~3 s
  net.setTimeout(3);           // SEGUNDOS no core 2.0.x (connect TCP)
  client.setTimeout(2000);     // ms — comandos MQTT (CONNACK, SUBACK…)
  client.begin(mqtt_server, net);
  client.setKeepAlive(20);
  client.setWill(topic_online.c_str(), "0", true, 1); // LWT: broker publica "0" retido se cair sem aviso
  reconnectMQTT();
  tsMqttOK = millis();         // conta os 30 min do degrau d a partir do boot

  pingOK = pingInternet();
  if (pingOK) ultimoPingOK = millis();
  tsPing = millis();

  // 5) Watchdog do loop principal
  esp_err_t wdt = esp_task_wdt_init(WDT_TIMEOUT_S, true);   // reconfigura se o core já iniciou
  if (wdt == ESP_OK) wdt = esp_task_wdt_add(NULL);
  Serial.printf("🐕 Watchdog %lus: %s\n", (unsigned long)WDT_TIMEOUT_S, wdt == ESP_OK ? "OK" : esp_err_to_name(wdt));
}

// ─── LOOP ─────────────────────────────────────────────────────────────────────
void loop() {
  esp_task_wdt_reset();
  client.loop();
  delay(10);

  if (wifiManager.getConfigPortalActive()) wifiManager.process();
  portalAtivo = wifiManager.getConfigPortalActive();   // inclui fechamento por timeout

  verificarRede();

  lerEPublicarEntradas();

  // ✅ Botão em função separada — chamado a cada iteração do loop
  verificarBotaoWifi();

  verificarAgenda();

  // LED controlado pela taskLED (FreeRTOS) — independente do loop()

  if (millis() - tsPing >= PING_MS) {
    tsPing = millis();
    pingOK = pingInternet();
    if (pingOK) ultimoPingOK = millis();
    else Serial.printf("❌ Ping falhou | sem internet há %lu min (só diagnóstico)\n",
                       (millis() - ultimoPingOK) / 60000UL);
  }

  static unsigned long tsDiag = 0;
  if (millis() - tsDiag >= DIAG_MS) {
    tsDiag = millis();
    if (client.connected()) publicarDiagnostico();
  }

  if (millis() - ts1 >= CICLO_PID_MS) {
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.printf("🔁 CICLO 30s | ESTADO: %s | Relé: %s | Wi-Fi: %s | MQTT: %s\n",
                  estadoControladora == 1 ? "AUTOMÁTICO" : "FORÇADO",
                  releAtivado ? "LIGADO" : "DESLIGADO",
                  WiFi.status() == WL_CONNECTED ? "OK" : "FORA",
                  client.connected() ? "OK" : "FORA");
    vagPID();
    salvarRTC();   // vagPid muda a cada Compute() mesmo sem mexer no atuador
    // ts1 marcado DEPOIS do Compute(): garante >= CICLO_PID_MS entre chamadas, senão o
    // PID_v1 (SetSampleTime igual ao ciclo) pularia um Compute() por jitter de 1 ms.
    ts1 = millis();
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  }
}
