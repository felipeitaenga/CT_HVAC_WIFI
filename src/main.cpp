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

// ─── Hardware ─────────────────────────────────────────────────────────────────
#define ONE_WIRE_BUS    13
#define PWM_PIN1        26
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

#define KP1  500
#define KI1  5
#define KD1  2

#define EEPROM_SIZE       16
#define ADDR_RELE1        0
// endereços 1–3: livres (antiga janela horária local — agenda agora vem só do supervisório, no NVS)
#define ADDR_ESTADO_CTRL  4   // estadoControladora (0=FORÇADO, 1=AUTOMÁTICO)
#define ADDR_SETPOINT     8   // float (4 bytes) — setpoint persistido

// ─── Versão do firmware ───────────────────────────────────────────────────────
#define FIRMWARE_VERSION "1.1.0"

// ─── OTA ──────────────────────────────────────────────────────────────────────
WebServer otaServer(80);
// OTA_AP_SENHA removida — AP aberto (sem senha)
#define OTA_TIMEOUT_MS 300000UL

// ─── Variáveis ────────────────────────────────────────────────────────────────
unsigned long buttonPressStartTime = 0;
bool          btnOtaArmed          = false;
const unsigned long buttonPressThreshold = 5000;

double temperatura, vag, setpoint = 23.0;   // default seguro (PID REVERSE: 0 → fan 100%)
PID pid1(&temperatura, &vag, &setpoint, KP1, KI1, KD1, REVERSE);

int  estadoControladora = 1;
bool releAtivado        = false;
volatile bool portalAtivo = false;   // escrito no loop/callbacks, lido na taskLED

int modoAnterior   = -1;
int statusAnterior = -1;

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

  Serial.printf("📂 Estado carregado | modo=%s | setpoint=%.1f°C\n",
                estadoControladora == 1 ? "AUTOMÁTICO" : "FORÇADO", setpoint);
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


// ─── MODO OTA ─────────────────────────────────────────────────────────────────
void entrarModoOTA() {
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("🔄 ENTRANDO EM MODO OTA");
  digitalWrite(RELE1_PIN, RELE_DESLIGADO);
  digitalWrite(RELE2_PIN, RELE_DESLIGADO);   // exaustor acompanha o fancoil
  client.publish(topic_online.c_str(), "0", true, 1); // saída deliberada (OTA) também marca offline
  client.disconnect();
  WiFi.mode(WIFI_AP_STA);  // mantém STA conectado + sobe AP
  delay(300);

  String apNome  = "HVAC-" + mqtt_id + "-OTA";
  String apSenha = "hvac" + mqtt_id + "ota";   // >= 8 chars — fecha flash anônimo por rádio
  WiFi.softAP(apNome.c_str(), apSenha.c_str());
  delay(500);

  IPAddress ip = WiFi.softAPIP();
  Serial.println("📶 AP OTA ATIVO");
  Serial.println("   SSID  : " + apNome);
  Serial.println("   Senha : " + apSenha);
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
    ESP.restart();
  });

  // ── Teste de conexão MQTT ────────────────────────────────────────────────────
  otaServer.on("/mqtt-test", HTTP_GET, []() {
    bool wifiOK = (WiFi.status() == WL_CONNECTED);
    bool mqttOK = false;
    unsigned long t0 = millis();

    if (wifiOK) {
      WiFiClient testNet;
      testNet.setTimeout(3000);
      MQTTClient testClient(256);
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
    ESP.restart();
  });

  ElegantOTA.setAuth("admin", apSenha.c_str());
  ElegantOTA.begin(&otaServer);
  otaServer.begin();

  unsigned long tOTA    = millis();
  unsigned long tPiscar = millis();

  while (millis() - tOTA < OTA_TIMEOUT_MS) {
    otaServer.handleClient();
    ElegantOTA.loop();
    if (millis() - tPiscar >= 50) {
      tPiscar = millis();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }

  Serial.println("⏱️ Timeout OTA. Reiniciando...");
  delay(500);
  ESP.restart();
}

// ─── Liga relé ────────────────────────────────────────────────────────────────
void ligarRele(const char* motivo) {
  Serial.printf("🔌 ligarRele() | motivo: %s | releAtivado: %s\n",
                motivo, releAtivado ? "já ligado" : "false");
  if (releAtivado) return;
  digitalWrite(RELE1_PIN, RELE_LIGADO);
  digitalWrite(RELE2_PIN, RELE_LIGADO);   // exaustor acompanha o fancoil
  salvarRele(RELE_LIGADO);
  releAtivado = true;
  pid1.SetMode(AUTOMATIC);
  Serial.println("✅ RELÉ LIGADO (fancoil + exaustor)");
  mqttPublish(topic_cmd, "true", true);
}

// ─── Desliga relé ─────────────────────────────────────────────────────────────
void desligarRele(const char* motivo) {
  Serial.printf("🔌 desligarRele() | motivo: %s | releAtivado: %s\n",
                motivo, releAtivado ? "true" : "já desligado");
  if (!releAtivado) return;
  pid1.SetMode(MANUAL);
  vag = 0;
  analogWrite(PWM_PIN1, 0);
  digitalWrite(RELE1_PIN, RELE_DESLIGADO);
  digitalWrite(RELE2_PIN, RELE_DESLIGADO);   // exaustor acompanha o fancoil
  salvarRele(RELE_DESLIGADO);
  releAtivado = false;
  Serial.println("⛔ RELÉ DESLIGADO (fancoil + exaustor)");
  mqttPublish(topic_vag, "0", true);
  mqttPublish(topic_cmd, "false", true);
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

// ─── Agenda: execução ─────────────────────────────────────────────────────────
// Aplica direto nas funções internas (funciona sem broker) e publica o novo estado retido
// para o supervisório acompanhar e para o retained não reverter o modo no próximo boot.
static void definirEstado(bool automatico) {
  int novo = automatico ? 1 : 0;
  if (estadoControladora != novo) { estadoControladora = novo; salvarEstadoCtrl(); }
  mqttPublish(topic_estado, automatico ? "true" : "false", true);
}

static void executarSlot(const SlotAgenda& s) {
  static const char* nomes[] = {"ESTADO", "CMD", "SETPOINT"};
  Serial.printf("⏰ AGENDA %02d:%02d → %s = %.1f\n", s.hora, s.minuto, nomes[s.acao], s.valor);
  switch (s.acao) {
    case ACAO_ESTADO:
      definirEstado(s.valor > 0);
      break;
    case ACAO_CMD:
      definirEstado(false);   // CMD só vale em modo FORÇADO
      if (s.valor > 0) ligarRele("agenda");
      else             desligarRele("agenda");
      break;
    case ACAO_SETPOINT:
      if (fabs(s.valor - setpoint) > 0.01) { setpoint = s.valor; salvarSetpoint(); }
      mqttPublish(topic_setpoint, String(setpoint, 1), true);
      break;
  }
}

// Aplica o último evento já passado de cada ação (até 7 dias atrás), do mais antigo ao mais novo.
// Cobre queda de energia (boot às 07:05 com slot das 07:00) e agenda recém-recebida.
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

  bool feito[3] = {false, false, false};
  for (int k = 0; k < 3; k++) {
    int melhor = -1;
    for (int a = 0; a < 3; a++)
      if (!feito[a] && idx[a] >= 0 && (melhor < 0 || ago[a] > ago[melhor])) melhor = a;
    if (melhor < 0) break;
    feito[melhor] = true;
    executarSlot(slots[idx[melhor]]);
  }
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

// ─── Entradas digitais → publica se mudou ────────────────────────────────────
void lerEPublicarEntradas() {
  int modoAtual = (digitalRead(BTN_AUTO_MANUAL) == LOW) ? 1 : 0;
  if (modoAtual != modoAnterior) {
    modoAnterior = modoAtual;
    Serial.printf("🔁 MODO (GPIO%d) → %s\n", BTN_AUTO_MANUAL, modoAtual ? "AUTO" : "MANUAL");
    mqttPublish(topic_modo, String(modoAtual));
  }

  int statusAtual = (digitalRead(BTN_STATUS) == LOW) ? 1 : 0;
  if (statusAtual != statusAnterior) {
    statusAnterior = statusAtual;
    Serial.printf("🔁 STATUS (GPIO%d) → %s\n", BTN_STATUS, statusAtual ? "LIGADO" : "DESLIGADO");
    mqttPublish(topic_status, String(statusAtual));
  }
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

      // ≥ 5s → reset WiFi (dispara enquanto pressionado)
      if (held >= buttonPressThreshold) {
        btnOtaArmed = false;
        Serial.println("🔄 5s atingido → RESET WI-FI");
        wifiManager.resetSettings();
        delay(500);
        ESP.restart();
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

// ─── ✅ Ping com timeout reduzido para não bloquear o botão ──────────────────
bool pingInternet() {
  WiFiClient pingClient;
  pingClient.setTimeout(800);    // 0.8s por host (total máx ~2.4s) — evita travar o loop()
  if (pingClient.connect("google.com", 80))     { pingClient.stop(); Serial.println("✅ Ping OK → google.com:80");  return true; }
  if (pingClient.connect("1.1.1.1", 80))        { pingClient.stop(); Serial.println("✅ Ping OK → 1.1.1.1:80");     return true; }
  if (pingClient.connect("208.67.222.222", 80)) { pingClient.stop(); Serial.println("✅ Ping OK → OpenDNS:80");     return true; }
  return false;
}

// ─── Reconexão MQTT ───────────────────────────────────────────────────────────
void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.println("🔄 Reconectando MQTT...");
  if (client.connect(mqtt_id.c_str(), mqtt_username, mqtt_password)) {
    Serial.println("✅ MQTT conectado!");
    client.subscribe(topic_cmd,      1); Serial.println("📋 Subscrito: " + topic_cmd);
    client.subscribe(topic_estado,   1); Serial.println("📋 Subscrito: " + topic_estado);
    client.subscribe(topic_setpoint, 1); Serial.println("📋 Subscrito: " + topic_setpoint);
    client.subscribe(topic_schedule, 1); Serial.println("📋 Subscrito: " + topic_schedule);
    modoAnterior   = -1;
    statusAnterior = -1;
    lerEPublicarEntradas();
    mqttPublish(topic_cmd,    releAtivado ? "true" : "false", true);
    // NÃO republicar topic_estado: ESTADO/SET é comando de entrada — só o SCADA escreve.
    // A controladora apenas consome o retido do broker; o valor local é persistido na EEPROM.
    client.publish(topic_online.c_str(), "1", true, 1); // confirma presença (retido, QoS 1)
    publicarEstadoAgenda();
  } else {
    Serial.printf("⚠️ MQTT falhou | erro: %d\n", client.lastError());
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
    if (payloadVerdadeiro(payload)) {
      if (estadoControladora != 1) { estadoControladora = 1; salvarEstadoCtrl(); }
      Serial.println("   🔄 Controladora → AUTOMÁTICO (CMD manual ignorado)");
    } else if (payloadFalso(payload)) {
      if (estadoControladora != 0) { estadoControladora = 0; salvarEstadoCtrl(); }
      Serial.println("   🔄 Controladora → FORÇADO (aceita TJS/CMD/SET)");
    } else {
      Serial.printf("   ⚠️ ESTADO payload desconhecido: \"%s\"\n", payload.c_str());
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
    if (estadoControladora == 0) {
      if (payloadVerdadeiro(payload))      ligarRele("CMD/SET = LIGAR");
      else if (payloadFalso(payload))      desligarRele("CMD/SET = DESLIGAR");
      else Serial.printf("   ⚠️ CMD payload desconhecido: \"%s\"\n", payload.c_str());
    } else {
      String estadoReal = releAtivado ? "true" : "false";
      if (payload != estadoReal) {
        Serial.printf("   🔄 Corrigindo ScadaBR: CMD real = \"%s\"\n", estadoReal.c_str());
        mqttPublish(topic_cmd, estadoReal, true);
      } else {
        Serial.println("   ℹ️ CMD ignorado → ScadaBR já sincronizado");
      }
    }
  }
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

// ─── PID + VAG ────────────────────────────────────────────────────────────────
float convertToPercentage(int v) { return (v / 255.0) * 100.0; }

void vagPID() {
  sensors.requestTemperatures();
  float leitura = sensors.getTempCByIndex(0);

  // DS18B20 retorna -127 em falha de comunicação e 85.0 no power-on reset
  if (leitura <= -100.0 || leitura >= 84.0) {
    Serial.printf("⚠️ Sensor DS18B20 FALHOU (leitura: %.1f°C) — PID suspenso\n", leitura);
    analogWrite(PWM_PIN1, 0);
    mqttPublish(topic_temperatura, "ERRO", false);
    return;
  }

  temperatura = leitura;

  if (releAtivado) {
    pid1.Compute();
    int   pwm = (int)vag;
    float pct = convertToPercentage(pwm);
    analogWrite(PWM_PIN1, pwm);
    Serial.printf("   🌡️  Temp: %.1f°C | Setpoint: %.1f°C | PWM: %d (%.1f%%)\n",
                  temperatura, setpoint, pwm, pct);
    mqttPublish(topic_temperatura, String(temperatura, 1), true);
    mqttPublish(topic_vag,         String(pct, 1), true);
  } else {
    analogWrite(PWM_PIN1, 0);
    Serial.printf("   🌡️  Temp: %.1f°C | Relé desligado\n", temperatura);
    mqttPublish(topic_temperatura, String(temperatura, 1), true);
    mqttPublish(topic_vag,         "0", true);
  }
}

// ─── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  carregarAgendaNVS();
  carregarEstadoCtrl();
  sensors.begin();

  client.onMessage(messageReceived);
  pid1.SetOutputLimits(0, 255);
  pid1.SetMode(MANUAL);

  pinMode(BTN_WIFI,         INPUT_PULLUP);
  pinMode(BTN_AUTO_MANUAL,  INPUT_PULLUP);
  pinMode(BTN_STATUS,       INPUT_PULLUP);
  pinMode(LED_PIN,          OUTPUT);
  pinMode(RELE1_PIN,        OUTPUT);
  pinMode(RELE2_PIN,        OUTPUT);
  pinMode(RELE3_PIN,        OUTPUT);
  pinMode(RELE4_PIN,        OUTPUT);
  pinMode(PWM_PIN1,         OUTPUT);

  digitalWrite(LED_PIN,   LOW);
  digitalWrite(RELE1_PIN, RELE_DESLIGADO);
  digitalWrite(RELE2_PIN, RELE_DESLIGADO);
  digitalWrite(RELE3_PIN, RELE_DESLIGADO);
  digitalWrite(RELE4_PIN, RELE_DESLIGADO);

  // ✅ ID gerado antes do autoConnect para usar no nome do AP
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

  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("🔧 CONTROLADORA HVAC | ID: " + mqtt_id + " | v" FIRMWARE_VERSION);
  Serial.printf( "   RELE1   GPIO%-2d  NIVEL=%s\n", RELE1_PIN, RELE_LIGADO == HIGH ? "HIGH" : "LOW");
  Serial.printf( "   DS18B20 GPIO%-2d\n", ONE_WIRE_BUS);
  Serial.printf( "   PWM     GPIO%-2d\n", PWM_PIN1);
  Serial.printf( "   STATUS  GPIO%-2d\n", BTN_STATUS);
  Serial.printf( "   AUTO/MN GPIO%-2d\n", BTN_AUTO_MANUAL);
  Serial.printf( "   BTN_WIFI GPIO%-2d | curto=OTA | longo(5s)=reset WiFi\n", BTN_WIFI);
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

  wifiManager.setAPCallback(cbPortalIniciado);
  wifiManager.setSaveConfigCallback(cbWifiSalvo);
  wifiManager.setConfigPortalTimeout(300);

  // ✅ Task LED iniciada ANTES do autoConnect (que é bloqueante)
  xTaskCreate(taskLED, "LED", 2048, NULL, 1, NULL);

  // ✅ AP com nome incluindo ID da controladora
  String apNome = "AutoConnect_" + mqtt_id;
  Serial.println("🔌 Conectando Wi-Fi...");
  Serial.println("   Se não conectar → AP: " + apNome);
  if (!wifiManager.autoConnect(apNome.c_str())) {
    Serial.println("❌ Portal AP timeout. Reiniciando...");
    delay(1000);
    ESP.restart();
  }
  Serial.print("✅ Wi-Fi OK | IP: ");
  Serial.println(WiFi.localIP());

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("🕐 Sincronizando NTP...");
  struct tm t;
  for (int i = 0; i < 10 && !getLocalTime(&t); i++) delay(1000);
  if (getLocalTime(&t))
    Serial.printf("✅ Hora: %02d:%02d:%02d\n", t.tm_hour, t.tm_min, t.tm_sec);
  else
    Serial.println("⚠️ NTP não sincronizou no setup");

  client.begin(mqtt_server, net);
  client.setKeepAlive(20);   // mais tolerante a bloqueios curtos do loop() (ping, reconnect)
  client.setWill(topic_online.c_str(), "0", true, 1); // LWT: broker publica "0" retido se cair sem aviso
  reconnectMQTT();

  // Sem janela local: restaura o último estado do relé; a agenda corrige no próximo evento
  if (EEPROM.read(ADDR_RELE1) == RELE_LIGADO) ligarRele("restauro pós-boot");

  pingOK = pingInternet();
  if (pingOK) ultimoPingOK = millis();
  tsPing = millis();
}

// ─── LOOP ─────────────────────────────────────────────────────────────────────
void loop() {
  client.loop();
  delay(10);

  static unsigned long tReconnect = 0;
  if (!client.connected() && millis() - tReconnect >= 5000UL) {
    tReconnect = millis();
    reconnectMQTT();
  }

  lerEPublicarEntradas();

  // ✅ Botão em função separada — chamado a cada iteração do loop
  verificarBotaoWifi();

  verificarAgenda();

  // LED controlado pela taskLED (FreeRTOS) — independente do loop()

  if (millis() - tsPing >= 60000UL) {
    tsPing = millis();
    pingOK = pingInternet();
    if (pingOK) ultimoPingOK = millis();
    else Serial.printf("❌ Ping falhou | sem internet há %lu min\n",
                       (millis() - ultimoPingOK) / 60000UL);
  }

  if (millis() - ultimoPingOK > 7200000UL) {
    Serial.println("❌ Sem internet há 2h. Reiniciando...");
    delay(100);
    ESP.restart();
  }

  if (millis() - ts1 >= 30000UL) {
    ts1 = millis();
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.printf("🔁 CICLO 30s | ESTADO: %s | Relé: %s\n",
                  estadoControladora == 1 ? "AUTOMÁTICO" : "FORÇADO",
                  releAtivado ? "LIGADO" : "DESLIGADO");
    vagPID();
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  }
}
