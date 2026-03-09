#include <WiFi.h>
#include <MQTT.h>
#include <OneWire.h>
#include <PID_v1.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <WiFiManager.h>
#include <time.h>
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

#define EEPROM_SIZE       7
#define ADDR_RELE1        0
#define ADDR_DIAS         1   // bitmap: bit0=Dom, bit1=Seg ... bit6=Sáb
#define ADDR_HORA_LIGA    2   // hora de ligar (0–23)
#define ADDR_HORA_DESLIGA 3   // hora de desligar (0–23)

// ─── Versão do firmware ───────────────────────────────────────────────────────
#define FIRMWARE_VERSION "1.0.1"

// ─── OTA ──────────────────────────────────────────────────────────────────────
WebServer otaServer(80);
// OTA_AP_SENHA removida — AP aberto (sem senha)
#define OTA_TIMEOUT_MS 300000UL

// ─── Variáveis ────────────────────────────────────────────────────────────────
unsigned long buttonPressStartTime = 0;
bool          btnOtaArmed          = false;
const unsigned long buttonPressThreshold = 5000;

double temperatura, vag, setpoint;
PID pid1(&temperatura, &vag, &setpoint, KP1, KI1, KD1, REVERSE);

int  estadoControladora = 1;
bool releAtivado        = false;
bool portalAtivo        = false;

int modoAnterior   = -1;
int statusAnterior = -1;

unsigned long ts1          = 0;
unsigned long tsPing       = 0;
unsigned long ultimoPingOK = 0;
bool pingOK = false;

// ─── Agenda (carregada da EEPROM) ─────────────────────────────────────────────
uint8_t schedDias        = 0b00111110; // padrão: Seg–Sex (bits 1–5)
uint8_t schedHoraLiga    = 7;
uint8_t schedHoraDesliga = 21;

WiFiClient        net;
MQTTClient        client;
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

void salvarAgenda() {
  EEPROM.write(ADDR_DIAS,         schedDias);
  EEPROM.write(ADDR_HORA_LIGA,    schedHoraLiga);
  EEPROM.write(ADDR_HORA_DESLIGA, schedHoraDesliga);
  EEPROM.commit();
  Serial.printf("💾 Agenda salva | dias=0b%08b | liga=%02dh | desliga=%02dh\n",
                schedDias, schedHoraLiga, schedHoraDesliga);
}

void carregarAgenda() {
  uint8_t d = EEPROM.read(ADDR_DIAS);
  uint8_t h = EEPROM.read(ADDR_HORA_LIGA);
  uint8_t f = EEPROM.read(ADDR_HORA_DESLIGA);
  // 0xFF = EEPROM virgem → aplicar padrão Seg–Sex 07h–21h
  schedDias        = (d == 0xFF)           ? 0b00111110 : d;
  schedHoraLiga    = (h == 0xFF || h > 23) ? 7          : h;
  schedHoraDesliga = (f == 0xFF || f > 23) ? 21         : f;
  Serial.printf("📂 Agenda carregada | dias=0b%08b | liga=%02dh | desliga=%02dh\n",
                schedDias, schedHoraLiga, schedHoraDesliga);
}

// ─── MQTT publish com log ─────────────────────────────────────────────────────
// retained=true -> broker guarda ultima mensagem e entrega imediatamente a novos subscribers
void mqttPublish(const String& topic, const String& payload, bool retained = false) {
  bool ok = client.publish(topic, payload, retained, 0);
  Serial.printf("­ƒôñ ENVIADO   [%s] ÔåÆ \"%s\" %s%s\n",
                topic.c_str(), payload.c_str(),
                ok ? "Ô£à" : "ÔØî FALHOU",
                retained ? " ­ƒôî" : "");
}


// ─── MODO OTA ─────────────────────────────────────────────────────────────────
void entrarModoOTA() {
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("🔄 ENTRANDO EM MODO OTA");
  digitalWrite(RELE1_PIN, RELE_DESLIGADO);
  client.disconnect();
  WiFi.mode(WIFI_AP_STA);  // mantém STA conectado + sobe AP
  delay(300);

  String apNome = "HVAC-" + mqtt_id + "-OTA";
  WiFi.softAP(apNome.c_str());  // sem senha
  delay(500);

  IPAddress ip = WiFi.softAPIP();
  Serial.println("📶 AP OTA ATIVO");
  Serial.println("   SSID  : " + apNome);
  Serial.println("   Senha : (aberto)");
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
      "<a class='btn btn2' href='/config'>⏱️ Configurar Agenda</a><br>"
      "<a class='btn btn4' href='/mqtt-test'>🔌 Testar Conexão MQTT</a><br>"
      "<a class='btn btn3' href='/restart' onclick=\"return confirm('Reiniciar a controladora?')\">🔄 Reiniciar Controladora</a>"
      "</body></html>";
    otaServer.send(200, "text/html", html);
  });

  // ── Página de configuração de agenda ────────────────────────────────────────
  otaServer.on("/config", HTTP_GET, []() {
    // Monta checkboxes pré-marcados conforme schedDias
    // bit0=Dom, bit1=Seg, bit2=Ter, bit3=Qua, bit4=Qui, bit5=Sex, bit6=Sáb
    const char* nomes[] = {"Dom","Seg","Ter","Qua","Qui","Sex","S&aacute;b"};
    const char* ids[]   = {"dom","seg","ter","qua","qui","sex","sab"};
    String checkboxes = "";
    for (int i = 0; i < 7; i++) {
      bool marcado = (schedDias >> i) & 1;
      checkboxes += "<label style='margin:6px;font-size:18px'>"
                    "<input type='checkbox' name='" + String(ids[i]) + "' value='1'" +
                    (marcado ? " checked" : "") +
                    "> " + nomes[i] + "</label> ";
    }
    String html =
      "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>body{font-family:sans-serif;text-align:center;padding:20px}"
      "h2{color:#333}label{display:inline-block;margin:4px}"
      ".bloco{background:#f4f4f4;border-radius:8px;padding:16px;margin:12px auto;max-width:400px}"
      "input[type=number]{width:60px;font-size:20px;text-align:center;padding:4px}"
      ".btn{display:inline-block;padding:12px 32px;margin-top:16px;"
      "background:#28a745;color:white;border:none;border-radius:6px;"
      "font-size:18px;cursor:pointer}"
      ".voltar{display:inline-block;margin-top:12px;color:#0078d4;font-size:14px}"
      "</style></head><body>"
      "<h2>⏱️ Configurar Agenda</h2>"
      "<p><b>ID:</b> " + mqtt_id + "</p>"
      "<form method='POST' action='/config/save'>"
      "<div class='bloco'>"
      "<b>Dias ativos:</b><br><br>" + checkboxes +
      "</div>"
      "<div class='bloco'>"
      "<b>Hora ligar:</b> "
      "<input type='number' name='hliga' min='0' max='23' value='" + String(schedHoraLiga) + "'> h"
      "&nbsp;&nbsp;&nbsp;"
      "<b>Hora desligar:</b> "
      "<input type='number' name='hdesliga' min='0' max='23' value='" + String(schedHoraDesliga) + "'> h"
      "</div>"
      "<button class='btn' type='submit'>💾 Salvar</button>"
      "</form>"
      "<br><a class='voltar' href='/'>← Voltar</a>"
      "</body></html>";
    otaServer.send(200, "text/html", html);
  });

  // ── Salvar configuração de agenda ───────────────────────────────────────────
  otaServer.on("/config/save", HTTP_POST, []() {
    // Monta bitmap de dias
    uint8_t dias = 0;
    const char* ids[] = {"dom","seg","ter","qua","qui","sex","sab"};
    for (int i = 0; i < 7; i++) {
      if (otaServer.hasArg(ids[i])) dias |= (1 << i);
    }
    schedDias = dias;

    // Lê horários
    if (otaServer.hasArg("hliga")) {
      int v = otaServer.arg("hliga").toInt();
      if (v >= 0 && v <= 23) schedHoraLiga = v;
    }
    if (otaServer.hasArg("hdesliga")) {
      int v = otaServer.arg("hdesliga").toInt();
      if (v >= 0 && v <= 23) schedHoraDesliga = v;
    }

    salvarAgenda();

    // Redireciona para /config com banner de sucesso
    String html =
      "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
      "<meta http-equiv='refresh' content='2;url=/config'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>body{font-family:sans-serif;text-align:center;padding:40px}"
      ".ok{background:#d4edda;border-radius:8px;padding:20px;font-size:20px}"
      "</style></head><body>"
      "<div class='ok'>✅ Agenda salva!<br><small>Redirecionando...</small></div>"
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

// ─── Horário NTP ──────────────────────────────────────────────────────────────
bool dentroDoHorario() {
  struct tm t;
  if (!getLocalTime(&t)) {
    Serial.println("⚠️ NTP indisponível — retornando false");
    return false;
  }
  // tm_wday: 0=Dom 1=Seg 2=Ter 3=Qua 4=Qui 5=Sex 6=Sáb
  const char* diasSemana[] = {"Dom","Seg","Ter","Qua","Qui","Sex","Sáb"};
  bool diaAtivo = (schedDias >> t.tm_wday) & 1;
  bool horario;
  if (schedHoraDesliga == 0) {
    // desliga=0 significa meia-noite → liga das X até 23h59
    horario = (t.tm_hour >= schedHoraLiga);
  } else if (schedHoraDesliga > schedHoraLiga) {
    // normal: liga das X até Y no mesmo dia
    horario = (t.tm_hour >= schedHoraLiga && t.tm_hour < schedHoraDesliga);
  } else {
    // cruza meia-noite: ex. liga 22h desliga 06h
    horario = (t.tm_hour >= schedHoraLiga || t.tm_hour < schedHoraDesliga);
  }
  bool dentro = diaAtivo && horario;
  Serial.printf("🕐 %s %02d:%02d | Dia ativo: %s | Janela %02dh-%02dh: %s → %s\n",
                diasSemana[t.tm_wday], t.tm_hour, t.tm_min,
                diaAtivo ? "SIM" : "NÃO",
                schedHoraLiga, schedHoraDesliga,
                horario ? "SIM" : "NÃO",
                dentro  ? "DENTRO" : "FORA");
  return dentro;
}

// ─── Liga relé ────────────────────────────────────────────────────────────────
void ligarRele(const char* motivo) {
  Serial.printf("🔌 ligarRele() | motivo: %s | releAtivado: %s\n",
                motivo, releAtivado ? "já ligado" : "false");
  if (releAtivado) return;
  digitalWrite(RELE1_PIN, RELE_LIGADO);
  salvarRele(RELE_LIGADO);
  releAtivado = true;
  pid1.SetMode(AUTOMATIC);
  Serial.println("✅ RELÉ LIGADO");
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
  salvarRele(RELE_DESLIGADO);
  releAtivado = false;
  Serial.println("⛔ RELÉ DESLIGADO");
  mqttPublish(topic_vag, "0", true);
  mqttPublish(topic_cmd, "false", true);
}

// ─── Aplica horário ───────────────────────────────────────────────────────────
void aplicarHorario() {
  Serial.println("📅 aplicarHorario()");
  if (dentroDoHorario()) ligarRele("ESTADO=1 horário 07h-21h");
  else                   desligarRele("ESTADO=1 fora do horário");
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
  pingClient.setTimeout(1500);   // ✅ reduzido de 3s → 1.5s por host (total max 4.5s)
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
    client.subscribe(topic_cmd);      Serial.println("📋 Subscrito: " + topic_cmd);
    client.subscribe(topic_estado);   Serial.println("📋 Subscrito: " + topic_estado);
    client.subscribe(topic_setpoint); Serial.println("📋 Subscrito: " + topic_setpoint);
    modoAnterior   = -1;
    statusAnterior = -1;
    lerEPublicarEntradas();
    mqttPublish(topic_cmd,    releAtivado ? "true" : "false", true);
    mqttPublish(topic_estado, estadoControladora == 1 ? "true" : "false", true);
  } else {
    Serial.printf("⚠️ MQTT falhou | erro: %d\n", client.lastError());
  }
}

// ─── Callback MQTT ────────────────────────────────────────────────────────────
void messageReceived(String &topic, String &payload) {
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.printf("📩 RECEBIDO  [%s] → \"%s\"\n", topic.c_str(), payload.c_str());

  if (topic == topic_setpoint) {
    setpoint = payload.toFloat();
    Serial.printf("   🌡️  Setpoint → %.1f°C\n", setpoint);
  }
  else if (topic == topic_estado) {
    if (payloadVerdadeiro(payload)) {
      estadoControladora = 1;
      Serial.println("   🔄 Controladora → HORÁRIO (07h liga / 21h desliga)");
      aplicarHorario();
    } else if (payloadFalso(payload)) {
      estadoControladora = 0;
      Serial.println("   🔄 Controladora → CMD (aguarda TJS/CMD/SET)");
    } else {
      Serial.printf("   ⚠️ ESTADO payload desconhecido: \"%s\"\n", payload.c_str());
    }
  }
  else if (topic == topic_cmd) {
    Serial.printf("   🔘 CMD | estadoControladora=%d (%s)\n",
                  estadoControladora, estadoControladora == 1 ? "HORÁRIO" : "CMD");
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
  temperatura = sensors.getTempCByIndex(0);

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
  Serial.begin(9600);
  EEPROM.begin(EEPROM_SIZE);
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
  xTaskCreate(taskLED, "LED", 1024, NULL, 1, NULL);

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
  reconnectMQTT();

  carregarAgenda();
  if (estadoControladora == 1) aplicarHorario();

  pingOK = pingInternet();
  if (pingOK) ultimoPingOK = millis();
  tsPing = millis();
}

// ─── LOOP ─────────────────────────────────────────────────────────────────────
void loop() {
  client.loop();
  delay(10);

  if (!client.connected()) reconnectMQTT();

  lerEPublicarEntradas();

  // ✅ Botão em função separada — chamado a cada iteração do loop
  verificarBotaoWifi();

  // LED controlado pela taskLED (FreeRTOS) — independente do loop()

  if (millis() - tsPing >= 30000UL) {
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
                  estadoControladora == 1 ? "HORÁRIO" : "CMD",
                  releAtivado ? "LIGADO" : "DESLIGADO");
    if (estadoControladora == 1) aplicarHorario();
    vagPID();
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  }
}
