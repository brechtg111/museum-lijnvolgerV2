// =============================================================
// MUSEUM LIJNVOLGER V2
// ESP32 - 3 IR-sensoren, SN754410NE motordriver, HC-SR04 + servo, WiFi + MQTT
// Volgt zwarte lijn; wijkt rechts uit bij obstakels < OBSTAKEL_AFSTAND cm.
// GPIO 12 = ADC2 → conflicteert met WiFi; batterij éénmalig meten vóór WiFi-start.
// =============================================================

#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// --- WiFi / MQTT ---
const char* ssid            = "brechtiot";
const char* password        = "brechtiot";
const char* mqtt_server     = "192.168.1.154";
const int   mqtt_port       = 1883;
const char* mqtt_user       = "wagon01";
const char* mqtt_pass       = "pi123";
const char* TOPIC_TELEMETRY = "museum/wagon/01/telemetry";
const char* TOPIC_ALERT     = "museum/wagon/01/event/error";
const char* TOPIC_CMD_STOP  = "museum/wagon/01/cmd/stop";
const char* TOPIC_CMD_START = "museum/wagon/01/cmd/start";
const char* TOPIC_BROADCAST = "museum/system/broadcast/stop";

WiFiClient   espClient;
PubSubClient mqttClient(espClient);
unsigned long lastMqtt      = 0;
const long    MQTT_INTERVAL = 300;  // telemetrie interval (ms)

String statusTekst = "START";
int    afstandCm   = -1;
bool   sL = false, sM = false, sR = false;

// --- Pinnen ---
#define IR_L 34
#define IR_M 35
#define IR_R 32
#define ENA 19
#define IN1 18
#define IN2  5
#define ENB 16
#define IN3  4
#define IN4  2
#define TRIG_PIN  25
#define ECHO_PIN  27
#define SERVO_PIN 33
#define BTN_START  15
#define BTN_STOP   26
#define BAT_PIN    12
#define LED_GREEN  21
#define LED_YELLOW 22
#define LED_RED    23
#define ENA_KANAAL 4   // PWM-kanaal linkermotor  (vermijdt servo-timers 0-3)
#define ENB_KANAAL 6   // PWM-kanaal rechtermotor

// --- Lijnvolg-parameters ---
// Q: majority-vote buffergrootte | baseSpeed/correction/pivotSpeed: PWM 0-255
#define Q 4
int baseSpeed  = 190;
int correction = 240;
int pivotSpeed = 210;

// --- Obstakel-parameters (tijden ms, afstanden cm) ---
#define SERVO_VOORUIT        90   // servo-hoek recht vooruit
#define SERVO_OBJECT        170   // servo-hoek zijwaarts naar obstakel
#define OBSTAKEL_AFSTAND     25   // detectiedrempel (cm)
#define OBJECT_ZIJ_MAX       25   // max afstand om object nog "aanwezig" te noemen
#define MAX_AFSTAND         100   // metingen boven dit worden -1
#define RIJD_SNELHEID       195
#define DRAAI_SNELHEID      200
#define DRAAI_90_MS         750   // ms voor ~90° draai
#define DRAAI_135_MS        400   // ms voor ~135° draai
#define SIDESTEP_MS        700   // ms zijwaarts rijden naast obstakel
#define EXTRA_MS            500   // ms extra vooruit na voorbijrijden
#define PAS_TIMEOUT_MS     1000   // veiligheidsgrens voorbijrij-tijd
#define ULTRA_INTERVAL      100   // meet-interval ultrasoon (ms)
#define LIJN_ZOEK_MS       2500   // max zoektijd lijn na ontwijking
#define OBSTAKEL_COOLDOWN_MS 10000 // wachttijd voor nieuwe obstakeldetectie

Servo scanServo;

// Majority-vote ringbuffers per IR-sensor
int bufL[Q] = {}, bufM[Q] = {}, bufR[Q] = {};
int bufIndex = 0;

int currentLeft = 0, currentRight = 0;
int lastLeft    = 0, lastRight    = 0;
int lastDir     = 1;  // 1 = rechts, -1 = links (laatste bekende rijrichting)

unsigned long lastUltra         = 0;
unsigned long obstakelNegeerTot = 0;

bool running                       = false;
unsigned long stopTime             = 0;
bool stopTimerActive               = false;
const unsigned long STOP_DURATION  = 30000;

float batteryV   = 8.4;
int   batteryPct = 100;

// Zoek- en kruispunttimers
#define ZOEK_FASE1_MS  1500  // ms draaien in lastDir bij lijn kwijt
#define ZOEK_FASE2_MS  1500  // ms draaien omgekeerde richting
#define DWARSLIJN_MS    200  // ms alle 3 sensoren actief voor écht kruispunt
unsigned long zoekStart      = 0;
unsigned long dwarslijnStart = 0;

enum State { STRAIGHT, TURN_L, TURN_R, PIVOT_L, PIVOT_R, NONE };
State prevState = NONE;


// =============================================================
// Hulpfuncties
// =============================================================

// true als meer dan helft van buffer HIGH is (ruisfilter)
bool majorityVote(int* buf) {
  int sum = 0;
  for (int i = 0; i < Q; i++) sum += buf[i];
  return sum > (Q / 2);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0); ledcWrite(ENB, 0);
  currentLeft = 0; currentRight = 0;
}

// Positief = vooruit, negatief = achteruit (-255 t/m 255)
void setMotors(int l, int r) {
  digitalWrite(IN1, l >= 0 ? LOW : HIGH);
  digitalWrite(IN2, l >= 0 ? HIGH : LOW);
  if (l < 0) l = -l;
  digitalWrite(IN3, r >= 0 ? LOW : HIGH);
  digitalWrite(IN4, r >= 0 ? HIGH : LOW);
  if (r < 0) r = -r;
  ledcWrite(ENA, constrain(l, 0, 255));
  ledcWrite(ENB, constrain(r, 0, 255));
  lastLeft = l; lastRight = r;
}

// Geleidelijk optrekken naar doelsnelheid (alleen rechtdoor, voorkomt wiel-slip)
void stepMotorsFade(int tL, int tR, int step = 8) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  if (currentLeft  < tL) currentLeft  = min(currentLeft  + step, tL);
  if (currentLeft  > tL) currentLeft  = max(currentLeft  - step, tL);
  if (currentRight < tR) currentRight = min(currentRight + step, tR);
  if (currentRight > tR) currentRight = max(currentRight - step, tR);
  ledcWrite(ENA, constrain(currentLeft,  0, 255));
  ledcWrite(ENB, constrain(currentRight, 0, 255));
  lastLeft = currentLeft; lastRight = currentRight;
}

void vooruit(int s) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  ledcWrite(ENA, s); ledcWrite(ENB, s);
}

// Draai ter plaatse t ms, daarna stop + 200 ms rust
void draaiRechts(int t) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, DRAAI_SNELHEID); ledcWrite(ENB, DRAAI_SNELHEID);
  delay(t); stopMotors(); delay(200);
}

void draaiLinks(int t) {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(ENA, DRAAI_SNELHEID); ledcWrite(ENB, DRAAI_SNELHEID);
  delay(t); stopMotors(); delay(200);
}

void vooruitTijd(int t) { vooruit(RIJD_SNELHEID); delay(t); stopMotors(); delay(200); }

// HC-SR04: geeft afstand in cm, of -1 bij geen echo / te ver
int meetAfstand() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long d = pulseIn(ECHO_PIN, HIGH, 30000);
  if (d == 0) return -1;
  int cm = d / 58;
  return (cm > MAX_AFSTAND) ? -1 : cm;
}

// Servo geleidelijk naar doel-hoek (15 ms/stap voor soepele beweging)
void servoNaar(int doel) {
  int h = scanServo.read();
  if (h < doel) for (; h <= doel; h++) { scanServo.write(h); delay(15); }
  else          for (; h >= doel; h--) { scanServo.write(h); delay(15); }
}


// =============================================================
// WiFi / MQTT
// =============================================================

void connectWiFi() {
  WiFi.mode(WIFI_STA); WiFi.begin(ssid, password);
  Serial.print("WiFi verbinden");
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) { delay(300); Serial.print("."); }
  if (WiFi.status() == WL_CONNECTED) { WiFi.setSleep(false); Serial.println(" OK: " + WiFi.localIP().toString()); }
  else Serial.println(" geen WiFi - rijdt zonder telemetrie");
}

// Herverbinden max 1x per 2s zodat rijlogica niet blokkeert
void mqttOnderhoud() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqttClient.connected()) { mqttClient.loop(); return; }
  static unsigned long lastTry = 0;
  if (millis() - lastTry < 2000) return;
  lastTry = millis();
  if (mqttClient.connect("ESP32-lijnauto", mqtt_user, mqtt_pass)) {
    mqttClient.subscribe(TOPIC_CMD_STOP);
    mqttClient.subscribe(TOPIC_CMD_START);
    mqttClient.subscribe(TOPIC_BROADCAST);
  }
}

void publishTelemetry() {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<256> doc;
  doc["status"] = statusTekst; doc["afstand"] = afstandCm;
  doc["ir_l"] = sL; doc["ir_m"] = sM; doc["ir_r"] = sR;
  doc["battery_v"] = batteryV; doc["battery_pct"] = batteryPct;
  char buf[256]; serializeJson(doc, buf);
  mqttClient.publish(TOPIC_TELEMETRY, buf);
  if (batteryPct <= 10) {
    StaticJsonDocument<64> alert;
    alert["type"] = "low_battery"; alert["battery"] = batteryPct;
    char ab[64]; serializeJson(alert, ab);
    mqttClient.publish(TOPIC_ALERT, ab);
  }
}

// Uitgebreide telemetrie specifiek voor de ontwijkfase
void publishOntwijkTelemetry(int stap, unsigned long duurMs, int kwijt = 0, const char* lijnSensor = "") {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<300> doc;
  doc["status"] = statusTekst; doc["afstand"] = afstandCm;
  doc["ir_l"] = sL; doc["ir_m"] = sM; doc["ir_r"] = sR;
  doc["stap"] = stap; doc["duur_ms"] = duurMs; doc["kwijt_teller"] = kwijt;
  if (strlen(lijnSensor) > 0) doc["lijn_sensor"] = lijnSensor;
  char buf[300]; serializeJson(doc, buf);
  mqttClient.publish(TOPIC_TELEMETRY, buf);
}


// =============================================================
// Obstakelontwijking
// stop 1s → 90°R → zijstap → 90°L → rijd langs → extra → 135°L → rijd tot lijn
// =============================================================
void ontwijkRechts() {
  unsigned long ontwijkStart = millis();

  statusTekst = "OBSTAKEL"; afstandCm = meetAfstand();
  publishTelemetry();
  Serial.printf("OBSTAKEL %d cm - stop 1s\n", afstandCm);
  stopMotors(); delay(1000);

  // Stap 1: zijwaarts naast obstakel plaatsen
  statusTekst = "DRAAI_RECHTS"; publishTelemetry(); draaiRechts(DRAAI_90_MS);
  statusTekst = "ZIJSTAP";      publishTelemetry(); vooruitTijd(SIDESTEP_MS);
  statusTekst = "DRAAI_LINKS";  publishTelemetry(); draaiLinks(DRAAI_90_MS);

  // Stap 2: rijd langs obstakel tot 3x buiten OBJECT_ZIJ_MAX of timeout
  statusTekst = "SCAN_OBJECT"; publishTelemetry();
  servoNaar(SERVO_OBJECT); delay(300);
  vooruit(RIJD_SNELHEID);
  unsigned long start = millis();
  int kwijt = 0; bool timeout = false;
  while (true) {
    int d = meetAfstand(); afstandCm = d;
    statusTekst = "NAAST_OBJECT"; mqttOnderhoud(); publishTelemetry();
    if (d == -1 || d > OBJECT_ZIJ_MAX) { if (++kwijt >= 3) break; }
    else kwijt = 0;
    if (millis() - start > PAS_TIMEOUT_MS) { timeout = true; break; }
    delay(60);
  }
  stopMotors();
  statusTekst = timeout ? "OBJECT_TIMEOUT" : "OBJECT_VOORBIJ"; publishTelemetry();

  // Stap 3: extra vooruit voor speling voor de terugdraai
  statusTekst = "EXTRA_VOORUIT"; publishTelemetry(); vooruitTijd(EXTRA_MS);

  // Stap 4a: servo terug, draai 135° links (schuin naar lijn)
  servoNaar(SERVO_VOORUIT);
  statusTekst = "TERUGDRAAIEN"; publishTelemetry(); draaiLinks(DRAAI_135_MS);

  // Stap 4b: rijd vooruit tot IR-sensor lijn detecteert (max LIJN_ZOEK_MS)
  statusTekst = "TERUG_VOORUIT"; publishTelemetry();
  vooruit(RIJD_SNELHEID);
  unsigned long zoekLijn = millis(); bool lijnGevonden = false;
  while (millis() - zoekLijn < LIJN_ZOEK_MS) {
    sL = digitalRead(IR_L); sM = digitalRead(IR_M); sR = digitalRead(IR_R);
    if (sL || sM || sR) { lijnGevonden = true; break; }
    delay(10);
  }
  stopMotors();

  // Stap 5: rapporteer welke sensor de lijn terugvond
  lastDir = -1;
  const char* lijnSensor = "geen";
  if (lijnGevonden) {
    if      (sL && sM && sR) lijnSensor = "L+M+R";
    else if (sL && sM)       lijnSensor = "L+M";
    else if (sM && sR)       lijnSensor = "M+R";
    else if (sL)             lijnSensor = "L";
    else if (sM)             lijnSensor = "M";
    else                     lijnSensor = "R";
  }
  statusTekst = lijnGevonden ? "LIJN_HERVAT" : "LIJN_NIET_GEVONDEN";
  publishTelemetry();
  Serial.printf("Ontwijking klaar: %s | sensor=[%s] | %lu ms\n",
                statusTekst.c_str(), lijnSensor, millis() - ontwijkStart);
}


// =============================================================
// Batterij
// =============================================================

// GPIO12 = ADC2: alleen bruikbaar vóór WiFi-start
float readBatteryVoltage() {
  const float divider = 3.34, adcRef = 3.3, adcMax = 4095.0;
  analogSetPinAttenuation(BAT_PIN, ADC_11db);
  long sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogRead(BAT_PIN); delay(2); }
  float v = ((sum / 16.0f) / adcMax) * adcRef * divider;
  Serial.printf("[BAT] raw=%ld → %.2f V\n", sum / 16, v);
  return v;
}

int voltageToPct(float v) {
  return constrain((int)((v - 6.4f) / (8.4f - 6.4f) * 100), 0, 100);
}

void updateBatteryLEDs(int pct) {
  digitalWrite(LED_GREEN,  pct > 30              ? HIGH : LOW);
  digitalWrite(LED_YELLOW, pct <= 30 && pct > 10 ? HIGH : LOW);
  digitalWrite(LED_RED,    pct <= 10             ? HIGH : LOW);
}


// =============================================================
// MQTT callback
// =============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  Serial.println("MQTT [" + t + "]");
  if (t == TOPIC_CMD_STOP || t == TOPIC_BROADCAST) {
    running = false; stopTimerActive = false;
    stopMotors(); statusTekst = "NOODSTOP";
    Serial.println("[NOODSTOP]");
  } else if (t == TOPIC_CMD_START) {
    running = true; stopTimerActive = false;
    statusTekst = "START"; Serial.println("[START]");
  }
}


// =============================================================
// Setup
// =============================================================
void setup() {
  Serial.begin(115200); delay(300);

  pinMode(IR_L, INPUT); pinMode(IR_M, INPUT); pinMode(IR_R, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(BTN_START, INPUT_PULLUP); pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(LED_GREEN, OUTPUT); pinMode(LED_YELLOW, OUTPUT); pinMode(LED_RED, OUTPUT);
  analogSetPinAttenuation(BAT_PIN, ADC_11db);

  // Servo eerst: allocateTimer(0) reserveert timer 0 zodat motoren kanalen 4+6 krijgen
  ESP32PWM::allocateTimer(0);
  scanServo.setPeriodHertz(50);
  scanServo.attach(SERVO_PIN, 500, 2400);
  scanServo.write(SERVO_VOORUIT); delay(500);

  // Motoren: 5 kHz PWM, 8-bit, vaste kanalen 4 en 6
  ledcAttachChannel(ENA, 5000, 8, ENA_KANAAL);
  ledcAttachChannel(ENB, 5000, 8, ENB_KANAAL);
  stopMotors();

  // Buffers vullen zodat majority-vote bij de eerste loop al correcte waarden heeft
  for (int i = 0; i < Q; i++) {
    bufL[i] = digitalRead(IR_L);
    bufM[i] = digitalRead(IR_M);
    bufR[i] = digitalRead(IR_R);
  }

  // Batterij meten VÓÓR WiFi (ADC2-conflict)
  batteryV   = readBatteryVoltage();
  batteryPct = voltageToPct(batteryV);
  updateBatteryLEDs(batteryPct);
  Serial.printf("[BAT init] %.2f V  %d%%\n", batteryV, batteryPct);

  connectWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  Serial.println("gereed");
}


// =============================================================
// Loop
// =============================================================
void loop() {
  mqttOnderhoud();
  if (millis() - lastMqtt > MQTT_INTERVAL) { lastMqtt = millis(); publishTelemetry(); }

  // Knoppen (edge-detect)
  static bool lastBtnStart = HIGH, lastBtnStop = HIGH;
  bool curStart = digitalRead(BTN_START), curStop = digitalRead(BTN_STOP);
  if (curStart == LOW && lastBtnStart == HIGH) {
    running = true; stopTimerActive = false; Serial.println("[KNOP] START");
  }
  if (curStop == LOW && lastBtnStop == HIGH) {
    running = false; stopTimerActive = false;
    stopMotors(); statusTekst = "NOODSTOP"; Serial.println("[KNOP] NOODSTOP");
  }
  lastBtnStart = curStart; lastBtnStop = curStop;

  if (stopTimerActive && millis() - stopTime >= STOP_DURATION) { running = true; stopTimerActive = false; }

  updateBatteryLEDs(batteryPct);

  // Noodstop bij kritiek laag batterijniveau
  if (batteryPct <= 10 && running) {
    running = false; stopTimerActive = false; stopMotors(); statusTekst = "ALARM_BATTERY";
  }

  if (!running) { stopMotors(); delay(10); return; }

  // Obstakeldetectie (elke ULTRA_INTERVAL ms)
  if (millis() - lastUltra > ULTRA_INTERVAL) {
    lastUltra = millis();
    int d = meetAfstand(); afstandCm = d;
    if (d != -1 && d < OBSTAKEL_AFSTAND && millis() > obstakelNegeerTot) {
      ontwijkRechts();
      obstakelNegeerTot = millis() + OBSTAKEL_COOLDOWN_MS;
      lastUltra = millis(); return;
    }
  }

  // IR-sensoren uitlezen + majority-vote
  bufL[bufIndex] = digitalRead(IR_L);
  bufM[bufIndex] = digitalRead(IR_M);
  bufR[bufIndex] = digitalRead(IR_R);
  bufIndex = (bufIndex + 1) % Q;
  bool L = majorityVote(bufL), M = majorityVote(bufM), R = majorityVote(bufR);
  sL = L; sM = M; sR = R;

  // Rijlogica op basis van sensorpatroon (L / M / R)
  if (!L && !M && !R) {
    // 000 – lijn kwijt: gefaseerd zoeken, daarna stop
    if (zoekStart == 0) zoekStart = millis();
    unsigned long zoekDuur = millis() - zoekStart;
    currentLeft = 0; currentRight = 0;
    if (zoekDuur < ZOEK_FASE1_MS) {
      (lastDir == -1) ? setMotors(pivotSpeed, -pivotSpeed) : setMotors(-pivotSpeed, pivotSpeed);
      statusTekst = "ZOEKEN_1";
    } else if (zoekDuur < ZOEK_FASE1_MS + ZOEK_FASE2_MS) {
      (lastDir == -1) ? setMotors(-pivotSpeed, pivotSpeed) : setMotors(pivotSpeed, -pivotSpeed);
      statusTekst = "ZOEKEN_2";
    } else {
      stopMotors(); statusTekst = "LIJN_KWIJT";
    }
    prevState = NONE;
  }
  else if (!L && M && !R) {
    // 010 – midden: rechtdoor met fade
    zoekStart = 0; dwarslijnStart = 0;
    stepMotorsFade(baseSpeed, baseSpeed);
    prevState = STRAIGHT; statusTekst = "RECHTDOOR";
  }
  else if (L && !M && !R) {
    // 100 – alleen links: harde pivot links
    zoekStart = 0; dwarslijnStart = 0; lastDir = -1;
    currentLeft = 0; currentRight = 0;
    setMotors(pivotSpeed, -pivotSpeed);
    prevState = PIVOT_L; statusTekst = "PIVOT_L";
  }
  else if (!L && !M && R) {
    // 001 – alleen rechts: harde pivot rechts
    zoekStart = 0; dwarslijnStart = 0; lastDir = 1;
    currentLeft = 0; currentRight = 0;
    setMotors(-pivotSpeed, pivotSpeed);
    prevState = PIVOT_R; statusTekst = "PIVOT_R";
  }
  else if (L && M && !R) {
    // 110 – links+midden: bocht links
    zoekStart = 0; dwarslijnStart = 0; lastDir = -1;
    currentLeft = 0; currentRight = 0;
    (prevState == STRAIGHT) ? setMotors(baseSpeed, -100) : setMotors(baseSpeed, -correction * 3 / 4);
    prevState = TURN_L; statusTekst = "BOCHT_L";
  }
  else if (!L && M && R) {
    // 011 – midden+rechts: bocht rechts
    zoekStart = 0; dwarslijnStart = 0; lastDir = 1;
    currentLeft = 0; currentRight = 0;
    (prevState == STRAIGHT) ? setMotors(-100, baseSpeed) : setMotors(-correction * 3 / 4, baseSpeed);
    prevState = TURN_R; statusTekst = "BOCHT_R";
  }
  else {
    // 111 – alles actief: bevestig kruispunt pas na DWARSLIJN_MS
    zoekStart = 0;
    if (dwarslijnStart == 0) dwarslijnStart = millis();
    if (millis() - dwarslijnStart >= DWARSLIJN_MS) {
      stopMotors(); running = false; prevState = NONE;
      statusTekst = "DWARSLIJN"; Serial.println("KRUISPUNT - gestopt");
    } else {
      // Nog te kort actief: waarschijnlijk brede bocht, rijd door
      (lastDir == -1) ? setMotors(baseSpeed, -100) : setMotors(-100, baseSpeed);
      prevState = NONE; statusTekst = "BOCHT_BREED";
    }
  }

  delay(10); // ~100 Hz lijnvolg-frequentie
}
