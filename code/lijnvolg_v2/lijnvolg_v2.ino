// =============================================================
// MUSEUM LIJNVOLGER V2
// ESP32 - 3 IR-sensoren, L298N motordriver, HC-SR04 ultrasoon,
// servo voor scanning, WiFi + MQTT telemetrie
//
// Werking: de robot volgt een zwarte lijn op een witte ondergrond.
// Ziet de ultrasone sensor een obstakel dichter dan OBSTAKEL_AFSTAND cm,
// dan wijkt de robot rechts uit, rijdt erlangs en zoekt daarna de lijn
// opnieuw op. Alle statusinfo wordt via MQTT verstuurd.
// =============================================================

#include <ESP32Servo.h>   // servo-aansturing zonder timer-conflict
#include <WiFi.h>
#include <PubSubClient.h> // MQTT client
#include <ArduinoJson.h>  // JSON opbouw voor telemetrie
// GPIO 12 = ADC2 → werkt NIET met analogRead() als WiFi actief is.
// Meting gebeurt daarom éénmalig vóór WiFi-start in setup().

// =============================================================
// WIFI / MQTT INSTELLINGEN
// Pas ssid, password en mqtt_server aan als het netwerk wijzigt.
// mqtt_port is standaard 1883 (onversleuteld MQTT).
// =============================================================
const char* ssid            = "brechtiot";
const char* password        = "brechtiot";
const char* mqtt_server     = "192.168.1.154";   // IP van de MQTT broker (Raspberry Pi)
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

unsigned long lastMqtt    = 0;
const long    MQTT_INTERVAL = 300; // telemetrie versturen elke 300 ms

// Live waarden die in elk telemetriebericht zitten
String statusTekst = "START";
int    afstandCm   = -1;   // -1 = geen geldig meetresultaat
bool   sL = false, sM = false, sR = false; // gefilterde IR-sensorwaarden

// =============================================================
// PINNEN
// Aanpassen als de bedrading op de ESP32 anders is.
// =============================================================
#define IR_L 34   // linker IR-sensor (digitaal: HIGH = lijn gezien)
#define IR_M 35   // midden IR-sensor
#define IR_R 32   // rechter IR-sensor

// L298N motordriver - linkermotor via ENA/IN1/IN2, rechtermotor via ENB/IN3/IN4
#define ENA 19    // PWM snelheidsregeling linkermotor (enable A)
#define IN1 18    // rijrichting linkermotor - bit 1
#define IN2  5    // rijrichting linkermotor - bit 2
#define ENB 16    // PWM snelheidsregeling rechtermotor (enable B)
#define IN3  4    // rijrichting rechtermotor - bit 1
#define IN4  2    // rijrichting rechtermotor - bit 2

#define TRIG_PIN  25  // ultrasone sensor: trigger (output)
#define ECHO_PIN  27  // ultrasone sensor: echo (input)
#define SERVO_PIN 33  // servo voor scan-richting

#define BTN_START  15  // startknop (INPUT_PULLUP → LOW = ingedrukt)
#define BTN_STOP   26  // stopknop  (INPUT_PULLUP → LOW = ingedrukt)
#define BAT_PIN    12  // batterijspanning via spanningsdeler (ADC1)
#define LED_GREEN  21  // groene LED  > 30 %
#define LED_YELLOW 22  // gele LED   10–30 %
#define LED_RED    23  // rode LED    ≤ 10 %

// PWM-kanalen voor de motoren (servo bezet timer 0 via ESP32Servo;
// kanaal 4 en 6 vermijden conflict met servo-timers 0-3)
#define ENA_KANAAL 4
#define ENB_KANAAL 6

// =============================================================
// LIJNVOLG-PARAMETERS
// Q = aantal samples in de majority-vote buffer (ruisfilter).
//   Hogere waarde = trager maar stabieler. Aanbevolen: 2-4.
// baseSpeed = basissnelheid rechtdoor (0-255).
//   Verhogen = sneller rijden, maar minder correctievermogen.
// correction = tegenwiel-rem bij een bocht (0-255).
//   Verhogen = scherpere bochten.
// pivotSpeed = draaisnelheid bij een pivot-turn (0-255).
//   Verlagen als de robot de lijn overshooit bij zoeken.
// =============================================================
#define Q 4
int baseSpeed  = 190;
int correction = 240;  // verhoogd: scherpere correctie in bochten
int pivotSpeed = 210;  // max vermogen voor pivot-bochten

// =============================================================
// OBSTAKEL-PARAMETERS
// Alle tijden zijn in milliseconden, afstanden in centimeter.
//
// OBSTAKEL_AFSTAND : obstakel wordt gemeld als afstand < deze waarde
//   Verhogen = eerder reageren (meer veiligheidsmarge)
// OBJECT_ZIJ_MAX   : max zijdelingse afstand waarbij object nog "zichtbaar" is
//   Verhogen = langer doorrijden langs het object
// RIJD_SNELHEID    : rijsnelheid tijdens ontwijkmanoeuvres
// DRAAI_SNELHEID   : motorsnelheid tijdens een draai-instructie
// DRAAI_90_MS      : ms nodig voor ca. 90° draaien
//   Aanpassen als de robot te ver of te weinig draait (batterijspanning beïnvloedt dit)
// SIDESTEP_MS      : ms rechtdoor rijden na de eerste 90°-draai (breedte van het object)
//   Verhogen als het object breder is
// EXTRA_MS         : extra vooruitrit nadat het object uit het zicht is
//   Verhogen om meer speling te geven voor de terugbocht
// PAS_TIMEOUT_MS   : maximale tijd voor het voorbijrijden (veiligheidsgrens)
// ULTRA_INTERVAL   : hoe vaak de ultrasone sensor gemeten wordt (ms)
// BOCHT_BINNEN     : snelheid binnenwiel bij de zachte terugbocht naar de lijn
// BOCHT_BUITEN     : snelheid buitenwiel bij de zachte terugbocht naar de lijn
//   Verschil BOCHT_BUITEN - BOCHT_BINNEN bepaalt hoe scherp de bocht is
// LIJN_ZOEK_MS     : maximale zoektijd om de lijn terug te vinden na ontwijking
// OBSTAKEL_COOLDOWN_MS : tijd na een ontwijking dat nieuwe obstakels genegeerd worden
// =============================================================
#define SERVO_VOORUIT        90   // servo-hoek voor vooruit kijken
#define SERVO_OBJECT        170   // servo-hoek om naar het obstakel te kijken (zijwaarts)
#define OBSTAKEL_AFSTAND     25   // cm - obstakeldetectie drempel
#define OBJECT_ZIJ_MAX       25   // cm - max afstand om object nog als "aanwezig" te beschouwen
#define MAX_AFSTAND         100   // cm - meetwaarden boven dit worden genegeerd (-1)
#define RIJD_SNELHEID       195   // PWM rijsnelheid tijdens ontwijking
#define DRAAI_SNELHEID      200   // PWM draaisnelheid
#define DRAAI_90_MS         650   // ms voor ~90° draai
#define DRAAI_135_MS        200   // ms voor ~135° draai (= DRAAI_90_MS × 1.5)
#define SIDESTEP_MS         1000   // ms rechtdoor na eerste draai
#define EXTRA_MS            500   // ms extra vooruit na voorbijrijden
#define PAS_TIMEOUT_MS     1000   // ms maximale voorbijrij-tijd
#define ULTRA_INTERVAL      100   // ms tussen ultrasone metingen
#define BOCHT_BINNEN        100   // PWM binnenwiel terugbocht
#define BOCHT_BUITEN        180   // PWM buitenwiel terugbocht
#define LIJN_ZOEK_MS       2500   // ms maximale zoektijd voor de lijn
#define OBSTAKEL_COOLDOWN_MS 10000 // ms geen nieuwe obstakeldetectie na ontwijking

Servo scanServo;

// Ringbuffers voor majority-vote ruisfiltering per sensor
int bufL[Q] = {}, bufM[Q] = {}, bufR[Q] = {};
int bufIndex = 0; // schrijfpositie in de ringbuffer

// Huidige en vorige motorsnelheden (gebruikt door stepMotorsFade)
int currentLeft  = 0, currentRight  = 0;
int lastLeft     = 0, lastRight     = 0;

// Laatste rijrichting: 1 = rechts, -1 = links
// Wordt gebruikt om bij lijn-kwijt de juiste zoekrichting te kiezen
int lastDir = 1;

unsigned long lastUltra        = 0; // tijdstip laatste ultrasone meting
unsigned long obstakelNegeerTot = 0; // tot wanneer obstakels genegeerd worden na ontwijking

// Rijstatus via knoppen / MQTT
bool running          = false;
unsigned long stopTime            = 0;
bool stopTimerActive              = false;
const unsigned long STOP_DURATION = 30000; // ms automatisch herstart na stop-knop

// Batterijwaarden voor telemetrie
float batteryV   = 8.4;
int   batteryPct = 100;

// Zoektimer: wordt gestart zodra alle IR-sensoren leeg zijn.
// Wordt gereset naar 0 zodra de lijn opnieuw gevonden wordt.
// Gebruik: gefaseerd zoeken met tijdslimiet zodat de robot niet eindeloos draait.
// ZOEK_FASE1_MS : tijd (ms) dat in de laatste bekende richting gezocht wordt
// ZOEK_FASE2_MS : extra tijd (ms) dat in de omgekeerde richting gezocht wordt
// Na ZOEK_FASE1_MS + ZOEK_FASE2_MS stopt de robot volledig (lijn niet gevonden).
#define ZOEK_FASE1_MS 1500  // ms draaien in lastDir    → verhogen voor meer geduld
#define ZOEK_FASE2_MS 1500  // ms draaien in -lastDir   → verhogen voor meer geduld
unsigned long zoekStart     = 0;
unsigned long dwarslijnStart = 0; // tijdstip waarop alle 3 sensoren voor het eerst actief werden
#define DWARSLIJN_MS 200          // ms dat alle sensoren actief moeten zijn voor een echt kruispunt

// Staten om bij te houden wat de robot de vorige loop-iteratie deed
// Wordt gebruikt om vloeiend van rechtdoor naar een bocht te schakelen
enum State { STRAIGHT, TURN_L, TURN_R, PIVOT_L, PIVOT_R, NONE };
State prevState = NONE;


// =============================================================
// majorityVote
// Geeft true als meer dan de helft van de buffer HIGH is.
// Filtert kortstondige vals-positieve of vals-negatieve IR-metingen.
// buf : pointer naar een int-array van lengte Q
// =============================================================
bool majorityVote(int* buf) {
  int sum = 0;
  for (int i = 0; i < Q; i++) sum += buf[i];
  return sum > (Q / 2);
}


// =============================================================
// stopMotors
// Zet beide motoren onmiddellijk stil door alle richting- en
// PWM-uitgangen laag te zetten. Reset ook currentLeft/Right naar 0
// zodat stepMotorsFade daarna correct kan optrekken.
// =============================================================
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
  currentLeft  = 0;
  currentRight = 0;
}


// =============================================================
// setMotors
// Stuurt de motoren direct aan op de opgegeven snelheid.
// Positieve waarde = vooruit, negatieve waarde = achteruit.
// Bereik: -255 tot 255. Wordt intern begrensd met constrain().
// Gebruik dit voor draaien en correcties (geen fade nodig).
// =============================================================
void setMotors(int leftSpeed, int rightSpeed) {
  // Richting linkermotor
  if (leftSpeed >= 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); leftSpeed = -leftSpeed; }

  // Richting rechtermotor
  if (rightSpeed >= 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  else { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); rightSpeed = -rightSpeed; }

  ledcWrite(ENA, constrain(leftSpeed,  0, 255));
  ledcWrite(ENB, constrain(rightSpeed, 0, 255));
  lastLeft  = leftSpeed;
  lastRight = rightSpeed;
}


// =============================================================
// stepMotorsFade
// Verhoogt of verlaagt de motorsnelheid geleidelijk naar de
// doelsnelheid met stappen van 'step' per aanroep.
// Gebruik dit voor vloeiend optrekken rechtdoor (minder slippen).
// targetLeft/Right : gewenste eindsnelheid (0-255)
// step             : grootte van elke versnellingsstap (standaard 8)
//   Kleiner = zachter optrekken maar trager bereiken van topsnelheid
// =============================================================
void stepMotorsFade(int targetLeft, int targetRight, int step = 8) {
  // Altijd vooruit bij fade (wordt alleen voor rechtdoor gebruikt)
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);

  // Stap richting doel
  if (currentLeft  < targetLeft)  currentLeft  = min(currentLeft  + step, targetLeft);
  if (currentLeft  > targetLeft)  currentLeft  = max(currentLeft  - step, targetLeft);
  if (currentRight < targetRight) currentRight = min(currentRight + step, targetRight);
  if (currentRight > targetRight) currentRight = max(currentRight - step, targetRight);

  ledcWrite(ENA, constrain(currentLeft,  0, 255));
  ledcWrite(ENB, constrain(currentRight, 0, 255));
  lastLeft  = currentLeft;
  lastRight = currentRight;
}


// =============================================================
// connectWiFi
// Probeert maximaal 10 seconden verbinding te maken met het
// opgegeven WiFi-netwerk. Als het mislukt, rijdt de robot verder
// zonder telemetrie (geen crash of blokkering).
// =============================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("WiFi verbinden");
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
    delay(300); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(false); // slaapstand uit voor stabielere verbinding
    Serial.println(" OK: " + WiFi.localIP().toString());
  } else {
    Serial.println(" geen WiFi - rijdt zonder telemetrie");
  }
}


// =============================================================
// mqttOnderhoud
// Houdt de MQTT-verbinding in stand en verwerkt inkomende berichten.
// Wordt elke loop-iteratie aangeroepen. Als de verbinding wegvalt,
// probeert het opnieuw te verbinden maar max 1 keer per 2 seconden
// (om de rijlogica niet te blokkeren).
// =============================================================
void mqttOnderhoud() {
  if (WiFi.status() != WL_CONNECTED) return; // geen WiFi = niets doen
  if (mqttClient.connected()) { mqttClient.loop(); return; }

  static unsigned long lastTry = 0;
  if (millis() - lastTry < 2000) return; // wacht 2 s voor herverbinding
  lastTry = millis();
  if (mqttClient.connect("ESP32-lijnauto", mqtt_user, mqtt_pass)) {
    mqttClient.subscribe(TOPIC_CMD_STOP);
    mqttClient.subscribe(TOPIC_CMD_START);
    mqttClient.subscribe(TOPIC_BROADCAST);
  }
}


// =============================================================
// publishTelemetry
// Verstuurt een JSON-bericht met de huidige status naar de broker.
// Formaat: {"status":"...","afstand":...,"ir_l":...,"ir_m":...,"ir_r":...}
// Wordt alleen verstuurd als er een actieve MQTT-verbinding is.
// =============================================================
void publishTelemetry() {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<256> doc;
  doc["status"]      = statusTekst;
  doc["afstand"]     = afstandCm;
  doc["ir_l"]        = sL; doc["ir_m"] = sM; doc["ir_r"] = sR;
  doc["battery_v"]   = batteryV;
  doc["battery_pct"] = batteryPct;
  char buf[256];
  serializeJson(doc, buf);
  mqttClient.publish(TOPIC_TELEMETRY, buf);

  if (batteryPct <= 10) {
    StaticJsonDocument<64> alert;
    alert["type"]    = "low_battery";
    alert["battery"] = batteryPct;
    char alertBuf[64];
    serializeJson(alert, alertBuf);
    mqttClient.publish(TOPIC_ALERT, alertBuf);
  }
}


// =============================================================
// publishOntwijkTelemetry
// Uitgebreide telemetrie specifiek voor de obstakelontwijking.
// Wordt enkel binnen ontwijkRechts() aangeroepen.
//
// Parameters:
//   stap       : huidige stap in de ontwijkreeks (1-5)
//   duurMs     : verstreken tijd (ms) sinds start van ontwijkRechts()
//   kwijt      : aantal opeenvolgende metingen buiten OBJECT_ZIJ_MAX
//                (alleen relevant tijdens stap 3 - naast het object rijden)
//   lijnSensor : welke sensor de lijn terugvond ("L","M","R","geen")
//                (alleen relevant tijdens stap 5 - lijn zoeken)
//
// Extra JSON-velden t.o.v. publishTelemetry():
//   stap, duur_ms, kwijt_teller, lijn_sensor
// =============================================================
void publishOntwijkTelemetry(int stap, unsigned long duurMs,
                              int kwijt = 0, const char* lijnSensor = "") {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<300> doc;
  doc["status"]      = statusTekst;
  doc["afstand"]     = afstandCm;
  doc["ir_l"]        = sL; doc["ir_m"] = sM; doc["ir_r"] = sR;
  doc["stap"]        = stap;        // deelstap binnen de ontwijking (1 t/m 5)
  doc["duur_ms"]     = duurMs;      // totale duur ontwijking tot nu toe (ms)
  doc["kwijt_teller"]= kwijt;       // hoe vaak object achter elkaar buiten bereik
  if (strlen(lijnSensor) > 0)
    doc["lijn_sensor"] = lijnSensor; // welke IR-sensor de lijn detecteerde
  char buf[300];
  serializeJson(doc, buf);
  mqttClient.publish(TOPIC_TELEMETRY, buf);
}


// =============================================================
// vooruit
// Zet beide motoren vooruit op snelheid s (0-255).
// Geen stopMotors() achteraf - gebruik vooruitTijd() voor een
// tijdgebonden vooruitrit met automatische stop.
// =============================================================
void vooruit(int s) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  ledcWrite(ENA, s);
  ledcWrite(ENB, s);
}


// =============================================================
// draaiRechts
// Draait de robot rechtsom ter plaatse gedurende t milliseconden.
// Linkermotor achteruit, rechtermotor vooruit = rechtsom draaien.
// t aanpassen als de robot meer of minder dan 90° draait:
//   te ver = t verlagen, te weinig = t verhogen.
// Na de draai: motors stop + 200 ms pauze om trilling te laten dalen.
// =============================================================
void draaiRechts(int t) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // linker achteruit
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); // rechter vooruit
  ledcWrite(ENA, DRAAI_SNELHEID);
  ledcWrite(ENB, DRAAI_SNELHEID);
  delay(t);
  stopMotors(); delay(200);
}


// =============================================================
// draaiLinks
// Draait de robot linksom ter plaatse gedurende t milliseconden.
// Linkermotor vooruit, rechtermotor achteruit = linksom draaien.
// Zie draaiRechts() voor aanpassingstips.
// =============================================================
void draaiLinks(int t) {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); // linker vooruit
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // rechter achteruit
  ledcWrite(ENA, DRAAI_SNELHEID);
  ledcWrite(ENB, DRAAI_SNELHEID);
  delay(t);
  stopMotors(); delay(200);
}


// =============================================================
// vooruitTijd
// Rijdt vooruit op RIJD_SNELHEID gedurende t milliseconden,
// daarna stop + 200 ms pauze.
// =============================================================
void vooruitTijd(int t) {
  vooruit(RIJD_SNELHEID);
  delay(t);
  stopMotors(); delay(200);
}


// =============================================================
// meetAfstand
// Meet de afstand met de HC-SR04 ultrasone sensor.
// Geeft de afstand in cm terug, of -1 bij geen echo of te ver.
// Timeout van 30 000 µs = max ~5 m meetbereik (voldoende voor indoor).
// =============================================================
int meetAfstand() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10); // 10 µs puls = standaard HC-SR04
  digitalWrite(TRIG_PIN, LOW);
  long duur = pulseIn(ECHO_PIN, HIGH, 30000); // wacht op echo, timeout 30 ms
  if (duur == 0) return -1;                   // geen echo ontvangen
  int cm = duur / 58;                         // omrekening: 58 µs per cm (heen en terug)
  return (cm > MAX_AFSTAND) ? -1 : cm;        // te ver = ongeldig
}


// =============================================================
// servoNaar
// Beweegt de servo geleidelijk naar hoek 'doel' (0-180°).
// Stap voor stap met 15 ms vertraging = ca. 1°/stap voor soepele beweging.
// Sneller: delay verlagen (maar dan meer trilling).
// SERVO_VOORUIT (90°) = recht vooruit
// SERVO_OBJECT (170°) = zijwaarts naar het obstakel kijken
// =============================================================
void servoNaar(int doel) {
  int h = scanServo.read(); // huidige positie uitlezen
  if (h < doel) for (; h <= doel; h++) { scanServo.write(h); delay(15); }
  else          for (; h >= doel; h--) { scanServo.write(h); delay(15); }
}


// =============================================================
// ontwijkRechts
// Volledige ontwijkingsroutine wanneer een obstakel gedetecteerd wordt.
//
// Stappen:
//   1. Stop + 1 s wachten (veiligheidsrust)
//   2. Rechts draaien (DRAAI_90_MS) → SIDESTEP_MS vooruit → links draaien (DRAAI_90_MS)
//      → robot staat naast het obstakel, parallel aan de lijn
//   3. Servo naar het obstakel, vooruitrijden tot obstakel 3x buiten OBJECT_ZIJ_MAX cm valt
//      → robot is voorbij het obstakel
//   4. Servo terug vooruit → 90° links draaien (DRAAI_90_MS) → SIDESTEP_MS vooruit
//      → robot rijdt exact dezelfde zijstap terug, richting originele lijnpositie
//   5. Sensorcheck: rapporteer welke IR-sensor de lijn ziet en hervat het lijnvolgen
//
// AANPASSEN ALS:
//   - robot draait te ver/weinig bij terugkeer: pas DRAAI_90_MS aan
//   - robot komt te ver/dichtbij de lijn uit: pas SIDESTEP_MS aan
//   - robot staat scheef na de ontwijking: batterijspanning beïnvloedt draaitijd
// =============================================================
void ontwijkRechts() {
  unsigned long ontwijkStart = millis();

  // --- Obstakel gedetecteerd ---
  statusTekst = "OBSTAKEL";
  afstandCm = meetAfstand();
  publishTelemetry();
  Serial.printf("OBSTAKEL gezien op %d cm - stop 1 seconde\n", afstandCm);
  stopMotors(); delay(1000);

  // -------------------------------------------------------
  // STAP 1: zijwaarts rechts naast het obstakel plaatsen
  //   draai 90° rechts → rijdt SIDESTEP_MS zijwaarts → draai 90° links terug
  //   Robot staat nu parallel aan de lijn, naast het obstakel
  // -------------------------------------------------------
  statusTekst = "DRAAI_RECHTS"; publishTelemetry();
  Serial.println("Stap 1a: draai 90° rechts");
  draaiRechts(DRAAI_90_MS);

  statusTekst = "ZIJSTAP"; publishTelemetry();
  Serial.printf("Stap 1b: zijstap vooruit (%d ms)\n", SIDESTEP_MS);
  vooruitTijd(SIDESTEP_MS);

  statusTekst = "DRAAI_LINKS"; publishTelemetry();
  Serial.println("Stap 1c: draai 90° links - staat naast het obstakel");
  draaiLinks(DRAAI_90_MS);

  // -------------------------------------------------------
  // STAP 2: servo richting obstakel, rijden tot obstakel voorbij is
  //   Stopt zodra het obstakel 3x achter elkaar buiten OBJECT_ZIJ_MAX cm valt,
  //   of als PAS_TIMEOUT_MS bereikt wordt (veiligheidsgrens)
  // -------------------------------------------------------
  statusTekst = "SCAN_OBJECT"; publishTelemetry();
  Serial.println("Stap 2: servo naar obstakel, vooruitrijden");
  servoNaar(SERVO_OBJECT);
  delay(300); // wacht tot servo stil staat voor eerste meting

  vooruit(RIJD_SNELHEID);
  unsigned long start = millis();
  int kwijt = 0;
  bool timeout = false;

  while (true) {
    int d = meetAfstand();
    afstandCm = d;
    statusTekst = "NAAST_OBJECT";
    mqttOnderhoud(); publishTelemetry();
    Serial.printf("  naast obstakel: %d cm | kwijt=%d\n", d, kwijt);

    if (d == -1 || d > OBJECT_ZIJ_MAX) {
      if (++kwijt >= 3) break; // 3x buiten bereik = obstakel voorbij
    } else {
      kwijt = 0;               // obstakel terug in beeld, teller resetten
    }
    if (millis() - start > PAS_TIMEOUT_MS) { timeout = true; break; }
    delay(60);
  }
  stopMotors();

  statusTekst = timeout ? "OBJECT_TIMEOUT" : "OBJECT_VOORBIJ";
  publishTelemetry();
  Serial.printf("Stap 2 klaar: %s na %lu ms\n", statusTekst.c_str(), millis() - start);

  // -------------------------------------------------------
  // STAP 3: extra stukje vooruit voor voldoende speling
  //   Zorgt dat de robot volledig naast het obstakel staat
  //   voor de terugdraai in stap 4 begint
  // -------------------------------------------------------
  statusTekst = "EXTRA_VOORUIT"; publishTelemetry();
  Serial.printf("Stap 3: extra vooruit (%d ms)\n", EXTRA_MS);
  vooruitTijd(EXTRA_MS);

  // -------------------------------------------------------
  // STAP 4: deterministische terugkeer naar de lijn
  //   Servo terug vooruit → 90° links draaien → SIDESTEP_MS vooruit
  //   Zelfde afstand als de zijstap in stap 1 zodat de robot
  //   terug op de originele lijnpositie terechtkomt
  // -------------------------------------------------------
  servoNaar(SERVO_VOORUIT);

  // -------------------------------------------------------
  // STAP 4a: draai 135° links → wagon benadert de lijn met een hoek
  //   135° i.p.v. 90° zodat de wagon schuin naar de lijn rijdt
  //   en de lijn geleidelijk raakt i.p.v. er recht op te botsen
  // -------------------------------------------------------
  statusTekst = "TERUGDRAAIEN"; publishTelemetry();
  Serial.println("Stap 4a: draai 135° links richting lijn");
  draaiLinks(DRAAI_135_MS);

  // -------------------------------------------------------
  // STAP 4b: rijd vooruit tot een IR-sensor de lijn detecteert
  //   Rijdt niet een vaste tijd maar stopt zodra de lijn gevonden is.
  //   Timeout van LIJN_ZOEK_MS als veiligheidsgrens.
  // -------------------------------------------------------
  statusTekst = "TERUG_VOORUIT"; publishTelemetry();
  Serial.println("Stap 4b: rijd naar lijn tot sensor detecteert");
  vooruit(RIJD_SNELHEID);
  unsigned long zoekLijn = millis();
  bool lijnGevonden = false;
  while (millis() - zoekLijn < LIJN_ZOEK_MS) {
    sL = digitalRead(IR_L);
    sM = digitalRead(IR_M);
    sR = digitalRead(IR_R);
    if (sL || sM || sR) { lijnGevonden = true; break; }
    delay(10);
  }
  stopMotors();

  // -------------------------------------------------------
  // STAP 5: sensorcheck en telemetrie
  // -------------------------------------------------------
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
  Serial.printf("Stap 5: %s | sensor=[%s] | totale duur: %lu ms\n",
                statusTekst.c_str(), lijnSensor, millis() - ontwijkStart);
}


// =============================================================
// readBatteryVoltage / voltageToPct / updateBatteryLEDs
// Leest de spanning via een externe spanningsdeler (factor 4.87),
// berekent het percentage en stuurt de drie status-LEDs aan.
// =============================================================
// Batterij meten — werkt ALLEEN vóór WiFi start (GPIO 12 = ADC2, conflicteert met WiFi)
// Wordt éénmalig aangeroepen in setup() vóór connectWiFi().
// Daarna blijft batteryV/batteryPct op de gemeten waarde staan.
float readBatteryVoltage() {
  const float divider = 3.34;  // gecalibreerd op gemeten vs. werkelijke spanning (7.3V / 6.7V)
  const float adcRef  = 3.3;
  const float adcMax  = 4095.0;
  analogSetPinAttenuation(BAT_PIN, ADC_11db);
  long sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogRead(BAT_PIN);
    delay(2);
  }
  float v = ((sum / 16.0f) / adcMax) * adcRef * divider;
  Serial.printf("[BAT] raw gem=%ld  →  %.2f V\n", sum / 16, v);
  return v;
}

int voltageToPct(float v) {
  const float full  = 8.4;
  const float empty = 6.4;
  int pct = (int)((v - empty) / (full - empty) * 100);
  return constrain(pct, 0, 100);
}

void updateBatteryLEDs(int pct) {
  digitalWrite(LED_GREEN,  pct > 30              ? HIGH : LOW);
  digitalWrite(LED_YELLOW, pct <= 30 && pct > 10 ? HIGH : LOW);
  digitalWrite(LED_RED,    pct <= 10             ? HIGH : LOW);
}


// =============================================================
// mqttCallback
// Verwerkt inkomende MQTT-commando's voor start en stop.
// =============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  Serial.println("MQTT ontvangen [" + t + "]");
  if (t == TOPIC_CMD_STOP || t == TOPIC_BROADCAST) {
    // Noodstop: geen auto-herstart, wacht op expliciet START-commando
    running         = false;
    stopTimerActive = false;
    stopMotors();
    statusTekst = "NOODSTOP";
    Serial.println("[NOODSTOP] Wagon gestopt via MQTT — wacht op START");
  } else if (t == TOPIC_CMD_START) {
    running         = true;
    stopTimerActive = false;
    statusTekst     = "START";
    Serial.println("[START] Wagon gestart via MQTT");
  }
}


// =============================================================
// setup
// Eenmalige initialisatie bij opstarten:
//   - pinmodi instellen
//   - servo initialiseren (timer 0 via ESP32Servo, vóór de motoren)
//   - motoren initialiseren op aparte PWM-kanalen (4 en 6)
//   - IR-buffers vullen met de actuele sensorwaarden
//   - WiFi en MQTT-server instellen
// =============================================================
void setup() {
  Serial.begin(115200); delay(300);

  pinMode(IR_L, INPUT); pinMode(IR_M, INPUT); pinMode(IR_R, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP,  INPUT_PULLUP);
  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED,    OUTPUT);
  analogSetPinAttenuation(BAT_PIN, ADC_11db);

  // Servo EERST initialiseren: allocateTimer(0) reserveert timer 0 voor de servo
  // zodat ledcAttachChannel voor de motoren andere timers/kanalen krijgt
  ESP32PWM::allocateTimer(0);
  scanServo.setPeriodHertz(50);           // 50 Hz = standaard RC-servo
  scanServo.attach(SERVO_PIN, 500, 2400); // pulsbreedtes in µs (500=0°, 2400=180°)
  scanServo.write(SERVO_VOORUIT); delay(500); // servo naar startpositie

  // Motoren op vaste kanalen 4 en 6 (buiten servo-timerbereik 0-3)
  // Let op: ledcWrite() gebruikt pinnummers (ENA/ENB), niet kanaalnummers
  ledcAttachChannel(ENA, 5000, 8, ENA_KANAAL); // 5000 Hz PWM, 8-bit (0-255)
  ledcAttachChannel(ENB, 5000, 8, ENB_KANAAL);
  stopMotors();

  // Buffers vullen zodat majority-vote bij de eerste loop al correcte waarden heeft
  for (int i = 0; i < Q; i++) {
    bufL[i] = digitalRead(IR_L);
    bufM[i] = digitalRead(IR_M);
    bufR[i] = digitalRead(IR_R);
  }

  // Batterij meten VÓÓR WiFi — GPIO 12 (ADC2) werkt enkel zonder WiFi via analogRead
  batteryV   = readBatteryVoltage();
  batteryPct = voltageToPct(batteryV);
  updateBatteryLEDs(batteryPct);
  Serial.printf("[BAT init] %.2f V  %d %%\n", batteryV, batteryPct);

  connectWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);

  Serial.println("gereed - lijnvolgen + obstakelontwijking + MQTT + knoppen + batterij");
}


// =============================================================
// loop
// Hoofdlus - wordt continu herhaald. Volgorde per iteratie:
//   1. MQTT verbinding onderhouden + periodiek telemetrie sturen
//   2. Obstakelcheck (elke ULTRA_INTERVAL ms)
//      → obstakel gevonden: ontwijkRechts() aanroepen en terugkeren
//   3. IR-sensoren uitlezen en majority-vote toepassen
//   4. Motoraansturing op basis van sensorpatroon:
//      000 = geen lijn → zoeken in laatste bekende richting
//      010 = midden    → rechtdoor (met fade)
//      100 = links     → pivot links
//      001 = rechts    → pivot rechts
//      110 = links+mid → bocht links
//      011 = mid+rechts→ bocht rechts
//      111 = alles     → dwarslijn, stop
// =============================================================
void loop() {
  // MQTT verbinding onderhouden en periodiek telemetrie versturen
  mqttOnderhoud();
  if (millis() - lastMqtt > MQTT_INTERVAL) {
    lastMqtt = millis();
    publishTelemetry();
  }

  // --- Fysieke knoppen (edge-detect via static) ---
  static bool lastBtnStart = HIGH;
  static bool lastBtnStop  = HIGH;
  bool curStart = digitalRead(BTN_START);
  bool curStop  = digitalRead(BTN_STOP);

  if (curStart == LOW && lastBtnStart == HIGH) {
    Serial.println("[KNOP] START → wagon rijdt");
    running         = true;
    stopTimerActive = false;
  }
  if (curStop == LOW && lastBtnStop == HIGH) {
    Serial.println("[KNOP] NOODSTOP → wacht op START-knop");
    running         = false;
    stopTimerActive = false;  // geen auto-herstart
    stopMotors();
    statusTekst = "NOODSTOP";
  }
  lastBtnStart = curStart;
  lastBtnStop  = curStop;

  // Automatisch herstart na STOP_DURATION ms
  if (stopTimerActive && (millis() - stopTime >= STOP_DURATION)) {
    running         = true;
    stopTimerActive = false;
  }

  // --- Batterij ---
  // GPIO 12 = ADC2: kan niet worden uitgelezen terwijl WiFi actief is.
  // Waarde is éénmalig gemeten in setup() vóór WiFi-start en blijft ongewijzigd.
  // LEDs bijwerken op basis van de opgestarte waarde (elke loop, kost niets).
  updateBatteryLEDs(batteryPct);

  // Noodstop bij kritiek laag batterijniveau (op basis van opstartmeting)
  if (batteryPct <= 10 && running) {
    running         = false;
    stopTimerActive = false;
    stopMotors();
    statusTekst = "ALARM_BATTERY";
  }

  // Als de robot gestopt is, niets rijden
  if (!running) {
    stopMotors();
    delay(10);
    return;
  }

  // --- Obstakeldetectie ---
  // Elke ULTRA_INTERVAL ms meten. Als er een obstakel dichterbij is dan
  // OBSTAKEL_AFSTAND en de cooldown-periode voorbij is, ontwijken.
  if (millis() - lastUltra > ULTRA_INTERVAL) {
    lastUltra = millis();
    int d = meetAfstand();
    afstandCm = d;
    if (d != -1 && d < OBSTAKEL_AFSTAND && millis() > obstakelNegeerTot) {
      ontwijkRechts();
      obstakelNegeerTot = millis() + OBSTAKEL_COOLDOWN_MS; // nieuwe ontwijking pas na cooldown
      lastUltra = millis();
      return; // rest van de loop overslaan, meteen opnieuw beginnen
    }
  }

  // --- IR-sensoren uitlezen (ringbuffer + majority vote) ---
  bufL[bufIndex] = digitalRead(IR_L);
  bufM[bufIndex] = digitalRead(IR_M);
  bufR[bufIndex] = digitalRead(IR_R);
  bufIndex = (bufIndex + 1) % Q; // volgende schrijfpositie (rondlopend)

  bool L = majorityVote(bufL); // true = linker sensor ziet de lijn
  bool M = majorityVote(bufM);
  bool R = majorityVote(bufR);
  sL = L; sM = M; sR = R; // opslaan voor telemetrie

  // --- Rijlogica op basis van sensorpatroon ---

  if (!L && !M && !R) {
    // Geen sensor ziet de lijn: robot is de lijn kwijt.
    // Gefaseerd zoeken met tijdslimiet om eindeloos ronddraaien te voorkomen:
    //   Fase 1 (0 – ZOEK_FASE1_MS)       : pivot in lastDir (meest waarschijnlijke kant)
    //   Fase 2 (ZOEK_FASE1_MS – FASE1+2) : pivot in omgekeerde richting
    //   Fase 3 (> FASE1+FASE2)           : volledig stoppen — robot geeft het op
    if (zoekStart == 0) zoekStart = millis(); // start de zoektimer bij eerste kwijt-iteratie
    unsigned long zoekDuur = millis() - zoekStart;

    currentLeft = 0; currentRight = 0;

    if (zoekDuur < ZOEK_FASE1_MS) {
      // Fase 1: draai in de richting waar de lijn het laatst gezien werd
      if (lastDir == -1) setMotors(pivotSpeed, -pivotSpeed);
      else               setMotors(-pivotSpeed, pivotSpeed);
      statusTekst = "ZOEKEN_1";
    } else if (zoekDuur < ZOEK_FASE1_MS + ZOEK_FASE2_MS) {
      // Fase 2: draai de andere kant op (lijn eventueel aan de andere zijde)
      if (lastDir == -1) setMotors(-pivotSpeed, pivotSpeed);
      else               setMotors(pivotSpeed, -pivotSpeed);
      statusTekst = "ZOEKEN_2";
    } else {
      // Fase 3: beide fasen mislukt — stop volledig om schade te vermijden
      stopMotors();
      statusTekst = "LIJN_KWIJT";
    }
    prevState = NONE;
  }
  else if (!L && M && !R) {
    // Alleen midden: robot rijdt recht over de lijn.
    // stepMotorsFade zorgt voor geleidelijk optrekken (minder wiel-slip).
    zoekStart = 0; dwarslijnStart = 0;
    stepMotorsFade(baseSpeed, baseSpeed);
    prevState  = STRAIGHT;
    statusTekst = "RECHTDOOR";
  }
  else if (L && !M && !R) {
    // Alleen links: lijn is ver naar links, harde pivot links.
    zoekStart = 0; dwarslijnStart = 0;
    lastDir = -1;
    currentLeft = 0; currentRight = 0;
    setMotors(pivotSpeed, -pivotSpeed);
    prevState  = PIVOT_L;
    statusTekst = "PIVOT_L";
  }
  else if (!L && !M && R) {
    // Alleen rechts: lijn is ver naar rechts, harde pivot rechts.
    zoekStart = 0; dwarslijnStart = 0;
    lastDir = 1;
    currentLeft = 0; currentRight = 0;
    setMotors(-pivotSpeed, pivotSpeed);
    prevState  = PIVOT_R;
    statusTekst = "PIVOT_R";
  }
  else if (L && M && !R) {
    // Links + midden: lijn buigt naar links.
    // Vanuit rechtdoor: zacht ingaan (halve snelheid, binnenste wiel stopt).
    // Vanuit een eerdere bocht: agressievere correctie.
    zoekStart = 0; dwarslijnStart = 0;
    lastDir = -1;
    currentLeft = 0; currentRight = 0;
    if (prevState == STRAIGHT) setMotors(baseSpeed, -100);
    else                       setMotors(baseSpeed, -correction * 3/4);
    prevState  = TURN_L;
    statusTekst = "BOCHT_L";
  }
  else if (!L && M && R) {
    // Midden + rechts: lijn buigt naar rechts. Spiegeling van bovenstaand.
    zoekStart = 0; dwarslijnStart = 0;
    lastDir = 1;
    currentLeft = 0; currentRight = 0;
    if (prevState == STRAIGHT) setMotors(-100, baseSpeed);
    else                       setMotors(-correction * 3/4, baseSpeed);
    prevState  = TURN_R;
    statusTekst = "BOCHT_R";
  }
  else {
    // Alle drie sensoren actief: bocht of kruispunt.
    // Kruispunt alleen bevestigen als de toestand langer dan DWARSLIJN_MS aanhoudt.
    zoekStart = 0;
    if (dwarslijnStart == 0) dwarslijnStart = millis();

    if (millis() - dwarslijnStart >= DWARSLIJN_MS) {
      stopMotors();
      running     = false;
      prevState   = NONE;
      statusTekst = "DWARSLIJN";
      Serial.println("KRUISPUNT gedetecteerd - gestopt");
    } else {
      // Nog te kort actief: waarschijnlijk een bocht, rijd door
      if (lastDir == -1) setMotors(baseSpeed, -100);
      else               setMotors(-100, baseSpeed);
      prevState   = NONE;
      statusTekst = "BOCHT_BREED";
    }
  }

  delay(10); // 20 ms per loop = ~50 Hz lijnvolg-frequentie
             // Verlagen = snellere reactie maar meer CPU-belasting
}
