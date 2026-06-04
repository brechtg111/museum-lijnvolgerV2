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
const char* TOPIC_TELEMETRY = "museum/wagon/01/telemetry"; // topic waarop de robot publiceert

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
#define IR_R 33   // rechter IR-sensor

// L298N motordriver - linkermotor via ENA/IN1/IN2, rechtermotor via ENB/IN3/IN4
#define ENA 19    // PWM snelheidsregeling linkermotor (enable A)
#define IN1 18    // rijrichting linkermotor - bit 1
#define IN2  5    // rijrichting linkermotor - bit 2
#define ENB 16    // PWM snelheidsregeling rechtermotor (enable B)
#define IN3  4    // rijrichting rechtermotor - bit 1
#define IN4  2    // rijrichting rechtermotor - bit 2

#define TRIG_PIN  25  // ultrasone sensor: trigger (output)
#define ECHO_PIN  27  // ultrasone sensor: echo (input)
#define SERVO_PIN 14  // servo voor scan-richting

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
#define Q 2
int baseSpeed  = 180;
int correction = 150;
int pivotSpeed = 250;

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
#define OBSTAKEL_AFSTAND     15   // cm - obstakeldetectie drempel
#define OBJECT_ZIJ_MAX       30   // cm - max afstand om object nog als "aanwezig" te beschouwen
#define MAX_AFSTAND         200   // cm - meetwaarden boven dit worden genegeerd (-1)
#define RIJD_SNELHEID       195   // PWM rijsnelheid tijdens ontwijking
#define DRAAI_SNELHEID      210   // PWM draaisnelheid
#define DRAAI_90_MS         600   // ms voor ~90° draai
#define SIDESTEP_MS         400   // ms rechtdoor na eerste draai
#define EXTRA_MS            200   // ms extra vooruit na voorbijrijden
#define PAS_TIMEOUT_MS     4000   // ms maximale voorbijrij-tijd
#define ULTRA_INTERVAL      100   // ms tussen ultrasone metingen
#define BOCHT_BINNEN        150   // PWM binnenwiel terugbocht
#define BOCHT_BUITEN        200   // PWM buitenwiel terugbocht
#define LIJN_ZOEK_MS       2500   // ms maximale zoektijd voor de lijn
#define OBSTAKEL_COOLDOWN_MS 3000 // ms geen nieuwe obstakeldetectie na ontwijking

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

unsigned long lastUltra       = 0; // tijdstip laatste ultrasone meting
unsigned long obstakelNegeerTot = 0; // tot wanneer obstakels genegeerd worden na ontwijking

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
  if (leftSpeed >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); leftSpeed = -leftSpeed; }

  // Richting rechtermotor
  if (rightSpeed >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); rightSpeed = -rightSpeed; }

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
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

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
  mqttClient.connect("ESP32-lijnauto", mqtt_user, mqtt_pass);
}


// =============================================================
// publishTelemetry
// Verstuurt een JSON-bericht met de huidige status naar de broker.
// Formaat: {"status":"...","afstand":...,"ir_l":...,"ir_m":...,"ir_r":...}
// Wordt alleen verstuurd als er een actieve MQTT-verbinding is.
// =============================================================
void publishTelemetry() {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<200> doc;
  doc["status"]  = statusTekst;
  doc["afstand"] = afstandCm;
  doc["ir_l"] = sL; doc["ir_m"] = sM; doc["ir_r"] = sR;
  char buf[200];
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
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
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
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); // linker achteruit
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // rechter vooruit
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
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // linker vooruit
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); // rechter achteruit
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
// Stappen:
//   1. Stop + 1 s wachten (veiligheidsrust)
//   2. Rechts draaien, zijwaarts rijden, links draaien
//      → robot staat nu naast het obstakel
//   3. Servo naar het obstakel draaien en vooruitrijden
//      tot het obstakel 3x achter elkaar buiten OBJECT_ZIJ_MAX cm valt
//   4. Extra stukje vooruit (EXTRA_MS) voor voldoende speling
//   5. Servo terug vooruit, zachte bocht naar links
//      → stopt zodra een IR-sensor de lijn detecteert
//
// AANPASSEN ALS:
//   - robot draait te ver/weinig: pas DRAAI_90_MS aan
//   - robot rijdt te ver/weinig naast object: pas SIDESTEP_MS aan
//   - robot vindt de lijn niet terug: pas BOCHT_BINNEN/BOCHT_BUITEN of LIJN_ZOEK_MS aan
// =============================================================
void ontwijkRechts() {
  statusTekst = "OBSTAKEL"; publishTelemetry();
  Serial.println("OBSTAKEL gezien - stop 1 seconde");
  stopMotors(); delay(1000);

  statusTekst = "ONTWIJKEN"; publishTelemetry();

  // Stap 1: zijwaarts rechts naast het object plaatsen
  draaiRechts(DRAAI_90_MS);
  vooruitTijd(SIDESTEP_MS);
  draaiLinks(DRAAI_90_MS);

  // Stap 2: servo richting object, vooruitrijden tot object voorbij is
  servoNaar(SERVO_OBJECT);
  delay(300); // wacht tot servo stil staat voor eerste meting
  vooruit(RIJD_SNELHEID);
  unsigned long start = millis();
  int kwijt = 0; // teller: hoe vaak het object al buiten bereik was
  while (true) {
    int d = meetAfstand();
    afstandCm = d;
    mqttOnderhoud(); publishTelemetry();
    if (d == -1 || d > OBJECT_ZIJ_MAX) {
      if (++kwijt >= 3) break; // 3 opeenvolgende metingen buiten bereik = object voorbij
    } else {
      kwijt = 0; // object terug in beeld, teller resetten
    }
    if (millis() - start > PAS_TIMEOUT_MS) break; // veiligheidsgrens
    delay(60); // wacht tussen metingen
  }
  stopMotors();

  // Stap 3: extra stukje vooruit zodat de robot volledig voorbij het object is
  vooruitTijd(EXTRA_MS);

  // Stap 4: servo terug vooruit, zachte bocht naar links richting de lijn
  // Stopt zodra een IR-sensor de lijn ziet (geen blinde vaste draai)
  servoNaar(SERVO_VOORUIT);
  statusTekst = "LIJN_ZOEKEN"; publishTelemetry();
  lastDir = -1; // na ontwijking rechts ligt de lijn links van de robot
  unsigned long tZoek = millis();
  while (millis() - tZoek < LIJN_ZOEK_MS) {
    if (digitalRead(IR_L) || digitalRead(IR_M) || digitalRead(IR_R)) break; // lijn gevonden
    setMotors(BOCHT_BINNEN, BOCHT_BUITEN); // zachte bocht: buitenwiel sneller dan binnenwiel
    delay(10);
  }
  stopMotors();

  statusTekst = "LIJN_HERVAT"; publishTelemetry();
  Serial.println("terug op de lijn - lijnvolgen hervat");
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

  connectWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);

  Serial.println("gereed - lijnvolgen + obstakelontwijking + MQTT");
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
    // Draai ter plaatse in de laatste bekende richting om de lijn te zoeken.
    currentLeft = 0; currentRight = 0;
    if (lastDir == -1) setMotors(pivotSpeed, -pivotSpeed); // pivot links
    else               setMotors(-pivotSpeed, pivotSpeed); // pivot rechts
    prevState  = NONE;
    statusTekst = "ZOEKEN";
  }
  else if (!L && M && !R) {
    // Alleen midden: robot rijdt recht over de lijn.
    // stepMotorsFade zorgt voor geleidelijk optrekken (minder wiel-slip).
    stepMotorsFade(baseSpeed, baseSpeed);
    prevState  = STRAIGHT;
    statusTekst = "RECHTDOOR";
  }
  else if (L && !M && !R) {
    // Alleen links: lijn is ver naar links, harde pivot links.
    lastDir = -1;
    currentLeft = 0; currentRight = 0;
    setMotors(pivotSpeed, -pivotSpeed);
    prevState  = PIVOT_L;
    statusTekst = "PIVOT_L";
  }
  else if (!L && !M && R) {
    // Alleen rechts: lijn is ver naar rechts, harde pivot rechts.
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
    lastDir = -1;
    currentLeft = 0; currentRight = 0;
    if (prevState == STRAIGHT) setMotors(baseSpeed / 2, 0);
    else                       setMotors(baseSpeed, -correction / 2);
    prevState  = TURN_L;
    statusTekst = "BOCHT_L";
  }
  else if (!L && M && R) {
    // Midden + rechts: lijn buigt naar rechts. Spiegeling van bovenstaand.
    lastDir = 1;
    currentLeft = 0; currentRight = 0;
    if (prevState == STRAIGHT) setMotors(0, baseSpeed / 2);
    else                       setMotors(-correction / 2, baseSpeed);
    prevState  = TURN_R;
    statusTekst = "BOCHT_R";
  }
  else {
    // Alle drie sensoren actief tegelijk = dwarslijn of kruispunt.
    // Robot stopt volledig. Aanpassen als je kruispunten anders wil behandelen.
    stopMotors();
    prevState  = NONE;
    statusTekst = "DWARSLIJN";
  }

  delay(20); // 20 ms per loop = ~50 Hz lijnvolg-frequentie
             // Verlagen = snellere reactie maar meer CPU-belasting
}
