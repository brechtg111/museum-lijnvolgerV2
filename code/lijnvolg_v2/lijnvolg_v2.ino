// =============================================
// LIJNVOLGTEST - 3 DIGITALE IR SENSOREN
// Met pivot turn, fade en achteruit test
// =============================================

#define IR_L 34
#define IR_M 35
#define IR_R 33

#define ENA 19
#define IN1 18
#define IN2 5
#define ENB 16
#define IN3 4
#define IN4 2

#define Q 2

int baseSpeed  = 180;
int correction = 150;
int pivotSpeed = 250;

// Buffers voor majority vote ruisfiltering
int bufL[Q] = {};
int bufM[Q] = {};
int bufR[Q] = {};
int bufIndex = 0;

// Huidige en laatste motorsnelheden bijhouden
int currentLeft  = 0;
int currentRight = 0;
int lastLeft     = 0;
int lastRight    = 0;
int lastDir      = 1; // 1 = rechts, -1 = links

// Staten om te weten wat de auto vorige loop deed
enum State { STRAIGHT, TURN_L, TURN_R, PIVOT_L, PIVOT_R, NONE };
State prevState = NONE;

// ---------------------------------------------
// Majority vote: geeft true als meer dan de helft
// van de buffer-waarden 1 zijn (lijn gedetecteerd)
// ---------------------------------------------
bool majorityVote(int* buf) {
  int sum = 0;
  for (int i = 0; i < Q; i++) sum += buf[i];
  return sum > (Q / 2);
}

// ---------------------------------------------
// Stopt beide motoren volledig
// en reset de huidige snelheden naar 0
// ---------------------------------------------
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
  currentLeft  = 0;
  currentRight = 0;
}

// ---------------------------------------------
// Zet beide motoren op een opgegeven snelheid
// Negatieve waarde = achteruit voor dat wiel
// Slaat de snelheid op in lastLeft/lastRight
// ---------------------------------------------
void setMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    rightSpeed = -rightSpeed;
  }
  ledcWrite(ENA, constrain(leftSpeed,  0, 255));
  ledcWrite(ENB, constrain(rightSpeed, 0, 255));
  lastLeft  = leftSpeed;
  lastRight = rightSpeed;
}

// ---------------------------------------------
// Verhoogt of verlaagt de motorsnelheid geleidelijk
// naar de doelsnelheid met stapjes van 'step'
// Wordt gebruikt voor vloeiend optrekken rechtdoor
// ---------------------------------------------
void stepMotorsFade(int targetLeft, int targetRight, int step = 8) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

  if (currentLeft  < targetLeft)  currentLeft  = min(currentLeft  + step, targetLeft);
  if (currentLeft  > targetLeft)  currentLeft  = max(currentLeft  - step, targetLeft);
  if (currentRight < targetRight) currentRight = min(currentRight + step, targetRight);
  if (currentRight > targetRight) currentRight = max(currentRight - step, targetRight);

  ledcWrite(ENA, constrain(currentLeft,  0, 255));
  ledcWrite(ENB, constrain(currentRight, 0, 255));

  lastLeft  = currentLeft;
  lastRight = currentRight;
}

// ---------------------------------------------
// Testfunctie bij opstarten: rijdt 1 seconde achteruit
// om te controleren of de motoren correct werken
// ---------------------------------------------
void testAchteruit() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, 100);
  ledcWrite(ENB, 100);
  delay(1000);
  stopMotors();
  delay(500);
}

// ---------------------------------------------
// Setup: pinnen instellen, PWM initialiseren,
// sensorbuffers voorvullen en achteruitTest uitvoeren
// ---------------------------------------------
void setup() {
  pinMode(IR_L, INPUT);
  pinMode(IR_M, INPUT);
  pinMode(IR_R, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcAttach(ENA, 5000, 8);
  ledcAttach(ENB, 5000, 8);

  // Buffers vullen met huidige sensorwaarden zodat majority vote meteen klopt
  for (int i = 0; i < Q; i++) {
    bufL[i] = digitalRead(IR_L);
    bufM[i] = digitalRead(IR_M);
    bufR[i] = digitalRead(IR_R);
  }

  testAchteruit();
}

// ---------------------------------------------
// Hoofdlus: sensoren uitlezen, majority vote toepassen
// en op basis van het sensorpatroon de motoren aansturen
// ---------------------------------------------
void loop() {
  // Sensorwaarden in de buffer opslaan (ringbuffer)
  bufL[bufIndex] = digitalRead(IR_L);
  bufM[bufIndex] = digitalRead(IR_M);
  bufR[bufIndex] = digitalRead(IR_R);
  bufIndex = (bufIndex + 1) % Q;

  // Gefilterde sensorwaarden berekenen via majority vote
  bool L = majorityVote(bufL);
  bool M = majorityVote(bufM);
  bool R = majorityVote(bufR);

  // Geen enkele sensor ziet de lijn: draai in laatste bekende richting
  if (!L && !M && !R) {
    currentLeft = 0; currentRight = 0;
    if (lastDir == -1) setMotors(pivotSpeed, -pivotSpeed);
    else               setMotors(-pivotSpeed, pivotSpeed);
    prevState = NONE;
  }
  // Alleen midden ziet de lijn: rechtdoor met fade
  else if (!L && M && !R) {
    stepMotorsFade(baseSpeed, baseSpeed);
    prevState = STRAIGHT;
  }
  // Alleen links ziet de lijn: pivot links ter plaatse
  else if (L && !M && !R) {
    lastDir = -1;
    currentLeft = 0; currentRight = 0;
    setMotors(pivotSpeed, -pivotSpeed);
    prevState = PIVOT_L;
  }
  // Alleen rechts ziet de lijn: pivot rechts ter plaatse
  else if (!L && !M && R) {
    lastDir = 1;
    currentLeft = 0; currentRight = 0;
    setMotors(-pivotSpeed, pivotSpeed);
    prevState = PIVOT_R;
  }
  // Midden en links zien de lijn: bocht naar links
  // Eerste detectie vanuit rechtdoor = zacht ingaan, daarna agressief
  else if (L && M && !R) {
    lastDir = -1;
    currentLeft = 0; currentRight = 0;
    if (prevState == STRAIGHT)
      setMotors(baseSpeed / 2, 0);       // Vertraagde ingang vanuit rechte lijn
    else
      setMotors(baseSpeed, -correction / 2); // Agressieve correctie
    prevState = TURN_L;
  }
  // Midden en rechts zien de lijn: bocht naar rechts
  // Eerste detectie vanuit rechtdoor = zacht ingaan, daarna agressief
  else if (!L && M && R) {
    lastDir = 1;
    currentLeft = 0; currentRight = 0;
    if (prevState == STRAIGHT)
      setMotors(0, baseSpeed / 2);       // Vertraagde ingang vanuit rechte lijn
    else
      setMotors(-correction / 2, baseSpeed); // Agressieve correctie
    prevState = TURN_R;
  }
  // Alle drie sensoren actief (dwarslijn): stop
  else {
    stopMotors();
    prevState = NONE;
  }
  delay(20);
}
