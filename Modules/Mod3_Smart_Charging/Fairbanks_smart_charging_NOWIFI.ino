#include <LiquidCrystal.h>

// ── LCD ──────────────────────────────────────────────────────
const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// ── Pins ─────────────────────────────────────────────────────
const int MOSFET_PIN = 2;

// ── Battery / Charging ───────────────────────────────────────
const float Ebat    = 10.0;   // kWh
const float Pmax    = 7.2;    // kW
const float eta     = 0.92;
const float SOC_req = 0.80;
const float SOC_res = 0.10;

// ── TOU ──────────────────────────────────────────────────────
const float BASE = 0.12;

// ── Time Discretization ──────────────────────────────────────
const float dt = 0.5;   // 30 min
const int   N  = 48;

// ── USER INPUT VARIABLES (SET VIA SERIAL) ────────────────────
float START_TIME = 17.5;   // hours
float SOC0       = 0.20;
float ARRIVAL    = 17.5;
float DEPARTURE  = 7.0;

// ── State ────────────────────────────────────────────────────
float P_schedule[N];
float rate[N];
float currentSOC;

unsigned long startMillis;
bool configured = false;

// ============================================================
// TOU RATE
// ============================================================
float getRate(float h) {
  if (h >= 23 || h < 6) return 0.65 * BASE;
  if (h >= 17 && h < 21) return 1.60 * BASE;
  return 0.90 * BASE;
}

// ============================================================
// BUILD SCHEDULE (cleaned version)
// ============================================================
void buildSchedule() {
  for (int i = 0; i < N; i++) {
    float h = i * dt;
    rate[i] = getRate(h);
    P_schedule[i] = 0;
  }

  int a = (int)(ARRIVAL / dt);
  int d = (int)(DEPARTURE / dt);

  int idx[N];
  int len = 0;

  if (d < a) {
    for (int i = a; i < N; i++) idx[len++] = i;
    for (int i = 0; i <= d; i++) idx[len++] = i;
  } else {
    for (int i = a; i <= d; i++) idx[len++] = i;
  }

  float Ereq = max((SOC_req - SOC0) * Ebat, 0.0);
  float Erem = Ereq;
  float SOC  = SOC0;

  // sort cheapest first
  for (int i = 0; i < len - 1; i++) {
    for (int j = i + 1; j < len; j++) {
      if (rate[idx[j]] < rate[idx[i]]) {
        int tmp = idx[i];
        idx[i] = idx[j];
        idx[j] = tmp;
      }
    }
  }

  for (int i = 0; i < len && Erem > 0; i++) {
    int k = idx[i];

    float Emax = Pmax * dt * eta;
    float Eadd = min(Emax, Erem);

    P_schedule[k] = Eadd / (dt * eta);
    SOC += Eadd / Ebat;
    Erem -= Eadd;
  }

  currentSOC = SOC0;
}

// ============================================================
// TIME (manual + millis)
// ============================================================
float getCurrentHour() {
  float elapsed = (millis() - startMillis) / 3600000.0;
  float h = START_TIME + elapsed;

  while (h >= 24) h -= 24;
  return h;
}

// ============================================================
// SERIAL CONFIG INPUT
// Example:
// T=17.5
// SOC=0.3
// A=18
// D=7
// ============================================================
void checkSerial() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');

  if (s.startsWith("T=")) START_TIME = s.substring(2).toFloat();
  if (s.startsWith("SOC=")) SOC0 = s.substring(4).toFloat();
  if (s.startsWith("A=")) ARRIVAL = s.substring(2).toFloat();
  if (s.startsWith("D=")) DEPARTURE = s.substring(2).toFloat();

  Serial.println("Updated parameter.");
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  pinMode(MOSFET_PIN, OUTPUT);

  lcd.begin(20,4);
  lcd.print("Send config:");

  Serial.println("Enter:");
  Serial.println("T=17.5 (time)");
  Serial.println("SOC=0.2");
  Serial.println("A=17.5");
  Serial.println("D=7");
  Serial.println("Then type: START");

  while (!configured) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');

      if (s.startsWith("T=")) START_TIME = s.substring(2).toFloat();
      else if (s.startsWith("SOC=")) SOC0 = s.substring(4).toFloat();
      else if (s.startsWith("A=")) ARRIVAL = s.substring(2).toFloat();
      else if (s.startsWith("D=")) DEPARTURE = s.substring(2).toFloat();
      else if (s.startsWith("START")) configured = true;
    }
  }

  buildSchedule();
  startMillis = millis();

  lcd.clear();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  checkSerial();

  float hour = getCurrentHour();
  int step = (int)(hour / dt);

  bool shouldCharge = P_schedule[step] > 0.01;

  if (currentSOC < SOC_res) {
    digitalWrite(MOSFET_PIN, HIGH);
  }
  else if (currentSOC >= SOC_req) {
    digitalWrite(MOSFET_PIN, LOW);
  }
  else if (shouldCharge) {
    digitalWrite(MOSFET_PIN, HIGH);
  }
  else {
    digitalWrite(MOSFET_PIN, LOW);
  }

  // crude SOC update
  if (digitalRead(MOSFET_PIN)) {
    float dSOC = (Pmax * 0.75 / 3600.0) / Ebat;
    currentSOC += dSOC;
  }

  // LCD
  lcd.setCursor(0,0);
  lcd.print("Hr:");
  lcd.print(hour,1);

  lcd.setCursor(0,1);
  lcd.print("SOC:");
  lcd.print(currentSOC*100,0);

  lcd.setCursor(0,2);
  lcd.print("Step:");
  lcd.print(step);

  lcd.setCursor(0,3);
  lcd.print("P:");
  lcd.print(P_schedule[step],1);

  delay(750);
}
