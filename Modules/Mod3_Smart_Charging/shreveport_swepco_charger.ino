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

// ── SWEPCO TOU Rates (Shreveport) ────────────────────────────
// $0.1095  5:00am  - 3:00pm
// $0.1604  3:00pm  - 7:00pm
// $0.1095  7:00pm  - 11:00pm
// $0.0318  11:00pm - 5:00am

// ── Time Discretization ──────────────────────────────────────
const float dt = 0.5;   // 30 min slots
const int   N  = 48;

// ── USER INPUT VARIABLES (SET VIA SERIAL) ────────────────────
float START_TIME = 17.5;   // current hour (24h decimal)
float SOC0       = 0.20;   // initial SOC (0.0 - 1.0)
float ARRIVAL    = 17.5;   // plug-in time (24h decimal)
float DEPARTURE  = 7.0;    // departure time (24h decimal)

// ── State ────────────────────────────────────────────────────
float P_schedule[N];
float rate[N];
float currentSOC;

unsigned long startMillis;
bool configured = false;

// ============================================================
// SWEPCO TOU RATE  (Shreveport, LA)
// ============================================================
float getRate(float h) {
  // 11:00pm (23:00) to 5:00am  → off-peak night
  if (h >= 23.0 || h < 5.0)  return 0.0318;
  // 5:00am  to 3:00pm (15:00)  → mid rate
  if (h >= 5.0  && h < 15.0) return 0.1095;
  // 3:00pm  to 7:00pm (19:00)  → peak
  if (h >= 15.0 && h < 19.0) return 0.1604;
  // 7:00pm  to 11:00pm         → mid rate
  return 0.1095;
}

// ============================================================
// BUILD OPTIMAL SCHEDULE
// ============================================================
void buildSchedule() {
  for (int i = 0; i < N; i++) {
    float h = i * dt;
    rate[i] = getRate(h);
    P_schedule[i] = 0;
  }

  int a = (int)(ARRIVAL   / dt);
  int d = (int)(DEPARTURE / dt);

  int idx[N];
  int len = 0;

  // handle overnight window (arrival > departure across midnight)
  if (d < a) {
    for (int i = a; i < N; i++) idx[len++] = i;
    for (int i = 0; i <= d; i++) idx[len++] = i;
  } else {
    for (int i = a; i <= d; i++) idx[len++] = i;
  }

  float Ereq = max((SOC_req - SOC0) * Ebat, 0.0f);
  float Erem = Ereq;

  // sort slots cheapest first (bubble sort)
  for (int i = 0; i < len - 1; i++) {
    for (int j = i + 1; j < len; j++) {
      if (rate[idx[j]] < rate[idx[i]]) {
        int tmp = idx[i];
        idx[i]  = idx[j];
        idx[j]  = tmp;
      }
    }
  }

  for (int i = 0; i < len && Erem > 0; i++) {
    int k = idx[i];
    float Emax = Pmax * dt * eta;
    float Eadd = min(Emax, Erem);
    P_schedule[k] = Eadd / (dt * eta);
    Erem -= Eadd;
  }

  currentSOC = SOC0;
}

// ============================================================
// SIMULATED CLOCK
// ============================================================
float getCurrentHour() {
  float elapsed = (millis() - startMillis) / 3600000.0;
  float h = START_TIME + elapsed;
  while (h >= 24.0) h -= 24.0;
  return h;
}

// ============================================================
// SERIAL CONFIG  (live updates during run)
// ============================================================
void checkSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();

  if (s.startsWith("T="))   START_TIME = s.substring(2).toFloat();
  if (s.startsWith("SOC=")) { SOC0 = s.substring(4).toFloat(); currentSOC = SOC0; }
  if (s.startsWith("A="))   ARRIVAL    = s.substring(2).toFloat();
  if (s.startsWith("D="))   DEPARTURE  = s.substring(2).toFloat();
  if (s.startsWith("REBUILD")) { buildSchedule(); Serial.println("Schedule rebuilt."); }

  Serial.println("Parameter updated.");
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  pinMode(MOSFET_PIN, OUTPUT);

  lcd.begin(20, 4);
  lcd.print("SWEPCO - Shreveport");
  lcd.setCursor(0, 1);
  lcd.print("Send config...");

  Serial.println("=== SHREVEPORT (SWEPCO) EV CHARGER ===");
  Serial.println("SWEPCO TOU Rates:");
  Serial.println("  $0.0318  11pm - 5am");
  Serial.println("  $0.1095   5am - 3pm");
  Serial.println("  $0.1604   3pm - 7pm");
  Serial.println("  $0.1095   7pm - 11pm");
  Serial.println("---------------------------------------");
  Serial.println("Enter parameters (one per line):");
  Serial.println("  T=<current hour 0-23.9>  e.g. T=17.5");
  Serial.println("  SOC=<0.0-1.0>            e.g. SOC=0.2");
  Serial.println("  A=<arrival hour>         e.g. A=17.5");
  Serial.println("  D=<departure hour>       e.g. D=7.0");
  Serial.println("Then type: START");

  while (!configured) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();

      if      (s.startsWith("T="))   START_TIME = s.substring(2).toFloat();
      else if (s.startsWith("SOC=")) SOC0       = s.substring(4).toFloat();
      else if (s.startsWith("A="))   ARRIVAL    = s.substring(2).toFloat();
      else if (s.startsWith("D="))   DEPARTURE  = s.substring(2).toFloat();
      else if (s == "START")         configured = true;

      if (!configured) Serial.println("OK - send more or type START");
    }
  }

  buildSchedule();
  startMillis = millis();

  Serial.println("Schedule built. Running...");
  Serial.println("(Live: T= SOC= A= D= REBUILD accepted anytime)");
  lcd.clear();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  checkSerial();

  float hour = getCurrentHour();
  int   step = (int)(hour / dt);
  if (step >= N) step = N - 1;

  bool shouldCharge = (P_schedule[step] > 0.01);

  // Charging logic with reserve / full cutoffs
  if (currentSOC < SOC_res) {
    digitalWrite(MOSFET_PIN, HIGH);   // emergency charge
  } else if (currentSOC >= SOC_req) {
    digitalWrite(MOSFET_PIN, LOW);    // full
  } else if (shouldCharge) {
    digitalWrite(MOSFET_PIN, HIGH);   // scheduled slot
  } else {
    digitalWrite(MOSFET_PIN, LOW);    // off-peak wait
  }

  // Crude SOC integration (based on 75% of Pmax assumption)
  if (digitalRead(MOSFET_PIN)) {
    float dSOC = (Pmax * eta * 0.75 / 3600.0) / Ebat;
    currentSOC += dSOC;
    if (currentSOC > 1.0) currentSOC = 1.0;
  }

  // ── LCD Display ──────────────────────────────────────────
  lcd.setCursor(0, 0);
  lcd.print("Hr:");
  lcd.print(hour, 1);
  lcd.print("  Rate:$");
  lcd.print(rate[step], 4);

  lcd.setCursor(0, 1);
  lcd.print("SOC:");
  lcd.print(currentSOC * 100, 0);
  lcd.print("%  ");
  lcd.print(digitalRead(MOSFET_PIN) ? "CHG " : "OFF ");

  lcd.setCursor(0, 2);
  lcd.print("Slot:");
  lcd.print(step);
  lcd.print("  P:");
  lcd.print(P_schedule[step], 1);
  lcd.print("kW  ");

  lcd.setCursor(0, 3);
  // Show cheapest upcoming slot rate for reference
  lcd.print("Depart:");
  lcd.print(DEPARTURE, 1);
  lcd.print("  Arr:");
  lcd.print(ARRIVAL, 1);

  delay(750);
}
