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

// ── Cedar City TOU Rates (effective Dec 1 2025) ──────────────
// On-peak  (weekdays 6pm-10pm):
//   June-Sep  → 32.0834 ¢/kWh
//   Oct-May   → 28.3924 ¢/kWh
// Off-peak (all other times):
//   June-Sep  →  7.1296 ¢/kWh
//   Oct-May   →  6.3094 ¢/kWh

// ── Time Discretization ──────────────────────────────────────
const float dt = 0.5;   // 30 min slots
const int   N  = 48;

// ── USER INPUT VARIABLES (SET VIA SERIAL) ────────────────────
float START_TIME = 17.5;  // current hour (24h decimal)
float SOC0       = 0.20;  // initial SOC (0.0 - 1.0)
float ARRIVAL    = 17.5;  // plug-in time (24h decimal)
float DEPARTURE  = 7.0;   // departure time (24h decimal)
int   MONTH      = 7;     // 1=Jan ... 12=Dec  (determines season)
int   DOW        = 1;     // Day of week: 0=Sun, 1=Mon...5=Fri, 6=Sat
                          // On-peak only applies Mon-Fri

// ── State ────────────────────────────────────────────────────
float P_schedule[N];
float rate[N];
float currentSOC;

unsigned long startMillis;
bool configured = false;

// ============================================================
// CEDAR CITY TOU RATE
// Peak window: weekdays (Mon-Fri) 6pm (18:00) - 10pm (22:00)
// Season:  June(6)-Sep(9) = summer,  Oct(10)-May(5) = winter
// ============================================================
bool isSummer() {
  return (MONTH >= 6 && MONTH <= 9);
}

bool isWeekday() {
  return (DOW >= 1 && DOW <= 5);   // Mon=1 ... Fri=5
}

float getRate(float h) {
  // On-peak window: 18:00 - 22:00, weekdays only
  bool onPeak = isWeekday() && (h >= 18.0 && h < 22.0);

  if (isSummer()) {
    return onPeak ? 0.320834 : 0.071296;
  } else {
    return onPeak ? 0.283924 : 0.063094;
  }
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

  // handle overnight window
  if (d < a) {
    for (int i = a; i < N; i++) idx[len++] = i;
    for (int i = 0; i <= d;  i++) idx[len++] = i;
  } else {
    for (int i = a; i <= d;  i++) idx[len++] = i;
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
// HELPER: DOW name string
// ============================================================
const char* dowName() {
  const char* days[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  if (DOW < 0 || DOW > 6) return "???";
  return days[DOW];
}

// ============================================================
// SERIAL CONFIG  (live updates during run)
// ============================================================
void checkSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();

  bool rebuild = false;

  if (s.startsWith("T="))   { START_TIME = s.substring(2).toFloat(); }
  if (s.startsWith("SOC=")) { SOC0 = s.substring(4).toFloat(); currentSOC = SOC0; }
  if (s.startsWith("A="))   { ARRIVAL    = s.substring(2).toFloat(); rebuild = true; }
  if (s.startsWith("D="))   { DEPARTURE  = s.substring(2).toFloat(); rebuild = true; }
  if (s.startsWith("M="))   { MONTH      = (int)s.substring(2).toFloat(); rebuild = true; }
  if (s.startsWith("DOW=")) { DOW        = (int)s.substring(4).toFloat(); rebuild = true; }
  if (s.startsWith("REBUILD")) rebuild = true;

  if (rebuild) {
    buildSchedule();
    Serial.print("Schedule rebuilt. Season: ");
    Serial.print(isSummer() ? "Summer" : "Winter");
    Serial.print("  Day: ");
    Serial.print(isWeekday() ? "Weekday" : "Weekend");
    Serial.println(isWeekday() ? " (peak 6-10pm)" : " (no peak today)");
  }

  Serial.println("Parameter updated.");
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  pinMode(MOSFET_PIN, OUTPUT);

  lcd.begin(20, 4);
  lcd.print("Cedar City EV Chrgr");
  lcd.setCursor(0, 1);
  lcd.print("Send config...");

  Serial.println("=== CEDAR CITY EV CHARGER ===");
  Serial.println("TOU Rates (effective Dec 1 2025):");
  Serial.println("  On-peak  (wkdy 6pm-10pm):");
  Serial.println("    Summer (Jun-Sep): 32.0834 c/kWh");
  Serial.println("    Winter (Oct-May): 28.3924 c/kWh");
  Serial.println("  Off-peak (all other times):");
  Serial.println("    Summer (Jun-Sep):  7.1296 c/kWh");
  Serial.println("    Winter (Oct-May):  6.3094 c/kWh");
  Serial.println("-------------------------------");
  Serial.println("Enter parameters (one per line):");
  Serial.println("  T=<current hour 0-23.9>  e.g. T=17.5");
  Serial.println("  SOC=<0.0-1.0>            e.g. SOC=0.2");
  Serial.println("  A=<arrival hour>         e.g. A=17.5");
  Serial.println("  D=<departure hour>       e.g. D=7.0");
  Serial.println("  M=<month 1-12>           e.g. M=7  (July)");
  Serial.println("  DOW=<0=Sun...6=Sat>      e.g. DOW=1 (Mon)");
  Serial.println("Then type: START");

  while (!configured) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();

      if      (s.startsWith("T="))   START_TIME = s.substring(2).toFloat();
      else if (s.startsWith("SOC=")) SOC0       = s.substring(4).toFloat();
      else if (s.startsWith("A="))   ARRIVAL    = s.substring(2).toFloat();
      else if (s.startsWith("D="))   DEPARTURE  = s.substring(2).toFloat();
      else if (s.startsWith("M="))   MONTH      = (int)s.substring(2).toFloat();
      else if (s.startsWith("DOW=")) DOW        = (int)s.substring(4).toFloat();
      else if (s == "START")         configured = true;

      if (!configured) {
        Serial.print("OK [M=");
        Serial.print(MONTH);
        Serial.print(" DOW=");
        Serial.print(dowName());
        Serial.println("] - send more or type START");
      }
    }
  }

  buildSchedule();
  startMillis = millis();

  Serial.print("Schedule built. Season: ");
  Serial.print(isSummer() ? "Summer" : "Winter");
  Serial.print("  Day: ");
  Serial.println(isWeekday() ? "Weekday (peak 6-10pm)" : "Weekend (no peak)");
  Serial.println("(Live: T= SOC= A= D= M= DOW= REBUILD accepted anytime)");

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
  bool onPeak = isWeekday() && (hour >= 18.0 && hour < 22.0);

  // Charging logic with reserve / full cutoffs
  if (currentSOC < SOC_res) {
    digitalWrite(MOSFET_PIN, HIGH);   // emergency charge
  } else if (currentSOC >= SOC_req) {
    digitalWrite(MOSFET_PIN, LOW);    // full
  } else if (shouldCharge) {
    digitalWrite(MOSFET_PIN, HIGH);   // scheduled slot
  } else {
    digitalWrite(MOSFET_PIN, LOW);    // waiting for cheaper slot
  }

  // Crude SOC integration
  if (digitalRead(MOSFET_PIN)) {
    float dSOC = (Pmax * eta * 0.75 / 3600.0) / Ebat;
    currentSOC += dSOC;
    if (currentSOC > 1.0) currentSOC = 1.0;
  }

  // ── LCD Display ──────────────────────────────────────────
  lcd.setCursor(0, 0);
  lcd.print(dowName());
  lcd.print(" M:");
  lcd.print(MONTH);
  lcd.print(isSummer() ? " SUM" : " WIN");
  lcd.print(onPeak     ? " PK " : "    ");

  lcd.setCursor(0, 1);
  lcd.print("Hr:");
  lcd.print(hour, 1);
  lcd.print(" $");
  lcd.print(rate[step], 4);

  lcd.setCursor(0, 2);
  lcd.print("SOC:");
  lcd.print(currentSOC * 100, 0);
  lcd.print("%  ");
  lcd.print(digitalRead(MOSFET_PIN) ? "CHG " : "OFF ");

  lcd.setCursor(0, 3);
  lcd.print("P:");
  lcd.print(P_schedule[step], 1);
  lcd.print("kW Dep:");
  lcd.print(DEPARTURE, 1);

  delay(750);
}
