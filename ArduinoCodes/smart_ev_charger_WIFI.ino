#include <WiFiS3.h>
#include <time.h>
#include <LiquidCrystal.h>

// ── WiFi Credentials ─────────────────────────────────────────
char ssid[] = "YOUR_WIFI";
char pass[] = "YOUR_PASSWORD";

// ── LCD ──────────────────────────────────────────────────────
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// ── Hardware ─────────────────────────────────────────────────
const int MOSFET_PIN = 2;
const int CUR_PIN = A3;
const int VOLT_PIN = A2;

// ── Battery ──────────────────────────────────────────────────
const float Ebat = 10.0;   // kWh
const float Pmax = 7.2;
const float eta  = 0.92;

float currentSOC = 0.2;

// ── TOU ──────────────────────────────────────────────────────
const float BASE = 0.12;
const float dt = 0.5;
const int N = 48;

float P_schedule[N];
float rate[N];

// ── Time ─────────────────────────────────────────────────────
long gmtOffset_sec = -5 * 3600; // EST
int daylightOffset_sec = 3600;

// ============================================================
// TOU RATE
// ============================================================
float getRate(float h) {
  if (h >= 23 || h < 6) return 0.65 * BASE;
  if (h >= 17 && h < 21) return 1.60 * BASE;
  return 0.90 * BASE;
}

// ============================================================
// BUILD SCHEDULE
// ============================================================
void buildSchedule() {
  for (int i = 0; i < N; i++) {
    float h = i * dt;
    rate[i] = getRate(h);
    P_schedule[i] = 0;
  }

  float SOC_req = 0.8;
  float ARRIVAL = 17.5;
  float DEPART  = 7.0;

  int a = ARRIVAL / dt;
  int d = DEPART / dt;

  int idx[N], len = 0;

  if (d < a) {
    for (int i = a; i < N; i++) idx[len++] = i;
    for (int i = 0; i <= d; i++) idx[len++] = i;
  } else {
    for (int i = a; i <= d; i++) idx[len++] = i;
  }

  float Erem = (SOC_req - currentSOC) * Ebat;

  // sort cheapest
  for (int i = 0; i < len - 1; i++) {
    for (int j = i + 1; j < len; j++) {
      if (rate[idx[j]] < rate[idx[i]]) {
        int t = idx[i]; idx[i] = idx[j]; idx[j] = t;
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
}

// ============================================================
// READ SENSORS
// ============================================================
float readVoltage() {
  return analogRead(VOLT_PIN) * (5.0 / 16383.0) * 12.0;
}

float readCurrent() {
  return (analogRead(CUR_PIN) * (5.0 / 16383.0) - 2.5) / 0.4;
}

// ============================================================
// SOC ESTIMATION (hybrid)
// ============================================================
void updateSOC() {
  float current = readCurrent();
  float dt_h = 0.75 / 3600.0;

  // Coulomb count
  currentSOC += (current * dt_h) / (Ebat * 1000.0);

  // Voltage correction (rough)
  float V = readVoltage();
  float soc_est = (V - 300) / (420 - 300); // rough EV pack mapping

  // Blend
  currentSOC = 0.9 * currentSOC + 0.1 * soc_est;

  currentSOC = constrain(currentSOC, 0.0, 1.0);
}

// ============================================================
// GET REAL TIME
// ============================================================
float getHour() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return 0;

  return timeinfo.tm_hour + timeinfo.tm_min / 60.0;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(9600);
  pinMode(MOSFET_PIN, OUTPUT);

  lcd.begin(20,4);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");

  buildSchedule();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  float hour = getHour();
  int step = hour / dt;

  bool shouldCharge = P_schedule[step] > 0.01;

  if (currentSOC < 0.1) {
    digitalWrite(MOSFET_PIN, HIGH);
  }
  else if (currentSOC >= 0.8) {
    digitalWrite(MOSFET_PIN, LOW);
  }
  else if (shouldCharge) {
    digitalWrite(MOSFET_PIN, HIGH);
  }
  else {
    digitalWrite(MOSFET_PIN, LOW);
  }

  updateSOC();

  // LCD
  lcd.setCursor(0,0);
  lcd.print("Time:");
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