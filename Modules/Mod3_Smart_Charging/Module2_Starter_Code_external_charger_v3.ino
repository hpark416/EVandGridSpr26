#include <LiquidCrystal.h>

// -------------------------
// Measurements
// -------------------------
double BatVolt = 0.0;
double BatCur  = 0.0;
double BatPower = 0.0;

double CharVolt = 0.0;
double CharCur  = 0.0;
double CharPower = 0.0;

double totalWh = 0.0;

// -------------------------
// Hardware constants
// -------------------------
const double ADC_REF_V = 5.0;
const double ADC_MAX = 16383.0; // 14-bit ADC on Uno R4
const double ACS712_BAT_ZERO = 2.49;
const double ACS712_BAT_SENS = 0.066; // V/A
const double ACS712_CHG_ZERO = 2.49;
const double ACS712_CHG_SENS = 0.4;   // V/A

const double R1 = 11000.0;
const double R2 = 1000.0;
const double VOLT_DIV_GAIN = (R1 + R2) / R2;

// J1772 / control pins
const int CHARGE_ENABLE_PIN = 2;
const int PROX_PIN = A4;
const int PILOT_PIN = A5;

// -------------------- LCD Pins --------------------
const int rs = 3, en = 4, d4 = 5, d5 = 6, d6 = 7, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// -------------------------
// Charge scheduling settings
// -------------------------
const float Ebat_kWh = 10.0;   // usable battery capacity estimate
const float Pmax_kW  = 7.2;    // max external charging power estimate
const float eta      = 0.92;   // charging efficiency
const float SOC_req  = 0.80;   // stop target
const float SOC_res  = 0.10;   // emergency minimum
const float BASE     = 0.12;   // $/kWh base rate
const float SLOT_HR  = 0.5;    // 30-minute schedule slots
const int   N_SLOTS  = 48;

float START_TIME = 17.5; // wall-clock start time in hours
float SOC0       = 0.20; // initial SOC estimate
float ARRIVAL    = 17.5; // plug-in time
float DEPARTURE  = 7.0;  // unplug time

float P_schedule[N_SLOTS];
float rate[N_SLOTS];
float currentSOC = SOC0;
bool configured = false;
bool scheduleEnabled = true;

// -------------------------
// Timing / state
// -------------------------
unsigned long startMillis = 0;
unsigned long lastUpdateMs = 0;
bool chargeRequestLatched = false;

// -------------------------
// Helper functions
// -------------------------
double readScaledVoltage(int pin) {
  double raw = analogRead(pin);
  return raw * ADC_REF_V / ADC_MAX * VOLT_DIV_GAIN;
}

double readScaledCurrent(int pin, double zeroOffset, double sensitivity) {
  double raw = analogRead(pin);
  double sensedV = raw * ADC_REF_V / ADC_MAX;
  return (sensedV - zeroOffset) / sensitivity;
}

double readPilotHighVoltage() {
  double pHigh = 0.0;
  for (int i = 0; i < 100; i++) {
    double reading = analogRead(PILOT_PIN);
    if (reading > pHigh) pHigh = reading;
  }
  return pHigh * 12.0 / ADC_MAX;
}

double readProximityVoltage() {
  double raw = analogRead(PROX_PIN);
  return raw * ADC_REF_V / ADC_MAX;
}

float getRate(float h) {
  if (h >= 23 || h < 6) return 0.65f * BASE;
  if (h >= 17 && h < 21) return 1.60f * BASE;
  return 0.90f * BASE;
}

float getCurrentHour() {
  float elapsed = (millis() - startMillis) / 3600000.0f;
  float h = START_TIME + elapsed;
  while (h >= 24.0f) h -= 24.0f;
  while (h < 0.0f) h += 24.0f;
  return h;
}

void buildSchedule() {
  for (int i = 0; i < N_SLOTS; i++) {
    float h = i * SLOT_HR;
    rate[i] = getRate(h);
    P_schedule[i] = 0.0f;
  }

  int a = (int)(ARRIVAL / SLOT_HR);
  int d = (int)(DEPARTURE / SLOT_HR);

  int idx[N_SLOTS];
  int len = 0;

  if (d < a) {
    for (int i = a; i < N_SLOTS; i++) idx[len++] = i;
    for (int i = 0; i <= d; i++) idx[len++] = i;
  } else {
    for (int i = a; i <= d; i++) idx[len++] = i;
  }

  float Ereq = max((SOC_req - SOC0) * Ebat_kWh, 0.0f);
  float Erem = Ereq;

  // cheapest slots first
  for (int i = 0; i < len - 1; i++) {
    for (int j = i + 1; j < len; j++) {
      if (rate[idx[j]] < rate[idx[i]]) {
        int tmp = idx[i];
        idx[i] = idx[j];
        idx[j] = tmp;
      }
    }
  }

  for (int i = 0; i < len && Erem > 0.0f; i++) {
    int k = idx[i];
    float Emax = Pmax_kW * SLOT_HR * eta;
    float Eadd = min(Emax, Erem);
    P_schedule[k] = Eadd / (SLOT_HR * eta); // commanded kW in this slot
    Erem -= Eadd;
  }

  currentSOC = SOC0;
}

void printSchedule() {
  Serial.println("Schedule rebuilt.");
  for (int i = 0; i < N_SLOTS; i++) {
    if (P_schedule[i] > 0.01f) {
      Serial.print("Slot ");
      Serial.print(i);
      Serial.print(" (");
      Serial.print(i * SLOT_HR, 1);
      Serial.print(" h): ");
      Serial.print(P_schedule[i], 2);
      Serial.println(" kW");
    }
  }
}

void checkSerial() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  if (s.startsWith("T=")) START_TIME = s.substring(2).toFloat();
  else if (s.startsWith("SOC=")) SOC0 = s.substring(4).toFloat();
  else if (s.startsWith("A=")) ARRIVAL = s.substring(2).toFloat();
  else if (s.startsWith("D=")) DEPARTURE = s.substring(2).toFloat();
  else if (s.equalsIgnoreCase("SCHED=ON")) scheduleEnabled = true;
  else if (s.equalsIgnoreCase("SCHED=OFF")) scheduleEnabled = false;
  else if (s.equalsIgnoreCase("REBUILD")) {
    buildSchedule();
    printSchedule();
  }

  Serial.println("Updated parameter.");
}

void printMeasurements(double pilotV, double proxV, bool chargeEnabled, bool shouldCharge, int step, float hour) {
  Serial.print("Battery Voltage = "); Serial.println(BatVolt, 3);
  Serial.print("Battery Current = "); Serial.println(BatCur, 3);
  Serial.print("Battery Power = ");   Serial.println(BatPower, 3);

  Serial.print("Charger Voltage = "); Serial.println(CharVolt, 3);
  Serial.print("Charger Current = "); Serial.println(CharCur, 3);
  Serial.print("Charger Power = ");   Serial.println(CharPower, 3);

  Serial.print("Pilot Voltage = ");   Serial.println(pilotV, 3);
  Serial.print("Proximity Voltage = "); Serial.println(proxV, 3);

  Serial.print("Hour = "); Serial.println(hour, 2);
  Serial.print("Slot = "); Serial.println(step);
  Serial.print("Scheduled Power (kW) = "); Serial.println(P_schedule[step], 3);
  Serial.print("Should Charge = "); Serial.println(shouldCharge ? "YES" : "NO");
  Serial.print("Charge Enabled = "); Serial.println(chargeEnabled ? "YES" : "NO");
  Serial.print("SOC = "); Serial.println(currentSOC, 4);
  Serial.print("Total Energy (Wh) = "); Serial.println(totalWh, 5);
  Serial.println();
}

void updateLCD(bool chargingEnabled, float hour, int step, bool shouldCharge) {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("CHG:");
  lcd.print(chargingEnabled ? "ON " : "OFF");
  lcd.print(" Hr:");
  lcd.print(hour, 1);

  lcd.setCursor(0, 1);
  lcd.print("SOC:");
  lcd.print(currentSOC * 100.0, 0);
  lcd.print("% P:");
  lcd.print(CharPower, 0);

  lcd.setCursor(0, 2);
  lcd.print("Slot:");
  lcd.print(step);
  lcd.print(" Cmd:");
  lcd.print(P_schedule[step], 1);

  lcd.setCursor(0, 3);
  lcd.print(shouldCharge ? "Sched:YES " : "Sched:NO  ");
  lcd.print(totalWh, 1);
  lcd.print("Wh");
}

void setup() {
  Serial.begin(9600);
  analogReadResolution(14);

  pinMode(CHARGE_ENABLE_PIN, OUTPUT);
  digitalWrite(CHARGE_ENABLE_PIN, LOW);

  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("External charger");
  lcd.setCursor(0, 1);
  lcd.print("Send config/START");

  Serial.println("Enter optional config:");
  Serial.println("T=17.5  SOC=0.2  A=17.5  D=7");
  Serial.println("SCHED=ON or SCHED=OFF");
  Serial.println("Then type START");

  while (!configured) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.startsWith("T=")) START_TIME = s.substring(2).toFloat();
      else if (s.startsWith("SOC=")) SOC0 = s.substring(4).toFloat();
      else if (s.startsWith("A=")) ARRIVAL = s.substring(2).toFloat();
      else if (s.startsWith("D=")) DEPARTURE = s.substring(2).toFloat();
      else if (s.equalsIgnoreCase("SCHED=ON")) scheduleEnabled = true;
      else if (s.equalsIgnoreCase("SCHED=OFF")) scheduleEnabled = false;
      else if (s.equalsIgnoreCase("START")) configured = true;
    }
  }

  buildSchedule();
  printSchedule();

  startMillis = millis();
  lastUpdateMs = startMillis;

  lcd.clear();
}

void loop() {
  checkSerial();

  // Measurements
  BatVolt = readScaledVoltage(A0);
  BatCur = readScaledCurrent(A1, ACS712_BAT_ZERO, ACS712_BAT_SENS);
  BatPower = BatVolt * BatCur;

  CharVolt = readScaledVoltage(A2);
  CharCur = readScaledCurrent(A3, ACS712_CHG_ZERO, ACS712_CHG_SENS);
  CharPower = CharVolt * CharCur;

  double pilotV = readPilotHighVoltage();
  double proxV = readProximityVoltage();

  bool connectorPresent = (proxV >= 1.40 && proxV < 2.00);
  bool evseReady = (pilotV >= 7.5);
  bool chargingState = (pilotV >= 5.0 && pilotV < 7.5);

  float hour = getCurrentHour();
  int step = (int)(hour / SLOT_HR);
  if (step < 0) step = 0;
  if (step >= N_SLOTS) step = N_SLOTS - 1;

  bool shouldCharge = (!scheduleEnabled) || (P_schedule[step] > 0.01f);

  // Request logic: respect J1772 connection state, but also incorporate
  // scheduled charging and SOC limits from the other code.
  bool wantCharge = false;
  if (currentSOC < SOC_res) {
    wantCharge = true;                  // emergency override
  } else if (currentSOC >= SOC_req) {
    wantCharge = false;                 // target reached
  } else if (shouldCharge) {
    wantCharge = true;                  // scheduled charging slot
  }

  if (!connectorPresent) {
    chargeRequestLatched = false;
  } else if (!wantCharge) {
    chargeRequestLatched = false;
  } else if (evseReady || chargingState) {
    chargeRequestLatched = true;
  }

  digitalWrite(CHARGE_ENABLE_PIN, chargeRequestLatched ? HIGH : LOW);
  bool chargeEnabled = chargeRequestLatched;

  // Integrate energy and update estimated SOC from measured charger power
  unsigned long now = millis();
  double dtHours = (now - lastUpdateMs) / 3600000.0;
  lastUpdateMs = now;

  if (chargeEnabled && CharPower > 0.0) {
    totalWh += CharPower * dtHours;
    currentSOC += (CharPower * dtHours) / (Ebat_kWh * 1000.0);
    if (currentSOC > 1.0f) currentSOC = 1.0f;
  }

  // Status text
  if (!connectorPresent) {
    Serial.println("Status: connector not connected");
  } else if (!wantCharge) {
    Serial.println("Status: connected, not requesting charge");
  } else if (evseReady) {
    Serial.println("Status: requesting external charge");
  } else if (chargingState) {
    Serial.println("Status: charging");
  } else {
    Serial.println("Status: connected, waiting");
  }

  printMeasurements(pilotV, proxV, chargeEnabled, shouldCharge, step, hour);
  updateLCD(chargeEnabled, hour, step, shouldCharge);

  delay(500);
}
