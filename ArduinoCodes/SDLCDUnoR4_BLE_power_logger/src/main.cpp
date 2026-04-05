// Merged sketch: SDLCDUnoR4 + Uno R4 WiFi Power Logger with BLE
// - Uses existing LCD + SD logging and kWh integration from SDLCDUnoR4.ino
// - Adds BLE peripheral and Serial CSV logging from uno_r4_wifi_power_logger.ino
//   (compatible with Expo app Data Logger tab)

#include <Arduino.h>
#include <cstring>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <SD.h>
#include <ArduinoBLE.h>

// -------------------- Data Vars --------------------
double BatVolt;
double BatCur;
double BatPower;
double KWh = 0.0;  // integrated battery energy (kWh)

double CharVolt;
double CharCur;
double CharPower;

int R1 = 11000;
int R2 = 1000;

// -------------------- LCD Pins --------------------
const int rs = 3, en = 4, d4 = 5, d5 = 6, d6 = 7, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// -------------------- SD --------------------
const int SD_CS_PIN = 10;
File logFile;
char logFilename[13] = "LOG000.CSV";
bool sdAvailable = false;

// -------------------- Timing --------------------
const unsigned long LOG_INTERVAL_MS = 500;
const unsigned long LCD_INTERVAL_MS = 400;
const unsigned long SERIAL_INTERVAL_MS = 100;  // BLE + Serial update rate

unsigned long lastLogTime = 0;
unsigned long lastLcdTime = 0;
unsigned long lastEnergyTime = 0;
unsigned long lastSerialTime = 0;
unsigned long runStartMs = 0;
unsigned long startTime = 0;  // for Serial time_s

// -------------------- Spinner --------------------
const char spinnerChars[4] = {'|', '/', '-', '\\'};
byte spinnerIndex = 0;

// -------------------- BLE (from uno_r4_wifi_power_logger) --------------------
const char *BLE_NAME = "ESP32H2-2";  // keep same so app can connect as Device 4
const char *SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab";
const char *ANALOG_CHAR_UUID = "12345678-1234-1234-1234-1234567890af";

BLEService dataLoggerService(SERVICE_UUID);
// Store CSV "v100,i100,p100,wh100" in characteristic (as bytes)
BLECharacteristic analogChar(ANALOG_CHAR_UUID, BLERead, 48);

bool bleConnected = false;
unsigned long lastLedToggle = 0;
bool ledState = false;

// -------------------- Helpers --------------------
void padTo20(char *line) {
  int len = strlen(line);
  for (int i = len; i < 20; i++) line[i] = ' ';
  line[20] = '\0';
}

void printRow(byte row, const char *line20) {
  lcd.setCursor(0, row);
  lcd.print(line20);
}

// -------------------- Create Log File --------------------
bool createNewLogFile() {
  for (int i = 0; i <= 999; i++) {
    snprintf(logFilename, sizeof(logFilename), "LOG%03d.CSV", i);

    if (!SD.exists(logFilename)) {
      File f = SD.open(logFilename, FILE_WRITE);
      if (!f) return false;

      f.println("ms,run_s,dt_s,BatVolt_V,BatCur_A,BatPower_W,KWh,CharVolt_V,CharCur_A,CharPower_W");
      f.flush();
      f.close();
      return true;
    }
  }
  return false;
}

void logCsvRow(unsigned long nowMs, double run_s, double dt_s) {
  if (!sdAvailable) return;

  logFile = SD.open(logFilename, FILE_WRITE);
  if (!logFile) {
    sdAvailable = false;
    return;
  }

  logFile.print(nowMs);
  logFile.print(',');
  logFile.print(run_s, 3);
  logFile.print(',');
  logFile.print(dt_s, 3);
  logFile.print(',');

  logFile.print(BatVolt, 5);
  logFile.print(',');
  logFile.print(BatCur, 5);
  logFile.print(',');
  logFile.print(BatPower, 5);
  logFile.print(',');
  logFile.print(KWh, 8);
  logFile.print(',');

  logFile.print(CharVolt, 5);
  logFile.print(',');
  logFile.print(CharCur, 5);
  logFile.print(',');
  logFile.print(CharPower, 5);

  logFile.println();
  logFile.flush();
  logFile.close();
}

// -------------------- BLE Helpers --------------------
// Build CSV for BLE: "v100,i100,p100,wh100"
// voltage/current/power from battery channel, energy from KWh (converted to Wh)
void updateBleAnalogValue(double voltage, double current, double power, double kwh) {
  double Wh = kwh * 1000.0;  // convert kWh -> Wh

  int v100 = (int)(voltage * 100.0 + 0.5);
  int i100 = (int)(current * 100.0 + 0.5);
  int p100 = (int)(power * 100.0 + 0.5);
  int wh100 = (int)(Wh * 100.0 + 0.5);

  char buf[48];
  int len = snprintf(buf, sizeof(buf), "%d,%d,%d,%d", v100, i100, p100, wh100);
  if (len > 0 && len < (int)sizeof(buf)) {
    analogChar.writeValue((const uint8_t *)buf, (unsigned int)len);
  }
}

// LED: blink when disconnected, solid when connected (like ESP32-S3 status)
void updateStatusLed() {
  if (bleConnected) {
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  unsigned long now = millis();
  if (now - lastLedToggle >= 500) {
    lastLedToggle = now;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  lcd.begin(20, 4);
  lcd.noAutoscroll();
  lcd.noCursor();
  lcd.noBlink();

  // Use 14-bit ADC resolution on Uno R4 (0–16383)
  analogReadResolution(14);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  if (SD.begin(SD_CS_PIN) && createNewLogFile()) {
    sdAvailable = true;
  }

  runStartMs = millis();
  startTime = runStartMs;
  lastEnergyTime = runStartMs;
  lastLogTime = runStartMs;
  lastLcdTime = runStartMs;
  lastSerialTime = runStartMs;

  lcd.clear();

  // ----- BLE init -----
  if (!BLE.begin()) {
    Serial.println(F("BLE init failed. Check board."));
    // Fast blink LED to indicate BLE failure
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  BLE.setLocalName(BLE_NAME);
  BLE.setAdvertisedService(dataLoggerService);
  dataLoggerService.addCharacteristic(analogChar);
  BLE.addService(dataLoggerService);
  BLE.advertise();

  // Initial value (so first read returns something valid)
  const char *initValue = "0,0,0,0";
  analogChar.writeValue((const uint8_t *)initValue, (unsigned int)strlen(initValue));

  Serial.println(F("BLE advertising as ESP32H2-2. Connect from app Data Logger tab."));
  Serial.println(F("Time_s,BatVolt_V,BatCur_A,BatPower_W,Wh"));
}

// -------------------- Main Loop --------------------
void loop() {
  unsigned long now = millis();

  // -------- BLE connection state and status LED --------
  BLEDevice central = BLE.central();
  if (central) {
    bleConnected = central.connected();
  } else {
    bleConnected = false;
  }
  updateStatusLed();

  // -------- Read Sensors (same as SDLCDUnoR4.ino) --------
  BatVolt = analogRead(A0);
  BatVolt = BatVolt * 5.0 / 16383.0 * ((R1 + R2) / (double)R2);

  BatCur = analogRead(A1);
  BatCur = ((BatCur * 5.0 / 16383.0) - 2.49) / 0.066;

  BatPower = BatVolt * BatCur;

  CharVolt = analogRead(A2);
  CharVolt = CharVolt * 5.0 / 16383.0 * ((R1 + R2) / (double)R2);

  CharCur = analogRead(A3);
  CharCur = ((CharCur * 5.0 / 16383.0) - 2.49) / 0.4;

  CharPower = CharVolt * CharCur;

  // -------- Energy Integration (kWh) --------
  double dtEnergy = (now - lastEnergyTime) / 1000.0;  // seconds
  lastEnergyTime = now;
  KWh += BatPower * (dtEnergy / 3600000.0);           // W * s -> kWh

  // -------- SD Logging --------
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    unsigned long prev = lastLogTime;
    lastLogTime = now;

    double dtLog = (now - prev) / 1000.0;
    double run_s = (now - runStartMs) / 1000.0;

    logCsvRow(now, run_s, dtLog);
  }

  // -------- LCD Update (fixed-width safe) --------
  if (now - lastLcdTime >= LCD_INTERVAL_MS) {
    lastLcdTime = now;

    unsigned long runSeconds = (now - runStartMs) / 1000UL;
    unsigned int minutes = runSeconds / 60;
    unsigned int seconds = runSeconds % 60;
    if (minutes > 99) minutes = 99;

    char line0[21], line1[21], line2[21], line3[21];
    char sdRight[6];

    if (sdAvailable) {
      snprintf(sdRight, sizeof(sdRight), "L%c%c%c",
               logFilename[3], logFilename[4], logFilename[5]);
    } else {
      snprintf(sdRight, sizeof(sdRight), "NO-SD");
    }

    // Row 0
    snprintf(line0, sizeof(line0), "BAT:%6.2fV", BatVolt);
    padTo20(line0);
    memcpy(&line0[20 - strlen(sdRight)], sdRight, strlen(sdRight));
    printRow(0, line0);

    // Row 1
    snprintf(line1, sizeof(line1), "CUR:%6.2fA", BatCur);
    padTo20(line1);
    snprintf(&line1[15], 6, "%02u:%02u", minutes, seconds);
    printRow(1, line1);

    // Row 2
    snprintf(line2, sizeof(line2), "PWR:%7.1fW", BatPower);
    padTo20(line2);
    printRow(2, line2);

    // Row 3
    snprintf(line3, sizeof(line3), "kWh:%9.4f", KWh);
    padTo20(line3);
    line3[19] = spinnerChars[spinnerIndex];
    spinnerIndex = (spinnerIndex + 1) % 4;
    printRow(3, line3);
  }

  // -------- BLE + Serial CSV logging (battery channel only) --------
  if (now - lastSerialTime >= SERIAL_INTERVAL_MS) {
    lastSerialTime = now;

    double timeSec = (now - startTime) / 1000.0;
    double Wh = KWh * 1000.0;

    // Update BLE characteristic for app ("v100,i100,p100,wh100")
    updateBleAnalogValue(BatVolt, BatCur, BatPower, KWh);

    // Serial CSV: Time_s,BatVolt_V,BatCur_A,BatPower_W,Wh
    Serial.print(timeSec, 3);
    Serial.print(',');
    Serial.print(BatVolt, 2);
    Serial.print(',');
    Serial.print(BatCur, 2);
    Serial.print(',');
    Serial.print(BatPower, 2);
    Serial.print(',');
    Serial.println(Wh, 4);
  }
}
