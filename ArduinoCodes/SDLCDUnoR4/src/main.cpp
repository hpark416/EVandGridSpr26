#include <Arduino.h>
#include <cstring>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <SD.h>

// -------------------- Data Vars --------------------
double BatVolt;
double BatCur;
double BatPower;
double KWh = 0.0;

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

unsigned long lastLogTime = 0;
unsigned long lastLcdTime = 0;
unsigned long lastEnergyTime = 0;
unsigned long runStartMs = 0;

// -------------------- Spinner --------------------
const char spinnerChars[4] = {'|', '/', '-', '\\'};
byte spinnerIndex = 0;

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

  logFile.print(nowMs); logFile.print(',');
  logFile.print(run_s, 3); logFile.print(',');
  logFile.print(dt_s, 3); logFile.print(',');

  logFile.print(BatVolt, 5);  logFile.print(',');
  logFile.print(BatCur, 5);   logFile.print(',');
  logFile.print(BatPower, 5); logFile.print(',');
  logFile.print(KWh, 8);      logFile.print(',');

  logFile.print(CharVolt, 5); logFile.print(',');
  logFile.print(CharCur, 5);  logFile.print(',');
  logFile.print(CharPower, 5);

  logFile.println();
  logFile.flush();
  logFile.close();
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  lcd.begin(20, 4);
  lcd.noAutoscroll();
  lcd.noCursor();
  lcd.noBlink();

  analogReadResolution(14);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  if (SD.begin(SD_CS_PIN) && createNewLogFile()) {
    sdAvailable = true;
  }

  runStartMs = millis();
  lastEnergyTime = runStartMs;
  lastLogTime = runStartMs;
  lastLcdTime = runStartMs;

  lcd.clear();
}

// -------------------- Main Loop --------------------
void loop() {
  unsigned long now = millis();

  // -------- Read Sensors --------
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

  // -------- Energy Integration --------
  double dtEnergy = (now - lastEnergyTime) / 1000.0;
  lastEnergyTime = now;
  KWh += BatPower * (dtEnergy / 3600000.0);

  // -------- SD Logging --------
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    unsigned long prev = lastLogTime;
    lastLogTime = now;

    double dtLog = (now - prev) / 1000.0;
    double run_s = (now - runStartMs) / 1000.0;

    logCsvRow(now, run_s, dtLog);
  }

  // -------- LCD Update (10 Hz, fixed-width safe) --------
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
}
