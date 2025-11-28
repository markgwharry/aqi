#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_BME680.h>

// -------------------- Pins --------------------
const int OLED_SDA = 20;
const int OLED_SCL = 21;

const int BME_SDA  = 3;
const int BME_SCL  = 4;

const int PMS_RX_PIN = 6;  // ESP32-C3 RX1  <- PMSA003 TX
const int PMS_TX_PIN = 7;  // ESP32-C3 TX1  -> PMSA003 RX (optional)

// -------------------- I2C buses ----------------
TwoWire WireBME = TwoWire(1);   // second I2C bus for BME688

// -------------------- OLED (SH1106 via U8g2) ---
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0,
  U8X8_PIN_NONE
);

// -------------------- BME688 -------------------
Adafruit_BME680 bme;   // BME688 works with this driver
bool bmePresent = false;

// -------------------- Measurements -------------
float pm1_0 = 0.0f;
float pm2_5 = 0.0f;
float pm10  = 0.0f;

float temperature = 0.0f;
float humidity    = 0.0f;
float gasOhms     = 0.0f;
float vocIndex    = 0.0f;   // crude gas-based index

unsigned long lastReadMs = 0;
const unsigned long SENSOR_INTERVAL_MS = 5000;  // 5 seconds

// -------------------- Air quality state --------
enum AirQualityState {
  AQ_GOOD,
  AQ_MODERATE,
  AQ_WARNING,
  AQ_DANGEROUS
};

AirQualityState currentState = AQ_GOOD;

// -------------------- Forward declarations -----
bool readPMSA003();
void readBME688();
float computeVocIndex(float gasResistanceOhms);
AirQualityState calculateAirQualityState();
void drawOledScreen(AirQualityState state, bool blinkWarning);

// =================================================
// setup()
// =================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C for OLED
  Wire.begin(OLED_SDA, OLED_SCL);

  // OLED init (U8g2)
  u8g2.begin();

  // I2C for BME688
  WireBME.begin(BME_SDA, BME_SCL);

  // BME688 init on WireBME
  if (!bme.begin(0x76, &WireBME)) {   // change to 0x77 if needed
    Serial.println(F("BME688 not found at 0x76"));
    bmePresent = false;
  } else {
    bmePresent = true;
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);  // 320°C for 150 ms
  }

  // PMSA003 UART
  Serial1.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);

  // Splash
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "Garage AQ monitor");
  u8g2.drawStr(0, 28, "OLED+BME688+PMSA003");
  u8g2.sendBuffer();
  delay(800);
}

// =================================================
// loop()
// =================================================
void loop() {
  unsigned long now = millis();

  if (now - lastReadMs >= SENSOR_INTERVAL_MS) {
    lastReadMs = now;

    // 1) Read PM sensor
    bool pmOk = readPMSA003();
    if (!pmOk) {
      Serial.println(F("PMSA003: no valid frame"));
    }

    // 2) Read BME688
    readBME688();

    // 3) Classify air quality
    currentState = calculateAirQualityState();

    // 4) Blink banner in warning/danger
    bool blinkWarning = false;
    if (currentState >= AQ_WARNING) {
      blinkWarning = ((now / 500) % 2 == 0);
    }

    // 5) Update OLED
    drawOledScreen(currentState, blinkWarning);

    // Debug print
    Serial.print(F("PM2.5=")); Serial.print(pm2_5);
    Serial.print(F(" ug/m3, VOC idx=")); Serial.print(vocIndex);
    Serial.print(F(", T=")); Serial.print(temperature);
    Serial.print(F("C, RH=")); Serial.print(humidity);
    Serial.print(F("%, state=")); Serial.println((int)currentState);
  }

  delay(10);
}

// =================================================
// PMSA003 reading
// =================================================
bool readPMSA003() {
  const int FRAME_LEN = 32;

  if (Serial1.available() < FRAME_LEN) {
    return false;
  }

  uint8_t buffer[FRAME_LEN];

  // Sync to 0x42 0x4D header
  while (Serial1.available() >= FRAME_LEN) {
    if (Serial1.peek() == 0x42) {
      Serial1.readBytes(buffer, FRAME_LEN);
      if (buffer[0] == 0x42 && buffer[1] == 0x4D) {
        // Checksum
        uint16_t sum = 0;
        for (int i = 0; i < FRAME_LEN - 2; i++) {
          sum += buffer[i];
        }
        uint16_t checksum = ((uint16_t)buffer[FRAME_LEN - 2] << 8) | buffer[FRAME_LEN - 1];
        if (sum == checksum) {
          pm1_0 = ((uint16_t)buffer[10] << 8) | buffer[11];
          pm2_5 = ((uint16_t)buffer[12] << 8) | buffer[13];
          pm10  = ((uint16_t)buffer[14] << 8) | buffer[15];
          return true;
        } else {
          return false; // bad checksum
        }
      }
    } else {
      Serial1.read();  // discard one byte and resync
    }
  }
  return false;
}

// =================================================
// BME688 reading
// =================================================
void readBME688() {
  if (!bmePresent) return;

  if (!bme.performReading()) {
    Serial.println(F("BME688 reading failed"));
    return;
  }

  temperature = bme.temperature;
  humidity    = bme.humidity;
  gasOhms     = bme.gas_resistance;

// 🔍 Print the raw gas resistance
Serial.print("Gas resistance: ");
Serial.print(gasOhms);
Serial.println(" ohms");

  vocIndex = computeVocIndex(gasOhms);
}

// Map gas resistance to a crude VOC index (0 good → 500 bad)
float computeVocIndex(float gasResistanceOhms) {
  // Calibrated using your baseline of ~42kΩ
  const float CLEAN_AIR_GAS = 42000.0f;   // your room's normal level
  const float DIRTY_AIR_GAS = 8000.0f;    // typical VOC-heavy air (IPA, resin)
  
  float g = gasResistanceOhms;

  // Clamp values
  if (g > CLEAN_AIR_GAS) g = CLEAN_AIR_GAS;
  if (g < DIRTY_AIR_GAS) g = DIRTY_AIR_GAS;

  // Map clean -> 0, dirty -> 500
  float norm = (CLEAN_AIR_GAS - g) / (CLEAN_AIR_GAS - DIRTY_AIR_GAS);
  float idx = norm * 500.0f;

  return idx;
}



// =================================================
// Air quality classification
// =================================================
AirQualityState calculateAirQualityState() {
  int pmScore  = 0;
  int vocScore = 0;

  // PM2.5 thresholds (ug/m3)
  if      (pm2_5 < 12)  pmScore = 0;
  else if (pm2_5 < 35)  pmScore = 1;
  else if (pm2_5 < 55)  pmScore = 2;
  else                  pmScore = 3;

  // VOC index thresholds
  if      (vocIndex < 100) vocScore = 0;
  else if (vocIndex < 200) vocScore = 1;
  else if (vocIndex < 350) vocScore = 2;
  else                     vocScore = 3;

  int overall = max(pmScore, vocScore);

  switch (overall) {
    case 0: return AQ_GOOD;
    case 1: return AQ_MODERATE;
    case 2: return AQ_WARNING;
    default: return AQ_DANGEROUS;
  }
}

// =================================================
// OLED drawing via U8g2
// =================================================
void drawOledScreen(AirQualityState state, bool blinkWarning) {
  u8g2.clearBuffer();

  const char* statusText;
  switch (state) {
    case AQ_GOOD:      statusText = "GOOD";   break;
    case AQ_MODERATE:  statusText = "OK";     break;
    case AQ_WARNING:   statusText = "WARN";   break;
    case AQ_DANGEROUS: statusText = "DANGER"; break;
    default:           statusText = "???";    break;
  }

  // Top banner: in WARN/DANGER we can blink by hiding text every other frame
  if (!(state >= AQ_WARNING && blinkWarning)) {
    u8g2.setFont(u8g2_font_helvB18_tf);
    u8g2.drawStr(0, 22, statusText);
  }

  u8g2.setFont(u8g2_font_6x12_tf);

  char line[32];
  snprintf(line, sizeof(line), "PM2.5: %.1f ug/m3", pm2_5);
  u8g2.drawStr(0, 36, line);

  snprintf(line, sizeof(line), "VOC idx: %.0f", vocIndex);
  u8g2.drawStr(0, 48, line);

  snprintf(line, sizeof(line), "T: %.1fC  RH: %.0f%%", temperature, humidity);
  u8g2.drawStr(0, 60, line);

  u8g2.sendBuffer();
}
