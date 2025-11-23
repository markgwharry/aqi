#include <SPI.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold12pt7b.h>

// --- E-paper pin definitions ---
static const int EPD_CS   = 7;
static const int EPD_DC   = 0;
static const int EPD_RST  = 5;
static const int EPD_BUSY = 10;

// Use the panel type that exists in your GxEPD2 version:
// GxEPD2_213_GDEY0213B74
GxEPD2_BW<GxEPD2_213_GDEY0213B74, GxEPD2_213_GDEY0213B74::HEIGHT> display(
  GxEPD2_213_GDEY0213B74(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY)
);

// --- Soil sensor ---
const int MOIST_PIN = 1;

// TEMP calibration – we’ll tune these later
int DRY_VALUE = 3800;
int WET_VALUE = 1700;

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 60000; // 60 s

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("Plant monitor debug sketch starting...");

  pinMode(MOIST_PIN, INPUT);

  // SPI on your chosen pins: SCLK=4, MOSI=6, MISO unused
  SPI.begin(4, -1, 6, -1);

  display.init(115200);   // you can lower this if needed
  display.setRotation(1);
  display.setFont(&FreeMonoBold12pt7b);
  display.setTextColor(GxEPD_BLACK);

  drawScreen(0, 0);  // initial screen
}

void loop() {
  int raw = analogRead(MOIST_PIN);
  int moisture = map(raw, DRY_VALUE, WET_VALUE, 0, 100);
  moisture = constrain(moisture, 0, 100);

  Serial.print("Raw ADC: ");
  Serial.print(raw);
  Serial.print("  | Moisture: ");
  Serial.print(moisture);
  Serial.println("%");

  unsigned long now = millis();
  if (now - lastDisplayUpdate > DISPLAY_INTERVAL) {
    drawScreen(moisture, raw);
    lastDisplayUpdate = now;
  }

  delay(1000);
}

void drawScreen(int moisture, int raw) {
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);

    display.setCursor(5, 30);
    display.print("Moisture:");

    display.setCursor(5, 70);
    display.print(moisture);
    display.print("%");

    display.setCursor(5, 110);
    display.print("Raw:");
    display.print(raw);
  } while (display.nextPage());
}
