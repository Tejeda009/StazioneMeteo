#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <Adafruit_SGP30.h>
#include <BH1750.h>
#include <virtuabotixRTC.h>
#include <Servo.h>

// --- PIN DEFINITIONS ---
#define PIN_DHT         2
#define PIN_BTN_MODE    3
#define PIN_BTN_ACT     4
#define PIN_LED_OK      5
#define PIN_LED_WARN    6
#define PIN_LED_ALARM   7
#define PIN_BUZZER      8
#define PIN_SERVO       9
#define PIN_RTC_CLK     10
#define PIN_RTC_DAT     11
#define PIN_RTC_RST     12
#define PIN_LM35        A0
#define PIN_WATER       A1

// --- SENSOR CONFIG ---
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);
Adafruit_SGP30 sgp;
BH1750 lightMeter;
virtuabotixRTC myRTC(PIN_RTC_CLK, PIN_RTC_DAT, PIN_RTC_RST);
Servo irrigationServo;

// --- DISPLAYS ---
// Use Page Buffer (_1_) for SH1106 (1.3") and SSD1306 (0.91") to save SRAM
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2_main(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2_status(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// --- GLOBAL VARIABLES ---
float temp_dht, hum_dht, temp_lm35;
uint16_t tvoc, eco2, lux;
int water_level;
bool isAutoMode = true;
bool isIrrigating = false;
unsigned long lastUpdate = 0;
const long interval = 2000; 

// --- THRESHOLDS ---
const float TEMP_HIGH = 35.0;
const uint16_t ECO2_HIGH = 1000;
const int WATER_LOW = 300; // Threshold for soil moisture

void setup() {
  Serial.begin(9600);
  Serial.println(F("Avvio....(Speriamo che funzioni)"));

  pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_BTN_ACT, INPUT_PULLUP);
  pinMode(PIN_LED_OK, OUTPUT);
  pinMode(PIN_LED_WARN, OUTPUT);
  pinMode(PIN_LED_ALARM, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  dht.begin();
  sgp.begin();
  Wire.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  
  irrigationServo.attach(PIN_SERVO);
  irrigationServo.write(90); // Neutral/Stop for 360 servo

  u8g2_main.begin();
  u8g2_status.begin();

  // Test buzzer
  tone(PIN_BUZZER, 2000, 100);
}

void loop() {
  handleButtons();

  if (millis() - lastUpdate >= interval) {
    readSensors();
    processLogic();
    updateDisplays();
    logSerial();
    lastUpdate = millis();
  }
}

void handleButtons() {
  static bool lastModeState = HIGH;
  bool modeState = digitalRead(PIN_BTN_MODE);
  
  if (modeState == LOW && lastModeState == HIGH) {
    isAutoMode = !isAutoMode;
    delay(200); 
  }
  lastModeState = modeState;

  if (!isAutoMode) {
    if (digitalRead(PIN_BTN_ACT) == LOW) {
      startIrrigation();
    } else {
      stopIrrigation();
    }
  }
}

void readSensors() {
  temp_dht = dht.readTemperature();
  hum_dht = dht.readHumidity();
  
  // LM35 conversion (10mV per degree)
  temp_lm35 = (analogRead(PIN_LM35) * 5.0 * 100.0) / 1024.0;

  if (sgp.IAQmeasure()) {
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
  }

  lux = lightMeter.readLightLevel();
  water_level = analogRead(PIN_WATER);
  myRTC.updateTime();
}

void processLogic() {
  bool alarm = false;

  if (isAutoMode) {
    if (water_level < WATER_LOW) startIrrigation();
    else stopIrrigation();
  }

  if (temp_lm35 > TEMP_HIGH || eco2 > ECO2_HIGH) {
    alarm = true;
    tone(PIN_BUZZER, 1000, 200);
  }

  digitalWrite(PIN_LED_OK, (!alarm && isAutoMode) ? HIGH : LOW);
  digitalWrite(PIN_LED_WARN, !isAutoMode ? HIGH : LOW);
  digitalWrite(PIN_LED_ALARM, (alarm || isIrrigating) ? HIGH : LOW);
}

void startIrrigation() {
  isIrrigating = true;
  irrigationServo.write(180); 
}

void stopIrrigation() {
  isIrrigating = false;
  irrigationServo.write(90); 
}

void updateDisplays() {
  // 1.3" Main Display
  u8g2_main.firstPage();
  do {
    u8g2_main.setFont(u8g2_font_6x10_tf);
    u8g2_main.setCursor(0, 10);
    u8g2_main.print(F("Temp: ")); u8g2_main.print(temp_lm35); u8g2_main.print(F(" C"));
    u8g2_main.setCursor(0, 22);
    u8g2_main.print(F("Hum:  ")); u8g2_main.print(hum_dht); u8g2_main.print(F(" %"));
    u8g2_main.setCursor(0, 34);
    u8g2_main.print(F("CO2:  ")); u8g2_main.print(eco2); u8g2_main.print(F(" ppm"));
    u8g2_main.setCursor(0, 46);
    u8g2_main.print(F("Lux:  ")); u8g2_main.print(lux);
    u8g2_main.setCursor(0, 58);
    u8g2_main.print(myRTC.hours); u8g2_main.print(F(":")); 
    if(myRTC.minutes < 10) u8g2_main.print(F("0"));
    u8g2_main.print(myRTC.minutes);
  } while (u8g2_main.nextPage());

  // 0.91" Status Display
  u8g2_status.firstPage();
  do {
    u8g2_status.setFont(u8g2_font_6x10_tf);
    u8g2_status.setCursor(0, 10);
    u8g2_status.print(isAutoMode ? F("MODO: AUTO") : F("MODO: MANU"));
    u8g2_status.setCursor(0, 25);
    u8g2_status.print(isIrrigating ? F("IRRIGA: ON") : F("IRRIGA: OFF"));
  } while (u8g2_status.nextPage());
}

void logSerial() {
  Serial.print(F("T:")); Serial.print(temp_lm35);
  Serial.print(F(" H:")); Serial.print(hum_dht);
  Serial.print(F(" CO2:")); Serial.print(eco2);
  Serial.print(F(" Lux:")); Serial.print(lux);
  Serial.print(F(" H2O:")); Serial.print(water_level);
  Serial.print(F(" Mode:")); Serial.println(isAutoMode ? F("A") : F("M"));
}
