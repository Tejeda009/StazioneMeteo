//codice per gestire la stazione meteo e irrigazione 'piante'
//controlla temperatura, umidita, CO2, luminosita, livello d'acqua, orario e irriga con un servo
//include anche un buzzer per avvisi acustici
//ha due display, uno principale per le letture e uno secondario per lo stato
//ha due pulsanti, uno per cambiare modalita e uno per attivare l'irrigazione

//librerie
//!!!!!!!!!LEGGETE LE DOCUMENTAZIONI DELLE LIBRERIE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <Adafruit_SGP30.h>
#include <BH1750.h>
#include <virtuabotixRTC.h>
#include <Servo.h>
#include <EEPROM.h

//struct (perchè il cavolo di sgp30 non accetta baseline più vecchie di 7 giorni)
struct baseline{
  uuint16_t eeprom_eco2;
  uuint16_t eeprom_tvoc;
  uuint32_t timestamp;
}

// ---PIN ---
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

// --- CONFIGURAZIONI SENSORI ---
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);
Adafruit_SGP30 sgp;
BH1750 lightMeter;
virtuabotixRTC myRTC(PIN_RTC_CLK, PIN_RTC_DAT, PIN_RTC_RST);
Servo irrigationServo;

// --- SCHERMI ---
// Usa buffer di paginazione (_1_) per SH1106 (1.3") e SSD1306 (0.91") per salvare SRAM
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2_main(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2_status(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// --- VARIABILI GLOBALI ---
float temp_dht, hum_dht, temp_lm35, hi;
uint16_t tvoc, eco2, lux;
int water_level;
bool isAutoMode = true;
bool isIrrigating = false;
unsigned long lastUpdate = 0;
unsigned long eeprom_lastUpdate = 0;
int eeprom_addr = 0;
float kalman_temp = 0;

//----kalman----
float Q = 0.022;  // Incertezza del processo (quanto pensi che cambi la temp velocemente)    //-----------indicazioni su come usarlo--------
float R = 0.5;    // Incertezza della misura (rumore dei sensori)                           if troppo lento a cambiare, ++Q o --R
float P = 1.0;    // Errore stimato iniziale                                                if valore oscilla ++R --Q
float K = 0;      // Guadagno di Kalman
float temp_guess = 25.0; // Valore iniziale della temperatura stim
//pesi per kalman
const float PESO_DHT = 0.3;
const float PESO_LM = 0.7; // per le altri costanti guarda calculate_temp()

// --- SOGLIE ---
const float TEMP_HIGH = 35.0;
const uint16_t ECO2_HIGH = 1000;
const int WATER_LOW = 300; // soglia per livello d'acqua
const long interval = 2000; 
const long eeprom_interval = 60 * 60 * 60 * 60; //1 ora
const float alfa = 0.1;
const uuint32_t settegiorni = 7 * 24 * 60 *60;

void setup() {
  Serial.begin(9600);
  Serial.println(F("Avvio....(Speriamo che funzioni)"));

  sensors_setup();

  // test del buzzer
  tone(PIN_BUZZER, 2000, 100);

  //lettura iniziale dei sensori per inizializzazione media mobile
  readSensors();
}

void sensors_setup(){
  //configurazione dei vari pin
  pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_BTN_ACT, INPUT_PULLUP);
  pinMode(PIN_LED_OK, OUTPUT);
  pinMode(PIN_LED_WARN, OUTPUT);
  pinMode(PIN_LED_ALARM, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  //start dei sensori con eventuali controlli
  if(!dht.begin()) Serial.println(F("DHT11 non trovato"));
  
  Wire.begin();
  if(!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) Serial.println(F("BH1750 non trovato"));

  setup_sgp30();

  //servo
  irrigationServo.attach(PIN_SERVO);
  irrigationServo.write(90); //posizione neutra del servo

  u8g2_main.begin();
  u8g2_status.begin();
}

void setup_sgp30() {
  if(!sgp.begin()) {
    Serial.println(F("Sgp30 non trovato!"));
  }
  //provo a forzare un soft reset
  if(!sgp.softReset()) {
    Serial.println(F("Soft reset fallito"));
  }
  else{
    sgp.IAQinit(); // se funziona allora lo faccio ripartire
  }


  //lettura valori per sgp30 da eeprom per calibrazione e settaggio baseline per sgp30
  baseline eeprom_baseline;
  EEPROM.get(eeprom_addr, eeprom_baseline);
  //primo controllo validità
  if(!isnan(eeprom_baseline.eeprom_tvoc) && !isnan(eeprom_baseline.eprom_eco2) && !isnan(eeprom_baseline.timestamp)){
    DateTime now = rtc.now();
    uuint32_t tillnow = now.unixTime() - eeprom_baseline.timstamp;
    if(tillnow < settegiorni){
      sgp.setIAQbaseline(eeprom_eco2, eeprom_tvoc); //settaggi con controllo
    }
    else{
      Serial.println(F("Baselina trovata ma scaduta"));
    }
  }
}

void loop() {
  handleButtons();

  //logica per aggiornare i valori
  if (millis() - lastUpdate >= interval) {
    readSensors();
    processLogic();
    updateDisplays();
    logSerial();
    lastUpdate = millis();
  }
  if (millis() - eeprom_lastUpdate >= eeprom_interval){
    save_baseline();
    eeprom_lastUpdate = millis();
  }
}

void save_baseline() {
  baseline tosave;
  if(sgp.getIAQbaseline(&tosave.eeprom_eco2, &tosave.eeprom_tvoc)){
    tosave.timestamp = rtc.now().unixTime();
    EEPROM.update(eeprom_addr, tosave); //update modifica solo se sono diversi (max 100.000 cicli r/w per Eeprom)
    Serial.println(F("Salvataggio baseline riuscito"));
  }
  else Serial.println(F("Salvataggio baseline non riuscito"));
}

void handleButtons() {
  //gestione dei pulsanti
  static bool lastModeState = HIGH;
  bool modeState = digitalRead(PIN_BTN_MODE);
  
  //scelta modalità automatica o manuale
  if (modeState == LOW && lastModeState == HIGH) {
    isAutoMode = !isAutoMode;
    delay(200); 
  }
  lastModeState = modeState;

  //logica irrigazione manuale
  if (!isAutoMode) {
    if (digitalRead(PIN_BTN_ACT) == LOW) {
      startIrrigation();
    } else {
      stopIrrigation();
    }
  }
}

//media mobile
void mme(float raw, float& filtered){
  if(raw)
    filtered = (alfa * raw) + ((1 - alfa) * filtered);
  else filtered = raw; 
}

uuint32_t abs_hum(float temp, float hum) {
  //calcolo umidità assoluta (g/cm3 e non %) per il cavolo di sgp30
  float abs_hum = (6.112 * pow(2.71828, (17.67 * ktemp) / (ktemp + 243.5)) * hum * 2.1674) / (273.15 + ktemp);
  return (uint32_t)(1000 * absHum); //perchè la vuole in mg/m3 per qualche motivo assurdo
}

void readSensors() {
  //tentativo lettura valori grezzi
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  //calcolo con media mobile esponenziale e controlli di validità
  if(!isnan(t)){
    mme(t,temp_dht);
  }
  if(!isnan(h)){
    mme(h,hum_dht);
  }

  hi = dht.computeHeatIndex(temp_dht, hum_dht, false); //calcolo indice di calore con valori già filtrati, (isFahreheit = false)
  
  // calcolo della temperatura analogico
  float raw = analogRead(PIN_LM35);

  if(!isnan(raw)) {
    raw = (raw * 5.0 * 100.0) / 1024.0;
    mme(raw,temp_lm35);
  }

  //unione valori della temperatura con kalman
  kalman_temp = calculate_temp(temp_dht,temp_lm35);

  //settaggio umidità per sgp30
  if(!sgp.setHumidity(abs_hum(kalman_temp, hum_dht))) Serial.prinln(F("All'sgp30 non garba la tua umidità"));

  //calcolo TVOC e eCO2
  if (sgp.IAQmeasure()) {
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
  }

  //update luce, livello d'acqua e orario
  secure_ligthRead();
  water_level = analogRead(PIN_WATER);
  myRTC.updateTime();
}

void secure_ligthRead() {
  if (lightMeter.measurementReady()) {
    lux = lightMeter.readLightLevel();

    //taken from the bh1750.h docs
    if (lux < 0) {
      Serial.println(F("Error condition detected"));
    } else {
      if (lux > 40000.0) {
        // reduce measurement time - needed in direct sun light
        if (lightMeter.setMTreg(32)) {
          Serial.println(
              F("Setting MTReg to low value for high light environment"));
        } else {
          Serial.println(
              F("Error setting MTReg to low value for high light environment"));
        }
      } else {
        if (lux > 10.0) {
          // typical light environment
          if (lightMeter.setMTreg(69)) {
            Serial.println(F(
                "Setting MTReg to default value for normal light environment"));
          } else {
            Serial.println(F("Error setting MTReg to default value for normal "
                             "light environment"));
          }
        } else {
          if (lux <= 10.0) {
            // very low light environment
            if (lightMeter.setMTreg(138)) {
              Serial.println(
                  F("Setting MTReg to high value for low light environment"));
            } else {
              Serial.println(F("Error setting MTReg to high value for low "
                               "light environment"));
            }
          }
        }
      }
    }
  }
}

//calcolo della temperatura unendo i valori filtrati di dht11 e lm35
float calculate_temp(float dht, float lm35) {

  float sum = (lm35 * PESO_LM) + (dht * PESO_DHT);

  P = P + Q;

  // Fase di Aggiornamento (Update)
  K = P / (P + R);
  temp_guess = temp_guess + K * (sum - temp_guess);
  P = (1 - K) * P;

  return temp_guess;
}

void processLogic() {
  bool alarm = false;

  //gestione irrigazione automatica
  if (isAutoMode) {
    if (water_level < WATER_LOW) startIrrigation();
    else stopIrrigation();
  }

  //allarme per CO2 e temperatura
  if (kalman_temp > TEMP_HIGH || eco2 > ECO2_HIGH) {
    alarm = true;
    tone(PIN_BUZZER, 1000, 200);
  }
 
  //gestione led
  digitalWrite(PIN_LED_OK, (!alarm && isAutoMode) ? HIGH : LOW); //LED verde - OK
  digitalWrite(PIN_LED_WARN, !isAutoMode ? HIGH : LOW);         // LED giallo - Manuale
  digitalWrite(PIN_LED_ALARM, alarm ? HIGH : LOW); // LED rosso - Allarme
  if(isIrrigating) digitalWrite(PIN_BUZZER, HIGH); else digitalWrite(PIN_BUZZER, LOW);
}

//irrigazione
void startIrrigation() {
  isIrrigating = true;
  irrigationServo.write(180); 
}

//stop irrigazione
void stopIrrigation() {
  isIrrigating = false;
  irrigationServo.write(90); 
}

//gestione display con librerie e F() per risparmiare SRAM
void updateDisplays() {
  // 1.3" Display principale
  u8g2_main.firstPage();
  do {
    u8g2_main.setFont(u8g2_font_6x10_tf); 
    u8g2_main.setCursor(0, 10); //setta le coordinate da cui partire
    u8g2_main.print(F("Temp: ")); u8g2_main.print(F(kalman_temp)); u8g2_main.print(F(" C"));
    u8g2_main.setCursor(0, 22);
    u8g2_main.print(F("Hum:  ")); u8g2_main.print(F(hum_dht)); u8g2_main.print(F(" %"));
    u8g2_main.setCursor(0, 34);
    u8g2_main.print(F("CO2:  ")); u8g2_main.print(F(eco2)); u8g2_main.print(F(" ppm"));
    u8g2_main.setCursor(0, 46);
    u8g2_main.print(F("Lux:  ")); u8g2_main.print(F(lux));
    u8g2_main.setCursor(0, 58);
    u8g2_main.print(F("HI:  ")); u8g2_main.print(F(hi)); u8g2_main.print(F(" C"));
    u8g2_main.setCursor(0, 70);
    u8g2_main.print(myRTC.hours); u8g2_main.print(F(":")); 
    if(myRTC.minutes < 10) u8g2_main.print(F("0"));
    u8g2_main.print(F(myRTC.minute)s);
  } while (u8g2_main.nextPage());

  // 0.91" Display di stato
  u8g2_status.firstPage();
  do {
    u8g2_status.setFont(u8g2_font_6x10_tf);
    u8g2_status.setCursor(0, 10);
    u8g2_status.print(isAutoMode ? F("MODO: AUTO") : F("MODO: MANU"));
    u8g2_status.setCursor(0, 25);
    u8g2_status.print(isIrrigating ? F("IRRIGA: ON") : F("IRRIGA: OFF"));
  } while (u8g2_status.nextPage());
}

//stampa seriale
void logSerial() {
  Serial.print(F("T:")); Serial.print(kalman_temp);
  Serial.print(F(" H:")); Serial.print(hum_dht);
  Serial.print(F(" CO2:")); Serial.print(eco2);
  Serial.print(F(" Lux:")); Serial.print(lux);
  Serial.print(F(" HI:")); Serial.print(hi);
  Serial.print(F(" H2O:")); Serial.print(water_level);
  Serial.print(F(" Mode:")); Serial.println(isAutoMode ? F("A") : F("M")); 
}

