//codice per gestire la stazione meteo e irrigazione 'piante'
//controlla temperatura, umidita, CO2, luminosita, livello d'acqua, orario e irriga con un servo
//include anche un buzzer per avvisi acustici
//ha due display, uno principale per le letture e uno secondario per lo stato
//ha due pulsanti, uno per cambiare modalita e uno per attivare l'irrigazione

//task
#define _TASK_TIMECRITICAL       // Enable monitoring scheduling overruns
#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_SELF_DISTRUCT
#define _TASK_MICRO_RES
#define _TASK_STATUS_REQUEST
#define _TASK_PRIORITY

//librerie
//!!!!!!!!!LEGGETE LE DOCUMENTAZIONI DELLE LIBRERIE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <Adafruit_SGP30.h>
#include <BH1750.h>
#include <Servo.h>
#include <EEPROM.h>
#include <TaskScheduler.h> //scheduler per ottimizzazione task

//segnali
Scheduler runner;
StatusRequest ready;
StatusRequest read;

//prototipi
void sensors_setup();
void runner_setup();
void process_logic();
void save_baseline_func();
void disable_logic();
void disable_setup();
void updateDisplays();
void graph();
void first_page();
void second_page();

//struct per salvataggio nella eeprom
struct baseline{
  uint16_t eeprom_eco2;
  uint16_t eeprom_tvoc;
};

// ---------------------------------------------------------------------- PIN ----------------------------------
#define PIN_DHT         2
#define PIN_BTN_MODE    3
#define PIN_BTN_ACT     4
#define PIN_LED_OK      5
#define PIN_LED_WARN    6
#define PIN_LED_ALARM   7
#define PIN_BUZZER      8
#define PIN_SERVO       9
#define PIN_LM35        A0
#define PIN_WATER       A1
#define WEAR            100

// ------------------------------------------------------------- CONFIGURAZIONI SENSORI ---------------------------
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);
Adafruit_SGP30 sgp;
BH1750 lightMeter;
Servo irrigationServo;

// ----------------------------------------------------------------- SCHERMO ---
// Usa buffer di paginazione (_1_) per SH1106 (1.3")per salvare SRAM
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2_main(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ---------------------------------------------------------------- VARIABILI GLOBALI ---------------------------
float temp_dht, hum_dht, temp_lm35, hi;
uint16_t tvoc, eco2, lux;
int water_level;
bool isAutoMode = true;
bool isIrrigating = false;
unsigned int eeprom_addr = 0;
float kalman_temp = 0;
unsigned int wear_cont = 0; //per salvare l'eeprom dall'usura
bool safe_data = true;
float history[128];
unsigned int i = 0;

//---------------------------------------------------------------------- KALMAN ------------------------
float Q = 0.022;  // Incertezza del processo (quanto pensi che cambi la temp velocemente)    //-----------indicazioni su come usarlo--------
float R = 0.5;    // Incertezza della misura (rumore dei sensori)                           if troppo lento a cambiare, ++Q o --R
float P = 1.0;    // Errore stimato iniziale                                                if valore oscilla ++R --Q
float K = 0;      // Guadagno di Kalman
float temp_guess = 25.0; // Valore iniziale della temperatura stim
//pesi per kalman
const float PESO_DHT = 0.3;
const float PESO_LM = 0.7; // per le altri costanti guarda calculate_temp()

// ----_------------------------------------------------------------------ SOGLIE ----------------------------------------------
const float TEMP_HIGH = 35.0;
const uint16_t ECO2_HIGH = 1000;
const int WATER_LOW = 300; // soglia per livello d'acqua
const long interval = 2000; 
const unsigned long eeprom_interval = 3600000UL; // 1 ora
const float alfa = 0.1;
const uint32_t settegiorni = 604800UL; // 7 giorni in secondi
const int buttons_interval = 50;

//-------------------------------------------------------------------- TASK SCHEDULER -------------------------------------------
Task t_sensor(TASK_IMMEDIATE, TASK_ONCE, &sensors_setup, &runner, true, NULL, &disable_setup);
Task t_logic(interval, TASK_FOREVER, &process_logic, &runner);
Task t_save_baseline(eeprom_interval, TASK_FOREVER, &save_baseline_func, &runner);
Task t_update(TASK_IMMEDIATE, TASK_ONCE, &updateDisplays, &runner);
Task t_graph(TASK_IMMEDIATE, TASK_ONCE, &graph, &runner);
Task t_first_page(TASK_IMMEDIATE, TASK_ONCE, &first_page, &runner);
Task t_second_page(TASK_IMMEDIATE, TASK_ONCE, &second_page, &runner);
Task t_buttons(buttons_interval, TASK_FOREVER, &handleButtons, &runner);
Task t_serial(100, TASK_FOREVER, &handleSerial, &runner);


//---------------------------------------------------------------------- SETUP -------------------------------------
void setup() {
  Serial.begin(9600);
  while(!Serial); //attesa avvio seriale
  Serial.println(F("Avvio....(Speriamo che funzioni)"));

  runner_setup();

  // test del buzzer
  tone(PIN_BUZZER, 2000, 100);
}

void PrepareStatus() {
  read.setWaiting();
}

void PrepareStatus_setup() {
  ready.setWaiting();
}

void disable_setup() {
  PrepareStatus_setup();
  t_sensor.restartDelayed();
}

void disable_logic() {
  PrepareStatus();
  t_logic.restartDelayed();
}

void runner_setup() {
  runner.init();
  PrepareStatus();
  PrepareStatus_setup();
  t_update.waitFor(&read);
  t_logic.waitFor(&ready);
  t_save_baseline.waitFor(&ready);
  t_buttons.waitFor(&ready);
  
  t_sensor.enable();
  t_logic.enable();
  t_save_baseline.enable();
  t_buttons.enable();
  t_serial.enable();
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
  dht.begin();
  
  Wire.begin();
  if(!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) Serial.println(F("BH1750 non trovato"));

  setup_sgp30();

  //servo
  irrigationServo.attach(PIN_SERVO);
  irrigationServo.write(90); //posizione neutra del servo

  u8g2_main.begin();

  ready.signalComplete();
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
  if(!isnan(eeprom_baseline.eeprom_tvoc) && !isnan(eeprom_baseline.eeprom_eco2) ){
    sgp.setIAQBaseline(eeprom_baseline.eeprom_eco2, eeprom_baseline.eeprom_tvoc); //settaggi con controllo
    wear_cont++;
  }
  else Serial.println(F("Lettura dalla eeprom fallita"));
}

// ---------------------------------------------------------------------- FINE SETUP --------------------------------------------------

// ---------------------------------------------------------------------- LOGICA ------------------------------------------------
void loop() {
  runner.execute();
}

void process_logic() {
  readSensors();
  process_logic_internal();
}

//gestione bottoni
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

//logica di allarme
void process_logic_internal() {
  bool alarm = false;

  //gestione irrigazione automatica 
  if (isAutoMode) {
    if (hum_dht < 40 && water_level > WATER_LOW) startIrrigation();
    else if (kalman_temp < TEMP_HIGH || hum_dht > 75) stopIrrigation();
  }

  //allarme per CO2 e temperatura (emergenza grave)
  if (kalman_temp > TEMP_HIGH || eco2 > ECO2_HIGH) {
    alarm = true;
    if (eco2 > (ECO2_HIGH * 2)) {
      tone(PIN_BUZZER, 1000, 200);
    }
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

//----------------------------------------------------------------------------------- SENSORI -------------------------------------------------------------

void readSensors() {
  safe_data = true;
  dht11_read();
  
  lm35_read();

  //unione valori della temperatura con kalman
  if(safe_data){
    kalman_temp = calculate_temp(temp_dht,temp_lm35);
  }

  sgp30_read();

  //update luce, livello d'acqua e orario
  secure_ligthRead();
  water_level = analogRead(PIN_WATER);

  read.signalComplete();
}

// -------------------------- SGP30 (TVOC - eCO2) ------------------

//funzione per salvae la baseline nella eeprom
void save_baseline_func() {
  baseline tosave;
  if(sgp.getIAQBaseline(&tosave.eeprom_eco2, &tosave.eeprom_tvoc)){
    if(wear_cont >  WEAR){
      eeprom_addr+=sizeof(baseline);
      wear_cont = 0;
    }
    EEPROM.put(eeprom_addr, tosave); // put è meglio di update per le struct
    Serial.println(F("Salvataggio baseline riuscito"));
    wear_cont++;
  }
  else Serial.println(F("Salvataggio baseline non riuscito"));
}

//calcolo umidità assoluta
uint32_t abs_hum(float temperature, float humidity) {
  //calcolo umidità assoluta (mg/cm3 e non %) per il cavolo di sgp30
  //presa da adafruit sgp30 doc
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
  return absoluteHumidityScaled;
}

//lettura sgp30
void sgp30_read() {
  //settaggio umidità per sgp30
  if(!sgp.setHumidity(abs_hum(kalman_temp, hum_dht))) Serial.println(F("All'sgp30 non garba la tua umidità"));

  //calcolo TVOC e eCO2
  if (sgp.IAQmeasure()) {
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
  }
  else Serial.println(F("Lettura sgp30 fallita"));
}

// -------------------- DHT11 (temperatura - umidità) -------------

void dht11_read() {
  //tentativo lettura valori grezzi
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  //calcolo con media mobile esponenziale e controlli di validità
  if(!isnan(t)){
    mme(t,temp_dht);
  }
  else {
    Serial.println(F("Lettura temperatura dht11 fallita"));
    safe_data = false;
  }
  if(!isnan(h)){
    mme(h,hum_dht);
  }
  else {
    Serial.println(F("Lettura umidità dht11 fallita"));
    safe_data = false;
  }

  hi = dht.computeHeatIndex(temp_dht, hum_dht, false); //calcolo indice di calore con valori già filtrati, (isFahreheit = false)
}

// ------------------ LM35 (temperatura) ---------------------------

void lm35_read() {
  // calcolo della temperatura analogico
  float raw = analogRead(PIN_LM35);

  if(!isnan(raw)) {
    raw = (raw * 5.0 * 100.0) / 1024.0;
    mme(raw,temp_lm35);
  }
  else Serial.println(F("Lettura LM35 fallita"));
}

// ---------------------- BH1750 (luce) -------------------------

void secure_ligthRead() {
  if (lightMeter.measurementReady()) {
    lux = lightMeter.readLightLevel();

    //preso dai doc della libreria
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

// ----------------------------------------------------------------------- FILTRI e MEDIE ---------------------------------------------------------------

//media mobile
void mme(float raw, float& filtered){
  if(raw)
    filtered = (alfa * raw) + ((1 - alfa) * filtered);
  else filtered = raw; 
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

// -------------------------------------------------------------------- DISPLAY e SERIALE ---------------------------------------------------------

//gestione display 
void updateDisplays() {
  // 1.3" Display principale 
  first_page();
  logSerial();
  t_update.disable();
  t_graph.enable();
}

//------------bitmap ---------------------
static const unsigned char image_Layer_13_bits[] PROGMEM = {0x80,0x00,0x84,0x10,0x08,0x08,0xc0,0x01,0x31,0x46,0x12,0x24,0x08,0x08,0x08,0x08,0x08,0x08,0x12,0x24,0x31,0x46,0xc0,0x01,0x08,0x08,0x84,0x10,0x80,0x00,0x00,0x00};
static const unsigned char image_Layer_4_bits[] PROGMEM = {0x38,0x00,0x44,0x40,0xd4,0xa0,0x54,0x40,0xd4,0x1c,0x54,0x06,0xd4,0x02,0x54,0x02,0x54,0x06,0x92,0x1c,0x39,0x01,0x75,0x01,0x7d,0x01,0x39,0x01,0x82,0x00,0x7c,0x00};
static const unsigned char image_Layer_5_bits[] PROGMEM = {0x20,0x00,0x20,0x00,0x30,0x00,0x70,0x00,0x78,0x00,0xf8,0x00,0xfc,0x01,0xfc,0x01,0x7e,0x03,0xfe,0x02,0xff,0x06,0xff,0x07,0xfe,0x03,0xfe,0x03,0xfc,0x01,0xf0,0x00};
static const unsigned char image_ButtonCenter_bits[] PROGMEM = {0x1c,0x22,0x5d,0x5d,0x5d,0x22,0x1c};


//prima pagina (dati)
void first_page() {
  u8g2_main.firstPage();
  do{
    u8g2_main.clearBuffer();
    u8g2_main.setFontMode(1);
    u8g2_main.setBitmapMode(1);

    u8g2_main.drawFrame(0, 0, 128, 64);
    u8g2_main.drawLine(0, 12, 127, 12);

    u8g2_main.setFont(u8g2_font_6x10_tf);
    u8g2_main.setCursor(12, 10); u8g2_main.print(F("-- Stazione Meteo --"));

    u8g2_main.drawXBM(5, 18, 16, 16, image_Layer_4_bits);
    u8g2_main.drawXBM(6, 41, 11, 16, image_Layer_5_bits);

    u8g2_main.drawLine(75, 13, 75, 63);
    u8g2_main.setCursor(23, 33); u8g2_main.print(kalman_temp, 1);

    u8g2_main.setCursor(23, 24); u8g2_main.print(F("Temp"));

    u8g2_main.setCursor(23, 47); u8g2_main.print(F("Hum"));

    u8g2_main.setCursor(22, 56); u8g2_main.print(hum_dht, 1);
    u8g2_main.setCursor(81, 45); u8g2_main.print(F("HI"));
    u8g2_main.setCursor(83, 53); u8g2_main.print(hi, 1);
    u8g2_main.drawXBM(93, 21, 15, 16, image_Layer_13_bits);

    u8g2_main.sendBuffer();
  }while(u8g2_main.nextPage());
  t_first_page.disable();
  t_second_page.enable();
}

//seconda pagina (altri dati)
void second_page() {
  u8g2_main.firstPage();
  do{
    u8g2_main.clearBuffer();
    u8g2_main.setFontMode(1);
    u8g2_main.setBitmapMode(1);

    u8g2_main.setFont(u8g2_font_6x10_tf);
    u8g2_main.setCursor(2, 8); u8g2_main.print(F("2026-04-30 08:47"));

    u8g2_main.setCursor(3, 23); u8g2_main.print(F("TVOC: ")); u8g2_main.print(tvoc);
    u8g2_main.setCursor(3, 36); u8g2_main.print(F("CO2: ")); u8g2_main.print(eco2);

    u8g2_main.setCursor(3, 63); u8g2_main.print(F("Mode: ")); u8g2_main.print(isAutoMode ? F("A") : F("M"));

    u8g2_main.drawLine(1, 51, 125, 51);
    u8g2_main.drawLine(1, 10, 125, 10);

    u8g2_main.drawXBM(115, 1, 7, 7, image_ButtonCenter_bits);

    u8g2_main.sendBuffer();
  }while(u8g2_main.nextPage());
}


//disegno su schermo principale TODO -------------------------------------------------------------------------------------
void graph() {
  u8g2_main.clearBuffer();

  //disegno
  u8g2_main.drawLine(0, 163, 128, 63); //asse x
  u8g2_main.drawBox(10, 20, 5, 40);

  t_first_page.enable();
  t_graph.disable();
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
  Serial.print(F(" Status: ")); Serial.println(isIrrigating ? F("IRR") : F("NO"));
}

//gestione comandi da seriale
void handleSerial() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'i') {
      isAutoMode = false;
      startIrrigation();
      Serial.println(F("Irrigazione manuale via seriale"));
    } else if (cmd == 's') {
      stopIrrigation();
      Serial.println(F("Stop irrigazione via seriale"));
    }
  }
}

