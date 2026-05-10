# Stazione Meteo

Progetto x Compito Sistemi e Reti 2025/2026 3Q - FreeSauro

## Componenti e Collegamenti

| Componente | Pin Arduino | Descrizione |
| :--- | :--- | :--- |
| **DHT11** | D2 | Umidità e Temperatura Digitale |
| **Pulsante MODALITÀ** | D3 | Cambio tra Auto e Manuale (Internal Pull-up) |
| **Pulsante AZIONE** | D4 | Override manuale / Selezione (Internal Pull-up) |
| **LED Verde** | D5 | Stato: Sistema OK |
| **LED Giallo** | D6 | Stato: Modalità Manuale / Avviso |
| **LED Rosso** | D7 | Stato: Allarme / Irrigazione attiva |
| **Buzzer** | D8 | Allarme acustico |
| **Servo MG946R** | D9 | Pompa irrigazione (360 gradi) |
| **DS1302 CLK** | D10 | RTC Clock |
| **DS1302 DAT** | D11 | RTC Data |
| **DS1302 RST** | D12 | RTC Reset |
| **LM35** | A0 | Temperatura analogica (Vout) |
| **Livello Acqua** | A1 | Sensore di livello / Umidità terreno |
| **I2C SDA** | A4 | Bus dati I2C (Display x2, BH1750, SGP30) |
| **I2C SCL** | A5 | Bus clock I2C (Display x2, BH1750, SGP30) |

## Librerie Necessarie
- `U8g2` (Oliver)
- `DHT sensor library` (Adafruit)
- `Adafruit SGP30`
- `BH1750` (Christopher Laws)
- `virtuabotixRTC`
- `Servo` (Integrata)

## Funzionamento
1. **Automatico**: Monitora i parametri. Se il terreno è secco (`water_level < 300`), attiva l'irrigazione. Se la temperatura o la CO2 sono troppo alte, suona l'allarme.
2. **Manuale**: Passa a questa modalità premendo il pulsante D3. Il LED Giallo si accende. Puoi attivare l'irrigazione tenendo premuto il pulsante D4.

## Monitoraggio Seriale
Oltre ai display OLED, potete  monitorare i dati dai vostri PC usando il Monitor Seriale dell'IDE di Arduino o usando lo script Python incluso:
1. Installate pyserial: `pip install pyserial`
2. Eseguite: `python monitor_seriale.py` (controllate il nome della porta seriale nello script).


