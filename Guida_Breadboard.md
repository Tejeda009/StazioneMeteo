# Guida alla Costruzione di Circuiti su Breadboard

Questa guida illustra le migliori pratiche, le misure di sicurezza e i passaggi fondamentali per assemblare circuiti stabili e sicuri su una breadboard.

## 1. Anatomia della Breadboard
Una breadboard standard è divisa in due sezioni principali:
- **Linee di Alimentazione (Rails):** Le righe orizzontali lungo i bordi (solitamente contrassegnate con linee rosse `+` e blu/nere `-`). Sono interconnesse orizzontalmente e servono per distribuire l'alimentazione (es. 5V o 3.3V) e la massa (GND) a tutto il circuito.
- **Area dei Componenti (Terminal Strips):** Le colonne centrali (numerate e con lettere A-E, F-J). I fori in una stessa colonna (es. A1, B1, C1, D1, E1) sono collegati elettricamente tra loro, ma isolati dalla colonna successiva e dalla metà inferiore. Il solco centrale serve per inserire i circuiti integrati (IC), isolando i pin di destra da quelli di sinistra per evitare cortocircuiti.

## 2. Alimentazione e Condensatori (Filtri)
Un'alimentazione stabile è cruciale per evitare riavvii anomali del microcontrollore o letture errate dei sensori, specialmente quando si usano componenti che assorbono molta corrente come motori o relè.

- **Condensatori di Bypass (Decoupling):** È buona norma inserire un condensatore ceramico da **100nF (0.1µF)** tra la linea del positivo (`+`) e la massa (`-`) il più vicino possibile ai pin di alimentazione dei circuiti integrati. Questo filtra i disturbi ad alta frequenza e i cali di tensione repentini.
- **Condensatori di Filtro Principali:** Sulle linee di alimentazione principali della breadboard (i rails), è utile aggiungere un condensatore elettrolitico più grande (es. tra **10µF e 100µF**) per gestire i picchi di assorbimento di corrente generali.
  - ⚠️ **ATTENZIONE:** I condensatori elettrolitici sono **polarizzati**. Il reoforo (pin) più lungo va al positivo (`+`), mentre la banda laterale con il segno meno (`-`) va collegata a massa. **Invertire la polarità di un condensatore elettrolitico può causarne la rottura o l'esplosione!**

## 3. Misure di Sicurezza Fondamentali

1. **Scollegare l'Alimentazione:** Non modificare **MAI** il circuito mentre è alimentato. Scollega l'alimentazione prima di aggiungere, rimuovere o spostare fili o componenti.
2. **Controllo della Polarità:** Verifica due volte la polarità di componenti critici prima di dare tensione:
   - **LED:** l'anodo è il pin lungo (`+`), il catodo è corto (`-` / lato piatto della lente).
   - **Diodi:** la banda argentata/nera indica il catodo (`-`).
   - **IC:** il pallino o la tacca a mezzaluna indicano l'orientamento per individuare il pin 1.
3. **Prevenzione dei Cortocircuiti:**
   - Non collegare mai direttamente la linea `5V` (o `3.3V`) alla linea `GND`. Questo causa un cortocircuito netto.
   - Assicurati che i terminali metallici nudi dei componenti (come le resistenze con gambe lunghe) non si tocchino accidentalmente tra loro.
4. **Resistenze per LED:** Usa SEMPRE una resistenza in serie ai LED (es. 220Ω, 330Ω, o 1kΩ) per limitare la corrente. Senza resistenza, il LED assorbirà troppa corrente e si brucerà istantaneamente.

## 4. Best Practices per il Cablaggio

- **Codice Colore dei Cavi:** Abituati a usare una logica rigorosa:
  - **Rosso:** Alimentazione positiva.
  - **Nero (o Blu/Marrone):** Massa (GND).
  - **Giallo/Verde/Bianco ecc.:** Segnali dati, I/O.
- **Ordine Logico di Assemblaggio:**
  1. Collega prima di tutto i rail di alimentazione. Se usi entrambi i lati della breadboard, ponticella i rail di destra con quelli di sinistra.
  2. Inserisci i componenti principali e i circuiti integrati a cavallo del solco centrale.
  3. Aggiungi i componenti passivi.
  4. Infine, esegui i collegamenti dei segnali logici.

## 5. Consigli Finali
- Se il circuito ha componenti di potenza (es. un servomotore), alimentalo con una sorgente esterna e **non usare il pin 5V della logica di controllo**. Quando usi più alimentazioni diverse, **ricordati sempre di collegare tra loro le masse (GND) di tutti gli alimentatori** per avere un riferimento di tensione comune.
- Fai sempre una revisione visiva di 30 secondi dell'intero circuito seguendo i percorsi di corrente PRIMA di accendere l'alimentazione.
