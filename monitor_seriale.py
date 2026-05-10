import serial
import time

# Configurazione della porta seriale (modifica 'COM3' o '/dev/ttyUSB0' in base al tuo sistema)
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 9600

def monitor_weather_station():
    try:
        print(f"Tentativo di connessione a {SERIAL_PORT}...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Attesa per il reset di Arduino
        print("Connessione riuscita, almeno qualcosa funziona Dav")
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(f"[{time.strftime('%H:%M:%S')}] {line}")
            time.sleep(0.1)
            
    except serial.SerialException as e:
        print(f"Errore Seriale: {e}")
    except KeyboardInterrupt:
        print("\nMonitoraggio interrotto dall'utente.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    monitor_weather_station()
