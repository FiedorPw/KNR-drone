from pymavlink import mavutil
import time

def main():
    # Konfiguracja połączenia MAVLink
    connection_string = '/dev/ttyACM0'  # Zmień na odpowiedni port szeregowy
    baud_rate = 57600  # Ustaw odpowiednią prędkość transmisji
    
    # Nawiązanie połączenia z kontrolerem lotu
    master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
    master.wait_heartbeat()
    print("Połączono z kontrolerem lotu")

    try:
        while True:
            # Odbieranie wiadomości
            msg = master.recv_match(type='BATTERY_STATUS', blocking=True)
            
            # Sprawdzanie, czy wiadomość o napięciu baterii została odebrana
            if msg:
                # Sprawdzanie dostępnych atrybutów wiadomości
                # print(msg.to_dict())  # Wyświetlenie wszystkich atrybutów wiadomości
                
                if 'voltages' in msg.to_dict():
                    voltage_battery = msg.voltages[0] / 1000.0  # Konwersja z mV na V, indeks 0 może wymagać dostosowania
                    print(f"Napięcie baterii: {voltage_battery:.2f} V")
                else:
                    print("Brak informacji o napięciu baterii w wiadomości")
            
            time.sleep(1)  # Czekaj 1 sekundę przed następnym odczytem
    
    except KeyboardInterrupt:
        print("Przerwano działanie skryptu")
    
    finally:
        master.close()

if __name__ == "__main__":
    main()
