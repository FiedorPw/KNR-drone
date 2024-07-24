import time
from pymavlink import mavutil

# Funkcja do połączenia z FC
def connect_to_fc(port='/dev/ttyS0', baud=115200):
    # Inicjalizacja połączenia MAVLink
    master = mavutil.mavlink_connection(port, baud=baud)
    
    # Poczekaj na pierwsze komunikaty typu heartbeat (potwierdzenie połączenia)
    master.wait_heartbeat()
    print("Połączono z FC!")
    
    return master

# Funkcja do odczytu parametrów lotu
def read_flight_parameters(master):
    while True:
        # Pobierz dane telemetryczne
        msg = master.recv_match(blocking=True)
        
        # Sprawdź typ wiadomości i wypisz odpowiednie parametry
        if not msg:
            continue
        if msg.get_type() == 'ATTITUDE':
            print(f"Roll: {msg.roll}, Pitch: {msg.pitch}, Yaw: {msg.yaw}")
        elif msg.get_type() == 'GLOBAL_POSITION_INT':
            print(f"Lat: {msg.lat}, Lon: {msg.lon}, Alt: {msg.alt}")
        elif msg.get_type() == 'VFR_HUD':
            print(f"Altitude: {msg.alt}, Airspeed: {msg.airspeed}, Climb: {msg.climb}")
        # Dodaj inne typy wiadomości i parametry, które chcesz odczytać
        else:
            print(f"Otrzymano wiadomość typu: {msg.get_type()}")

if __name__ == "__main__":
    # Połącz się z kontrolerem lotu
    master = connect_to_fc()

    # Czytaj i wypisuj parametry lotu
    read_flight_parameters(master)
