import math
import time

# Współrzędne początkowe środka kadru kamery
camera_width = 640  # szerokość kamery w pikselach
camera_height = 480  # wysokość kamery w pikselach
x_center = camera_width / 2
y_center = camera_height / 2

# Początkowa pozycja środka kamery (np. w metrach)
drone_x = 0.0
drone_y = 0.0

# Maksymalna odległość, która odpowiada krawędziom kadru kamery
max_distance = math.sqrt(x_center**2 + y_center**2)

# Minimalna i maksymalna prędkość
min_velocity = 0.1
max_velocity = 100.0

# Próg odległości, poniżej którego uznajemy, że dron dotarł do celu
arrival_threshold = 0.5  # w metrach

# Funkcja do obliczania prędkości na podstawie pozycji targetu
def calculate_velocity_to_target(x_target, y_target, drone_x, drone_y):
    # Obliczanie wektora od środka kadru do targetu
    delta_x = x_target - drone_x
    delta_y = y_target - drone_y
    
    # Obliczanie odległości od środka kadru do targetu
    distance = math.sqrt(delta_x**2 + delta_y**2)
    
    if distance == 0:
        return 0, 0, 0  # Dron nie rusza się, jeśli target jest dokładnie w środku kadru
    
    # Normalizacja wektora
    v_x_normalized = delta_x / distance
    v_y_normalized = delta_y / distance
    
    # Obliczanie prędkości, uzależniając ją od odległości
    # Skaluje prędkość pomiędzy min_velocity a max_velocity
    velocity = min_velocity + (max_velocity - min_velocity) * (distance / max_distance)
    velocity = min(max_velocity, max(velocity, min_velocity))  # Ograniczenie do min_velocity i max_velocity
    
    # Wektor prędkości
    velocity_x = velocity * v_x_normalized
    velocity_y = velocity * v_y_normalized
    
    return velocity_x, velocity_y, 0  # velocity_z = 0, bo dron porusza się po płaszczyźnie XY

# Funkcja do symulacji ruchu drona i sprawdzania dotarcia do celu
def fly_to_target(target_x, target_y, total_time, refresh_rate=1):
    global drone_x, drone_y
    
    for t in range(0, total_time, refresh_rate):
        # Obliczenie prędkości na podstawie aktualnej pozycji drona i targetu
        velocity_x, velocity_y, _ = calculate_velocity_to_target(target_x, target_y, drone_x, drone_y)
        
        # Aktualizacja pozycji drona
        drone_x += velocity_x * refresh_rate
        drone_y += velocity_y * refresh_rate
        
        # Obliczenie aktualnej odległości do celu
        distance_to_target = math.sqrt((target_x - drone_x)**2 + (target_y - drone_y)**2)
        
        # Sprawdzenie, czy dron dotarł do celu
        if distance_to_target <= arrival_threshold:
            print(f"Czas: {t}s, Prędkość: vx = {velocity_x:.2f} m/s, vy = {velocity_y:.2f} m/s, Pozycja: x = {drone_x:.2f}, y = {drone_y:.2f}")
            print("Dron dotarł do celu.")
            return True  # Dron dotarł do celu
        
        # Wyświetlenie nowych wartości prędkości
        print(f"Czas: {t}s, Prędkość: vx = {velocity_x:.2f} m/s, vy = {velocity_y:.2f} m/s, Pozycja: x = {drone_x:.2f}, y = {drone_y:.2f}")
        
        # Odczekanie sekundy (symulacja odświeżania co 1 sekundę)
        time.sleep(refresh_rate)
    
    # Jeśli po upływie całego czasu nie dotarł do celu
    print("Dron nie dotarł do celu.")
    return False  # Dron nie dotarł do celu

# Przykładowe współrzędne targetu (np. 10 metrów w prawo i 5 metrów w górę od pozycji startowej drona)
target_x = 10.0
target_y = 5.0

# Symulacja na 10 sekund
result = fly_to_target(target_x, target_y, total_time=20)
print("Wynik symulacji:", "Sukces" if result else "Niepowodzenie")
