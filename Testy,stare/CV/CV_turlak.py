import cv2
from time import sleep
import numpy as np
from CameraController import CameraController
from FlightControllerInterface import FlightControllerInterface
import RecieveTelemetry
'''
nie przetestowana w rzeczywistości:(

jest to druga najważniejsza klasa po main controllerze, 
przejmuje sterowanie od niego podczas schodzenia po piłkę danego koloru wykorzystując feedback z kamery
i na tej podstawie wydaje rozkazy przez metode FlightControllerIntreface 
'''

# Zestawy wartości HSV dla różnych obiektów, kolory ramki i priorytety
hsv_values = {
    "Zolta pilka": {"lower": np.array([20, 80, 0]), "upper": np.array([50, 255, 255]), "color": (0, 255, 255), "priority": 1},
    "Ceglana pilka": {"lower": np.array([40, 40, 50]), "upper": np.array([180, 50, 255]), "color": (0, 0, 255), "priority": 3},  # Czerwony kolor ramki
    "Niebieska pilka": {"lower": np.array([60, 60, 0]), "upper": np.array([110, 255, 255]), "color": (255, 0, 0), "priority": 4},
    "Fioletowa pilka": {"lower": np.array([120, 40, 0]), "upper": np.array([160, 255, 255]), "color": (255, 0, 255), "priority": 2},
    "Biala plachta": {"lower": np.array([0, 0, 180]), "upper": np.array([180, 50, 255]), "color": (0, 0, 255), "priority": 0}  # Niski priorytet, pomocniczy
}
def get_vectors_from_pic():
    pass


# hit mix do lądowania na piłce danego koloru, po wylądowaniu oddaje sterowanie do MainControllera(robi return z funkcji)
def land_on_ball(ball_colour, multiplyer, descent_speed, delay_betwen_move_and_photo):
    
    camera_controller = CameraController()
    flight_controller_interface = FlightControllerInterface()


    drone_landed = False
    barometr_says_we_landed_XD = False

    while not drone_landed:
        
        attitude = RecieveTelemetry.getattitude()
        if attitude <= 0.1: break
        
        # z dużej wysokości skanuje aktualne obrazy
        img = camera_controller.take_picture()
        x, y, _ = get_direction_from_img(img, ball_colour) # jak zwraca zero to jest poniżej thresholdu
        # im większy obiekt jest tym jesteśmy bliżej więc musimy zrobić mniejszy krok 
        step_size_multipylier = multiplyer * attitude
        # mnożymy x i y przez multiplyer
        x *= step_size_multipylier
        y *= step_size_multipylier

        flight_controller_interface.move(x, y, descent_speed)
        
        sleep(delay_betwen_move_and_photo)




# Funkcja do wykrywania koloru
def detect_color(frame, lower_color, upper_color, color, draw_rectangle):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    targets = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if draw_rectangle:
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        targets.append((x, y, w, h))
    return targets, mask

# Funkcja do regulacji pozycji prostokąta i rysowania linii
def adjust_rectangle_position(target, frame_center, frame):
    if target is None:
        return None, None, None  # Nie wykryto prostokąta, zwracamy None dla wszystkich wartości

    x, y, w, h = target
    target_center_x = x + w // 2
    target_center_y = y + h // 2

    # Oblicz błąd w pozycji X i Y
    error_x = frame_center[0] - target_center_x
    error_y = frame_center[1] - target_center_y

    # Stałe do regulacji
    movement_threshold = 30  # Minimalna odległość, aby ruch był wykonywany
    
    # Rysowanie linii od środka ekranu do celu
    cv2.line(frame, frame_center, (target_center_x, target_center_y), (0, 255, 0), 2)

    # Regulacja ruchu wzdłuż osi X
    if abs(error_x) > movement_threshold:
        if error_x > 0:
            move_left()  # Prostokąt przesunięty w prawo zgodnie do zwrotu x
        else:
            move_right()  # Prostokąt przesunięty przeciwnie do zwrotu x

    # Regulacja ruchu wzdłuż osi Y
    if abs(error_y) > movement_threshold:
        if error_y > 0:
            move_forward()  # Prostokąt przesunięty zgodnie do zwrotu y
        else:
            move_back()  # Prostokąt przesunięty przeciwnie do zwrotu y

    return error_x, error_y, w * h  # Zwracamy błędy x i y oraz powierzchnię prostokąta

def get_direction_from_img(image_path, target_color):
    # Wczytanie obrazu z pliku
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Unable to load image at {image_path}")
        return None, None, None  # Zwracamy None, jeśli nie można wczytać obrazu

    # Zmniejszenie rozmiaru obrazu
    scale_percent = 50  # Skaluje obraz do 50% oryginalnego rozmiaru
    width = int(frame.shape[1] * scale_percent / 100)
    height = int(frame.shape[0] * scale_percent / 100)
    dim = (width, height)
    draw_rectangle = False
    
    # Sprawdzenie, czy wybrany kolor istnieje w słowniku hsv_values
    if target_color not in hsv_values:
        print(f"Error: Target color '{target_color}' not found in hsv_values.")
        return None, None, None

    while True:
        # Skalowanie obrazu
        resized_frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
        frame_center = (resized_frame.shape[1] // 2, resized_frame.shape[0] // 2)

        # Wykrywanie białych obszarów
        lower_white = hsv_values["Biala plachta"]["lower"]
        upper_white = hsv_values["Biala plachta"]["upper"]
        white_color = hsv_values["Biala plachta"]["color"]
        white_targets, white_mask = detect_color(resized_frame, lower_white, upper_white, white_color, draw_rectangle)

        best_area = 0
        best_target = None
        best_mask = None
        best_color_name = None
        best_color = None

        for white_target in white_targets:
            x, y, w, h = white_target
            white_area_frame = resized_frame[y:y+h, x:x+w]

            # Wykrywanie wybranego koloru
            if target_color in hsv_values:
                hsv = hsv_values[target_color]
                lower_color = hsv["lower"]
                upper_color = hsv["upper"]
                color = hsv.get("color", (0, 0, 0))  # Default to black if color is not specified
                targets, mask = detect_color(white_area_frame, lower_color, upper_color, color, draw_rectangle)
                
                for target in targets:
                    tx, ty, tw, th = target
                    area = tw * th
                    if area > best_area:
                        best_area = area
                        best_target = (x + tx, y + ty, tw, th)
                        best_mask = mask
                        best_color_name = target_color
                        best_color = color

        # Wykonanie regulacji ruchu
        error_x, error_y, target_area = adjust_rectangle_position(best_target, frame_center, resized_frame)

        # Wyświetlanie zmniejszonego obrazu
        cv2.imshow('Resized Frame', resized_frame)
        cv2.imshow('White Mask', white_mask)

        # Sprawdzenie, czy prostokąt znajduje się w obrębie progu
        threshold_area = 50  # Definicja progu obszaru
        if target_area and target_area > threshold_area:
            print("Znaleziono cel w obrębie progu.")
            

        # Oczekiwanie na klawisz i zakończenie pętli w przypadku naciśnięcia klawisza 'q'
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            draw_rectangle = not draw_rectangle

    # Zamknięcie wszystkich okien
    cv2.destroyAllWindows()

    return error_x, error_y, target_area  # Zwracamy błędy x i y oraz powierzchnię obszaru


image_path = '' # PATH
target_color = "" # NAZWA KOLORU: Niebieska; Ceglana; Fioletowa; Zolta + pilka
get_direction_from_img(image_path, target_color)
