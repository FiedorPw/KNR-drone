import lgpio
import time
import tkinter as tk
from gpiozero import PWMOutputDevice, DigitalOutputDevice

def claim_and_release_pins(pins):
    # Open GPIO chip
    h = lgpio.gpiochip_open(0)

    # Claim and release the pins
    for pin in pins:
        try:
            lgpio.gpio_claim_output(h, 0, pin, 0)
            lgpio.gpio_free(h, pin)
        except Exception as e:
            print(f"Error releasing pin {pin}: {e}")

    # Close GPIO chip
    lgpio.gpiochip_close(h)

# List of GPIO pins to claim and release
pins_to_reset = [17, 22, 27]  # Replace with the pins you are using

# Claim and release the pins
claim_and_release_pins(pins_to_reset)

# Define the pins for the H-bridge
pinForward = 17  # Pin do przodu (GPIO 17)
pinBackward = 27 # Pin do tylu (GPIO 27)
pinPWM = 22
wartoscPWM = 0.8

# Set up the pins
forward = DigitalOutputDevice(pinForward)
backward = DigitalOutputDevice(pinBackward)
speed = PWMOutputDevice(pinPWM)

def motor_forward(speed_value=wartoscPWM):
    """Move forward with specified speed."""
    backward.off()
    forward.on()
    speed.value = speed_value  # Set speed (0.0 to 1.0)

def motor_backward(speed_value=wartoscPWM):
    """Move backward with specified speed."""
    forward.off()
    backward.on()
    speed.value = speed_value  # Set speed (0.0 to 1.0)

def motor_stop():
    """Stop the motor."""
    forward.off()
    backward.off()
    speed.value = 0

def on_key_press(event):
    """Handle key press events."""
    key = event.keysym
    if key == 'w':
        motor_forward()
        print("a forward")

    elif key == 's':
        motor_backward()
        print("d backward")

    elif key == 'space':
        motor_stop()
        print("s stop")

# Set up the tkinter GUI
root = tk.Tk()
root.title("Motor Control")

# Bind keys to functions
root.bind('<KeyPress>', on_key_press)

# Instructions label
instructions = tk.Label(root, text="Press 'W' to move forward, 'S' to move backward, 'Space' to stop")
instructions.pack()

# Run the tkinter main loop
root.mainloop()

