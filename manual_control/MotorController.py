import lgpio
import tkinter as tk
from gpiozero import PWMOutputDevice, DigitalOutputDevice

class MotorController:
    def __init__(self, pins_to_reset, pin_forward, pin_backward, pin_pwm):
        self.pins_to_reset = pins_to_reset
        self.pin_forward = pin_forward
        self.pin_backward = pin_backward
        self.pin_pwm = pin_pwm
        self.speed_value = 0.8

        self.claim_and_release_pins()
        self.forward = DigitalOutputDevice(self.pin_forward)
        self.backward = DigitalOutputDevice(self.pin_backward)
        self.speed = PWMOutputDevice(self.pin_pwm)

        self.setup_gui()

    def claim_and_release_pins(self):
        """Claim and release GPIO pins."""
        h = lgpio.gpiochip_open(0)
        for pin in self.pins_to_reset:
            try:
                lgpio.gpio_claim_output(h, 0, pin, 0)
                lgpio.gpio_free(h, pin)
            except Exception as e:
                print(f"Error releasing pin {pin}: {e}")
        lgpio.gpiochip_close(h)

    def motor_forward(self, speed_value=None):
        """Move forward with specified speed."""
        if speed_value is None:
            speed_value = self.speed_value
        self.backward.off()
        self.forward.on()
        self.speed.value = speed_value

    def motor_backward(self, speed_value=None):
        """Move backward with specified speed."""
        if speed_value is None:
            speed_value = self.speed_value
        self.forward.off()
        self.backward.on()
        self.speed.value = speed_value

    def motor_stop(self):
        """Stop the motor."""
        self.forward.off()
        self.backward.off()
        self.speed.value = 0

    def on_key_press(self, event):
        """Handle key press events."""
        key = event.keysym
        if key == 'w':
            self.motor_forward()
            print("Moving forward")
        elif key == 's':
            self.motor_backward()
            print("Moving backward")
        elif key == 'space':
            self.motor_stop()
            print("Stopping motor")

    def setup_gui(self):
        """Set up the tkinter GUI."""
        self.root = tk.Tk()
        self.root.title("Motor Control")
        self.root.bind('<KeyPress>', self.on_key_press)

        instructions = tk.Label(self.root, text="Press 'W' to move forward, 'S' to move backward, 'Space' to stop")
        instructions.pack()

        self.root.mainloop()
