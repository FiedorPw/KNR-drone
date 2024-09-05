class DroneMovement:
    def __init__(self, interface):
        self.interface = interface

    def move_left(self, speed):
        self.interface.move(-speed, 0, 0)
        print(f'Move left with speed: {speed}')

    def move_right(self, speed):
        self.interface.move(speed, 0, 0)
        print(f'Move right with speed: {speed}')

    def move_forward(self, speed):
        self.interface.move(0, speed, 0)
        print(f'Move forward with speed: {speed}')

    def move_back(self, speed):
        self.interface.move(0, -speed, 0)
        print(f'Move back with speed: {speed}')

    def move_up(self, speed):
        self.interface.move(0, 0, speed)
        print(f'Move up with speed: {speed}')

    def move_down(self, speed):
        self.interface.move(0, 0, -speed)
        print(f'Move down with speed: {speed}')

    def land(self):
        self.interface.land()
        print('Landing')

    def start(self):
        self.interface.start()
        print('Starting')

    def terminate(self):
        self.interface.terminate_handler()
        print('Terminating handler')
