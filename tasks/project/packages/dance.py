import time
from road_map import road_map


class Dance:
    def __init__(self, wheels, leds=None):
        self.wheels = wheels
        self.leds = leds

    def spin_right(self, duration=0.5):
        self.wheels.set_wheels_speed(0.5, -0.5)
        time.sleep(duration)

    def spin_left(self, duration=0.5):
        self.wheels.set_wheels_speed(-0.5, 0.5)
        time.sleep(duration)

    def forward(self, duration=0.3):
        self.wheels.set_wheels_speed(0.4, 0.4)
        time.sleep(duration)

    def backward(self, duration=0.3):
        self.wheels.set_wheels_speed(-0.4, -0.4)
        time.sleep(duration)

    def stop(self):
        self.wheels.set_wheels_speed(0.0, 0.0)

    def perform(self):
        """Full victory dance sequence"""
        self.spin_right(0.5)
        self.spin_left(0.5)
        self.spin_right(0.5)
        self.forward(0.3)
        self.backward(0.3)
        self.spin_right(0.8)
        self.stop()


