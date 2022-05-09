from picar_4wd.speed import Speed
from picar_4wd.pin import Pin
from picar_4wd.pwm import PWM
import RPi.GPIO as GPIO
import time, math
import threading

class Motor():
    STEP = 10
    DELAY = 0.1
    def __init__(self, pwm_pin, dir_pin, is_reversed=False):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self._is_reversed = is_reversed
        self._power = 0
        self._except_power = 0

    # def start_timer(self):
    #     self.t = threading.Timer(self.DELAY, self.adder_thread)
    #     self.t.start()

    def set_power(self, power):
        if power >= 0:
            direction = 0
        elif power < 0:
            direction = 1
        power = abs(power)
        if power != 0:
            power = int(power /2 ) + 20
        power = power

        direction = direction if not self._is_reversed else not direction
        self.dir_pin.value(direction)

        self.pwm_pin.pulse_width_percent(power)

left_front = Motor(PWM("P13"), Pin("D4"), is_reversed=True) # motor 1
right_front = Motor(PWM("P12"), Pin("D5"), is_reversed=True) # motor 2
left_rear = Motor(PWM("P8"), Pin("D11"), is_reversed=True) # motor 3
right_rear = Motor(PWM("P9"), Pin("D15"), is_reversed=True) # motor 4


def forward(power):
    left_front.set_power(power)
    left_rear.set_power(power)
    right_front.set_power(power)
    right_rear.set_power(power)

def turn_left(power):
    left_front.set_power(-power)
    left_rear.set_power(-power)
    right_front.set_power(power)
    right_rear.set_power(power)

def turn_right(power):
    left_front.set_power(power)
    left_rear.set_power(power)
    right_front.set_power(-power)
    right_rear.set_power(-power)

def stop():
    left_front.set_power(0)
    left_rear.set_power(0)
    right_front.set_power(0)
    right_rear.set_power(0)

def move_forward_10cm(n_10cm=1):
    speed4 = Speed(25)
    print("My test:", speed4.my_test)
    speed4.start()
    # foward(1) range(15) sleep(0.1) min_power:20 achieve 10cm +- 1cm
    forward(1)

    x = 0
    for i in range(15*n_10cm):
        time.sleep(0.1)
        speed = speed4()
        x += speed * 0.1
        print("%scm/s"%speed)
    print("%scm"%x)
    speed4.deinit()
    stop()

def turn_right_90():
    # turn_right(70) range(110) sleep(0.01)  min_power:20 achieve turn right 90
    turn_right(70)
    for i in range(106):
        time.sleep(0.01)
    stop()

def turn_left_90():
    # turn_right(70) range(117) sleep(0.01)  min_power:20 achieve turn left 90
    turn_left(70)
    for i in range(106):
        time.sleep(0.01)
    stop()

def turn_180():
    turn_left_90()
    turn_left_90()

if __name__ == "__main__":
    turn_right_90()
