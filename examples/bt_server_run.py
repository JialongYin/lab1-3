from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.speed import Speed
from picar_4wd.pin import Pin
from picar_4wd.pwm import PWM
from picar_4wd.filedb import FileDB
from picar_4wd.utils import pi_read
import RPi.GPIO as GPIO
import time, math
import threading
from collections import defaultdict
import numpy as np
import sys
import cv2
import bluetooth


class Motor():
    STEP = 10
    DELAY = 0.1
    def __init__(self, pwm_pin, dir_pin, is_reversed=False):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self._is_reversed = is_reversed
        self._power = 0
        self._except_power = 0

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

# Config File:
config = FileDB("config")
left_front_reverse = config.get('left_front_reverse', default_value = False)
right_front_reverse = config.get('right_front_reverse', default_value = False)
left_rear_reverse = config.get('left_rear_reverse', default_value = False)
right_rear_reverse = config.get('right_rear_reverse', default_value = False)
ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0))

# Init motors
left_front = Motor(PWM("P13"), Pin("D4"), is_reversed=left_front_reverse) # motor 1
right_front = Motor(PWM("P12"), Pin("D5"), is_reversed=right_front_reverse) # motor 2
left_rear = Motor(PWM("P8"), Pin("D11"), is_reversed=left_rear_reverse) # motor 3
right_rear = Motor(PWM("P9"), Pin("D15"), is_reversed=right_rear_reverse) # motor 4

left_rear_speed = Speed(25)
right_rear_speed = Speed(4)

# Init Ultrasonic
us = Ultrasonic(Pin('D8'), Pin('D9'))

# Init Servo
servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)

def start_speed_thread():
    # left_front_speed.start()
    # right_front_speed.start()
    left_rear_speed.start()
    right_rear_speed.start()

def end_speed_thread():
    left_rear_speed.deinit()
    right_rear_speed.deinit()

def speed_val():
    return (left_rear_speed() + right_rear_speed()) / 2.0

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
    # foward(1) range(15) sleep(0.1) min_power:20 achieve 10cm +- 1cm
    forward(1)
    x = 0
    slot = 17
    for i in range(slot*n_10cm):
        time.sleep(0.1)
        speed = speed_val()
        x += speed * 0.1
        # print("%scm/s"%speed)
    print("%scm"%x)
    stop()

def turn_right_90():
    # turn_right(70) range(110) sleep(0.01)  min_power:20 achieve turn right 90
    turn_right(70)
    for i in range(102):
        time.sleep(0.01)
    stop()

def turn_left_90():
    # turn_right(70) range(117) sleep(0.01)  min_power:20 achieve turn left 90
    turn_left(70)
    for i in range(102):
        time.sleep(0.01)
    stop()

def turn_180():
    turn_left_90()
    turn_left_90()

if __name__ == "__main__":
    hostMACAddress = "DC:A6:32:88:0A:61" # The address of Raspberry PI Bluetooth adapter on the server. The server might have multiple Bluetooth adapters.
    port = 0
    backlog = 1
    size = 1024
    s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    s.bind((hostMACAddress, port))
    s.listen(backlog)
    print("listening on port ", port)
    try:
        client, clientInfo = s.accept()
        while 1:
            print("server recv from: ", clientInfo)
            data = client.recv(size).decode('ascii')
            if data:
                print(data)
                if data == "stats":
                    stats = pi_read()
                    msg = "cpu_temperature:{:.2f}; battery:{:2f}".format(stats["cpu_temperature"], stats["battery"])
                elif data == "w":
                    move_forward_10cm(1)
                    msg = "Move 10cm forward SUCCESS!"
                elif data == "d":
                    turn_right_90()
                    msg = "turn right 90 SUCCESS!"
                elif data == "a":
                    turn_left_90()
                    msg = "turn left 90 SUCCESS!"
                else:
                    msg = "Not supported!"
                client.send(msg) # Echo back to client
    except:
        print("Closing socket")
        client.close()
        s.close()
