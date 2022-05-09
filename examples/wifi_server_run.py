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
import socket


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

def backward(power):
    left_front.set_power(-power)
    left_rear.set_power(-power)
    right_front.set_power(-power)
    right_rear.set_power(-power)

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

def move_backward_10cm(n_10cm=1):
    # foward(1) range(15) sleep(0.1) min_power:20 achieve 10cm +- 1cm
    backward(1)
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
    HOST = "192.168.0.104" # IP address of your Raspberry PI
    PORT = 65432          # Port to listen on (non-privileged ports are > 1023)

    print("RUN SUCCESS")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()

        try:
            dist_traveled = 0
            start_speed_thread()
            while 1:
                print("LISTENING")
                client, clientInfo = s.accept()
                print("server recv from: ", clientInfo)
                data = client.recv(1024).decode('ascii')      # receive 1024 Bytes of message in binary format
                if data:
                    print(data)
                    stats = pi_read()
                    if data == "w":
                        move_forward_10cm(1)
                        dist_traveled += 10
                        msg = "forward {:2f}cm/s {:2f}cm {:2f}C".format(speed_val(), dist_traveled, stats["cpu_temperature"])
                    elif data == "s":
                        move_backward_10cm(1)
                        dist_traveled += 10
                        msg = "backward {:2f}cm/s {:2f}cm {:2f}C".format(speed_val(), dist_traveled, stats["cpu_temperature"])
                    elif data == "d":
                        turn_right_90()
                        msg = "turnRight90 {:2f}cm/s {:2f}cm {:2f}C".format(speed_val(), dist_traveled, stats["cpu_temperature"])
                    elif data == "a":
                        turn_left_90()
                        msg = "turnLeft90 {:2f}cm/s {:2f}cm {:2f}C".format(speed_val(), dist_traveled, stats["cpu_temperature"])
                    else:
                        msg = "Not supported!"
                    client.sendall(msg.encode('ascii')) # Echo back to client

        except:
            end_speed_thread()
            print("Closing socket")
            client.close()
            s.close()
