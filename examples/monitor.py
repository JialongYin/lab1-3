from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
from picar_4wd.speed import Speed
from picar_4wd.pin import Pin
from picar_4wd.pwm import PWM
from picar_4wd.filedb import FileDB
import RPi.GPIO as GPIO
import time, math
import threading
from collections import defaultdict
import numpy as np
import sys
import cv2
from object_detector import ObjectDetector
from object_detector import ObjectDetectorOptions
np.set_printoptions(threshold=sys.maxsize)

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


"""Routing"""
left_front = Motor(PWM("P13"), Pin("D4"), is_reversed=True) # motor 1
right_front = Motor(PWM("P12"), Pin("D5"), is_reversed=True) # motor 2
left_rear = Motor(PWM("P8"), Pin("D11"), is_reversed=True) # motor 3
right_rear = Motor(PWM("P9"), Pin("D15"), is_reversed=True) # motor 4

prob_thresh = 0.3 # if prediction probability is over threshold, start sending
interval = 5 # 5s for debug; send a picture if dog detected; interval between two sends is 1 minute at least

def run_detector(model='efficientdet_lite0.tflite', camera_id=0, width=640, height=480, num_threads=4, enable_edgetpu=False):
    # Start capturing video input from the camera
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Initialize the object detection model
    options = ObjectDetectorOptions(
        num_threads=num_threads,
        score_threshold=0.3,
        max_results=3,
        enable_edgetpu=enable_edgetpu)
    detector = ObjectDetector(model_path=model, options=options)

    # Continuously capture images from the camera and run inference
    labels = set()
    if cap.isOpened():
        success, image = cap.read()
        if not success:
            sys.exit(
                'ERROR: Unable to read from webcam. Please verify your webcam settings.'
            )
        image = cv2.flip(image, 1)

        # Run object detection estimation using the model.
        detections = detector.detect(image)
        for detection in detections:
            category = detection.categories[0]
            class_name = category.label
            probability = round(category.score, 2)
            if probability > prob_thresh:
                labels.add(class_name)
    cap.release()
    cv2.destroyAllWindows()
    return labels, image

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

def move_forward_10cm(n_10cm=1):
    # foward(1) range(15) sleep(0.1) min_power:20 achieve 10cm +- 1cm
    global total_10cm
    forward(1)
    x = 0
    slot = 17
    for i in range(slot*n_10cm):
        time.sleep(0.1)
    stop()

def monitoring():
    labels, picture = run_detector()
    print("labels: ", labels)
    if "mouse" in labels: # dog
        cv2.imwrite('picture.jpg', picture)
        cv2.imshow('picture',picture)
        print("write picture")
        return picture
    else:
        return None

"""Run"""
def main():
    while True:
        picture = monitoring()
        time.sleep(1)
        if picture:
            # send picture to PC->smart phone
            time.sleep(interval)


if __name__ == "__main__":
    main()
