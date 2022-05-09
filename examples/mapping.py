from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
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

"""Mapping"""
ANGLE_RANGE = 180
STEP = 18
us_step = STEP
angle_distance = [0,0]
current_angle = 0
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2
scan_list = []

clearance = 0 # 0 or 1 or 2
dist_acc = 0.3 # 0.3-1
scanRound = 2

def get_distance_at(angle):
    global angle_distance
    servo.set_angle(angle)
    time.sleep(0.04)
    distance = us.get_distance()
    angle_distance = [angle, distance]
    return distance

def scan_step_angle_dist():
    global scan_list, current_angle, us_step
    current_angle += us_step
    if current_angle >= max_angle:
        current_angle = max_angle
        us_step = -STEP
    elif current_angle <= min_angle:
        current_angle = min_angle
        us_step = STEP

    scan_list.append((current_angle, get_distance_at(current_angle)))
    if current_angle == min_angle or current_angle == max_angle:
        if us_step < 0:
            # print("reverse")
            scan_list.reverse()
        # print(scan_list)
        tmp = scan_list.copy()
        scan_list = []
        return tmp
    else:
        return False

def min_index(index):
    return max(index-clearance, 0)
def max_index(index):
    return min(index+clearance, grid_size-1)+1

def mapping(current_x, current_y, offset, grid):
    angle_dists = defaultdict(list)
    for i in range(10*scanRound):
        scan_list = scan_step_angle_dist()
        if not scan_list:
            continue
        for item in scan_list:
            angle, dist = item
            angle_dists[int(angle)].append(dist)
    angle_distAvg = dict()
    for angle, dists in angle_dists.items():
        angle_distAvg[angle] = np.mean(dists) / gran
    print(angle_distAvg)
    pre_x, pre_y, pre_dist = None, None, None
    for i, (angle, dist) in enumerate(angle_distAvg.items()):
        obstacle_x = int(current_x - np.sin(np.deg2rad(angle+offset)) * dist)
        obstacle_y = int(current_y + np.cos(np.deg2rad(angle+offset)) * dist)
        line = [(obstacle_x, obstacle_y)]
        if pre_dist and abs(dist - pre_dist) <= dist_acc:
            xp, yp = list(zip(*sorted([(obstacle_x, obstacle_y), (pre_x, pre_y)])))
            for i in range(xp[0]+1, xp[1]):
                line.append((i, int(np.interp(i, xp, yp))))
        for x, y in line:
            if x >= 0 and x < grid_size and y >= 0 and y < grid_size:
                if clearance == 0:
                    grid[x, y] = 1
                else:
                    grid[min_index(x):max_index(x), min_index(y):max_index(y)] = 1
        pre_x, pre_y = obstacle_x, obstacle_y
        pre_dist = dist


"""A* search"""
gran = 10 # 10; 10cm can be achieved
grid_size = 50 // gran # 240//gran
start_x, start_y, start_angle = grid_size//2, 0, 0
target_x, target_y = start_x+0//gran, start_y+40//gran

START = 5
TARGET = 7
PATH = 4

def manhattanDist(x, y):
    return abs(x-target_x) + abs(y-target_y)

def angleDiff(pt1, pt2, angle):
    diff = abs(angle[pt1] - angle[pt2])
    return diff if diff < 270 else 90

def betternThanCur(pt, cur, g, angle, parent):
    if not cur:
        return True
    if g[pt]+manhattanDist(pt[0], pt[1]) < g[cur]+manhattanDist(cur[0], cur[1]):
        return True
    if g[pt]+manhattanDist(pt[0], pt[1]) == g[cur]+manhattanDist(cur[0], cur[1]) and \
        angleDiff(pt, parent[pt], angle) < angleDiff(cur, parent[cur], angle):
        return True
    return False

def neighbors(x, y, close, grid):
    result = []
    neighbor_offsets = [(0, 1, 0), (-1, 0, 90), (0, -1, 180), (1, 0, 270)]
    for x_offset, y_offset, ang in neighbor_offsets:
        new_x, new_y = x+x_offset, y+y_offset
        if new_x >= 0 and new_x < grid_size and new_y >= 0 and new_y < grid_size and grid[new_x, new_y] in [0, TARGET, PATH]:
            result.append(((new_x, new_y), ang))
    return result

def a_star(x, y, ang, grid):
    open = {(x, y)}
    close = set()
    g = {(x, y):0}
    angle = {(x, y): ang}
    parent = {(x, y):(x, y)}
    while (open):
        cur = None
        for pt in open:
            if betternThanCur(pt, cur, g, angle, parent):
                cur = pt
        if cur[0] == target_x and cur[1] == target_y:
            reconst_path = []
            while parent[cur] != cur:
                reconst_path.append(cur)
                cur = parent[cur]
                grid[cur[0], cur[1]] = PATH
            reconst_path.append((x, y))
            reconst_path.reverse()
            return reconst_path
        for nbr, ang in neighbors(cur[0], cur[1], close, grid):
            g_cur_nbr = g[cur] + 1
            if nbr not in open and nbr not in close:
                open.add(nbr)
                parent[nbr] = cur
                g[nbr] = g_cur_nbr
                angle[nbr] = ang
            else:
                if g_cur_nbr < g[nbr]:
                    g[nbr] = g_cur_nbr
                    parent[nbr] = cur
                    angle[nbr] = ang
                    if nbr in close:
                        close.remove(nbr)
                        open.add(nbr)
        open.remove(cur)
        close.add(cur)
    return []


"""Routing"""
left_front = Motor(PWM("P13"), Pin("D4"), is_reversed=True) # motor 1
right_front = Motor(PWM("P12"), Pin("D5"), is_reversed=True) # motor 2
left_rear = Motor(PWM("P8"), Pin("D11"), is_reversed=True) # motor 3
right_rear = Motor(PWM("P9"), Pin("D15"), is_reversed=True) # motor 4

map_every_gran = 3 # 5
detect_every_gran = 2 # detect stop sign every 2 grans; detect person every 10cm
total_10cm = 0
Nscreenshot = 3
prob_thresh = 0.3
person_wait_time = 5 # every 5s until person is gone
stop_wait_time = 2 # wait for some time and continue forwarding

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
    i = 0
    labels = set()
    while cap.isOpened() and i < Nscreenshot:
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
        i += 1
    cap.release()
    cv2.destroyAllWindows()
    return labels

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
    global total_10cm
    forward(1)
    x = 0
    slot = 17
    for i in range(slot*n_10cm):
        time.sleep(0.1)
        speed = speed_val()
        x += speed * 0.1
        # print("%scm/s"%speed)
        if (i+1) % slot == 0:
            total_10cm += 1
            while True:
                labels = run_detector()
                print("labels: ", labels)
                if "person" in labels:
                    stop()
                    time.sleep(person_wait_time)
                else:
                    forward(1)
                    break
            if total_10cm % detect_every_gran == 0:
                # print("total_10cm: ", total_10cm)
                stop_flag = False
                while True:
                    labels = run_detector()
                    print("labels: ", labels)
                    if not stop_flag and "stop sign" in labels:
                        stop()
                        time.sleep(stop_wait_time)
                        stop_flag = True
                    else:
                        forward(1)
                        break
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

def routing(path, ang_cur):
    offset_angle = {(0, 1): 0, (-1, 0): 90, (0, -1): 180, (1, 0): 270}
    n_10cm = 0
    for i in range(1, len(path)):
        x_offset = path[i][0] - path[i-1][0]
        y_offset = path[i][1] - path[i-1][1]
        ang_next = offset_angle[(x_offset, y_offset)]
        diff = ang_next - ang_cur
        diff %= 360
        if diff == 0:
            n_10cm += 1
        else:
            move_forward_10cm(n_10cm)
            n_10cm = 1
            if diff == 270:
                turn_right_90()
                ang_cur -= 90
            elif diff == 90:
                turn_left_90()
                ang_cur += 90
            else:
                turn_180()
                ang_cur -= 180
            ang_cur %= 360
    move_forward_10cm(n_10cm)
    return ang_cur


"""Run"""
def erase_grid(x, y, ang, grid):
    if ang == 0:
        grid[:, y:] == 0
    elif ang == 90:
        grid[0:x+1, :] = 0
    elif ang == 180:
        grid[:, :y+1] = 0
    else:
        grid[x:, :] = 0

def main():
    start_speed_thread()
    current_x, current_y, current_angle = start_x, start_y, start_angle
    grid = np.zeros((grid_size, grid_size), dtype=np.int8)
    mapping(current_x, current_y, current_angle, grid)
    print(np.rot90(grid))
    path = a_star(current_x, current_y, current_angle, grid)[:map_every_gran+1]
    grid[current_x, current_y] = START
    grid[target_x, target_y] = TARGET
    print(np.rot90(grid))
    print(path)
    while (current_x, current_y) != (target_x, target_y):
        current_angle = routing(path, current_angle)
        current_x, current_y = path[-1]
        erase_grid(current_x, current_y, current_angle, grid)
        mapping(current_x, current_y, current_angle, grid)
        grid = np.where(grid == PATH, 0, grid)
        print(np.rot90(grid))
        path = a_star(current_x, current_y, current_angle, grid)
        grid[current_x, current_y] = START
        grid[target_x, target_y] = TARGET
        print(np.rot90(grid))
        print(path)
    end_speed_thread()


if __name__ == "__main__":
    main()
