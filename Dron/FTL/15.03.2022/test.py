#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from asyncio import events
import rospy
from gs_flight import FlightController, CallbackEvent
from gs_board import BoardManager
from gs_navigation import NavigationManager
import math
import cv2
from ctypes import *

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
bridge = CvBridge()
rospy.init_node("flight_test_node")  # инициализируем ноду.


def detect(img):
    global darknet_width, darknet_height, config_file, thresh, weights, data_file
    frame_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    frame_resized = cv2.resize(frame_rgb, (darknet_width, darknet_height),
                                    interpolation=cv2.INTER_LINEAR)
    darknet_image = darknet.make_image(darknet_width, darknet_height, 3)
    darknet.copy_image_from_bytes(darknet_image, frame_resized.tobytes())

    network, class_names, class_colors = darknet.load_network(
                config_file,
                data_file,
                weights,
                batch_size=1
            )
    detections = darknet.detect_image(network, class_names, darknet_image, thresh=thresh)
    return detections

def label_from_detection(detection):
    return detection[0]

def coords_from_detection(detection):
    return detection[2]


def get_img():
    data = rospy.wait_for_message("/pioneer_max_camera/image_raw/", Image)
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    return cv_image

callbacks = []


def callback(event): 
  global callbacks
  next_callbacks = []
  for callback in callbacks:
    if callback(event):
      next_callbacks.append(callback)

def wait_for_event(waiting_event):
  global callbacks

  ready = False
  def callback(event):
    nonlocal ready
    if event.data == waiting_event:
      ready = True
      return False
    else:
      return True
  callbacks.append(callback)
  while not ready:
    rospy.sleep(0.1)



def takeoff():
    global start_coords
    print("Preflight")
    # start_coords = (, 0)
    ap.preflight()
    wait_for_event(CallbackEvent.ENGINES_STARTED)
    print("Takeoff")
    ap.takeoff()
    wait_for_event(CallbackEvent.TAKEOFF_COMPLETE)
    print("Takeoff completed")


def fly_to_global_point(x, y, z):
    print(f"Navigating to x={x} y={y} z={z}")
    ap.goToLocalPoint(x, y, z)
    wait_for_event(CallbackEvent.POINT_REACHED)
    print("Arrived")

def land():
    print("Landing...")
    ap.landing()
    wait_for_event(CallbackEvent.COPTER_LANDED)
    print("Landed")


# def fly_to_local_point(x, y, z):
#     global kx, ky, xy_invertion
#     print(f"local - Navigating to x={x} y={y} z={z}")
#     if xy_invertion:
#         fly_to_global_point(start_coords[0] + y * ky, start_coords[1] + x * kx, z)
#     else:
#         fly_to_global_point(start_coords[1] + x * kx, start_coords[0] + y * ky, z)


def fly_to_local_point(x, y, z):
    global kx, ky, xy_invertion
    print(f"local - Navigating to x={x} y={y} z={z}")
    if xy_invertion:
        fly_to_global_point(start_coords[1] + y * ky, start_coords[0] + x * kx, z)
    else:
        fly_to_global_point(start_coords[0] + x * kx, start_coords[1] + y * ky, z)


def calibrate_local(x, y, tolerance):
    global kx, ky, xy_invertion
    print("Calibrating...")
    coords1 = nav.lps.position()
    fly_to_local_point(x, y, coords1[2])
    coords2 = nav.lps.position()
    # print(math.fabs(coords2[0] - coords1[0], math.fabs(coords2[0] - coords1[0]) > y - tolerance and math.fabs(coords2[0] - coords1[0]) < y + tolerance))
    if not (math.fabs(coords2[0] - coords1[0]) > y - tolerance and math.fabs(coords2[0] - coords1[0]) < y + tolerance):
        print("Inverting x and y")
        xy_invertion = 1
    # elif math.fabs(coords2[1] - coords1[1]) > x - tolerance and math.fabs(coords2[1] - coords1[1]) < x + tolerance
    #     print("Inverting x and y")
    #     xy_invertion = 1
    # if coords2[0] - coords1[0] < 0:
    #     if xy_invertion:
    #         ky *= -1
    #     else:
    #         kx *= -1
    # if coords2[1] - coords1[1] < 0:
    #     if xy_invertion:
    #         kx *= -1
    #     else:
    #         ky *= -1
    print("Callibrating finished, returning to the start point")
    fly_to_global_point(coords1[0], coords1[1], coords1[2])
    print("Returned to the start point")

nav = NavigationManager()
board = BoardManager()
ap = FlightController(callback)
start_coords = nav.lps.position()

# led.changeAllColor(0,0,255.0)
# cargo.changeAllColor(0, 255, 0)
# cargo.on()
# cargo.off()


import json



# #how many cubes you want to add
# count = 10



def get_points():
    global fly_upper, altitude, PATH_TO_OBJECTS_JSON
    with open(PATH_TO_OBJECTS_JSON, "r") as f:
        objects = json.load(f)
        res = []
        for i in [0, 1, 2, 4]:
            cX = objects[i]["position"]["x"] 
            cY = objects[i]["position"]["y"] 
            # pos_z = objects[i]["scale"]["z"] / 2
            scale_x_dev = objects[i]["scale"]["x"] * 0.3
            scale_y_dev = objects[i]["scale"]["y"] * 0.3
            # scale_z = objects[i]["scale"]["z"]
            if i == 2 or i == 4:
                res.append([cX, cY, altitude])
            else:
                if not fly_upper:
                    res.append([cX - scale_x_dev, cY - scale_y_dev, altitude]) # левый верхний угол
                    res.append([cX - scale_x_dev, cY + scale_y_dev, altitude]) # правый верхний угол
                    res.append([cX + scale_x_dev, cY + scale_y_dev, altitude]) # правый нижний угол
                    res.append([cX + scale_x_dev, cY - scale_y_dev, altitude]) # левый нижний угол
                else:
                    res.append([cX, cY, altitude])
                    res.append([cX, cY, altitude + high_delta])
                    res.append([cX, cY, altitude])
        return res


        


# 000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000

# ------------------------- COPTER -----------------------------
local_start = 1
#kx, ky, xy_invertion = 1, -1, 1
kx, ky, xy_invertion = 1, 1, 0
high_delta = 0.5
altitude = 1.5
inp_points = ([1.91, 1.82, altitude], [1.8, 3.8, altitude], [4.27, 3.59, altitude], [6.68, 1.52, altitude])
fly_upper = 0
PATH_TO_OBJECTS_JSON = "./objects.json"
points = get_points()
print(points)
# points = ([1.91, 1.82, altitude], [1.8, 3.8, altitude], [4.27, 3.59, altitude], [6.68, 1.52, altitude])
scale  = ([])
# points = ([2.46 - 0.5, 2.37 - 0.5, altitude], [2.3- 0.5, 4.5- 0.5, altitude], [4.62- 0.5, 3.865- 0.5, altitude], [7.43- 0.5, 2.27- 0.5, altitude + 1])
photo_path = "imgs/bbb_img{}-{}.jpg"
# ------------------------- COPTER -----------------------------

#=====================================================================================================================================

# ------------------------ DETECTOR ----------------------------

enable_detector = 0
darknet_width = 416
darknet_height = 416

config_file = "~/yolo.cfg"
thresh = 0.7
weights = "~/yolo_last.weights"
data_file = "~/obj.data"

# ------------------------ DETECTOR ----------------------------



# 000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000

if not local_start:
    from gs_module import CargoController
    from gs_module import BoardLedController
    led = BoardLedController()
    cargo = CargoController()


if enable_detector:
    import darknet

print("Start coords: ", start_coords)
if not local_start:
    cargo.on()
takeoff()
if not local_start:
    cargo.changeAllColor(255,0,0)
    rospy.sleep(10)
    cargo.changeAllColor(0,0,0)

fly_to_global_point(start_coords[0], start_coords[1], altitude)
for n in range(len(points)):
    x, y, z =  points[n]
    fly_to_global_point(x, y, z)
    for i in range(10):
        cv2.imwrite(photo_path.format("-upper-" + str(n) ,i), get_img())
    if enable_detector:
        res = detect(get_img())
        for a in res:
            print(label_from_detection(a))

    # rospy.sleep(5)

if not local_start:
    cargo.off()
land()
# fly_to_point([2,2,3])
# fly_to_point([0,0,5])
# land()
# (2.5, 2.5, 1)
# (5, 3 , 1)