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
import std_msgs.msg

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
bridge = CvBridge()
rospy.init_node("flight_test_node")  # инициализируем ноду.
pub = rospy.Publisher('/label', std_msgs.msg.String, queue_size=10)
num = 0
def detect(img, network):
    global darknet_width, darknet_height, config_file, thresh, weights, data_file, num
    num += 1
    img_size = img.shape

    frame_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    frame_resized = cv2.resize(frame_rgb, (darknet_width, darknet_height),
                                    interpolation=cv2.INTER_LINEAR)
    darknet_image = darknet.make_image(darknet_width, darknet_height, 3)
    darknet.copy_image_from_bytes(darknet_image, frame_resized.tobytes())

    network, class_names, class_colors = network
    detections = darknet.detect_image(network, class_names, darknet_image, thresh=thresh)
    res = []
    for detection in detections:
        label, acc, coords = detection
        cX, cY, w, h = coords
        cX = cX / darknet_width * img_size[1]
        w  = w / darknet_width * img_size[1]
        cY = cY / darknet_height * img_size[0]
        h  = h / darknet_height * img_size[0]
        res.append([label, acc, [cX, cY, w, h]])
    print(f"detection {num}")
    # print(detections)
    # print(img.shape)
    cv2.imwrite(f"detection{num}.png", img)
    return res

def label_from_detection(detection):
    return detection[0]

def coords_from_detection(detection):
    return detection[2]


def get_img():
    rospy.sleep(0.5)
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


                    # point.append([cX - scale_x_dev, cY - scale_y_dev]) # левый верхний угол
                    # point.append([cX - scale_x_dev, cY + scale_y_dev]) # правый верхний угол
                    # point.append([cX + scale_x_dev, cY + scale_y_dev]) # правый нижний угол
                    # point.append([cX + scale_x_dev, cY - scale_y_dev]) # левый нижний угол

def is_point_in_rect (point, rect):
    b = (rect[0][0] < point[0] < rect[2][0]) and (rect[0][1] < point[1] < rect[2][1])
    return b


def fly_to_global_point(x, y, z, rect_zone=[]):
    print(f"Navigating to x={x} y={y} z={z}")
    if rect_zone != []:
        should_countinue = is_point_in_rect([x, y, z], rect_zone)
        if not should_countinue:
            print(f" - Navigation canceled, point {(x,y)} is not in rect_zone {rect_zone}")
            return False
    ap.goToLocalPoint(x, y, z)
    wait_for_event(CallbackEvent.POINT_REACHED)
    print(" - Arrived")
    return True

def land():
    print("Landing...")
    ap.landing()
    wait_for_event(CallbackEvent.COPTER_LANDED)
    print(" - Landed")

def disarm():
    print("Landing...")
    ap.disarm()
    time.sleep(0.5)
    print(" - Landed")


# def fly_to_local_point(x, y, z):
#     global kx, ky, xy_invertion
#     print(f"local - Navigating to x={x} y={y} z={z}")
#     if xy_invertion:
#         fly_to_global_point(start_coords[0] + y * ky, start_coords[1] + x * kx, z)
#     else:
#         fly_to_global_point(start_coords[1] + x * kx, start_coords[0] + y * ky, z)


def fly_to_relative_point(x, y, z, rect_zone):
    global kx, ky, xy_invertion
    coords = nav.lps.position()
    print(coords, (coords[0] - x, coords[1] - y, z))
    print(f"local navigating to x={x} y={y} z={z}")

    fly_to_global_point(x + coords[0], y + coords[1], z, rect_zone)

def fly_to_local_point(x, y, z, rect_zone=[]):
    fly_to_relative_point(-y, x, z, rect_zone)


def calibrate_local(x, y, tolerance):
    global kx, ky, xy_invertion
    print("Calibrating...")
    coords1 = nav.lps.position()
    # fly_to_local_point(x, y, coords1[2])
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
    #     else:cente
    #         ky *= -1
    print("Callibrating finished, returning to the start point")
    fly_to_global_point(coords1[0], coords1[1], coords1[2])
    print("Returned to the start point")

nav = NavigationManager()
board = BoardManager()
ap = FlightController(callback)

# led.changeAllColor(0,0,255.0)
# cargo.changeAllColor(0, 255, 0)
# cargo.on()
# cargo.off()


import json


def capture(name):
    print(f"captured {name}")
    cv2.imwrite(name, get_img())

def get_points():
    global fly_upper, altitude, PATH_TO_OBJECTS_JSON, high_delta1, high_delta2, need_edges_coords, edges_coords
    with open(PATH_TO_OBJECTS_JSON, "r") as f:
        objects = json.load(f)
        res = []
        for i in [1, 0, 2, 4]:
            cX = objects[i]["position"]["x"]
            cY = objects[i]["position"]["y"]
            # pos_z = objects[i]["scale"]["z"] / 2
            scale_x_dev = objects[i]["scale"]["x"]
            scale_y_dev = objects[i]["scale"]["y"]
            # scale_z = objects[i]["scale"]["z"]
            if i == 2 or i == 4:
                res.append([[cX, cY, altitude], 0])
            else:
                if not fly_upper:
                    # res.append([[cX - (scale_x_dev - 0.20), cY - (scale_y_dev - 0.2), altitude], 0])
                    # res.append([[cX - (scale_x_dev - 0.20), cY - (scale_y_dev - 0.2), altitude + high_delta2], 1]) # левый верхний угол
                    # res.append([[cX - (scale_x_dev - 0.20), cY + (scale_y_dev - 0.2), altitude + high_delta2], 1]) # правый верхний угол
                    # res.append([[cX + (scale_x_dev - 0.20), cY + (scale_y_dev - 0.2), altitude + high_delta2], 1]) # правый нижний угол
                    # res.append([[cX + (scale_x_dev - 0.20), cY - (scale_y_dev - 0.2), altitude + high_delta2], 1]) # левый нижний угол
                    # res.append([[cX + (scale_x_dev - 0.20), cY - (scale_y_dev - 0.2), altitude], 0])

                    res.append([[cX - scale_x_dev * 0.3, cY - scale_y_dev * 0.3, altitude], 0])
                    res.append([[cX - scale_x_dev * 0.3, cY - scale_y_dev * 0.3, altitude + high_delta2], 1]) # левый верхний угол
                    res.append([[cX - scale_x_dev * 0.3, cY + scale_y_dev * 0.3, altitude + high_delta2], 1]) # правый верхний угол
                    res.append([[cX + scale_x_dev * 0.3, cY + scale_y_dev * 0.3, altitude + high_delta2], 1]) # правый нижний угол
                    res.append([[cX + scale_x_dev * 0.3, cY - scale_y_dev * 0.3, altitude + high_delta2], 1]) # левый нижний угол
                    res.append([[cX + scale_x_dev * 0.3, cY - scale_y_dev * 0.3, altitude], 0])
                else:
                    res.append([[cX, cY, altitude], 0])
                    res.append([[cX, cY, altitude + high_delta1], 1])
                    res.append([[cX, cY, altitude], 0])
                if need_edges_coords:
                    point = []
                    dev = 0.2
                    point.append([cX - scale_x_dev - dev, cY - scale_y_dev - dev]) # левый верхний угол
                    point.append([cX - scale_x_dev - dev, cY + scale_y_dev + dev]) # правый верхний угол
                    point.append([cX + scale_x_dev + dev, cY + scale_y_dev + dev]) # правый нижний угол
                    point.append([cX + scale_x_dev + dev, cY - scale_y_dev - dev]) # левый нижний угол
                    edges_coords.append(point)


        return res


def center(coords, point_i, depth=0):
    global num, centering_accuracy, label_we_should_found
    # rospy.sleep(5)
    try:
        print("Centering...")
        print(f" - depth={depth}")
        print(f" - Center coords {coords}")
        kx1, ky1 = 1, 1
        cX, cY, w, h = coords
        img = get_img()

        res = detect(img, network)
        if len(res) > 0:
            nearest_label = min(res, key=lambda x: ((x[2][0] - 320) ** 2 + (x[2][1] - 240) ** 2) ** 0.5)
            print ("nearest label: ", nearest_label)
            # labels = [label_from_detection(nearest_label)]

            if label_from_detection(nearest_label) == label_we_should_found:
                coords1 = coords_from_detection(nearest_label)
                cX, cY, w, h = coords1
                cX, cY = round(cX), round(cY)
                print(" - Detected: " + label_from_detection(nearest_label))

                middleX, middleY = round(img.shape[1] / 2), round(img.shape[0] / 2)
                # y, x = img.shape[:2]
                case = -100
                print ("coords",cX, cY)
                capture(f"before_centering-{num}.jpg")
                if middleY - cY > 0 and middleX - cX > 0:
                    case = 1

                    kx1 =  -1
                    ky1 =  1
                elif middleY - cY > 0 and middleX - cX < 0:
                    case = 2

                    kx1 = 1
                    ky1 = 1
                elif middleY - cY < 0 and middleX - cX < 0:
                    case = 3


                    kx1 = 1
                    ky1 = -1
                elif middleY - cY < 0 and middleX - cX > 0:
                    case = 4

                    kx1 = -1
                    ky1 = -1

                # px_in_cm = 12.58474576271186
                cm_in_px = 0.1116541
                dx = math.fabs(cX - middleX)
                dy = math.fabs(cY - middleY)
                print("dx, dy = ", dx, dy)
                if (dx < centering_accuracy * cm_in_px and dy < centering_accuracy * cm_in_px) or depth > 3:
                    return
                print(f" - case {case}")
                fly_to_local_point(kx1 * (cm_in_px * dx / 100), ky1 * (cm_in_px * dy / 100), nav.lps.position()[2], edges_coords[point_i])
                # fly_to_local_point(kx1 * (cm_in_px * dx / 100), ky1 * (cm_in_px * dy / 100), nav.lps.position()[2], [])

                # rospy.sleep(5)
                capture(f"after_centering-{num}.jpg")
                num += 1
        if depth > 3:
            return
        center(coords, point_i, depth + 1)
    except Exception as e:
        print(f" - Error while centering occured: {e}")
        return



edges_coords = []
# 000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000

# ------------------------- COPTER -----------------------------

# AT SUBTASK 4 should_center=1, just_find_label_at_one_of_two_buildings=1 !!!!!!!!!!!!!!!!

local_start = 0
centering_accuracy = 2                                              # in cm
should_countinue_after_finding_label_at_one_of_two_buildings = 1     # subtask5
should_center  = 1                                                   # subtask4

label_we_should_found = "olymp"                                      # subtask3, subtask4

just_find_label_at_one_of_two_buildings = 1                          # subtask3, subtask4
just_detect_label_on_one_building = 0                                # subtask2

send_all_preds = 0

# kx, ky, xy_invertion = 1, -1, 1
# kx, ky, xy_invertion = 1, 1, 0
high_delta1, high_delta2 = 1, 0.05
altitude = 1.5
# inp_points = ([1.91, 1.82, altitude], [1.8, 3.8, altitude], [4.27, 3.59, altitude], [6.68, 1.52, altitude])
fly_upper = 0
PATH_TO_OBJECTS_JSON = "./objects.json"
need_edges_coords = 1
points = get_points()
print(points, "\n\n")
photo_path = "imgs/try12_img{}-{}.jpg"

# ------------------------- COPTER -----------------------------

#=====================================================================================================================================

# ------------------------ DETECTOR ----------------------------

enable_detector = 1
darknet_width = 416
darknet_height = 416

config_file = "./nn/yolo.cfg"
thresh = 0.5
weights = "./nn/yolo.weights"
data_file = "./nn/obj.data"

# ------------------------ DETECTOR ----------------------------

# 000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000

if enable_detector:
    import darknet
    network = darknet.load_network(
                    config_file,
                    data_file,
                    weights,
                    batch_size=1
                )


from threading import Thread
import time
class CountdownTask:
    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False

    def run(self):
        while self._running:
            print('LED: red')
            cargo.changeAllColor(255,0,0)
            time.sleep(1)
            print('LED: none')
            time.sleep(1)
            cargo.changeAllColor(0,0,0)


start_coords = nav.lps.position()
if not local_start:
    from gs_module import CargoController
    from gs_module import BoardLedController
    led = BoardLedController()
    cargo = CargoController()



print("\n\n\nStart coords: ", start_coords)
if not local_start:
    cargo.on()
    print("Cargo enabled")


takeoff()
if not local_start:
    cargo.changeAllColor(255,0,0)
    rospy.sleep(20)
    print("waiting")
    # time.sleep(10)
    print("finish")
    cargo.changeAllColor(0,0,0)

fly_to_global_point(start_coords[0], start_coords[1], altitude)
for point_n in range(len(points)):
    x, y, z =  points[point_n][0]
    need_detection = points[point_n][1]
    fly_to_global_point(x, y, z)
    for i in range(2):
        cv2.imwrite(photo_path.format("-upper-" + str(point_n) ,i), get_img())
    if enable_detector and need_detection:
        print("Detecting...")
        res = detect(get_img(), network)
        if len(res) > 0:
            nearest_label = min(res, key=lambda x: ((x[2][0] - 320) ** 2 + (x[2][1] - 240) ** 2) ** 0.5)
            print ("nearest label: ", nearest_label)
            labels = [label_from_detection(nearest_label)]
            print(" - Detected: " + label_from_detection(nearest_label))
            if send_all_preds:
                pub.publish(std_msgs.msg.String(labels[0]))
            if just_detect_label_on_one_building:

                if not local_start:
                    cargo.changeAllColor(0,255,0)
                    print(" - Sending label: " +labels[0])
                    if not send_all_preds:
                        pub.publish(std_msgs.msg.String(labels[0]))
                    rospy.sleep(10)
                    cargo.changeAllColor(0,0,0)
                else:
                    print(" - Sending label: " + labels[0])
                    if not send_all_preds:
                        pub.publish(std_msgs.msg.String(labels[0]))
                    rospy.sleep(10)
                print("Finished")
                break
            if just_find_label_at_one_of_two_buildings:
                if label_we_should_found in labels:

                    if not local_start:
                        cargo.changeAllColor(0,255,0)
                        print(" - Sending label: " + labels[0])
                        pub.publish(std_msgs.msg.String(labels[0]))
                        # print(labels[0])
                        rospy.sleep(10)
                        cargo.changeAllColor(0,0,0)
                    else:
                        print(" - Sending label: " + labels[0])
                        pub.publish(std_msgs.msg.String(labels[0]))
                        # labels[0]
                        rospy.sleep(10)
                    if not should_center:
                        print("Finished")
                        break
                    else:
                        print(point_n, len(edges_coords), edges_coords)


                        center(coords_from_detection(nearest_label), point_n)


                        if not local_start:
                            cargo.off()
                            print("Cargo turned off")
                            rospy.sleep(20)
                            print("Cargo turned off")
                        if not should_countinue_after_finding_label_at_one_of_two_buildings:
                            break
                        else:
                            fly_to_global_point(start_coords[0], start_coords[1], altitude)
                            fly_to_global_point(6.68, 1.52, altitude)
                            if not local_start:
                                c = CountdownTask()
                                t = Thread(target = c.run, args =())
                                t.start()
                            center(coords_from_detection(nearest_label), point_n)
                            rospy.sleep(5)
                            disarm()
                            if not local_start:
                                c.terminate()
                                t.join()
            # if (center_according_to_label in labels):
            #     center(res[], point_n)
        else:
           print(" - Nothing detected")
    print("\n")




if not should_countinue_after_finding_label_at_one_of_two_buildings:
    land()
