#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
from gs_flight import FlightController, CallbackEvent
from gs_board import BoardManager
from threading import Timer

rospy.init_node("flight_test_node")  # инициализируем ноду


coordinates = [  # массив координат точек
    [4.27, 3.59, 1.3],
    [4.27, 3.59, 2.1],
    
    [1.8, 3.8, 2], 
    [1.8, 3.8, 1.2],
    [1.8, 3.8, 2], 

    [1.91, 1.82 , 2],
    [1.91, 1.82 , 1], # 2

    [1.91, 3, 2],

    [6.68, 3, 2],
    [6.68, 1.52 , 2],

    # [4.17, 1.82 , 2.3],
    # [4.17, 1.82 , 1.9],

    # [6.68, 1.52, 1.9],
    # [6.68, 1.52, 1.2],
    # [6.68, 1.52, 2],

    [4.27, 3.59, 2],
]

run = True  # переменная отвечающая за работу программы
position_number = 0  # счетчик пройденных точек

bridge = CvBridge()

fourcc = cv2.VideoWriter_fourcc('M','J','P','G')

frame_width = int(1280)
frame_height = int(720)
size = (frame_width, frame_height)
out = cv2.VideoWriter('output.avi', fourcc, 30.0, size)

def callback(event):  # функция обработки событй Автопилота
    global ap
    global run
    global coordinates
    global position_number

    event = event.data
    if event == CallbackEvent.ENGINES_STARTED:  # блок обработки события запуска двигателя
        print("engine started")
        ap.takeoff()  # отдаем команду взлета
    elif event == CallbackEvent.TAKEOFF_COMPLETE:  # блок обработки события завершения взлета
        print("takeoff complite")
        position_number = 0
        ap.goToLocalPoint(coordinates[position_number][0], coordinates[position_number]
                          [1], coordinates[position_number][2])  # отдаем команду полета в точку
    elif event == CallbackEvent.POINT_REACHED:  # блок обработки события достижения точки
        print(f"point {position_number} reached")
        position_number += 1  # наращиваем счетчик точек
        # проверяем количество текущее количество точек с количеством точек в полетном задании
        if position_number < len(coordinates):
            t = Timer(3.0, lambda: ap.goToLocalPoint(
                coordinates[position_number][0], coordinates[position_number][1], coordinates[position_number][2]))
            t.start()
            # ap.goToLocalPoint(coordinates[position_number][0], coordinates[position_number][1], coordinates[position_number][2]) # отдаем команду полета в точку
        else:
            ap.landing()  # отдаем команду посадки
    elif event == CallbackEvent.COPTER_LANDED:  # блок обработки события приземления
        out.release()
        print("finish programm")
        run = False  # прекращем программу


def getVideo(data):
    global bridge
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        frame = cv2.resize(frame, (1280, 720))
        out.write(frame)
    except Exception as e:
        rospy.loginfo(str(e))


board = BoardManager()  # создаем объект бортового менеджера
ap = FlightController(callback)  # создаем объект управления полета

once = False  # переменная отвечающая за первое вхождение в начало программы

while not rospy.is_shutdown() and run:
    if board.runStatus() and not once:  # проверка подлкючения RPi к Пионеру
        print("start programm")
        img_topic = rospy.get_param(rospy.search_param("image_raw"))
        rospy.Subscriber(img_topic, Image, getVideo)
        ap.preflight()  # отдаем команду выполенения предстартовой подготовки
        once = True
    pass
