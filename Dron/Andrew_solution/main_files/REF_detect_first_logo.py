#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
from gs_flight import FlightController, CallbackEvent
from gs_module import CargoController
from gs_board import BoardManager
from threading import Timer
from cv.detection import Detector
from std_msgs.msg import String

rospy.init_node("your_node_name") # инициализация ноды

#загрузка модели
detector = Detector(        #инициализация весов для yolov4
    classnames_path='...',
    model_cfg='...',
    model_weights='...',
)


first_building_point = [1.5, 3.8, 2.4] # координаты здания №1

run = True
position_number = 0

bridge = CvBridge()

#запись видео
fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
out = cv2.VideoWriter('output5.avi', fourcc, 30.0, (1280, 720))


def after_timer():
    print("Выключение светодиодной индикации")
    cargo.changeAllColor(0, 0, 0)
    print("Пролёт до крыши здания №1")
    ap.goToLocalPoint(*first_building_point)


def end_task():
    print("Выключение светодиодной индикации.")
    cargo.changeAllColor(0, 0, 0)
    ap.landing()
    t = Timer(5.0, cargo.off)
    t.start()


def callback(event):  # функция обработки событй Автопилота
    global ap
    global run
    global coordinates
    global position_number
    global frame
    global pub

    event = event.data
    if event == CallbackEvent.ENGINES_STARTED:  # блок обработки события запуска двигателя
        print("engine started")
        ap.takeoff()  # отдаем команду взлета
        print("Взлет с крыши распределительного хаба на произвольную высоту вместе захваченным грузом")

    elif event == CallbackEvent.TAKEOFF_COMPLETE:  # блок обработки события завершения взлета
        print("Включение светодиодной индикации (цвет – красный)")
        cargo.changeAllColor(255, 0, 0)
        position_number = 0

        print("Зависание на 10 секунд")
        t = Timer(10.0, after_timer)
        t.start()

    elif event == CallbackEvent.POINT_REACHED:  # блок обработки события достижения точки
    
        '''
        Работа нейронной сети
        '''
        
        print("Детектирование логотипа получателя груза")
        bboxes = []
        while len(bboxes) == 0:
            #bboxes, detector_frame = detector.forward(frame)
            #out.write(cv2.resize(detector_frame, (1280, 720)))
            #print(bboxes[0].name)
            #pub.publish(bboxes[0].name)

            print("Включение светодиодной индикации (цвет – зелёный)")
            cargo.changeAllColor(0, 255, 0)
            print("Зависание на 10 секунд")
            t = Timer(10.0, lambda: end_task)
            t.start()

    elif event == CallbackEvent.COPTER_LANDED:  # блок обработки события приземления
        out.release()
        print("finish programm")
        run = False  # прекращем программу


def getVideo(data):
    '''
    Прием картинки
    '''
    global bridge
    global frame

    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        frame = cv2.resize(frame, (1280, 720))
    except Exception as e:
        rospy.loginfo(str(e))


board = BoardManager()  # создаем объект бортового менеджера
ap = FlightController(callback)  # создаем объект управления полета
cargo = CargoController() # создаем объект управления магнитного захвата

once = False  # переменная отвечающая за первое вхождение в начало программы

while not rospy.is_shutdown() and run:
    if board.runStatus() and not once:  # проверка подлкючения RPi к Пионеру
        print("Включение магнитного захвата")
        cargo.on()
        pub = rospy.Publisher('/label', String, queue_size=0)
        rospy.Subscriber("/pioneer_max_camera/image_raw", Image, getVideo)
        ap.preflight()  # отдаем команду выполенения предстартовой подготовки
        once = True
    pass
