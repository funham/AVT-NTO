#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gs_flight import FlightController, CallbackEvent
from gs_board import BoardManager

rospy.init_node("flight_test_node")  # инициализируем ноду

coordinates = [ # массив координат точек
    [0,1,1],
    [1,1,1],
    [1,0,1],
    [0,0,1]
]

run = True # переменная отвечающая за работу программы
position_number = 0 # счетчик пройденных точек

def callback(event): # функция обработки событй Автопилота
    global ap
    global run
    global coordinates
    global position_number

    event = event.data
    if event == CallbackEvent.ENGINES_STARTED: # блок обработки события запуска двигателя
        print("engine started")
        ap.takeoff() # отдаем команду взлета
    elif event == CallbackEvent.TAKEOFF_COMPLETE: # блок обработки события завершения взлета
        print("takeoff complite")
        position_number = 0
        ap.goToLocalPoint(coordinates[position_number][0], coordinates[position_number][1], coordinates[position_number][2]) # отдаем команду полета в точку
    elif event == CallbackEvent.POINT_REACHED: # блок обработки события достижения точки
        print(f"point {position_number} reached")
        position_number += 1 # наращиваем счетчик точек
        if position_number < len(coordinates): # проверяем количество текущее количество точек с количеством точек в полетном задании
            ap.goToLocalPoint(coordinates[position_number][0], coordinates[position_number][1], coordinates[position_number][2]) # отдаем команду полета в точку
        else:
            ap.landing() # отдаем команду посадки
    elif event == CallbackEvent.COPTER_LANDED: # блок обработки события приземления
        print("finish programm")
        run = False # прекращем программу

board = BoardManager() # создаем объект бортового менеджера
ap = FlightController(callback) # создаем объект управления полета

once = False # переменная отвечающая за первое вхождение в начало программы

while not rospy.is_shutdown() and run:
    if board.runStatus() and not once: # проверка подлкючения RPi к Пионеру
        print("start programm")
        ap.preflight() # отдаем команду выполенения предстартовой подготовки
        once = True
    pass
