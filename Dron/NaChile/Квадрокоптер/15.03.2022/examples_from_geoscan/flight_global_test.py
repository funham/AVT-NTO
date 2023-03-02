#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gs_flight import FlightController, CallbackEvent
from gs_board import BoardManager
from gs_navigation import NavigationManager

rospy.init_node("flight_global_test_node") # инициализируем ноду
gps = NavigationManager().gps # создаем объект для получения информации с глобальной системе координат
start_pos = gps.position() # получаем стартовые координаты

coordinates = [ # расчитываем новые координаты
    [start_pos[0], start_pos[1] + 0.00001, 20],
    [start_pos[0] + 0.00001, start_pos[1] + 0.00001, 20],
    [start_pos[0] + 0.00001, start_pos[1], 20],
    [start_pos[0], start_pos[1], 20],
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
        print(f"point {position_number} start")
        ap.goToPoint(coordinates[position_number][0], coordinates[position_number][1], coordinates[position_number][2]) # отдаем команду полета в точку
    elif event == CallbackEvent.POINT_REACHED: # блок обработки события достижения точки
        print(f"point {position_number} reached")
        position_number += 1 # наращиваем счетчик точек
        if position_number < len(coordinates): # проверяем количество текущее количество точек с количеством точек в полетном задании
            ap.goToPoint(coordinates[position_number][0], coordinates[position_number][1], coordinates[position_number][2]) # отдаем команду полета в точку
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
