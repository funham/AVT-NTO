#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gs_flight import FlightController, CallbackEvent
from gs_module import CargoController
from gs_board import BoardManager
from threading import Timer

rospy.init_node("flight_test_node")  # инициализируем ноду

run = True # переменная отвечающая за работу программы

def stop():
    cargo.changeAllColor(255, 0, 0)
    ap.landing()

def callback(event): # функция обработки событй Автопилота
    global ap
    global run

    event = event.data
    if event == CallbackEvent.ENGINES_STARTED: # блок обработки события запуска двигателя
        print("Copter armed")
        ap.takeoff() # отдаем команду взлета
    elif event == CallbackEvent.TAKEOFF_COMPLETE: # блок обработки события завершения взлета
        print("Takeoff completed")
        t = Timer(10.0, stop)
        t.start()
        cargo.changeAllColor(255, 0, 0)
    elif event == CallbackEvent.COPTER_LANDED: # блок обработки события приземления
        print("Flight plan finished")
        run = False # прекращем программу

board = BoardManager() # создаем объект бортового менеджера
cargo = CargoController()
ap = FlightController(callback) # создаем объект управления полета

once = False # переменная отвечающая за первое вхождение в начало программы

while not rospy.is_shutdown() and run:
    if board.runStatus() and not once: # проверка подлкючения RPi к Пионеру
        print("Starting...")
        cargo.on()
        ap.preflight() # отдаем команду выполенения предстартовой подготовки
        once = True
    pass
