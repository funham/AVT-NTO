#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gs_board import BoardManager
from rospy import sleep

rospy.init_node("board_test_node") # инициализируем ноду
board = BoardManager() # создаем объект бортового менеджера

run = True

while not rospy.is_shutdown() and run:
    if board.runStatus(): # проверка подлкючения RPi к Пионеру
        print("start of programm")
        print(board.boardNumber()) # выводим бортовой номер
        sleep(2)
        print(board.time()) # выводим время с момента включения коптера
        sleep(1)
        print(board.uptime()) # выводим время запуска для системы навигации
        sleep(1)
        print(board.flightTime()) # выводим время с начала полета

        parameters_dict = board.getParameters() # получаем словарь параметров автопилота
        for name in parameters_dict.keys():
            print(f"name: {name} value: {parameters_dict[name]}") # выводим значения параемтров

        new_param_dict = { # создаем словарь новых параметров
            "Board_number": 1.0 # меняем бортовой номер
        }
        board.setParameters(new_param_dict)    
        board.restart() # перезапускаем плату для применения всех параметров

        print("end of programm")
        run = False