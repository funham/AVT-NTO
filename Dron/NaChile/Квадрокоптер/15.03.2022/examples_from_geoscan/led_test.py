#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gs_board import BoardManager
from gs_module import BoardLedController
from time import sleep

rospy.init_node("led_test_node")
board=BoardManager()
led = BoardLedController()

run = True

while not rospy.is_shutdown() and run:
    if board.runStatus():
        print("start of programm")
        led.changeColor(0,0,255.0,0)
        sleep(1)
        led.changeColor(2,255.0,0,0)
        sleep(1)
        led.changeColor(3,0,0,255.0)
        sleep(1)
        led.changeAllColor(126.0,0,255.0)
        print("end of programm")
        run = False
