#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gs_sensors import SensorManager
from gs_board import BoardManager
from time import sleep

rospy.init_node("sensor_test_node")
board = BoardManager()
sensors = SensorManager()

run = True

while not rospy.is_shutdown() and run:
    if board.runStatus():
        print("start programm")
        print(sensors.gyro())
        print(sensors.accel())
        print(sensors.orientation())
        print(sensors.altitude())
        print(sensors.power())
        print(sensors.mag())
        print("end of programm")
        run = False