#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gs_board import BoardManager
from gs_logger import Logger
from time import sleep

rospy.init_node("logger_test_node")
board = BoardManager()
logger = Logger()

run = True

while not rospy.is_shutdown() and run:
    if board.runStatus():
        print("start of programm")
        print(logger.lastMsgs())
        print(board.time())
        print(logger.lastMsgs())
        print(logger.allMsgs())
        print("end of programm")
        run = False
