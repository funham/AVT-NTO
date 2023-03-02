#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gs_navigation import NavigationManager
from gs_board import BoardManager

rospy.init_node("navigation_test_node")
board = BoardManager()
navigation = NavigationManager()
run = True

while not rospy.is_shutdown() and run:
    if board.runStatus():
        print(navigation.system())
        print(navigation.gps.position())
        print(navigation.gps.satellites())
        print(navigation.gps.status())
        print(navigation.lps.position())
        print(navigation.lps.velocity())
        print(navigation.lps.yaw())
        print(navigation.opt.velocity())
        run = False