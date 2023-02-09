from functools import partial
from typing import *

import cv2
import Car

from LaneKeeper import LaneKeeper, Lane
from Model import Model, Detection, DetectionResults
from Detection import TrafficLight, Pedestrian, match_condition
from collections import deque


model = Model()
lane_keeper = LaneKeeper()

stop_conf = 0
cmd_buff = deque([], maxlen=CMD_BUFF_LEN)


def confidence_buffer(func: Callable) -> Callable:
    global cmd_buff

    def inner(frame: cv2.Mat):
        cmd = func(frame)

        if CMD_BUFF_LEN == 0:
            return cmd

        cmd_buff.append(cmd)

        stop_conf = sum(
            filter(lambda x: x == Command.STOP, cmd_buff)) / CMD_BUFF_LEN

        if stop_conf > 0.8:
            return Command.STOP

        return cmd if cmd != Command.STOP else ''

    return inner


@confidence_buffer
def calculate_command(frame: cv2.Mat) -> str:
    det: Detection = model.forward(frame)
    lane: Lane = lane_keeper.forward(frame)

    traffic_lights = (TrafficLight(detection)
                      for detection in det.TrafficLights)
    pedestrians = (Pedestrian(detection) for detection in det.Pedestrians)

    if \
            match_condition(traffic_lights, TrafficLight.stop_condition) or \
            match_condition(pedestrians, Pedestrian.stop_condition) or \
            lane.distance_travelled > Car.TARGET_DISTANCE or \
            lane.crossroad_distance < Car.CROSSROAD_STOP_DIST:
        return Command.STOP

    new_params = Car.calc_params(lane.deviation)

    return Command.str(*new_params)


class Command:
    STOP = 'SPEED:0\n'

    @staticmethod
    def str(speed, angle) -> str:
        return \
            f'SPEED:{speed}\n' if speed is not None else '' + \
            f'ANGLE:{angle}\n' if angle is not None else ''
