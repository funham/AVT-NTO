from functools import partial
from typing import Optional

import cv2
import Car

from beholder2048squad.Server import Server, ConnType
from LaneKeeper import LaneKeeper, LaneStatus
from Model import Model, Detection, DetectionResults
from Detection import TrafficLight, Pedestrian, match_condition

def Command(*, speed: Optional[int]=None, angle: Optional[int]=None) -> str:
    return \
        f'SPEED:{speed}\n' if speed is not None else '' + \
        f'ANGLE:{angle}\n' if angle is not None else ''


StopCmd = partial(Command, speed=0)

serv = Server(9090)
model = Model()
lane_keeper = LaneKeeper()

def get_command(frame: cv2.Mat, ) -> str:
    det: Detection = model.forward(frame)
    lane: LaneStatus = lane_keeper.forward(frame)

    traffic_lights = (TrafficLight(detection)
                      for detection in det.TrafficLights)
    pedestrians = (Pedestrian(detection) for detection in det.Pedestrians)

    # TODO confidence buffer
    if match_condition(traffic_lights, TrafficLight.stop_condition) or \
            match_condition(pedestrians, Pedestrian.stop_condition) or \
            lane.distance_travelled > Car.TARGET_DISTANCE:
        return StopCmd()

    Car.direct(lane.deviation)

    return Command(speed=Car.speed, angle=Car.angle)


def main_loop() -> None:
    frame = serv.chat_img()

    if cv2.waitKey(1) == ord('q') or frame is None:
        raise StopIteration

    cv2.imshow('frame', frame)

    cmd = get_command(frame)
    serv.chat_cmd(cmd, ConnType.C)


if __name__ == '__main__':
    try:
        while True:
            main_loop()

    except StopIteration:
        print('Exiting main loop...')
    finally:
        print('Dont get hit by a car')
        serv.chat_cmd(StopCmd())
        serv.close()
