import cv2

from beholder2048squad.Server import Server
from LaneKeeper import LaneKeeper, LaneStatus
from Detection import *

from dataclasses import dataclass

MAX_ANGLE: float = 45
MAX_SPEED: int = 255
MAX_DEVIATION: float = 5

k_dev = MAX_ANGLE / MAX_DEVIATION
k_break = .5


@dataclass
class Command:
    speed: int
    angle: int

    def __str__(self):
        return f'SPEED:{self.speed}\nANGLE:{self.angle}'


serv = Server(9090)
lane_keeper = LaneKeeper()


def get_command(frame: cv2.Mat, curr_speed: int, curr_angle: int) -> Command:
    cmd = Command(curr_speed, curr_angle)

    det: tuple = model.forward(frame)

    traffic_lights = (TrafficLight(detection)
                      for detection in det.traffic_light)

    pedestrians = (Pedestrian(detection)
                   for detection in det.pedestrians)

    # TODO confidence buffer
    if match_condition(traffic_lights, TrafficLight.stop_condition) or \
            match_condition(pedestrians, Pedestrian.stop_condition):
        return Command(speed=0, angle=curr_angle)

    lane_status = LaneKeeper.forward(frame)

    angle = int(lane_status.deviation * k_dev)
    speed = int(MAX_SPEED - angle * k_break)

    # TODO PID smoothing
    return Command(speed=speed, angle=angle)


speed, angle = 0, 0


def main_loop() -> None:
    frame = serv.recv_img()

    if cv2.waitKey(1) == ord('q'):
        raise StopIteration

    cv2.imshow('frame', frame)

    cmd = get_command(frame, speed, angle)

    speed, angle = cmd.speed, cmd.angle
    serv.send_str(str(cmd))


if __name__ == '__main__':
    try:
        while True:
            main_loop()

    except StopIteration:
        print('Exiting main loop...')
    finally:
        print('Dont get hit by a car')
        serv.send_str(str(Command(speed=0, angle=0)))
