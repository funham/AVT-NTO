"""
Represents a current status of the car.

Affectable by detection handlers and CarControl.
"""

import cfg
import numpy as np
import time

from enum import Enum
from typing import Callable, Optional


class CarStatus:
    class IntersectionTurnDirections(Enum):
        LEFT = 1
        RIGHT = 2
        STRAIGHT = 3

    _angle: float = 0
    _suspended: bool = False

    _set_speed_vals: list[float] = []
    _last_stop_time: float = 0
    _requested_stop: bool = False
    _turning_dir: IntersectionTurnDirections | None = None

    def __init__(self) -> None:
        self.reset()

    def reset(self):
        self._requested_stop = False
        self._set_speed_vals.clear()
        self.speed = cfg.CAR_MAX_SPEED

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, val):
        self._angle = np.clip(val, -cfg.CAR_MAX_ANGLE, cfg.CAR_MAX_ANGLE)

    @property
    def speed(self):
        return min(self._set_speed_vals)

    @speed.setter
    def speed(self, val):
        self._set_speed_vals.append(
            np.clip(val, cfg.CAR_MIN_SPEED, cfg.CAR_MIN_SPEED))

    @property
    def suspended(self) -> bool:
        """Returns True if car is stopped"""
        return self._suspended

    @property
    def stop_duration(self):
        return time.time() - self._last_stop_time

    def stop(self) -> None:
        """
        Requests a stop.

        Should be called by handler when stop condition is
        satisfied. For e.g. when a pedestrian detected on the road.
        """

        if self._suspended:
            return

        self._last_stop_time = time.time()
        self._suspended = True
        self._requested_stop = True

    @property
    def turning(self) -> IntersectionTurnDirections | None:
        return self._turning_dir

    def intersection_turn(self, dir: IntersectionTurnDirections):
        """Sets the car into the intersection passing mode"""
        self._turning_dir = dir

    def terminate_ride(self, callback: Optional[Callable] = None, *args, **kwargs):
        """
        Terminates ride when all the tasks of a run are done.

        For e.g. when the car has delivered the cargo to the sorting station
        or ran the requested distance, specified in cfg file.
        """
        print("Ride termination requested")

        if callback is not None:
            callback(*args, **kwargs)

        raise StopIteration
