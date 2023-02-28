import cfg
import numpy as np
import time

from abc import ABC, abstractmethod
from enum import Enum


class CarStatus:
    class IntersectionPassDirections(Enum):
        LEFT = 1
        RIGHT = 2
        STRAIGHT = 3

    _angle: float = 0
    _suspended: bool = False

    _set_speed_vals: list[float]
    _last_stop_time: float
    _requested_stop: bool

    def reset(self):
        """
        # DO NOT CALL
        (beyond `CarControl.get_command()`)
        """
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
        return self._suspended

    @property
    def stop_duration(self):
        return time.time() - self._last_stop_time

    def stop(self) -> None:
        if self._suspended:
            return

        self._last_stop_time = time.time()
        self._suspended = True
        self._requested_stop = True

    def set_intersection_passing(self, dir: IntersectionPassDirections):
        ...

    # TODO
    def terminate_ride(self):
        ...
