import cv2
import numpy as np

from enum import Enum
from abc import ABC, abstractmethod, abstractproperty
from typing import Iterable, Type
from itertools import chain
from dataclasses import dataclass


@dataclass
class Lane:
    deviation: float
    distance_travelled: int
    crossroad_distance: int


class LaneKeeper:
    dist: float = 0.0

    def forward(self, frame: cv2.Mat) -> Lane:
        layout = self._get_layout(frame)
        deviation = self._calc_deviation(layout)
        self.dist += self._calc_distance_increment(layout)

        return Lane(deviation=deviation,
                    distance_travelled=self.dist)

    def _calc_distance_increment(self, layout: cv2.Mat) -> float:
        return 0.0

    def _calc_deviation(self, layout: cv2.Mat) -> float:
        hist = layout.sum(axis=0)
        mid = hist.size() // 2
        ldev = 1 - hist[:mid].argmax() / mid
        rdev = hist[mid:].argmax() / mid

        return rdev - ldev

    def _get_layout(self, img: cv2.Mat) -> cv2.Mat:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        layout = cv2.inRange(img, (...), (...))  # binarized image
        dts = np.array([...])
        M = cv2.getPerspectiveTransform(dts, ...)
        layout = cv2.warpPerspective(layout, M, ...)

        return layout
