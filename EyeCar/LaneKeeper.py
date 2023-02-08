import cv2
import numpy as np

from enum import Enum
from abc import ABC, abstractmethod, abstractproperty
from dataclasses import dataclass
from typing import Iterable, Type
from itertools import chain


@dataclass
class LaneStatus:
    deviation: float
    distance_travelled: int


class LaneKeeper:
    dist: float = 0.0

    def __init__(self):
        ...

    def forward(self, frame: cv2.Mat) -> LaneStatus:
        layout = cv2.inRange(frame, (...), (...)) # binarized image
        dts = np.array([...])
        M = cv2.getPerspectiveTransform(dts, ...)

        layout = cv2.warpPerspective(layout, M, ...)
        hist = layout.sum(axis=0)

        

        return LaneStatus(..., ...)

