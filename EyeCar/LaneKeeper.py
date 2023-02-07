import cv2
from enum import Enum

from abc import ABC, abstractmethod, abstractproperty
from dataclasses import dataclass
from typing import Iterable, Type
from itertools import chain


class LaneStatus:
    deviation: float
    distance_travelled: int


class LaneKeeper:
    dist: float = 0.0

    def __init__(self):
        ...

    def forward(self, frame) -> LaneStatus:
        ...
