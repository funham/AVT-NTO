import cv2
from enum import Enum

from abc import ABC, abstractmethod, abstractproperty
from collections import namedtuple
from typing import Iterable, Type
from itertools import chain

LaneStatus = namedtuple("LaneStatus", "deviation distance_travelled")


class LaneKeeper:
    dist: float = 0.0

    def __init__(self):
        ...

    def forward(self, frame) -> Type[LaneStatus]:
        ...
