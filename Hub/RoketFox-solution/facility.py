# from facility_api import *
import numpy as np
import config as cfg
from Coords import CargoPosition


def MoveToHoarder(hoarder: int):

    if hoarder == 0:
        # ToFirstHoarder()
        print("Moving to first hoarder")
    elif hoarder == 1:
        # ToSecondHoarder()
        print("Moving to second hoarder")
    elif hoarder == 2:
        # ToThirdHoarder()
        print("Moving to third hoarder")
    else:  # hoarder == 3
        # ToFourthHoarder()
        print("Moving to fourth hoarder")


def send_to_drone(coords: tuple) -> None:
    coords = CargoPosition.pix_to_coords(coords)
    # MoveTo(coords)
    # MagnetFullDown()
    # MagnetOn()
    # MagnetUp()
    # MoveToHoarder(hoarder_num)
    # MagnetOff()
    # CarriageStart()
    # LiftUp()
    # LiftDown()
    print("Cargo sent to drone")
    return None
