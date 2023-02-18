# from facility_api import *
import numpy as np
import config as cfg


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


def __pix_to_coords(pix: np.ndarray) -> np.ndarray:
    '''
    Convert pixels to coordinates.
    '''
    x, y = pix
    coord_w, coord_h = cfg.CARRIAGE_SIZE
    img_w, img_h = cfg.IMG_SIZE

    return (x/img_w*coord_w, y/img_h*coord_h)


def send_to_drone(coords: np.ndarray) -> None:
    coords = __pix_to_coords(coords)
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
