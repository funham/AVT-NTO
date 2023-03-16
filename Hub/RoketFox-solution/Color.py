import numpy as np
import cv2


class Color:

    @staticmethod
    def get_color_code(color: np.ndarray) -> int:
        h_colors = {0: 0,    # 0 white
                    1: 0,    # 1 black
                    2: 22,   # 2 red
                    3: 78,   # 5 green
                    4: 144}  # 7 blue
        color = np.int32(color)


        h, s, v = color
        if v < (255*0.3):  # most likely black
            return 1
        elif s < (255*0.2) and v > (255*0.8):  # most likely white
            return 0
        else:
            for key, hue in h_colors.items():
                if h < hue:
                    return key
        return 2

    def marks_to_color(marks: list) -> tuple:
        color_idx = {0: "white",
                     1: "black",
                     2: "red",
                     3: "green",
                     4: "blue"}

        return tuple([color_idx[mark] for mark in marks])
