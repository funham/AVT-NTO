import numpy as np
import cv2


class Color:

    @staticmethod
    def get_color_code(color: np.ndarray) -> int:
        h_colors = {0: 0,    # 0 white
                    1: 0,    # 1 black
                    2: 15,   # 2 red
                    3: 22,   # 3 orange
                    4: 33,   # 4 yellow
                    5: 78,   # 5 green
                    6: 96,   # 6 cyan
                    7: 130,  # 7 blue
                    8: 144}  # 8 violet
        # 9: 170} # 9 magenta
        color = np.int32(color)


        h, s, v = color
        if v < (255*0.3):  # most likely black
            return 1
        elif s < (255*0.2) and v > (255*0.8):  # most likely white
            return 0
        elif s < (255*0.35):  # most likely fricking magenta
            return 9
        else:
            for key, hue in h_colors.items():
                if h < hue:
                    return key
        return 2

    def marks_to_color(marks: list) -> tuple:
        color_idx = {0: "white",
                     1: "black",
                     2: "red",
                     3: "orange",
                     4: "yellow",
                     5: "green",
                     6: "cyan",
                     7: "blue",
                     8: "violet",
                     9: "mag enta"}

        return tuple([color_idx[mark] for mark in marks])
