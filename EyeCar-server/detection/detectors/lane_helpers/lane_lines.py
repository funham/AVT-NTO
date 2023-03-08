import cv2
import numpy as np


class LaneLines():
    def __init__(self) -> None:
        # normal deviation from the right line, relative to an image size
        self.target_deviation = .5

    def get_deviation(self, line_index: int, img_width: int) -> float:
        # TODO
        return self.target_deviation - (line_index / img_width)

    def find_line_start_index(self, laytout: cv2.Mat) -> tuple[int, int]:
        ...
    
    def find_line_points(self, layout: cv2.Mat, nwindows: int) -> tuple[np.ndarray, np.ndarray]:
        ...

    def get_curvature_and_deviation(self, layout: cv2.Mat) -> tuple[float, float]:
        out_img = np.dstack((layout, layout, layout))
        
        nwindows = 9

        window_h = layout.shape[0] // nwindows
        window_w = 30
        midpoint = int(layout.shape[1] // 2)

        rlayout = layout[:, midpoint:]

        bottom_right = rlayout[-window_h:-1, :]
        hist = bottom_right.sum(axis=0) / bottom_right.max()
        
        line_x = [hist.argmax()]

        for i in range(1, nwindows):
            hist = rlayout[-window_h*(i+1):-window_h*i]
            line_pos = hist.argmax()
            line_x.append(line_pos)
            
        cv2.imshow('out_img', out_img)
        print(line_x)


lines = LaneLines()
img = cv2.imread(r"C:\Users\fun1h\Downloads\transformed.png", 0)
lines.get_curvature_and_deviation(img)
cv2.waitKey(0)
