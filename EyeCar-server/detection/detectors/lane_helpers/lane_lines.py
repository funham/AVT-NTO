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
        h, w = layout.shape
        nwindows = 9

        window_h = layout.shape[0] // nwindows
        window_w = 50
        

        rlayout = layout[:, w//2:]

        bottom_right = rlayout[-window_h:-1, :]
        hist = bottom_right.sum(axis=0) / bottom_right.max()
        
        line_x = [hist.argmax()]

        for i in range(1, nwindows):
            y1 = h - window_h * (i + 1)
            y2 = h - window_h * i

            x1 = line_x[-1]-window_w//2
            x2 = line_x[-1]+window_w//2

            hist = rlayout[y1:y2, :].sum(axis=0)            
            max_val = hist[x1:x2].max()
            
            if max_val < 10:
                last_diff = line_x[-1] - line_x[-2]
                line_x.append(line_x[-1] + last_diff)
                continue

            line_pos = hist[x1:x2].argmax() + x1
            line_x.append(line_pos)

            cv2.rectangle(out_img, (x1+w//2, y1), (x2+w//2, y2), (255,0,0), 2)
            cv2.rectangle(out_img, (line_pos+w//2-5, y1+window_h//3), (line_pos+w//2+5, y2-window_h//3), (0,0,255), 2)
            
        cv2.imshow('out_img', out_img)
        print(line_x)


lines = LaneLines()
img = cv2.imread(r"C:\Users\fun1h\Downloads\transformed.png", 0)
lines.get_curvature_and_deviation(img)
cv2.waitKey(0)
