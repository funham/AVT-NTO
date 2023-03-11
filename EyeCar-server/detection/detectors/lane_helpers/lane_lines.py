import cfg
import cv2
import numpy as np


def eval_poly(coeff_vector, x):
        deg = len(coeff_vector) - 1
        power_vector = list(range(deg+1)[::-1])
        arg_vector = np.array([x] * (deg+1))**power_vector  # [y^2, y^1, y^0]
        return np.dot(arg_vector, coeff_vector)

def draw_poly(out_img: cv2.Mat, coeff_vector: np.ndarray) -> None:
        h, w, _ = out_img.shape
        
        for y in range(h):
            x = int(eval_poly(coeff_vector, y))
            cv2.circle(out_img, (x, y), 1, (255, 0, 0), 2)


class LaneLines:
    def get_deviation_simple(self, layout: cv2.Mat, out_img: cv2.Mat) -> float:
        """Calculates the deviation of the car course from the center of the road.
        returns values from `-1.0` to `1.0`, where `0.0` is the center of the road.
        """

        assert len(layout.shape) == 2
        
        h, w = layout.shape
        y0 = int(h - h/10)
        hist = layout[y0:, :].sum(axis=0)
        imid = hist.size // 2

        # relative deviation of a left and right layout lines from center of an image
        lmaxi = hist[:imid].argmax()
        rmaxi = hist[imid:].argmax() + imid

        lmax = hist[lmaxi] / 255
        rmax = hist[rmaxi] / 255

        if rmax < 20:
            if cfg.DEBUG:
                print(f'deviation is not cosnsidered')
            return 0.0  # no deviation considered

        ldev = 1 - lmaxi / imid
        rdev = (rmaxi - imid) / imid

        deviation = rdev - ldev  # if lines are equally distanced from center then deviation is zero 

        if cfg.DEBUG:
            h, w = layout.shape
            lx, rx = imid - int(ldev * imid), imid + int(rdev * imid)

            # center, left line, right line
            cv2.line(out_img, (imid, y0), (imid, h), (255, 255, 0), 3, lineType=cv2.LINE_AA)
            cv2.line(out_img, (lx, y0), (lx, h), (0, 0, 255), 6, lineType=cv2.LINE_AA)
            cv2.line(out_img, (rx, y0), (rx, h), (0, 0, 255), 6, lineType=cv2.LINE_AA)

            print(f"{deviation=:.2f}")

        return deviation
    

    def get_deviation_only_right(self, layout: cv2.Mat, out_img: cv2.Mat) -> float:
        img_h, img_w = layout.shape
        line_x, line_y = self.get_line_points(layout, out_img, (15, 50))
        coeff_vector = np.polyfit(line_y, line_x, deg=3)

        y0 = img_h-10
        x0 = eval_poly(coeff_vector, y0)
        
        # ain't no statistics here ;)
        normal_deviaton = 70
        actual_deviation = x0 - img_w // 2

        deviation = actual_deviation - normal_deviaton 
        x_nd = img_w//2 + normal_deviaton

        if cfg.DEBUG:
            # draw_poly(out_img, coeff_vector)
            cv2.line(out_img, (int(x0), y0), (img_w//2, y0), (0, 255, 255), 1, lineType=cv2.LINE_AA)
            cv2.line(out_img, (img_w//2, y0), (img_w//2, y0-20), (0, 255, 0), 1, lineType=cv2.LINE_AA)
            cv2.circle(out_img, (int(x_nd), y0), 5, (255, 255, 0), 1)
        
        return 2 * deviation / img_w

    def get_line_points(self, layout: cv2.Mat, out_img: cv2.Mat, window_shape: tuple) -> tuple[float, float]:
        h, w = layout.shape

        window_h, window_w = window_shape
        nwindows = h // window_h

        rlayout = layout[:, w//2:]

        bottom_right = rlayout[-window_h:-1, :]
        hist = bottom_right.sum(axis=0) / bottom_right.max()

        maxv, maxi = hist.max(), hist.argmax()

        start_window_index = 0
        
        # trying to find start of the line
        for window_index in range(0, nwindows):
            y1 = h - window_h * (window_index + 1)
            y2 = h - window_h * window_index
            hist = rlayout[y1:y2, :].sum(axis=0)
            maxv, maxi = hist.max(), hist.argmax()

            if maxv > 10:
                start_window_index = window_index + 1
                cv2.rectangle(out_img, (maxi-window_w//2 + w//2, y1), (maxi+window_w//2 + w//2, y2), (30, 30, 30), 1)
                cv2.rectangle(out_img, (maxi-5 + w//2, y1), (maxi+5 + w//2, y2), (50, 30, 50), 1)
                break  # we found the line!
        
        line_x = [maxi]
        line_y = [layout.shape[0]-1-window_h//2]

        for window_index in range(start_window_index, nwindows):
            y1 = h - window_h * (window_index + 1)
            y2 = h - window_h * window_index

            x1 = line_x[-1]-window_w//2
            x2 = x1 + window_w

            x1 = np.clip(x1, 0, w//2-1)
            x2 = np.clip(x2, 0, w//2-1)

            if x2 - x1 < window_w * .5:
                break

            hist = rlayout[y1:y2, x1:x2].sum(axis=0)
            line_y.append(y2 - window_h // 2)
            
            
            # if no line detected in window
            if hist.max() / 255 < 5:
                if len(line_x) < 2:
                    # 0th order continuation
                    inc = 0
                elif len(line_x) == 2:
                    # 1st order continuation
                    dx = line_x[-1] - line_x[-2]
                    inc = dx
                else:
                    # 2nd order continuation
                    dx1 = line_x[-2] - line_x[-3]
                    dx2 = line_x[-1] - line_x[-2]
                    d2x = dx1 - dx2
                    inc = dx2 + d2x

                line_pos = line_x[-1] + inc
                
            else:
                line_pos = hist.argmax() + x1

            line_x.append(line_pos)

            cv2.rectangle(out_img, (x1+w//2, y1), (x2+w//2, y2), (30, 30, 30), 1)
            cv2.rectangle(out_img, (line_pos+w//2-5, y1), (line_pos+w//2+5, y2), (0,0,100), 1)
        
        line_x = np.array(line_x, dtype=np.int16) + w // 2
        line_y = np.array(line_y, dtype=np.int16)

        return line_x, line_y

    