import cfg
import cv2
import numpy as np


def eval_poly(coeff_vector, x):
    deg = len(coeff_vector) - 1
    power_vector = list(range(deg, -1, -1))
    arg_vector = np.array([x] * (deg + 1)).astype(np.int64) ** power_vector  # [..., y^2, y^1, y^0]
    return np.dot(arg_vector, coeff_vector)


def draw_poly(out_img: cv2.Mat, coeff_vector: np.ndarray) -> None:
    h, w, _ = out_img.shape
    for y in range(h):
        x = int(eval_poly(coeff_vector, y))
        cv2.circle(out_img, (x, y), 1, (255, 0, 0), 2)


class LaneLines:
    def get_deviation_only_right(self, layout: cv2.Mat, out_img: cv2.Mat, p1, p2) -> float:
        img_h, img_w = layout.shape
        x1, y1 = p1
        x2, y2 = p2
        layout[y1:y2, x1:x2] = 0
        
        lines = self.get_lines(layout, out_img)

        deviations = []
        for line in lines:
            line_x, line_y = line[:, 0], line[:, 1]

            coeff_vector = np.polyfit(line_y, line_x, deg=3)

            y0 = img_h - 10
            x0 = eval_poly(coeff_vector, y0)

            actual_deviation = x0 - img_w // 2

            deviation = actual_deviation - cfg.LINE_NORMAL_DEVIATION
            x_nd = img_w // 2 + cfg.LINE_NORMAL_DEVIATION

            deviations.append(deviation)

            draw_poly(out_img, coeff_vector)
            cv2.line(out_img, (int(x0), y0), (img_w // 2, y0), (0, 255, 255), 1, lineType=cv2.LINE_AA)
            cv2.line(out_img, (img_w // 2, y0), (img_w // 2, y0 - 20), (0, 255, 0), 1, lineType=cv2.LINE_AA)
            cv2.circle(out_img, (int(x_nd), y0), 5, (255, 255, 0), 1)

        return 2 * deviations[-1] / img_w

    def get_lines(self, layout: cv2.Mat, out_img: cv2.Mat):
        h_layout, w_layout = layout.shape
        w_window, h_window = cfg.LANE_WINDOW_SIZE

        line_left = self.get_local_line_points(layout[:, :w_layout // 2 - 1])

        line_right = self.get_local_line_points(layout[:, w_layout // 2:])
        line_right[:, 0] += w_layout // 2

        lines = [line_left, line_right]

        for line in lines:
            for x, y in line:
                x1, y1 = x - w_window // 2, y - h_window // 2
                x2, y2 = x + w_window // 2, y + h_window // 2
                cv2.rectangle(out_img, (x1, y1), (x2, y2), (30, 30, 30), 1)
                cv2.rectangle(out_img, (x - 5, y1), (x + 5, y2), (0, 0, 100), 1)
            cv2.rectangle(out_img, (line[-1][0] - 5, line[-1][1] - h_window // 2),
                          (line[-1][0] + 5, line[-1][1] + h_window // 2), (150, 130, 150), -1)

        return lines

    def get_local_line_points(self, lane_layout: cv2.Mat):
        h_lane_layout, w_lane_layout = lane_layout.shape
        w_window, h_window = cfg.LANE_WINDOW_SIZE
        nwindows = h_lane_layout // h_window

        bottom_right_window = lane_layout[-h_window:-1, :]
        hist = bottom_right_window.sum(axis=0) / bottom_right_window.max()

        start_window_index = 0
        maxv, maxi = hist.max(), hist.argmax()

        # trying to find start of the line
        for window_index in range(nwindows):
            y1 = h_lane_layout - h_window * (window_index + 1)
            y2 = h_lane_layout - h_window * window_index
            hist = lane_layout[y1:y2, :].sum(axis=0)
            maxv, maxi = hist.max(), hist.argmax()

            if maxv > 10:
                start_window_index = window_index + 1
                break  # we found the line!

        line_x = [maxi]
        line_y = [h_lane_layout - 1 - h_window // 2]

        for window_index in range(start_window_index, nwindows):
            y1 = h_lane_layout - h_window * (window_index + 1)
            y2 = h_lane_layout - h_window * window_index

            x1 = line_x[-1] - w_window // 2
            x2 = line_x[-1] + w_window // 2

            x1 = np.clip(x1, 0, w_lane_layout - 1)
            x2 = np.clip(x2, 0, w_lane_layout - 1)

            if x2 - x1 < w_window * .5:
                break

            hist = lane_layout[y1:y2, x1:x2].sum(axis=0)
            line_y.append(y2 - h_window // 2)

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

        line_x = np.flip(np.array(line_x, dtype=np.int16))
        line_y = np.flip(np.array(line_y, dtype=np.int16))

        line = np.concatenate([np.expand_dims(line_x, axis=0), np.expand_dims(line_y, axis=0)]).T

        return line
