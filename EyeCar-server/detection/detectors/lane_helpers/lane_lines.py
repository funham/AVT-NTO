import cv2
import numpy as np


class LaneLines():
    def __init__(self) -> None:
        # normal deviation from the right line, relative to an image size
        self.target_deviation = .3

    def get_line_points(self, layout: cv2.Mat, nwindows=9) -> tuple[float, float]:
        out_img = np.dstack((layout, layout, layout))
        h, w = layout.shape

        window_h = layout.shape[0] // nwindows
        window_w = 50
        

        rlayout = layout[:, w//2:]

        bottom_right = rlayout[-window_h:-1, :]
        hist = bottom_right.sum(axis=0) / bottom_right.max()
        
        line_x = [hist.argmax()]
        line_y = [layout.shape[0]-1-window_h//2]

        for i in range(1, nwindows):
            y1 = h - window_h * (i + 1)
            y2 = h - window_h * i

            x1 = line_x[-1]-window_w//2
            x2 = line_x[-1]+window_w//2

            hist = rlayout[y1:y2, :].sum(axis=0)
            try:
                max_val = hist[x1:x2].max()
            except:
                break
            
            if max_val / 255 < 5:
                if len(line_x) < 3:
                    # 1st order continuation
                    dx = line_x[-1] - line_x[-2]
                    line_x.append(line_x[-1] + dx)
                else:
                    # 2nd order continuation
                    dx1 = line_x[-2] - line_x[-3]
                    dx2 = line_x[-1] - line_x[-2]
                    d2x = dx1 - dx2
                    line_x.append(line_x[-1] + dx2 + d2x)
                
            else:
                line_pos = hist[x1:x2].argmax() + x1
                line_x.append(line_pos)

                cv2.rectangle(out_img, (x1+w//2, y1), (x2+w//2, y2), (255,0,0), 1)
                cv2.rectangle(out_img, (line_pos+w//2-5, y1+window_h//3), (line_pos+w//2+5, y2-window_h//3), (0,0,255), 1)

            line_y.append(y2 - window_h // 2)
            

        line_x = np.array(line_x, dtype=np.int16) + w // 2
        line_y = np.array(line_y, dtype=np.int16)
        return line_x, line_y, out_img
    
    def get_curvature_and_deviation(self, line_x: np.ndarray, line_y: np.ndarray, out_img: cv2.Mat) -> tuple[float, float]:
        a, b, c = np.polyfit(line_y, line_x, deg=2)

        img_h, img_w, *_ = out_img.shape

        y0 = img_h-1
        x0 = a*y0*y0 + b*y0 + c*1

        curvature = 2*a / (1 + (2*a*y0 + b)**2)**1.5
        deviation = 2*x0/img_w - 1 - self.target_deviation
        print(f'{deviation=}')
        print(f'{curvature=}')

        k = curvature / (1 + self.target_deviation*curvature*img_w)

        r = 1 / curvature
        R = 1 / k

        cv2.circle(out_img, np.int16([x0 + r, img_h+20]), int(r), (0, 255, 0), 2)
        cv2.circle(out_img, np.int16([x0 + r, img_h+20]), int(R), (0, 255, 0), 2)

        Y = np.arange(img.shape[0], step=1)
        X = a*Y*Y + b*Y+ c*1

        good_idxs = X < img.shape[1]

        X = X[good_idxs].astype(np.int32)
        Y = Y[good_idxs].astype(np.int32)

        line_pts = np.dstack([X, Y])[0]
        pts = np.array(list(filter(lambda p: (int(p[1]) < img.shape[0]) and (int(p[0]) < img.shape[1]), line_pts)), dtype=np.int32)
        pts = pts.reshape((-1, 1, 2))
        out_img = cv2.polylines(out_img, [pts], isClosed=False, color=(0, 125, 255), thickness=2, lineType=cv2.LINE_AA)

        return k * img_w, deviation * img_w, out_img

    def get_trajectory_curvature(self, layout: cv2.Mat) -> float:
        line_x, line_y, out_img = self.get_line_points(layout, 20)
        k, deviation, out_img = self.get_curvature_and_deviation(line_x, line_y, out_img)
        k_target = k - (deviation * .2)
        
        cv2.imshow('curv', out_img)

        return k_target
    
    def get_trajectory_curvature_simple(self, layout: cv2.Mat) -> float:
        h, w = layout.shape
        hist = layout[-20:, w//2:].sum(axis=0)

        line_val = hist.max()
        deviation = hist.argmax()

        if line_val / 255 < 10:
            return 0
        
        k = deviation / w * 0.2

        return k

if __name__ == "__main__":
    lines = LaneLines()
    img = cv2.imread(r"C:\Users\E X P L O R E R\Downloads\warped.png", 0)

    k = lines.get_trajectory_curvature(img)
    k1 = lines.get_trajectory_curvature_simple(img)



    print(f'{k=}, {k1=}')

    cv2.waitKey(0)
