import cfg
import cv2


class LineCurvature:
    def __init__(self):
        ...
			
    def get_deviation(self, layout: cv2.Mat) -> float:
        """Calculates the deviation of the car course from the center of the road.
        returns values from `-1.0` to `1.0`, where `0.0` is the center of the road.
        """
        assert len(layout.shape) == 2
        
        w, h = layout.shape
        hist = layout[h//2:, :].sum(axis=0)
        imid = hist.size // 2

        # relative deviation of a left and right layout lines from center of an image
        maxi_left = hist[:imid].argmax()
        maxi_right = hist[imid:].argmax() + imid

        lmax = hist[maxi_left]
        rmax = hist[maxi_right]

        ldev = 1 - maxi_left / imid if lmax > 10000 else 0
        rdev = (maxi_right - imid) / imid if rmax > 20000 else 0

        deviation = rdev - ldev  # if lines are equally distanced from center then deviation is zero 

        if cfg.DEBUG:
            h, w = layout.shape
            lx, rx = imid - int(ldev * imid), imid + int(rdev * imid)
            layout_lines = cv2.cvtColor(layout, cv2.COLOR_GRAY2BGR)
            layout_lines = cv2.line(layout_lines, (imid, 0), (imid, h), (255, 255, 0), 2)
            layout_lines = cv2.line(layout_lines, (lx, 0), (lx, h), (0, 0, 255), 2)
            layout_lines = cv2.line(layout_lines, (rx, 0), (rx, h), (0, 0, 255), 2)

            print(f"{deviation=:.2f}")

            cv2.imshow('layout lines', layout_lines[w//2:, :])

        return deviation
