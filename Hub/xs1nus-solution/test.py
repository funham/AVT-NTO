import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

W, H = 400, 460
TRACK_BARS = False

def transform_perspective(frame):
    in_pts = [[130, 42], [103, 400], [480, 400], [447, 40]]
    in_pts = np.int0(np.array(in_pts))
    out_pts = [(0, 0), (0, H), (W, H), (W, 0)]

    M = cv2.getPerspectiveTransform(np.float32(in_pts), np.float32(out_pts))

    return cv2.warpPerspective(frame, M, (W, H))



window = cv2.namedWindow("Parameters")

if TRACK_BARS:
    cv2.createTrackbar("treshold1", "Parameters", 0, 50, lambda _: ...)
    cv2.createTrackbar("treshold2", "Parameters", 0, 50, lambda _: ...)
    cv2.createTrackbar("gaus_ksize", "Parameters", 0, 20, lambda _: ...)


frame = cv2.imread(r"hub\res\hub1.png")
frame = transform_perspective(frame)


while True:
    tic = time.process_time()
    cv2.imshow('frame', frame)

    if TRACK_BARS:
        treshold1 = cv2.getTrackbarPos("treshold1", "Parameters")
        treshold1 *= 10
        treshold2 = cv2.getTrackbarPos("treshold2", "Parameters")
        treshold2 *= 10

        gaus_ksize = cv2.getTrackbarPos("gaus_ksize", "Parameters")
        gaus_ksize = gaus_ksize + (gaus_ksize%2) - 1 if gaus_ksize > 0 else 1
        cv2.setTrackbarPos("gaus_ksize", "Parameters", gaus_ksize)
    else:
        treshold1 = 20 * 10
        treshold2 = 10 * 10
        gaus_ksize = 7
    
    
    blur = cv2.GaussianBlur(frame, (gaus_ksize, gaus_ksize), 0)
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, treshold1, treshold2)
    
    contours, h = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    canvas = blur.copy()

    for cnt in contours:
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        if cv2.contourArea(box) > 2500:
            cv2.drawContours(canvas, [box], 0, (0, 0, 255), 2)

    

    toc = time.process_time()
    # edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    cv2.putText(edges, f"FPS:{(1 / (toc - tic + 0.000001)):.2f}", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    
    cv2.imshow("blur", canvas)
    cv2.imshow("Parameters", edges)

    if cv2.waitKey(1) == ord('q'):
        cv2.destroyAllWindows()
        break
    