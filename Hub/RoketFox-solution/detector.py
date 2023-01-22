import cv2
import numpy as np
import time
 

window = cv2.namedWindow("Parameters", cv2.WINDOW_GUI_NORMAL)
cv2.createTrackbar("treshold1", "Parameters", 0, 50, lambda x: None)
cv2.createTrackbar("treshold2", "Parameters", 0, 50, lambda x: None)
cv2.createTrackbar("gaus_ksize", "Parameters", 0, 20, lambda x: None)

img1 = cv2.imread(r"res\hub1.png")[110:390, 175:430]

while True:
    start_time = time.time()
    
    treshold1 = cv2.getTrackbarPos("treshold1", "Parameters")
    treshold1 *= 10
    treshold2 = cv2.getTrackbarPos("treshold2", "Parameters")
    treshold2 *= 10
    gaus_ksize = cv2.getTrackbarPos("gaus_ksize", "Parameters")
    
    
    gaus_ksize = gaus_ksize + (gaus_ksize%2) - 1 if gaus_ksize > 0 else 1
    cv2.setTrackbarPos("gaus_ksize", "Parameters", gaus_ksize)
    
    
    # grsc = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gaus = cv2.GaussianBlur(img1, (gaus_ksize, gaus_ksize), 0)
    # median = cv2.medianBlur(img, 7)
    # bilateral = cv2.bilateralFilter(img, 15, 80, 80)
    edges = cv2.Canny(gaus, treshold1, treshold2)
    edges = cv2.dilate(edges, None, iterations=2)
    edges = cv2.erode(edges, None, iterations=1)
    
    contours, h = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    gaus1 = gaus.copy()
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if h*w > 5000:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(gaus1,[box],0,(0,0,255),2)
            # cv2.rectangle(gaus1, (x, y), (x + w, y + h), (0, 0, 255), 2)

    end_time = time.time()
    edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    cv2.putText(edges, f"FPS:{(1 / (end_time - start_time + 0.000001)):.2f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    
    cv2.imshow("blur", gaus1)
    cv2.imshow("Parameters", edges)

    if cv2.waitKey(1) == 27:
        cv2.destroyAllWindows()
        break