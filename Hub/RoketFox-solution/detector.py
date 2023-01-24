import cv2
import numpy as np
import os
import math


def find_color(color):
    colors = {0: (255, 255, 255),  # white
              1: (40, 40, 40),  # black
              2: (95, 95, 230),  # red
              3: (80, 160, 250),  # orange
              4: (70, 240, 235),  # yellow
              5: (120, 200, 0),  # green
              6: (255, 205, 0),  # cyan
              7: (210, 120, 10),  # blue
              8: (225, 150, 90),  # violet
              9: (205, 145, 215)}  # magenta
    
    cur_col, min_dist = None, np.inf
    for num, col in colors.items():
        dist = np.linalg.norm(np.array(col) - np.array(color))
        if dist < min_dist:
            min_dist = dist
            cur_col = num
    return cur_col

def detect_colors(img) -> list:
    colors = []
    for x in range(2):
        for y in range(2):
            sector = img[x*100+40:x*100+50, y*100+40:y*100+50]
            color = np.mean(sector, axis=(0, 1))
            colors.append(find_color(color))
    return colors

def face_center(x, y, w, h):
    return np.array((int(x + w/2), int(y + h/2)))

def detect_face(img: np.ndarray):
    gaus_ksize = 7
    treshold1, treshold2 = 200, 100
    
    grsc = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gaus = cv2.GaussianBlur(grsc, (gaus_ksize, gaus_ksize), 0)
    edges = cv2.Canny(gaus, treshold1, treshold2)
    edges = cv2.dilate(edges, None, iterations=0)
    edges = cv2.erode(edges, None, iterations=0)
    
    cnts, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = filter(lambda cnt: cv2.boundingRect(cnt)[2]*cv2.boundingRect(cnt)[3] > 5000, cnts) # keeps cnts with a rect area more than 5000 pixels
    cnts = sorted(cnts, key=lambda cnt: cv2.boundingRect(cnt)[2]*cv2.boundingRect(cnt)[3]) # sort cnts by area (smaller first)
    centers  = list(map(lambda cnt: face_center(*cv2.boundingRect(cnt)), cnts)) # keeps centers of cnts
    
    bad_cnts = []
    for i in range(len(centers)):
        for j in range(i+1, len(centers)):
            if j in bad_cnts: continue
            if np.linalg.norm(centers[i] - centers[j]) < 50: # if centers are too close, they are probably the same face, so we remove one of them
                bad_cnts.append(j) # we remove the one with the larger area (the one with the larger index)
    
    cnts = [cnts[i] for i in range(len(cnts)) if i not in bad_cnts] # keeps only the good cnts
    
    width, height = 200, 200 # dimensions of the output face
    for cnt_id in range(len(cnts)):
        rect = cv2.minAreaRect(cnts[cnt_id])
        box = cv2.boxPoints(rect)
        box = np.float32(box)
        pts2 = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
        matrix = cv2.getPerspectiveTransform(box, pts2)
        out = cv2.warpPerspective(img, matrix, (width, height))
        print(detect_colors(out))
        cv2.imshow(f"{cnt_id}", out)
    
    cv2.imshow("img", img)
    cv2.imshow("edges", edges)
    

def clamp(n: int, minn: int, maxn: int) -> int:
    return max(min(n, maxn), minn)


if __name__ == "__main__":
    imgs = list(cv2.imread(r"res/"+i)[110:390, 175:430] for i in os.listdir(r"res") if i.endswith(".png")) # list of cropped imgs
    currImgId = 0
    
    
    while True:
        key = cv2.waitKey(0)
        
        if key == ord('n'):
            currImgId = clamp(currImgId + 1, 0, len(imgs) - 1)
            cv2.destroyAllWindows()
        
        if key == ord('p'):
            currImgId = clamp(currImgId - 1, 0, len(imgs) - 1)
            cv2.destroyAllWindows()
            
        if key == 27:
            cv2.destroyAllWindows()
            break
        
        detect_face(imgs[currImgId])