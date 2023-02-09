import cv2
import numpy as np
import os
import math


def find_color(color):
                        # 0 white
                        # 1 black
    colors = {2: 15,    # 2 red
              3: 22,    # 3 orange
              4: 33,    # 4 yellow
              5: 78,    # 5 green
              6: 96,    # 6 cyan
              7: 130,   # 7 blue
              8: 144}   # 8 violet
              #9: 170}   # 9 magenta
    
    h, s, v = color[0], color[1], color[2]
    if v < (255*0.5): # most likely black
        return 1
    elif s < (255*0.2) and v > (255*0.8): # most likely white
        return 0
    elif s < (255*0.35): # most likely fricking magenta
        return 9
    else:
        for key, hue in colors.items():
            if h < hue:
                return key
    return 2
    
def detect_colors(img) -> list:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    out_colors = []
    for x in range(2):
        for y in range(2):
            sector = hsv[x*100+40:x*100+50, y*100+40:y*100+50]
            color = np.mean(sector, axis=(0, 1))
            out_colors.append(find_color(color))
    out_colors = [[out_colors[0], out_colors[2]], [out_colors[1], out_colors[3]]]
    while not((out_colors[0][0] + out_colors[0][1] == 11 or out_colors[0][0] + out_colors[1][0] == 11) and out_colors[0][0] + out_colors[1][1] != 11):
         out_colors = list(map(list, zip(*out_colors[::-1])))
    return out_colors

def face_center(x, y, w, h):
    return np.array((x + w//2, y + h//2))

def detect_face(img: np.ndarray):
    # img = img[110:390, 175:430]
    
    perspective = np.float32([[172, 104],
                              [409, 104],
                              [428, 390],
                              [159, 390]])
    width  = 268
    height = 316
    unit_size = (4, 4)
    pts = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    matrix = cv2.getPerspectiveTransform(perspective, pts)
    croped_img = cv2.warpPerspective(img, matrix, (width, height))
    
    
    gaus_ksize = 7
    treshold1, treshold2 = 200, 100
    
    
    grsc = cv2.cvtColor(croped_img, cv2.COLOR_BGR2GRAY)
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
        box1 = np.int0(box)
        #cv2.drawContours(croped_img, [box1], 0, (0,0,255), 2)
        pts = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
        matrix = cv2.getPerspectiveTransform(box, pts)
        out = cv2.warpPerspective(croped_img, matrix, (width, height))
        
        out_coords = face_center(*cv2.boundingRect(cnts[cnt_id]))
        
        print(detect_colors(out))
        print(f"{out_coords[0]/croped_img.shape[0]:.2f}, {out_coords[1]/croped_img.shape[1]:.2f}")
        print(f"{out_coords[0]:.2f}, {out_coords[1]:.2f}")
        cv2.imshow(f"{cnt_id}", out)
    
    cv2.imshow("img", croped_img)
    cv2.imshow("edges", edges)
    

def clamp(n: int, minn: int, maxn: int) -> int:
    return max(min(n, maxn), minn)


if __name__ == "__main__":
    imgs = list(cv2.imread(r"res/"+i) for i in os.listdir(r"res") if i.endswith(".png")) # list of cropped imgs
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