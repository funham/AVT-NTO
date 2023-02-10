from config import *


def face_center(x, y, w, h):
    return np.array((x + w//2, y + h//2))

def get_cnts_and_coords(img) -> tuple:
    '''
    Returns the contours and coordinates (in pixels) of the objects in the image.
    '''
    gaus_ksize = 7
    canny_tres1, canny_tres2 = 200, 100
    
    grsc = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gaus = cv2.GaussianBlur(grsc, (gaus_ksize, gaus_ksize), 0)
    edges = cv2.Canny(gaus, canny_tres1, canny_tres2)
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
    
    cnts = tuple(cnts[i] for i in range(len(cnts)) if i not in bad_cnts) # keeps only the good cnts
    centers  = list(map(lambda cnt: face_center(*cv2.boundingRect(cnt)), cnts))
    return cnts, centers

if __name__ == '__main__':
    print(get_cnts_and_coords(cv2.imread(r"res\hub1.png")))