from config import *

def get_cropped_marks(img: np.ndarray) -> Generator:
    '''
    Returns a list of cropped images with uniform color.
    '''
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    for x, y in product(range(2), repeat=2):
            sector = img_hsv[x*100+40:x*100+50, y*100+40:y*100+50]
            yield sector