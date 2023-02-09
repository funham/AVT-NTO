from utils import *
from config import *
from detection_model import get_detected_objects
from recognition_model import get_cropped_marks



def img_preprocessing(img:np.ndarray) -> np.ndarray:
    ...
    
def get_marks_and_coords(img:np.ndarray) -> tuple:
    get_detected_objects()

def define_color() -> int:
    ...
    
def get_marks():
    get_cropped_marks()
    define_color()
    
def coords_to_commands():
    ...

def main(img):
    img_preprocessing()
    get_marks_and_coords()
    get_marks()


if __name__ == "__main__":
    imgs_folder = "res"
    imgs_l = list(cv2.imread(f"{imgs_folder}/{name}") for name in os.listdir(r"res") if name.endswith(".png"))
    currImgId = 0
    
    
    while True:
        key = cv2.waitKey(0)
        
        if key == ord('n'):
            currImgId = clamp(currImgId + 1, 0, len(imgs_l) - 1)
            cv2.destroyAllWindows()
        
        if key == ord('p'):
            currImgId = clamp(currImgId - 1, 0, len(imgs_l) - 1)
            cv2.destroyAllWindows()
            
        if key == 27:
            cv2.destroyAllWindows()
            break
        
        main(imgs_l[currImgId])