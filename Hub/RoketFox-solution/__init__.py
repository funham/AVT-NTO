import test_utils
from config import *
from detection_model import get_cnts_and_coords, face_center
from recognition_model import get_cropped_marks
from colors import get_color_by_hsv
import facility_api as fac
import facility_addon as facad


def preprocessing(img: np.ndarray) -> np.ndarray:
    '''
    Basic preprocessing of the image like crop, warp and resize.
    '''
    perspective = np.float32([[172, 104],  # left_top     | corners of the working area
                              [409, 104],  # right_top    |
                              [428, 390],  # right_bottom |
                              [159, 390]])  # left_bottom  |
    out_width, out_height = perspective.shape

    corners = np.float32(
        [[0, 0], [out_width, 0], [out_width, out_height], [0, out_height]])
    matrix = cv2.getPerspectiveTransform(perspective, corners)
    croped_img = cv2.warpPerspective(img, matrix, (out_width, out_height))
    return croped_img


def get_faces_and_coords(img: np.ndarray) -> tuple:
    '''
    Get the faces and their coordinates in pixels from the image.
    '''
    width, height = 200, 200  # dimensions of the output face

    cnts, coords = get_cnts_and_coords(img)

    faces = []
    for cnt_id in range(len(cnts)):
        rect = cv2.minAreaRect(cnts[cnt_id])
        box = cv2.boxPoints(rect)
        box = np.float32(box)
        corners = np.float32(
            [[0, 0], [width, 0], [width, height], [0, height]])
        matrix = cv2.getPerspectiveTransform(box, corners)
        face = cv2.warpPerspective(img, matrix, (width, height))
        faces.append(face)

    return faces, coords


def rotate_color_matrix(color_matrix: np.ndarray) -> np.ndarray:
    '''
    Rotate the color matrix until the rule is satisfied.

    m[[0,1], [2,3]]
    The rule: m[0] + m[1] = 11 and m[0] + m[3] != 11.
    '''
    while not (color_matrix[0][0] + color_matrix[0][1] == 11 and color_matrix[0][0] + color_matrix[1][1] != 11):
        color_matrix = list(map(list, zip(*color_matrix[::-1])))
    return color_matrix


def get_marking(face: np.ndarray) -> list:
    '''
    Get the marking from the face.
    '''
    cropped_sectors = get_cropped_marks(face)
    get_avg_color = partial(np.sum, axis=(0, 1))
    avg_colors = map(get_avg_color, cropped_sectors)
    colors = map(get_color_by_hsv, avg_colors)

    return np.reshape(colors, (2, 2))


def pix_to_coords(pix: np.ndarray, img_dimentions: tuple) -> np.ndarray:
    '''
    Convert the pixels to coordinates.
    '''
    coord_width, coord_height = 10, 15
    return (pix[0]/img_dimentions[0]*coord_width, pix[1]/img_dimentions[1]*coord_height)


def send_to_drone(coords: np.ndarray, hoarder_num: int) -> None:
    coords = pix_to_coords(coords)
    fac.MoveTo(coords)
    fac.MagnetFullDown()
    fac.MagnetOn()
    fac.MagnetUp()
    facad.moveToHoarder(hoarder_num)
    fac.MagnetOff()
    fac.CarriageStart()
    fac.LiftUp()
    fac.LiftDown()
    print("All is done")


def main():
    if not TESTING:
        cap = cv2.VideoCapture(0)
    else:
        cap = test_utils.ImageCapture('res')

    while True:
        ret, frame = cap.read()

        if not ret:
            print('Error: Cannot read frame')
            break

        frame = preprocessing(frame)
        faces, coords = get_faces_and_coords(frame)

        marks = map(get_marking, faces)

        for marking, coord in zip(marks, coords):
            if marking == TARGET_MARKING:  # can be changed in config.py
                send_to_drone(coord)
                break


if __name__ == "__main__":
    main()
