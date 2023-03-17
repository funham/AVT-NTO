import cv2
import time
from arduino import Arduino

CAMERA_ID = '/dev/video0'
CAR_SPEED = 1430
ARDUINO_PORT = '/dev/ttyUSB0'
arduino = Arduino(ARDUINO_PORT, baudrate=115200, timeout=10)
time.sleep(1)
arduino.set_angle(90)

captured = 0

# открываем видеокамеру
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

def cross_intersection():
    arduino.dist(CAR_SPEED, 400)

def get_trafficlight_img(frame):
    x1, y1 = (2, 23)
    x2, y2 = (4, 5)

    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2, cv2.LINE_AA)
    cv2.imwrite(f'tl_images/rects/img{captured}.png', frame)

    cropped = frame[y1:y2, x1:x2]
    cv2.imwrite(f'tl_images/cropped/img{captured}.png', cropped)

    return cropped


def get_signal(cropped):
    ...

for _ in range(10):
    cap.read()

while True:
    # получаем кадр
    ret, frame = cap.read()
    if not ret:
        break

    cropped = get_trafficlight_img(frame)
    is_red = get_signal(cropped)

    if is_red:
        cross_intersection()

