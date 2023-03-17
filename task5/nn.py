import cv2
import yolopy
import time
from arduino import Arduino

CAMERA_ID = '/dev/video0'
CAR_SPEED = 1430
ARDUINO_PORT = '/dev/ttyUSB0'
TRAFFIC_LIGHT_CLASSID = 0

# путь до кватнтованной модели
model_file = '../anything_yolov4-tiny-obj_uint8.tmfile'
# загружаем модель. 
# use_uint8 должно быть True, если ваша модель квантована. 
# use_timvx должно быть True, если вы запускаете модель на NPU.
# cls_num должно быть равно количеству классов, на которых вы обучали сеть
model = yolopy.Model(model_file, use_uint8=True, use_timvx=True, cls_num=1)
arduino = Arduino(ARDUINO_PORT, baudrate=115200, timeout=10)
time.sleep(1)

# открываем видеокамеру
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

def cross_intersection():
    arduino.set_angle(90)
    arduino.dist(CAR_SPEED, 400)

def get_signal():
    ...

while True:
    # получаем кадр
    ret, frame = cap.read()
    if not ret:
        break
    
    # прогон изображения через нейронную сетьd
    # classes хранит список из id классов всех сдетектированных объектов
    # scores уверенность в точности распознавания объектов
    # boxes - список рамок для всех объектов 
    classes, scores, boxes = model.detect(frame)

    detections = zip(classes, scores, boxes)
    print(detections)

    for det in detections:
        cls, scr, box = det

        if cls == TRAFFIC_LIGHT_CLASSID:
            if get_signal() == True:
               cross_intersection()

    # печатаем id классов объектов
