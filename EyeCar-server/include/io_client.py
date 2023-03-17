"""
IOClient interface takes care of all the IO communication with the server
such as reading frames from the camera and sending commands to a client.
"""


from enum import Enum
from abc import ABC, abstractmethod
from typing import Iterator, Optional, Union
from include.arduino import Arduino
from include.vid_writer import VideoWriter

import time
import cv2
import os
import cfg


class InputType(Enum):
    LOCAL_CAMERA = 0
    IMAGE_FOLDER = 1
    SERVER_CAMERA = 2
    VIDEO_PLAYER = 3
    EYECAR = 4


class IOClient(ABC):
    def read_frame(self):
        print('-----------------')

    def send_msg(self, command: str) -> None:
        print(f'Sending command:\n{command}')


    def handle_keyboard_input(self):
        key = cv2.waitKey(1)

        if key in (27, ord('q')):
            raise StopIteration(f'[{chr(key) if key!=27 else "ESC"}] pressed, exiting...')


class LocalCameraClient(IOClient):
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_W)

    def read_frame(self) :
        super().read_frame()
        ret, frame = self.cap.read()

        return cv2.resize(frame, cfg.IMG_SHAPE) if ret else None
    

class VideoPlayerClient(IOClient):
    def __init__(self, path, fps):
        self.path = path
        self.fps = fps
        self.paused = False

        self._reset()

    def _reset(self):
        self.cap = cv2.VideoCapture(self.path)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_H)

    def read_frame(self) :
        super().read_frame()

        ret, frame = self.cap.read()
        time.sleep(1/self.fps)

        if not ret:
            print("[VideoPlayerClient]: The end of the video.\n")
            raise StopIteration

        return cv2.resize(frame, cfg.IMG_SHAPE) if ret else None
    
    def handle_keyboard_input(self):
        key = cv2.waitKey(1)
        
        if key == ord('p'):
            self.paused = not self.paused
            key = cv2.waitKey(0)
        
        if key in (27, ord('q')):
            raise StopIteration(f'[{chr(key) if key!=27 else "ESC"}] pressed, exiting...')


        if key == ord('r'):
            self._reset()


class ImageFolderClient(IOClient):
    def __init__(self, path):
        self.path = path
        self.reader = self.__reader()

    def read_frame(self) :
        super().read_frame()
        return next(self.reader)

    def handle_keyboard_input(self):
        key = None
        while key not in (27, ord('q'), ord('n'), ord('p')):
            if key is not None:
                print('Unknown key. Are you on ENG layout?')
            key = cv2.waitKey(0)

        if key in (ord('q'), 27):
            raise StopIteration(f'[{chr(key) if key!=27 else "ESC"}] pressed, exiting...')


        self.last_pressed_key = key

    def __reader(self):
        path_list = [path for path in os.listdir(
            self.path) if path.endswith('.png') or path.endswith('.jpg')]

        img_idx = 0
        path = f'{self.path}/{path_list[img_idx]}'

        yield cv2.resize(cv2.imread(path), cfg.IMG_SHAPE)

        while True:
            if self.last_pressed_key == ord('n'):
                img_idx += 1

            if self.last_pressed_key == ord('p'):
                img_idx -= 1

            img_idx = img_idx % len(path_list)

            path = f'{self.path}/{path_list[img_idx]}'
            img = cv2.imread(path)

            yield cv2.resize(img, cfg.IMG_SHAPE)


class EyeCarClient(IOClient):
    def __init__(self):
        self.arduino = Arduino(cfg.ARDUINO_PORT, baudrate=115200, timeout=10)
        time.sleep(1)
        self.cap = cv2.VideoCapture(cfg.CAMERA_ID, cv2.CAP_V4L2)

    def read_frame(self) :
        ret, frame = self.cap.read()
        
        if not ret:
            return None

        VideoWriter().write('run', frame)        
        frame = cv2.resize(frame, cfg.IMG_SHAPE)

        return frame

    def send_msg(self, command: str) -> None:
        super().send_msg(command)

        if command == "PARK":
            self._park_car()
            if cfg.DROP_CARGO:
                self._drop_cargo()
            raise StopIteration("Endpoint reached")

        for cmd in command.split('\n'):
            sign, args = cmd[:5], cmd[6:]
            
            if sign == 'SPEED':
                k = 1
                b = 0
                speed = int(args) * k + b
                
                if speed < 30:
                    self.arduino.stop()
                    continue

                self.arduino.set_speed(speed)

            elif sign == 'ANGLE':
                angle = int(args)
                self.arduino.set_angle(90 - angle)

    def _park_car(self):
        self.arduino.set_speed(cfg.CAR_MIN_SPEED)
        self.arduino.set_angle(90 + 10)
        time.sleep(2)
        self.arduino.set_angle(90 - 10)
        time.sleep(2)

        self.arduino.set_angle(90)
        self.arduino.stop()

    def _drop_cargo(self):
        self.arduino.drop()

            

def create_io_client(in_mode, args) -> IOClient:
    if in_mode == InputType.LOCAL_CAMERA:
        return LocalCameraClient()
    elif in_mode == InputType.IMAGE_FOLDER:
        return ImageFolderClient(path=args.img_source_folder_path)
    elif in_mode == InputType.VIDEO_PLAYER:
        return VideoPlayerClient(path=os.path.join(args.video_source_path, args.video_source_file), fps=cfg.FPS)
    elif in_mode == InputType.EYECAR:
        return EyeCarClient()

    raise ValueError('Invalid input mode')
