"""
IOClient interface takes care of all the IO communication with the server
such as reading frames from the camera and sending commands to a client.
"""


from enum import Enum
from abc import ABC, abstractmethod
from typing import Iterator

import time
import beholder2048squad.Server
import cv2
import os
import cfg


class InputType(Enum):
    LOCAL_CAMERA = 0
    IMAGE_FOLDER = 1
    SERVER_CAMERA = 2
    VIDEO_PLAYER = 3


class IOClient(ABC):
    def read(self) -> cv2.Mat | None:
        print('-----------------')

    def send_msg(self, command: str) -> None:
        print(f'Sending command:\n{command}')

    def handle_keyboard_input(self):
        key = cv2.waitKey(1)

        if key in (27, ord('q')):
            raise StopIteration


class LocalCameraClient(IOClient):
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_W)

    def read(self) -> cv2.Mat | None:
        super().read()
        ret, frame = self.cap.read()

        return cv2.resize(frame, cfg.IMG_SHAPE) if ret else None
    
class VideoPlayerClient(IOClient):
    def __init__(self, path, fps):
        self.cap = cv2.VideoCapture(path)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_H)

        self.path = path
        self.fps = fps
        self.paused = False

    def read(self) -> cv2.Mat | None:
        super().read()
        ret, frame = self.cap.read()
        time.sleep(1/self.fps)

        if not ret:
            self.cap = cv2.VideoCapture(self.path)
            ret, frame = self.cap.read()

        return cv2.resize(frame, cfg.IMG_SHAPE) if ret else None
    
    def handle_keyboard_input(self):
        key = cv2.waitKey(1)
        
        if key == ord('p'):
            self.paused = not self.paused
            key = cv2.waitKey(0)
        
        if key in (27, ord('q')):
            raise StopIteration

        if key == ord('r'):
            self.paused = False


class ImageFolderClient(IOClient):
    def __init__(self, path):
        self.path = path
        self.reader = self.__reader()

    def read(self) -> cv2.Mat | None:
        super().read()
        return next(self.reader)

    def handle_keyboard_input(self):
        key = None
        while key not in (27, ord('q'), ord('n'), ord('p')):
            if key is not None:
                print('Unknown key. Are you on ENG layout?')
            key = cv2.waitKey(0)

        if key in (ord('q'), 27):
            raise StopIteration

        self.last_pressed_key = key

    def __reader(self) -> Iterator[cv2.Mat | None]:
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


class ServerCameraClient(IOClient):
    def __init__(self, udp_host: str,
                 udp_port: int,
                 tcp_host: str,
                 tcp_port: int):

        self._server = beholder2048squad.Server.Server(
            udp_host, udp_port, tcp_host, tcp_port)

    def read(self) -> cv2.Mat | None:
        super().read()
        return cv2.resize(self._server.recv_img(), cfg.IMG_SHAPE)

    def send_msg(self, command: str) -> None:
        super().send_msg(command)
        self._server.send_msg(command)


def get_io_client(in_mode) -> IOClient:
    if in_mode == InputType.LOCAL_CAMERA:
        return LocalCameraClient()
    elif in_mode == InputType.IMAGE_FOLDER:
        return ImageFolderClient(path=cfg.IMG_SOURCE_FOLDER_PATH)
    elif in_mode == InputType.SERVER_CAMERA:
        return ServerCameraClient(udp_host=cfg.UDP_HOST, udp_port=cfg.UDP_PORT,
                                  tcp_host=cfg.TCP_HOST, tcp_port=cfg.TCP_PORT)
    elif in_mode == InputType.VIDEO_PLAYER:
        return VideoPlayerClient(path=f'{cfg.VIDEO_SOURCE_PATH}/{cfg.VIDEO_SOURCE_FILE}', fps=cfg.FPS)

    raise ValueError('Invalid input mode')
