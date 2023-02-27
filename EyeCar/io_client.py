from enum import Enum
from abc import ABC, abstractmethod
from typing import *

import beholder2048squad.Server
import cv2
import os
import cfg

class InputType(Enum):
    LOCAL_CAMERA = 0
    IMAGE_FOLDER = 1
    SERVER_CAMERA = 2



class IOClient(ABC):
    @abstractmethod
    def read(self) -> cv2.Mat | None:
        pass

    @abstractmethod
    def send_msg(self, command: str):
        pass

    def handle_keyboard_input(self):
        key = cv2.waitKey(1)

        if key in (27, ord('q')):
            raise StopIteration


class LocalCameraClient(IOClient):
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    def read(self) -> cv2.Mat | None:
        ret, frame = self.cap.read()
    
        return frame if ret else None
    
    def send_msg(self, command: str) -> None:
        print(f'Sending command:\n {command}')
        print('-----------------')

class ImageFolderClient(IOClient):
    def __init__(self, path):
        self.path = path
        self.reader = self.__reader()

    def read(self) -> cv2.Mat | None:
        return next(self.reader)
    
    def send_msg(self, command: str) -> None:
        print('-----------------')
        print(f'Sending command:\n {command}')

    def handle_keyboard_input(self):
        pass

    def __reader(self) -> Iterator[cv2.Mat | None]:
        path_list = [path for path in os.listdir(
            self.path) if path.endswith('.png') or path.endswith('.jpg')]

        img_idx = 0
        path = f'{self.path}\{path_list[img_idx]}'

        yield cv2.imread(path)

        while True:
            key = cv2.waitKey(0)

            if key == 27 or key == ord('q'):
                cv2.destroyAllWindows()
                yield None

            if key == ord('n'):
                img_idx += 1

            if key == ord('p'):
                img_idx -= 1

            img_idx = img_idx % len(path_list)

            path = f'{self.path}\{path_list[img_idx]}'
            img = cv2.imread(path)
            yield img


class ServerCameraClient(IOClient):
    def __init__(self, udp_host: str,
                       udp_port: int,
                       tcp_host: str,
                       tcp_port: int):
        
        self._server = beholder2048squad.Server.Server(udp_host, udp_port, tcp_host, tcp_port)

    def read(self) -> cv2.Mat | None:
        return self._server.recv_img()
    
    def send_msg(self, command: str) -> None:
        self._server.send_msg(command)


def get_io_client(in_mode) -> IOClient:
    if in_mode == InputType.LOCAL_CAMERA:
        return LocalCameraClient()
    elif in_mode == InputType.IMAGE_FOLDER:
        return ImageFolderClient(path=cfg.IMG_SOURCE_FOLDER_PATH)
    elif in_mode == InputType.SERVER_CAMERA:
        return ServerCameraClient(udp_host=cfg.UDP_HOST, udp_port=cfg.UDP_PORT,
                                  tcp_host=cfg.TCP_HOST, tcp_port=cfg.TCP_PORT)
    
    raise ValueError('Invalid input mode')