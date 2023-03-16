import cfg
import cv2


class Stream:
    def __init__(self, name: str, shape: tuple):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.name = name
        self.cap = cv2.VideoWriter(f'res/vids/{name}.avi', fourcc, 20, shape[:2][::-1])
        self.img_shape = shape

class VideoWriter:
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(VideoWriter, cls).__new__(cls)
            cls.streams = {}
        return cls.instance

    def write(self, name: str, frame: cv2.Mat):
        if name not in self.streams:
            self.streams[name] = Stream(name, frame.shape)
        
        if frame.shape != self.streams[name].img_shape:
            raise ValueError(f'Wrong window shape. Expected {self.window_shapes[name]}. Got {frame.shape[:2][::-1]}')
        
        self.streams[name].cap.write(frame)
    
    def __del__(self):
        print("releasing streaming...")
        for stream in self.streams.values():
            stream.cap.release()
