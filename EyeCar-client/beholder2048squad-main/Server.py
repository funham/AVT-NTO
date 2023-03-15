import cv2
import socket
import pickle
import numpy as np
from typing import Iterator


class Server:
    __conn: socket.socket | None = None

    __MAX_LENGTH = 65540

    def __init__(self, udp_host: str, udp_port: int, tcp_host: str, tcp_port: int):
        self.__udp_host = udp_host
        self.__udp_port = udp_port

        self.__tcp_host = tcp_host
        self.__tcp_port = tcp_port

        self.__udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__udp_sock.bind((self.__udp_host, self.__udp_port))

        self.__tcp_sock = socket.socket(socket.AF_INET,  # Internet
                                  socket.SOCK_STREAM)  # TCP

        self.__tcp_sock.bind((self.__tcp_host, self.__tcp_port))
        self.__tcp_sock.listen()

    def __recv_packs(self, num_packs: int) -> Iterator[bytes]:
        for _ in range(num_packs):
            data, addr = self.__udp_sock.recvfrom(Server.__MAX_LENGTH)
            yield data

    def recv_img(self) -> cv2.Mat | None:
        data, address = self.__udp_sock.recvfrom(Server.__MAX_LENGTH)
        
        try:
            frame_info = pickle.loads(data)
        except:
            return None

        if len(data) >= 100 or not frame_info:
            return None

        buffer = b''.join(self.__recv_packs(num_packs=frame_info["packs"]))

        frame = np.frombuffer(buffer, dtype=np.uint8)
        frame = frame.reshape(frame.shape[0], 1)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        return frame
    
    def __tcp_reset(method):
        def wrapper(self, *args, **kwargs):
            self.__conn, (client_name, client_port) = self.__tcp_sock.accept()
            ret = method(self, *args, **kwargs)
            self.__conn.close()

            return ret
        
        return wrapper

    @__tcp_reset
    def send_msg(self, msg: str):
        self.__conn.sendall(msg.encode('utf-8'))


if __name__ == "__main__":
    server = Server(udp_host="0.0.0.0", udp_port=5000,
                    tcp_host="192.168.0.12", tcp_port=5001)
    
    while True:
        frame = server.recv_img()
        frame = cv2.flip(frame, 1)

        if frame is not None:
            cv2.imshow("Stream", frame)
            if cv2.waitKey(1) == 27:
                break

        server.send_msg('kekw :3')

    cv2.destroyAllWindows()