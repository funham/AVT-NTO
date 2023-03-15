import cv2
import socket
import math
import pickle


class Client:
    __MAX_LENGTH = 65000
    __tcp_sock: socket.socket
    
    def __init__(self, udp_host: str, udp_port: int, tcp_host: str, tcp_port: int):
        self.__udp_host = udp_host
        self.__udp_port = udp_port
        self.__udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.__tcp_host = tcp_host
        self.__tcp_port = tcp_port

    def send_img(self, img: cv2.Mat):
        retval, buffer = cv2.imencode(".jpg", img)

        if not retval:
            raise ValueError("couldn't ecode image")

        buffer = buffer.tobytes()
        buffer_size = len(buffer)
        num_of_packs = math.ceil(buffer_size / Client.__MAX_LENGTH)

        frame_info = {"packs": num_of_packs}
        self.__udp_sock.sendto(pickle.dumps(frame_info), (self.__udp_host, self.__udp_port))

        left = 0
        right = Client.__MAX_LENGTH

        for _ in range(num_of_packs):
            # truncate data to send
            data = buffer[left:right]
            left = right
            right += Client.__MAX_LENGTH

            # send the frames accordingly
            self.__udp_sock.sendto(data, (self.__udp_host, self.__udp_port))
        
    @staticmethod
    def __tcp_reset(method):
        def wrapper(self, *args, **kwargs):
            self.__tcp_sock = socket.socket(socket.AF_INET,  # Internet
                                      socket.SOCK_STREAM)  # TCP
            self.__tcp_sock.connect((self.__tcp_host, self.__tcp_port))
            ret = method(self, *args, **kwargs)
            self.__tcp_sock.close()

            return ret

        return wrapper
    
    @__tcp_reset
    def recv_msg(self) -> str:
        msg = self.__tcp_sock.recv(1024)
        return msg.decode('utf-8')


if __name__ == "__main__":
    client = Client(udp_host='192.168.0.12', udp_port=5000, 
                    tcp_host='192.168.0.12', tcp_port=5001)
    cap = cv2.VideoCapture(0)

    while True:
        ret, img = cap.read()

        if not ret:
            print('bruh')
            continue

        client.send_img(img)
        
        try:
            msg = client.recv_msg()
            print(f'recieved command: {msg}')
        except ConnectionResetError:
            print("Connection reset by host")
            break