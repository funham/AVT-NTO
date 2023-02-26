import cv2
import socket
import math
import pickle


class Client:
    MAX_LENGTH = 65000
    tcp_sock: socket.socket
    
    def __init__(self, udp_host: str, udp_port: int, tcp_host: str, tcp_port: int):
        self.udp_host = udp_host
        self.udp_port = udp_port
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.tcp_host = tcp_host
        self.tcp_port = tcp_port

    def send_img(self, img: cv2.Mat):
        retval, buffer = cv2.imencode(".jpg", img)

        if not retval:
            raise ValueError("couldn't ecode image")

        buffer = buffer.tobytes()
        buffer_size = len(buffer)
        num_of_packs = math.ceil(buffer_size / Client.MAX_LENGTH)

        frame_info = {"packs": num_of_packs}
        self.udp_sock.sendto(pickle.dumps(frame_info), (self.udp_host, self.udp_port))

        left = 0
        right = Client.MAX_LENGTH

        for _ in range(num_of_packs):
            # truncate data to send
            data = buffer[left:right]
            left = right
            right += Client.MAX_LENGTH

            # send the frames accordingly
            self.udp_sock.sendto(data, (self.udp_host, self.udp_port))
        
    @staticmethod
    def __tcp_reset(method):
        def wrapper(self, *args, **kwargs):
            self.tcp_sock = socket.socket(socket.AF_INET,  # Internet
                                      socket.SOCK_STREAM)  # TCP
            self.tcp_sock.connect((self.tcp_host, self.tcp_port))
            ret = method(self, *args, **kwargs)
            self.tcp_sock.close()

            return ret

        return wrapper
    
    @__tcp_reset
    def recv_msg(self) -> str:
        msg = self.tcp_sock.recv(1024)
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
