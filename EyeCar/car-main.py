"""
The entry point of a car side (executes on RaspberryPI):

Setting up LAN client, sending frames to server (PC), then recieving
corresponding command and sending it to the car's controller (Arduino)
by serial port.
"""

import cv2
from beholder2048squad.Client import Client
import serial

HOST_RAMZI = '192.168.0.12'
HOST_ONEPLUS = '192.168.17.46'

if __name__ == "__main__":
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = '/dev/ttyUSB0'
    ser.open()

    client = Client(udp_host=HOST_ONEPLUS, udp_port=5000, 
                    tcp_host=HOST_ONEPLUS, tcp_port=5001)
    
    cap = cv2.VideoCapture(0)

    while True:
        ret, img = cap.read()

        if not ret:
            print('bruh')
            continue

        client.send_img(img)
        
        try:
            msg = client.recv_msg()
            print(f'recieved command:\n{msg}')
            print('------------------')
        except ConnectionResetError:
            print("Connection reset by host")
            break
        except KeyboardInterrupt:
            print()
            print("Interrupted with keyboard")
            print("exiting")
            break

