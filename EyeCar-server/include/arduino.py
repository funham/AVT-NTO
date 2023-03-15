import serial


class Arduino:  # класс ардуино! При создании объекта, задаёт порт и скорость обмена данными, для общения по UART
    def __init__(self, port, baudrate=9600, timeout=None, coding='utf-8'):
        self.serial_connected = False
        self.serial = serial.Serial(port, baudrate=baudrate, timeout=timeout, write_timeout=1)

        #self.serial.flush()
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        self.serial_connected = True
        self.coding = coding

    def send_data(self, data: str):  # метод класса для отправки данных через UART
        print('Sent to Arduino:', data)
        data += '\n'
        try:
            self.serial.write(data.encode(self.coding))
        except serial.SerialTimeoutException:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

    def read_data(self):  # метод класса для чтения данных через UART !!! Не используется в программе. !!!
        line = self.serial.readline().decode(self.coding).strip()
        return line

    def set_speed(self, speed):
        msg = f'GO:{speed}'
        self.send_data(msg)

    def set_angle(self, angle):
        msg = f'ANGLE:{angle}'
        self.send_data(msg)

    def dist(self, speed, turn):
        msg = f'DIST:{speed}:{turn}'
        self.send_data(msg)

    def stop(self):
        msg = 'STOP'
        self.send_data(msg)

    def drop(self):
        msg = 'DROP'
        self.send_data(msg)

    def __del__(self):
        if self.serial_connected:
            self.serial.close()
