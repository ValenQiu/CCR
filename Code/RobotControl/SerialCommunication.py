import sys
import serial
import serial.tools.list_ports

ports_list = list(serial.tools.list_ports.comports())  # 获取所有串口设备实例
if len(ports_list) <= 0:
    print("No serial devices avaliable!")
else:
    print("avaliable serial devices:")
    for port in ports_list:  # 依次输出每个设备对应的串口号和描述信息
        print(list(port)[0], list(port)[1])  # COM4 USB-SERIAL CH340 (COM4)

class SerialCommunication:
    def __init__(self, port, baudrate=9600, timeout=1):
        """Initialize serial communication."""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.connect()

    def connect(self):
        """Establish the serial connection."""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")

    def send(self, data):
        """Send data through the serial port."""
        if self.serial and self.serial.is_open:
            self.serial.write(data)
            print(f"Sent: {[hex(byte) for byte in data]}")
        else:
            print("Serial port is not open.")

    def read(self):
        """Read data from the serial port."""
        if self.serial and self.serial.in_waiting > 0:
            response = self.serial.read(self.serial.in_waiting)
            print(f"Received: {[hex(byte) for byte in response]}")
            return response
        else:
            print("No data available.")
            return None

    def close(self):
        """Close the serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Serial connection closed.")