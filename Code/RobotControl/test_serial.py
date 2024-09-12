import time
import serial

# Open the serial port
ser = serial.Serial('COM3', 9600)

# Add a delay to ensure the device is ready
time.sleep(1)  # Adjust the time as necessary

# Send the command to control motor speed to 50%
# command = [0x55, 0x55, 0x08, 0x03, 0x01, 0xf4, 0x01, 0xb, 0xe8, 0x03]
# command = [85, 85, 8, 3, 1, 244, 1, 11, 232, 3]     # 1 motor, 1000
# command = [85, 85, 8, 3, 1, 244, 1, 11, 244, 1]    # 1 motor, 500
# command = [85, 85, 11, 3, 2, 244, 1, 11, 244, 1, 12, 244, 1]    # 2 motors, 500 500
# command = [85, 85, 11, 3, 2, 244, 1, 11, 232, 3, 12, 232, 3]    # 2 motors, 1000 1000
command = [85, 85, 17, 3, 2, 244, 1, 11, 232, 3, 12, 232, 3, 21, 232, 3, 22, 232, 3]    # 4 motors, 1000
# command = [85, 85, 17, 3, 2, 244, 1, 11, 244, 1, 12, 244, 1, 21, 244, 1, 22, 244, 1]    # 4 motors, 500
print(command)
ser.write(command)
time.sleep(0.31)

# Close the serial port
ser.close()