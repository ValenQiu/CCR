import numpy as np
import time
from Controller import Controller

# parameters of the robot
port = 'COM3'       # port number
m1x = 11            # motor id for the 1st segment, x axis
m1y = 12            # motor id for th 1st segment, y axis
m2x = 21            # motor id for the 2nd segment, x axis
m2y = 22            # motor id for th 2nd segment, y axis

controller = Controller(port)

# This code is aim to let the robt to draw a circle in a 2D plane
omega = np.linspace(0, 2*np.pi, 100)
A1 = 500       # amplitude of the segment 1's circle, represents the radius of segment 1
A2 = 500        # amplitude of the segment 2's circle, represents the radius of segment 2
x1 = A1 * np.cos(omega)
y1 = A1 * np.sin(omega)
x2 = A2 * np.cos(omega)
y2 = A2 * np.sin(omega)

x1 = x1.astype(int) + 500
y1 = y1.astype(int) + 500
x2 = x2.astype(int) + 500
y2 = y2.astype(int) + 500

print(x1, y1, x2, y2)

speed = 200

# initial the position
controller.set_servo_position(m1x, 1000, speed)
controller.set_servo_position(m1y, 500, speed)
controller.set_servo_position(m2x, 1000, speed)
controller.set_servo_position(m2y, 500, speed)
time.sleep(3)

for i in range (0, len(x1)):
    controller.set_servo_position(m1x, x1[i], speed)
    controller.set_servo_position(m1y, y1[i], speed)
    controller.set_servo_position(m2x, x2[i], speed)
    controller.set_servo_position(m2y, y2[i], speed)
    time.sleep(0.2)

# while True:
#     """
#     the range of servo position should be between 400-640
#     """
#     controller.set_servo_position(11, 500, 500) # 1为舵机ID。500为舵机中位位置。800为运行时间，单位毫秒。
#     controller.set_servo_position(12, 500, 800)  # 1为舵机ID。500为舵机中位位置。800为运行时间，单位毫秒。
#     time.sleep(2)  # 延时2s

#     """read the servo id"""
#     hand_pos = controller.get_servo_position(1)
#     print(hand_pos)

#     controller.set_servo_position(11, 0, 500)
#     controller.set_servo_position(12, 400, 500)
#     time.sleep(2) # 延时2s

#     """read the servo id"""
#     hand_pos = controller.get_servo_position(1)
#     print(hand_pos)

#     controller.set_servo_position(11, 1000, 1000)
#     controller.set_servo_position(12, 640, 1000)
#     time.sleep(2) # 延时2s

#     """read the servo id"""
#     hand_pos = controller.get_servo_position(1)
#     print(hand_pos)
