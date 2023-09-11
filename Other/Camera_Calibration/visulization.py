from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
import os
import time
import imageio


def build_frame2(n):
    # 改变的数据
    x = np.linspace(-10, 10, 101)
    y = np.sin(x + n*np.pi/50)
    # 不变的长度
    p = plt.plot(x, y, color='blue')
    # 使用 blit 时，build frame 函数必须显式返回对象
    return p


fig = plt.figure()
ani = FuncAnimation(fig, build_frame2, interval=10, blit=True)
plt.show()

#
# # 执行 plt.show() 时会自动递增 build_frame 函数中的参数 n
# # 并将绘制的每一帧以时间间隔 interval(ms) 逐帧显示
# plt.show()