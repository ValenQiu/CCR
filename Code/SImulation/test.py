import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.animation import FuncAnimation
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Rectangle
import numpy as np
import pandas as pd

def convert_angle_to_radian(angle):
    radian_angle = angle * math.pi / 180
    return radian_angle

# target
alpha_target = 120  # radian

# parameters of the continuum robot
l_bb = 300      # length of the backbone, mm
n_s = 10        # number of the spacer disks
r_td = 10.5     # distance of tendon routing and backbone, mm

alpha_bb = convert_angle_to_radian(alpha_target)    # angle of backbone, radian
r_bb = None         # arch radius of backbone

ls = l_bb       # segment length

# piecewise parameters between the spacer disks
lsp = ls / n_s
alpha_p = alpha_bb / n_s
r_bb = l_bb / n_s / alpha_bb

