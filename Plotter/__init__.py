from matplotlib import pyplot as plt
import numpy as np
import matplotlib.patches as patches
from math import sqrt,cos,sin,atan2,atan,exp,tan,pi
from matplotlib.animation import FuncAnimation
import json
import os

# plt.rcParams.update({
#     "text.usetex": True,
#     "font.family": "Helvetica"
# })

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['axes.unicode_minus']=False
plt.rcParams['text.usetex'] = True 

plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams['axes.linewidth'] = 1.2  # 坐标轴加粗
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'
plt.rcParams['xtick.major.size'] = 5
plt.rcParams['ytick.major.size'] = 5