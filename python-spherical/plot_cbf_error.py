import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
import pandas as pd
import os

folderName = "/home/robolab/python-spherical/15Jul_rand_im/"
# error = pd.read_csv('cbf_center_error.csv', header = None)
error = pd.read_csv(folderName + 'cbf_obs_error.csv', header = None)

# error = pd.read_csv('error8drones.csv', header = None)

errorLen = len(error)

time = range(errorLen)

column = len(error.columns)

for i in range(column):
    plt.plot(time, error[i])
plt.axhline(y = 5, color = 'r', linestyle = '--')

plt.xlabel('Time (sec)')
# plt.ylabel('Distance to Center')
plt.ylabel('Min. Distance to Obstacles (m)')
plt.ylim([4, 10])
plt.xlim([0, 299])
# plt.legend(loc='upper right')
plt.savefig(folderName+'distance_to_obs.png')

plt.cla()

droneError = pd.read_csv(folderName+'cbf_drone_error.csv', header = None)
droneErrorLen = len(droneError)

time = range(droneErrorLen)

column = len(droneError.columns)

for i in range(column):
    plt.plot(time, droneError[i], label='drone{0}'.format("{:02d}".format(i+1)))
plt.axhline(y = 3.5, color = 'r', linestyle = '--' )

plt.xlabel('Time (sec)')
plt.ylabel('Min. Distance to Other Agents (m)')
plt.ylim([3, 23])
plt.xlim([0, 299])
# plt.legend(loc='lower right')
plt.savefig(folderName+'distance_to_drone.png')
plt.cla()

centerError = pd.read_csv(folderName+'cbf_center_error.csv', header = None)
centerErrorLen = len(centerError)

time = range(centerErrorLen)

column = len(centerError.columns)
colors = ['k','y','g','c','b','m','r','#da8200']

for i in range(column):
    plt.plot(time, centerError[i], label='drone{0}'.format("{:02d}".format(i+1)), color=colors[i])
plt.axhline(y = 20, color = 'r', linestyle = '--')

plt.xlabel('Time (sec)')
plt.ylabel('Min. Distance to Center (m)')
plt.ylim([19, 23])
plt.xlim([0, 299])
plt.legend(loc='upper right')
plt.savefig(folderName+'distance_to_center.png')
plt.cla()