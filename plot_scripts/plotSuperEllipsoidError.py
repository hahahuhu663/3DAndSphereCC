import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
import pandas as pd

folder_name = 'superEllipsoid2'
error = pd.read_csv(folder_name+'/droneOneObstacles.csv', header = None)

errorLen = len(error)

# time = range(errorLen)        

column = len(error.columns)
# objectNumber = 16
for i in range(1,17):
    objectNumber = i
    plt.plot(error[32], error[objectNumber*2 -2], label='Minimum Distance')
    # plt.plot(error[3], error[1], label='Minimum Distance')

    # plt.axhline(y = 8.7, color = 'r', linestyle = '--', label='Safety Radius')
    plt.plot(error[32], error[objectNumber*2 -1], color = 'r', linestyle = '--', label='Safety Radius')


    x_max = max(error[1])
    # y_max = max(error[0])
    y_max = max(error[0])
    plt.xlabel('time (sec)')
    plt.ylabel('cbf error (m)')
    plt.xlim([0, 250])
    plt.ylim([6, 30])
    plt.legend(loc='center right')
    # plt.xticks(np.arange(min(error[1]), max(error[1])+1, 2))
    # plt.yticks(np.arange(min(error[0]), max(error[0])+1, 4))
    # plt.legend(loc='upper right')

    plt.savefig('/home/robolab/plot_result/'+ folder_name +'/error_cbf_drone1_'+ str(objectNumber) +'.png')
    plt.close()
# plt.savefig('/home/robolab/plot_result/'+ folder_name +'/error_cbf_obs.png')
# plt.show()
