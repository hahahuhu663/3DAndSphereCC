import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
import pandas as pd

folder_name = 'sym2'
error = pd.read_csv(folder_name+'/error.csv', header = None)

errorLen = len(error)

# time = range(errorLen)        

column = len(error.columns)

plt.plot(error[3], error[0], label='Minimum Distance')
# plt.plot(error[3], error[1], label='Minimum Distance')

# plt.axhline(y = 8.7, color = 'r', linestyle = '--', label='Safety Radius')
plt.axhline(y = 5.5, color = 'r', linestyle = '--', label='Safety Radius')


x_max = max(error[1])
# y_max = max(error[0])
y_max = max(error[0])
plt.xlabel('time (sec)')
plt.ylabel('cbf error (m)')
plt.xlim([0, 150])
plt.ylim([5, 20])
plt.legend(loc='center right')
# plt.xticks(np.arange(min(error[1]), max(error[1])+1, 2))
# plt.yticks(np.arange(min(error[0]), max(error[0])+1, 4))
# plt.legend(loc='upper right')

plt.savefig('/home/robolab/plot_result/'+ folder_name +'/error_cbf_drone.png')
# plt.savefig('/home/robolab/plot_result/'+ folder_name +'/error_cbf_obs.png')
# plt.show()
