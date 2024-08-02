import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
import pandas as pd


folder_name = 'asym3'
error = pd.read_csv(folder_name + '/droneError.csv', header = None)

errorLen = len(error)

time = range(errorLen)

column = len(error.columns)

for x in range(column-1):
    plt.plot(error[8], error[x], label='drone'+str(x))

# plt.plot(error[8], error[7], label='drone'+str(3))
# x_max = max(error[errorLen-1])
# y_max = max(error[0])
plt.xlabel('time (sec)')
plt.ylabel('error (m)')
plt.xlim([0, 150])
plt.ylim([0, 25])           
plt.legend(loc='center right')

plt.savefig('/home/robolab/plot_result/'+folder_name+'/droneError_checkSum.png')
# plt.show()
