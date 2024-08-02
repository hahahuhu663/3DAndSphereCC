import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
import pandas as pd

error = pd.read_csv('error.csv', header = None)

errorLen = len(error)

time = range(errorLen)

column = len(error.columns)

for x in range(column):
    plt.plot(time, error[x], label='drone'+str(x))

plt.xlabel('time')
plt.ylabel('error')
plt.legend(loc='upper right')

plt.savefig('/home/robolab/plot_result/cvt_error/error.png')
# plt.show()
