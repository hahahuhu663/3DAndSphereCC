import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
import pandas as pd

d1 = pd.read_csv('csv_file.csv', header = None)
# d2 = pd.read_csv('d2.csv', header = None)
# d3 = pd.read_csv('d3.csv', header = None)
# d4 = pd.read_csv('d4.csv', header = None)
# d5 = pd.read_csv('d5.csv', header = None)
# d6 = pd.read_csv('d6.csv', header = None)
# d7 = pd.read_csv('d7.csv', header = None)
# d8 = pd.read_csv('d8.csv', header = None)


fig = plt.figure()
ax = Axes3D(fig)
ax.set_zlim(0, 40)
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_xlabel('x',fontsize=18)
ax.set_ylabel('y',fontsize=18)
ax.set_zlabel('z',fontsize=18)

# d1Len = len(d1)
ax.plot(d1[0],d1[1], d1[2], c='k', label='drone1')
# for x in range(d1Len):
#     ax.scatter(d1[0][x], d1[1][x], d1[2][x], c='k', label='drone1')
ax.plot(d1[3],d1[4], d1[5], c='y', label='drone2')
ax.plot(d1[6],d1[7], d1[8], c='g', label='drone3')
ax.plot(d1[9],d1[10], d1[11], c='c', label='drone4')
ax.plot(d1[12],d1[13], d1[14], c='b', label='drone5')
ax.plot(d1[15],d1[16], d1[17], c='m', label='drone6')
ax.plot(d1[18],d1[19], d1[20], c='r', label='drone7')
ax.plot(d1[21],d1[22], d1[23], c='#da8200', label='drone8')



plt.legend()
# plt.show()
plt.savefig('/home/robolab/python-test-project/result/trajectory.png')
# print(d1[0])
