import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import sys

'''
v0.1 Oct. 7, 2017
  - calc Voronoi (3D)
'''

inx = np.linspace(0, 2, 3, endpoint=True)
iny = np.linspace(0, 2, 3, endpoint=True)
inz = np.linspace(0, 2, 3, endpoint=True)
# print(np.meshgrid(inx, iny, inz))
mx, my, mz = np.meshgrid(inx, iny, inz)

arr = []
for ax in range(len(mx)):
    for ay in range(len(my)):
        for az in range(len(mz)):
            arr += [[mx[ax][ay][az], my[ax][ay][az], mz[ax][ay][az]]]
print(arr)
points = np.array(arr)
vor = Voronoi(points)

#voronoi_plot_2d(vor)
#plt.show()

#for idx, pts in enumerate(vor.points):
#    print("point " + str(idx) + "(" + str(pts) + ")")

for idx, vts in enumerate(vor.vertices):
    print("vertices " + str(idx) + "(" + str(vts) + ")")
# points = np.array([[0, 1, 1], [0, 2, 0], [1, 0, 1], [1, 1, 0], [1, 2, 1]])
# # points = np.array([[-0.5, -0.5, -0.5], [0.5, -0.5, 0.5],[0,0,0] , [-0.5, 0.5, -0.5], [0.5, 0.5, 0.5]])
#
#
# vor = Voronoi(points)
#
# # vor = Voronoi(points)
# print(vor.vertices)
