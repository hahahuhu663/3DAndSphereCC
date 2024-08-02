import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
import time
import pandas as pd
from tess import Container
from scipy.spatial import ConvexHull

folder_name = 'sym2'
d1 = pd.read_csv(folder_name +'/trajectories.csv', header = None)
# d2 = pd.read_csv('d2.csv', header = None)
# d3 = pd.read_csv('d3.csv', header = None)
# d4 = pd.read_csv('d4.csv', header = None)
# d5 = pd.read_csv('d5.csv', header = None)
# d6 = pd.read_csv('d6.csv', header = None)
# d7 = pd.read_csv('d7.csv', header = None)
# d8 = pd.read_csv('d8.csv', header = None)

frame = d1.shape[0]
voronoiLimit = [(-20,-20, 0),(20,20,40)]
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
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
ax.plot(d1[6],d1[7], d1[8], c='y', label='drone2')
ax.plot(d1[12],d1[13], d1[14], c='g', label='drone3')
ax.plot(d1[18],d1[19], d1[20], c='c', label='drone4')
ax.plot(d1[24],d1[25], d1[26], c='b', label='drone5')
ax.plot(d1[30],d1[31], d1[32], c='m', label='drone6')
ax.plot(d1[36],d1[37], d1[38], c='r', label='drone7')
ax.plot(d1[42],d1[43], d1[44], c='#da8200', label='drone8')


plt.legend()
# plt.show()
plt.savefig('/home/robolab/plot_result/'+folder_name+'/trajectories.png')

def resetAxes():
    ax.clear()
    ax.set_zlim(0, 40)
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)

colorList = ['k','y','g','c','b','m','r','w']

# pipesCombined = [
#     [-11.5,-10.1667, 0],
#     [-11.5, -9.8333, 0], 
#     [-8.5, -9.8333, 0], 
#     [-8.5, -10.1667, 0], 
#     [-11.5,-10.1667, 40],
#     [-11.5, -9.8333, 40], 
#     [-8.5, -9.8333, 40], 
#     [-8.5, -10.1667, 40]
# ]

# nearPipeCrate = [
#     [-9.5,-9.5,0],
#     [-9.5,-4.5,0],
#     [-4.5,-4.5,0],
#     [-4.5,-9.5,0],
#     [-9.5,-9.5,5],
#     [-9.5,-4.5,5],
#     [-4.5,-4.5,5],
#     [-4.5,-9.5,5]
# ]

# cratesStack = [
#     [5,5,0],
#     [5,15,0],
#     [15,15,0],
#     [15,5,0],
#     [5,5,10],
#     [5,15,10],
#     [15,15,10],
#     [15,5,10],
# ]
# topCrate = [
#     [7.5,7.5,10],
#     [12.5, 7.5,10],
#     [12.5, 12.5,10],
#     [7.5,12.5,10],
#     [7.5,7.5,15],
#     [12.5, 7.5,15],
#     [12.5, 12.5,15],
#     [7.5,12.5,15]
# ]

# lonelyCrate = [
#     [7.5,-7.5,0],
#     [12.5, -7.5,0],
#     [12.5, -12.5,0],
#     [7.5,-12.5,0],
#     [7.5,-7.5,5],
#     [12.5, -7.5,5],
#     [12.5, -12.5,5],
#     [7.5,-12.5,5]
# ]
   #set 1 symmetry
# obstacles = [
#     [15, 15, 5],
#     [15, 15, 15],
#     [15, 15, 25],
#     [15, 15, 35],
#     [15, -15, 5],
#     [15, -15, 15],
#     [15, -15, 25],
#     [15, -15, 35],
#     [-15, 15, 5],
#     [-15, 15, 15],
#     [-15, 15, 25],
#     [-15, 15, 35],
#     [-15, -15, 5],
#     [-15, -15, 15],
#     [-15, -15, 25],
#     [-15, -15, 35],
# ]
    #set 2 symmetry
obstacles = [
    [15, 5, 5],
    [15, -5, 5],
    [15, 0, 15],
    [15, 0, 25],
    [-15, -5, 5],
    [-15, 5, 5],
    [-15, 0, 15],
    [-15, 0, 25],
    [5, 15, 5],
    [-5, 15, 5],
    [0, 15, 15],
    [0, 15, 25],
    [5, -15, 5],
    [-5, -15, 5],
    [0, -15, 15],
    [0, -15, 25],
]
#set 3 symemetry
# obstacles = [
#     [15, 5, 5],
#     [15, -5, 5],
#     [15, 0, 15],
#     [-15, -5, 5],
#     [-15, 5, 5],
#     [-15, 0, 15],
#     [5, 15, 5],
#     [-5, 15, 5],
#     [0, 15, 15],
#     [5, -15, 5],
#     [-5, -15, 5],
#     [0, -15, 15],
#     [0, 0, 5],
#     [0, 0, 15],
#     [0, 0, 25],
#     [0, 0, 35],
# ]
    #set 1 asymmetry
# obstacles = [
#     [-15, 15, 5],
#     [-5, 15, 5],
#     [-10, 15, 15],
#     [-10, 15, 25],
#     [15, 15, 5],
#     [15, 15, 15],
#     [15, 15, 25],
#     [-15, -5, 5],
#     [-15, -5, 15],
#     [-15, -5, 25],
#     [15, -5, 5],
#     [15, -5, 15],
#     [15, -5, 25],
#     [-5, -15, 5],
#     [-5, -15, 15],
#     [-5, -15, 25]
# ]
    #set 2 asymmetry
# obstacles = [
#     [-15, 15, 5],
#     [-5, 15, 5],
#     [-5, 15, 15],
#     [-5, 15, 25],
#     [5, 15, 5],
#     [5, 15, 15],
#     [15, 15, 5],
#     [-15, -15, 5],
#     [-15, -15, 15],
#     [-5, -15, 5],
#     [-5, -15, 15],
#     [5, -15, 5],
#     [5, -15, 15],
#     [5, -15, 25],
#     [15, -5, 5],
#     [15, -5, 15]
# ]
#set 3 asymemetry
# obstacles = [
#     [-15,5,5],
#     [-15, -5, 5],
#     [-15, 0, 15],
#     [-15, -15, 5],
#     [-15, -10, 15],
#     [-15, -5, 25],
#     [-5, -15, 5],
#     [-5, -15, 15],
#     [5, -15, 5],
#     [15, -15, 5],
#     [15, -15, 15],
#     [5, 15, 5],
#     [5, 15, 15],
#     [15, 5, 5],
#     [15, 5, 15],
#     [5, 15, 25]
# ]
def get_cube():   
    phi = np.arange(1,10,2)*np.pi/4
    Phi, Theta = np.meshgrid(phi, phi)

    cube_x = np.cos(Phi)*np.sin(Theta)
    cube_y = np.sin(Phi)*np.sin(Theta)
    cube_z = np.cos(Theta)/np.sqrt(2)
    return cube_x, cube_y, cube_z

# obstacles = np.array([
#      pipesCombined, cratesStack, topCrate, nearPipeCrate, lonelyCrate
#     ])
cube_x, cube_y, cube_z = get_cube()
# collection = []

def plotVoronoi(ax, position, colorIndex, destination, vertices=[], faceVertices=[]):
    # poly3d = [[vertices[faceVertices[ix][iy]] for iy in range(len(faceVertices[ix]))] for ix in range(len(faceVertices))]
    # ax.add_collection3d(Poly3DCollection(poly3d,edgecolors='k',facecolors=colorList[colorIndex],linewidths=1, alpha=0.2))
    # ax.scatter(centroid[0], centroid[1], centroid[2], c='r', s=30)

    # for tri in collection:
    #         ax.add_collection3d(tri)

    ax.scatter(destination[0], destination[1], destination[2], c='g', s=30)
    ax.scatter(position[0], position[1], position[2], c='r', s=30)
    ax.plot([position[0],destination[0]],[position[1],destination[1]],[position[2],destination[2]],color = 'k', linestyle='dashed')

for i in range(frame):
    ax.clear()
    # ax = fig.add_subplot(111, projection='3d')
    ax.set_zlim(0, 40)
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    for obstacle in obstacles:
        obx = obstacle[0]
        oby = obstacle[1]
        obz = obstacle[2]
        ax.plot_surface((cube_x*10)+obx,(cube_y*10)+oby,(cube_z*10)+obz, color='b', rstride = 1, cstride =1, antialiased=True, shade = True, alpha = 0.1)
        # ax.plot_surface((cube_x*10)+obx,(cube_y*10)+oby,(cube_z*10)+obz, color='b', antialiased=False, shade=False)


    u1 = [d1[0][i], d1[1][i], d1[2][i]]
    u2 = [d1[6][i], d1[7][i], d1[8][i]]
    u3 = [d1[12][i], d1[13][i], d1[14][i]]
    u4 = [d1[18][i], d1[19][i], d1[20][i]]
    u5 = [d1[24][i], d1[25][i], d1[26][i]]
    u6 = [d1[30][i], d1[31][i], d1[32][i]]
    u7 = [d1[36][i], d1[37][i], d1[38][i]]
    u8 = [d1[42][i], d1[43][i], d1[44][i]]
    positions = [u1,u2,u3,u4,u5,u6,u7,u8]
    c1 = [d1[3][i], d1[4][i], d1[5][i]]
    c2 = [d1[9][i], d1[10][i], d1[11][i]]
    c3 = [d1[15][i], d1[16][i], d1[17][i]]
    c4 = [d1[21][i], d1[22][i], d1[23][i]]
    c5 = [d1[27][i], d1[28][i], d1[29][i]]
    c6 = [d1[33][i], d1[34][i], d1[35][i]]
    c7 = [d1[39][i], d1[40][i], d1[41][i]]
    c8 = [d1[45][i], d1[46][i], d1[47][i]]
    destinations = [c1,c2,c3,c4,c5,c6,c7,c8]
    container = Container(positions, limits=voronoiLimit, periodic=False)
    j = 0
    for cell in container:
        
        plotVoronoi(ax, positions[j], j, destinations[j], cell.vertices(), cell.face_vertices())
        j +=1
    # for j in range(len(positions)):
    #     plotVoronoi(ax, positions[j], j, destinations[j])
        

    # now = d1[48][i]
    # print(now)
    # for obstacle in obstacles:
    #     obx = obstacle[0]
    #     oby = obstacle[1]
    #     obz = obstacle[2]
    #     # ax.plot_surface((cube_x+obx)*10,(cube_y+oby)*10,(cube_z+obz)*10, color='b')
    #     ax.plot_surface((cube_x*10)+obx,(cube_y*10)+oby,(cube_z*10)+obz, color='b', antialiased=False, shade=False)

    currentTime = time.time()
    plt.savefig('/home/robolab/plot_result/cbf_cvt/sym2/' + "{:09d}".format(i) + '.png')






# print(d1[0])
