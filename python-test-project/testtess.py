import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from tess import Container
import math
import matplotlib
matplotlib.use('Agg')

positions = [[-2,1,-2],[0,-1,-2],[1,1,-2],[-1,-1,-2],[1,0,-2],[2,0,-2],[0,2,-2],[1,2,-2]]

def getLineLen(x_len, y_len, z_len):
    return math.sqrt((x_len**2) + (y_len**2) + (z_len**2))

def movePosition(i, initials, destinations, speed = 10):
    popSize = len(initials)
    new_positions = []
    x_initial = initials[i][0]
    y_initial = initials[i][1]
    z_initial = initials[i][2]
    x_destination = destinations[0]
    y_destination = destinations[1]
    z_destination = destinations[2]
    x_len = x_destination - x_initial
    y_len = y_destination - y_initial
    z_len = z_destination - z_initial
    line_len = getLineLen(x_len, y_len, z_len)
    if(line_len <= speed):
        new_positions = [x_destination, y_destination, z_destination]
    else:
        z_velo = ((z_destination - z_initial)/line_len)*speed
        y_velo = ((y_destination - y_initial)/line_len)*speed
        x_velo = ((x_destination - x_initial)/line_len)*speed
        new_positions = [x_initial + x_velo, y_initial + y_velo, z_initial + z_velo]
    return new_positions

def plotVoronoi(c, ax):
    i = 0
    j = 7
    colorList = ['k','y','g','c','b','m', 'r','w']
    for v in c:
        vertices = v.vertices()
        faceVertices = v.face_vertices()
        poly3d = [[vertices[faceVertices[ix][iy]] for iy in range(len(faceVertices[ix]))] for ix in range(len(faceVertices))]
        # print("-----------------")
        # print(vertices)
        # print(faceVertices)
        # print("-----------------")
        # print(poly3d)
        ax.add_collection3d(Poly3DCollection(poly3d,edgecolors='k',facecolors=colorList[i],linewidths=1, alpha=0.2))
        # ax.scatter(v.centroid()[0], v.centroid()[1], v.centroid()[2], c='r', s=50)
        # ax.scatter(positions[i][0], positions[i][1], positions[i][2], c='b', s=50)
        # ax.plot([positions[i][0],v.centroid()[0]],[positions[i][1],v.centroid()[1]],[positions[i][2],v.centroid()[2]],color = 'k', linestyle='dashed')
        # positions[i] =  movePosition(i, positions, v.centroid())
        i += 1
        j -= 1

def resetAxes(ax):
    ax.clear()
    ax.set_zlim(-4, 4)
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)

fig = plt.figure()
ax = Axes3D(fig)
ax.set_zlim(-4, 4)
ax.set_xlim(-4, 4)
ax.set_ylim(-4, 4)

loopOn = True

while loopOn:
# for i in range(100):
    c = Container(positions, limits=[(-4,-4,-4),(4,4,4)], periodic=False)
    plotVoronoi(c, ax)
    # plt.show()
    plt.savefig('/home/robolab/python-test-project/result/FIG_'+str("dfgdfgd34534")+'.png')
    resetAxes(ax)
    loopOn = False



# def plotting(fig, ax, iter, save_fig=False):
#     major_locator=MultipleLocator(4)
#     ax.set_xlim([-12,12])
#     ax.set_ylim([-12,12])
#
#     #change x,y labels
#     y_ticks = np.arange(-4, 100, 4)
#     x_ticks = np.arange(-4, 100, 4)
#     ax.set_yticklabels(y_ticks)
#     ax.set_xticklabels(x_ticks)
#
#     font = {'size':13}
#     ax.xaxis.set_major_locator(major_locator)
#     ax.yaxis.set_major_locator(major_locator)
#     ax.set_xlabel('x(m)', font, labelpad=15)
#     ax.set_ylabel('y(m)', font, labelpad=15)
#
#     if save_fig:
#         plt.savefig('/home/robolab/raspi_ws/src/CVT_CBF_gazebo/Data/FIG/FIG_'+str(iter)+'.png')
#     #plt.title(str(j)  +"th itr")
#     plt.tick_params(labelsize=13) #设置刻度字体大小
#     plt.pause(0.001)
#     ax.clear()
