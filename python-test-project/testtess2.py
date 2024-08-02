import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from tess import Container
import math

# positions = [[-70,50,-70],[0,-50,-70],[50,50,-70],[-50,-50,-70],[50,0,-70],[70,0,-70],[0,70,-70],[50,70,-70]]
# positions = [[-70,50,-99],[0,-50,-99],[50,50,-99],[-50,-50,-99],[50,0,-99],[70,0,-99]]
positions = [[2,2,2],[1,3,5],[4,4,4]]
positions = [[2,2,2],[1,3,5],[4,4,4]]
sigma = 1.0

def point_inside_polyhedron(point, faceVertices, vertices):
    for face in faceVertices:
        vertices_face = np.array([vertices[face[iy]] for iy in range(len(face))])
        normal = np.cross(vertices_face[1] - vertices_face[0], vertices_face[2] - vertices_face[0])
        dot_product = np.dot(point - vertices_face[0], normal)
        if dot_product < 0: 
            return False
    return True

def centroid_calculation(region_value, ori_centroid):
    sum_value = [0,0,0]
    for i in region_value:
        sum_value += i
    poly_centroid = []
    poly_size = len(region_value)
    if poly_size == 0:
        # print("same")
        return ori_centroid
    else:
        poly_dense_x = sum_value[0]/poly_size
        poly_dense_y = sum_value[1]/poly_size
        poly_dense_z = sum_value[2]/poly_size
        poly_centroid =[poly_dense_x,poly_dense_y,poly_dense_z]
        # print("change")
        # print(poly_centroid)

    return poly_centroid

def value_contribution(vertices, faceVertices, point_samples):
    region_value =[]
    saved_indices = []
    # print(point_samples)
    # for i in range(face):

    for i in range(len(point_samples)):
        # if(in_poly_hull_single(vertices, point_samples[i])):
        if (point_inside_polyhedron(point_samples[i], faceVertices, vertices)):
            region_value.append(point_samples[i])
            saved_indices.append(i)
    # print(point_samples)
    remaining_points = np.delete(point_samples, saved_indices, 0)
    # remaining_points = point_samples
    # region_value, remaining_points = point_inside_polyhedron(point_samples, faceVertices, vertices, colorIndex)

    # return region_value, remaining_points, len(saved_indices)
    return region_value, remaining_points


def density_function(vertices, faceVertices, ori_centroid, point_samples):
    # starttime = time.time()
    # region_value, remaining_point_samples, saved_size = value_contribution(vertices, faceVertices, point_samples, colorIndex) 
    region_value, remaining_point_samples = value_contribution(vertices, faceVertices, point_samples) 

    # print("check position in the region:")
    # print(time.time()-starttime)
    # starttime = time.time()
    poly_centroid = centroid_calculation(region_value, ori_centroid)
    # print("calculate centroid:")
    # print(time.time()-starttime)

    # return poly_centroid, x_unit, y_unit, z_unit
    return poly_centroid, remaining_point_samples


def getLineLen(x_len, y_len, z_len):
    return math.sqrt((x_len**2) + (y_len**2) + (z_len**2))

def movePosition(i, initials, destinations, speed = 3):
    # popSize = len(initials)
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
    # colorList = ['k','y','g','c','b','m', 'r','w']
    colorList = ['y','r','g','b']

    for v in c:
        vertices = v.vertices()
        faceVertices = v.face_vertices()
        # print(faceVertices)
        # print("-----------")
        poly3d = [[vertices[faceVertices[ix][iy]] for iy in range(len(faceVertices[ix]))] for ix in range(len(faceVertices))]
        # print("-----------------")
        # print(vertices)
        # print(faceVertices)
        # print("-----------------")
        # print(poly3d)
        ax.add_collection3d(Poly3DCollection(poly3d,edgecolors='k',facecolors=colorList[i],linewidths=1, alpha=0.1))
        centroid = v.centroid()
        x = np.array([])
        y = np.array([])
        z = np.array([])
        for v in vertices:
            x = np.append(x, np.array(v[0]))
            y = np.append(y, np.array(v[1]))
            z = np.append(z, np.array(v[2]))

    # Compute the density of the current cell using a Gaussian density function
        # density = gaussian_density(x, y, z, centroid[0], centroid[1], centroid[2])

        # print(density)
    # Update the centroid based on the new density
        sample_size = 3000
        deviation = 1
        target = [6,6,6]
        x_unit = np.random.normal(target[0], deviation, sample_size) 
        y_unit = np.random.normal(target[1], deviation, sample_size)
        z_unit = np.random.normal(target[2], deviation, sample_size)
        point_samples = np.vstack((np.array(x_unit), np.array(y_unit), np.array(z_unit))).T
        new_centroid, point_samples = density_function(vertices, faceVertices, centroid, point_samples)
        # print("asdfghjkl")
        # new_centroid = calculate_new_centroid(centroid, vertices, faceVertices)
        # print("fdgdfgdfg")
        # print(density)
        # print(new_centroid)
        # print(centroid)
        print("Centroid:", new_centroid)
        ax.scatter(new_centroid[0], new_centroid[1], new_centroid[2], c='r', s=30)
        # ax.scatter(new_centroid[0], new_centroid[1], new_centroid[2], c='r', s=50)
        # ax.scatter(ppp[i][0], ppp[i][1], ppp[i][2], c='b', s=10)
        # ax.plot([positions[i][0],new_centroid[0]],[positions[i][1],new_centroid[1]],[positions[i][2],new_centroid[2]],color = 'k', linestyle='dashed')
        # ax.plot([ppp[i][0],centroid[0]],[ppp[i][1],centroid[1]],[ppp[i][2],centroid[2]],color = 'k', linestyle='dashed')

        positions[i] =  movePosition(i, positions, new_centroid)
        # positions[i] =  movePosition(i, positions, centroid)
        i += 1
        j -= 1

def resetAxes(ax):
    ax.clear()
    ax.set_zlim(-100, 100)
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_zlim(0, 6)
ax.set_xlim(0, 6)
ax.set_ylim(0, 6)

# print(positions)
c = Container(positions, limits=[(0,0,0),(6,6,6)], periodic=False)
    # c.set_density_func(gaussian_density)
    # c.compute()
plotVoronoi(c, ax)
    
plt.show()
    # plt.savefig('/home/robolab/python-test-project/result/FIG_'+str(i)+'.png')
    # resetAxes(ax)
    # loopOn = False



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
