import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from scipy.spatial import SphericalVoronoi, geometric_slerp



def getTheta(x,y,z): #Latitude // Polar
    theta = np.arctan2(z, np.sqrt(x**2 + y**2))
    # theta = np.arcsin(z/radius)
    return theta

def getPhi(x,y): #Longtitude // Azimuth
    phi = np.arctan2(y,x)
    if phi < 0:
        phi += 2 * np.pi
    # phi = np.arctan2(y, x)
    return phi

def getThetaScaled(z):
    theta = np.arcsin(z/radius)
    return theta

def getPhiScaled(x,y):
    phi = np.arctan2(y, x)
    return phi

def getScaledPoint(p):
    theta = getTheta(p[0],p[1],p[2])
    phi = getPhi(p[0],p[1])
    x = radius*np.cos(phi)*np.cos(theta)
    y = radius*np.sin(phi)*np.cos(theta)
    z = radius*np.sin(theta)
    return np.array([x,y,z])

radius = 20
center = [0,0,0]
# fn = input('Enter folder name:') + '/'
fn = "10Jul_rand3/"
folder_name = "/home/robolab/python-spherical/" + fn
d1 = pd.read_csv(folder_name +'/trajectories.csv', header = None)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
t_vals = np.linspace(0, 1, 2000)

ax.set_zlim(-20, 20)
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_xlabel('x',fontsize=18)
ax.set_ylabel('y',fontsize=18)
ax.set_zlabel('z',fontsize=18)

allObs = []

obs = np.array([[20,0,0],[0,20,0],[0,-20,0],[-20,0,0],[0,0,20],[0,0,-20]])
for ob in obs:
    
    d = np.linalg.norm(np.array(center) - np.array(ob))

    a = (radius**2 - 5**2 + d**2) / (2 * d)
    h = np.sqrt(radius**2 - a**2)

    p0 = np.array(center) + a * (np.array(ob) - np.array(center)) / d
    normal = np.cross(np.array(center) - np.array(ob), [1, 0, 0])

    u, v = np.mgrid[0:2*np.pi:100j, 0:np.pi:50j]
    # x_circle = p0[0] + h * np.cos(u) * np.sin(v)
    # y_circle = p0[1] + h * np.sin(u) * np.sin(v)
    # z_circle = p0[2] + h * np.cos(v)
    # ax.plot_surface(x_circle, y_circle, z_circle, color='g', alpha=0.5, rstride=5, cstride=5)
    circle_points = []
    for theta in np.linspace(0, 2*np.pi, 100):
        for phi in np.linspace(0, np.pi, 100):
            x_circle = p0[0] + h * np.cos(theta) * np.sin(phi)
            y_circle = p0[1] + h * np.sin(theta) * np.sin(phi)
            z_circle = p0[2] + h * np.cos(phi)
            if np.linalg.norm(np.array([x_circle, y_circle, z_circle]) - np.array(center)) <= radius:
                circle_points.append([x_circle, y_circle, z_circle])

    circle_points = np.array(circle_points)
    allObs.append(circle_points)
    # Plot the intersection circle on the surface of sphere 1
    # u,v=np.mgrid[0:2*np.pi:10j, 0:np.pi:10j]
    # x=np.cos(u)*np.sin(v)*(5) + ob[0]
    # y=np.sin(u)*np.sin(v)*(5) + ob[1]
    # z=np.cos(v)*(5) + ob[2]
    # ax.plot_wireframe(x, y, z, color='g', linewidth=0.5)

u,v=np.mgrid[0:2*np.pi:20j, 0:np.pi:20j]
x=np.cos(u)*np.sin(v)*(radius) + center[0]
y=np.sin(u)*np.sin(v)*(radius) + center[1]
z=np.cos(v)*(radius) + center[2]


# x,y = np.meshgrid(np.linspace(-5,5,20),np.linspace(-5,5,20))
# z = np.sqrt(((39*(x**2)-(16000*x)+16000+(39*(y**2)))/-39 ))


# ax.plot_surface(x,y,z)


frame = d1.shape[0]
for i in range(frame):
    for o in allObs:
        ax.plot(o[:, 0], o[:, 1], o[:, 2], 'g', alpha = 0.5)
    ax.plot_wireframe(x, y, z, color='lightskyblue', linewidth=0.5)
    ax.scatter(d1[0][i],d1[1][i], d1[2][i], c='b')
    # for x in range(d1Len):
    #     ax.scatter(d1[0][x], d1[1][x], d1[2][x], c='k', label='drone1')
    ax.scatter(d1[6][i],d1[7][i], d1[8][i], c='b')
    ax.scatter(d1[12][i],d1[13][i], d1[14][i], c='b')
    ax.scatter(d1[18][i],d1[19][i], d1[20][i], c='b')
    ax.scatter(d1[24][i],d1[25][i], d1[26][i], c='b')
    ax.scatter(d1[30][i],d1[31][i], d1[32][i], c='b')
    ax.scatter(d1[36][i],d1[37][i], d1[38][i], c='b')
    ax.scatter(d1[42][i],d1[43][i], d1[44][i], c='b')

    ax.scatter(d1[3][i],d1[4][i], d1[5][i], c='r')
    ax.scatter(d1[9][i],d1[10][i], d1[11][i], c='r')
    ax.scatter(d1[15][i],d1[16][i], d1[17][i], c='r')
    ax.scatter(d1[21][i],d1[22][i], d1[23][i], c='r')
    ax.scatter(d1[27][i],d1[28][i], d1[29][i], c='r')
    ax.scatter(d1[33][i],d1[34][i], d1[35][i], c='r')
    ax.scatter(d1[39][i],d1[40][i], d1[41][i], c='r')
    ax.scatter(d1[45][i],d1[46][i], d1[47][i], c='r')

    scaledPoints = np.array([])
    for p_i in range(8):
        p = [d1[p_i*6][i], d1[(p_i*6) + 1][i],d1[(p_i*6)+2][i]]
        scaledPoint = getScaledPoint(p)
        # if i == 83:
        #     print("point : ", p)
        #     print("scaled point : ", scaledPoint)
        if scaledPoints.size == 0:
            scaledPoints = np.hstack((scaledPoints, scaledPoint))
        else:    
            scaledPoints = np.vstack((scaledPoints, scaledPoint))   
        g = np.array([d1[(p_i*6) + 3][i], d1[(p_i*6) + 4][i],d1[(p_i*6)+5][i]])/radius
        if (scaledPoint/radius != g).any():
            slerp = geometric_slerp(scaledPoint/radius, g, t_vals)*radius
            ax.plot(slerp[...,0], slerp[...,1], slerp[...,2], c='k') 

    # if i%3 == 0:
    #     points = scaledPoints

    sv = SphericalVoronoi(scaledPoints, radius, center)
    # if i == 0:
    #     points += 0.05
    sv.sort_vertices_of_regions()
        
    regionCenters = []
    # initial_velos = []
    # j = 0
    pCounter = 0
    for region in sv.regions:
        # region_vertices = [sv.vertices[j] for j in region]
        # dis_x = scaled_center[0] - points[i][0]
        # dis_y = scaled_center[1] - points[i][1]
        # dis_z = scaled_center[2] - points[i][2]
        # displacement = getDisplacement(dis_x,dis_y,dis_z)
        # initial_velo_x = (dis_x*velo)/displacement
        # initial_velo_y = (dis_y*velo)/displacement
        # initial_velo_z = (dis_z*velo)/displacement
        # initial_velos.append([initial_velo_x,initial_velo_y,initial_velo_z])

        # plot region
        
        # start = sv.vertices[region[0]]/radius
        # end = sv.vertices[region[-1]]/radius
        # region_slerp = geometric_slerp(start, end, t_vals)*radius
        # ax.plot(region_slerp[..., 0], region_slerp[..., 1], region_slerp[..., 2], c='g')
        # if region_plot:

        for region_i in range(len(region)):
            start = sv.vertices[region[region_i]]/radius
            end = sv.vertices[region[region_i-1]]/radius
            if (start != end).any():
                region_slerp = geometric_slerp(start, end, t_vals)*radius
                ax.plot(region_slerp[..., 0], region_slerp[..., 1], region_slerp[..., 2], c='g')
            # region_plot = False

        # ax.plot([points[pCounter][0],scaled_center[0]],[points[pCounter][1],scaled_center[1]],[points[pCounter][2],scaled_center[2]],color = 'k', linestyle='dashed')
        # start=np.array([0,0,1])
        # end= np.array([0,1,0])
        # slerp = geometric_slerp(start, end, t_vals)

        
        pCounter +=1
    
    plt.savefig('/home/robolab/python-spherical/result/' + fn + 'FIG_'+'{:05}'.format(i)+'.png')
    ax.clear()
    ax.set_zlim(-20, 20)
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    print(i)
print("DOOOOOOOOOOOONEe")
