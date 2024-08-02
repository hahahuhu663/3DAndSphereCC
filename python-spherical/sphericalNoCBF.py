import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
from scipy.special import comb
from scipy.spatial import SphericalVoronoi, geometric_slerp
from mpl_toolkits.mplot3d import proj3d
from cvxopt import matrix, sparse
from cvxopt.solvers import qp, options
import csv
import time

CBF_drone_data = []
f = open('/home/robolab/python-spherical/cbf_drone_error.csv', 'w')
writer = csv.writer(f)
CBF_center_data = []
f1 = open('/home/robolab/python-spherical/cbf_center_error.csv', 'w')
writer1 = csv.writer(f1)

radius = 20 
center = np.array([0, 0, 0])
t_vals = np.linspace(0, 1, 2000)
velo = 0.5
safety_radius = 0.01
options['show_progress'] = False
# options['reltol'] = 1e-6
# options['abstol'] = 1e-6

def getArcFrom2Circle(r):
    return np.multiply(np.arcsin(np.divide(r,2*radius)), 4*radius)

def haversine(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    
    # Radius of the Earth in kilometers (change it accordingly)

    
    # Calculate the arc length
    arc_length = radius * c

    return arc_length

def getDisplacement(x,y,z):
    return(np.sqrt(np.power(x,2)+np.power(y,2)+np.power(z,2)))
# set input data

# points = np.array([[0,2.4,20], [0, 6, 20], [10, 10, 20],
#                    [5, 5, 20], [7, 4, 20], [13, 11, 20]])
# points = np.array([[20,20,20],[-20,-20,20],[-20,20,20],[20,-20,20],[10,-20,20],[10,20,20],[20,10,20],[-20,10,20]])

def getTheta(x,y,z): #Latitude // Polar
    theta = np.arctan2(z, np.sqrt(x**2 + y**2))
    return theta

def getPhi(x,y): #Longtitude // Azimuth
    phi = np.arctan2(y,x)
    if phi < 0:
        phi += 2 * np.pi
    return phi

def getScaledPoint(p):
    theta = getTheta(p[0],p[1],p[2])
    phi = getPhi(p[0],p[1])
    x = radius*np.cos(phi)*np.cos(theta)
    y = radius*np.sin(phi)*np.cos(theta)
    z = radius*np.sin(theta)
    return np.array([x,y,z])


points = np.array([])
rand_xyz = np.random.uniform(-20, 20, size=(8, 3))

for p in rand_xyz:
    # displacement = getDisplacement(p[0],p[1],p[2])
    # converter = np.divide(radius,displacement)
    # scaledX = p[0]*converter
    # scaledY = p[1]*converter
    # scaledZ = p[2]*converter
    scaledPoint = getScaledPoint(p)
    if points.size == 0:
        points = np.hstack((points, scaledPoint))
        # points = np.hstack((points, np.array([scaledX,scaledY,scaledZ])))
    else:    
        points = np.vstack((points, scaledPoint))
        # points = np.vstack((points, np.array([scaledX,scaledY,scaledZ])))   




# points = np.array([[20,20,20],[-20,-20,20],[-20,20,20],[20,-20,20],[-10,-10,20]])


def si_position_controller(xi, positions, x_velocity_gain=1, y_velocity_gain=1, z_velocity_gain=1, velocity_magnitude_limit=velo):

    """
    xi: 2xN numpy array (of single-integrator states of the robots)
    points: 2xN numpy array (of desired points each robot should achieve)
    -> 2xN numpy array (of single-integrator control inputs)
    """
    #Check user input types
    assert isinstance(xi, np.ndarray), "In the si_position_controller function created by the create_si_position_controller function, the single-integrator robot states (xi) must be a numpy array. Recieved type %r." % type(xi).__name__
    assert isinstance(positions, np.ndarray), "In the si_position_controller function created by the create_si_position_controller function, the robot goal points (positions) must be a numpy array. Recieved type %r." % type(positions).__name__
    #Check user input ranges/sizes
    assert xi.shape[0] == 3
    # assert xi.shape[0] == 2, "In the si_position_controller function created by the create_si_position_controller function, the dimension of the single-integrator robot states (xi) must be 2 ([x;y]). Recieved dimension %r." % xi.shape[0]
    assert positions.shape[0] == 3
    # assert positions.shape[0] == 2, "In the si_position_controller function created by the create_si_position_controller function, the dimension of the robot goal points (positions) must be 2 ([x_goal;y_goal]). Recieved dimension %r." % positions.shape[0]
    assert xi.shape[1] == positions.shape[1], "In the si_position_controller function created by the create_si_position_controller function, the number of single-integrator robot states (xi) must be equal to the number of robot goal points (positions). Recieved a single integrator current position input array of size %r x %r and desired position array of size %r x %r." % (xi.shape[0], xi.shape[1], positions.shape[0], positions.shape[1])
    _,N = np.shape(xi)
    dxi = np.zeros((3, N))
    # Calculate control input
    #Set ratio for velocity in each axes according to velocity gain
    dxi[0][:] = x_velocity_gain*(positions[0][:]-xi[0][:])
    dxi[1][:] = y_velocity_gain*(positions[1][:]-xi[1][:])
    dxi[2][:] = z_velocity_gain*(positions[2][:]-xi[2][:])
    # Threshold magnitude
    norms = np.linalg.norm(dxi, axis=0)
    #Set velocity base on magnitude limit
    idxs = np.where(norms > velocity_magnitude_limit)
    if norms[idxs].size != 0:
        dxi[:, idxs] *= velocity_magnitude_limit/norms[idxs]
    return dxi

def barrier(dxi, x, obstacles, safety_radius, obstacles_safety_radii, barrier_gain=1, unsafe_barrier_gain = 1e6, magnitude_limit=velo):
    #Check user input types
    assert isinstance(dxi, np.ndarray), "In the function created by the create_single_integrator_barrier_certificate2 function, the single-integrator robot velocity command (dxi) must be a numpy array. Recieved type %r." % type(dxi).__name__
    assert isinstance(x, np.ndarray), "In the function created by the create_single_integrator_barrier_certificate2 function, the robot states (x) must be a numpy array. Recieved type %r." % type(x).__name__

    #Check user input ranges/sizes
    assert x.shape[0] == 3
    # assert x.shape[0] == 2, "In the function created by the create_single_integrator_barrier_certificate2 function, the dimension of the single integrator robot states (x) must be 2 ([x;y]). Recieved dimension %r." % x.shape[0]
    assert dxi.shape[0] == 3
    # assert dxi.shape[0] == 2, "In the function created by the create_single_integrator_barrier_certificate2 function, the dimension of the robot single integrator velocity command (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % dxi.shape[0]
    assert x.shape[1] == dxi.shape[1], "In the function created by the create_single_integrator_barrier_certificate2 function, the number of robot states (x) must be equal to the number of robot single integrator velocity commands (dxi). Recieved a current robot pose input array (x) of size %r x %r and single integrator velocity array (dxi) of size %r x %r." % (x.shape[0], x.shape[1], dxi.shape[0], dxi.shape[1])


    # Initialize some variables for computational savings
    N = dxi.shape[1] #Number of drones
    num_constraints = int(comb(N, 2))
    Ag= np.zeros((num_constraints, 3*N))
    
    bg = np.zeros(num_constraints)
    H = sparse(matrix(2*np.identity(3*N)))
    if (obstacles.size != 0):
        obSize = obstacles.shape[1]
        Aob = np.zeros((N*obSize, 3*N))
        bob = np.zeros(N*obSize)
    ACenter = np.zeros((N, 3*N))
    bCenter = np.zeros(N)
    exceed_barrier = 0
    countAgents = 0
    countObstacles = 0
    for i in range(N): #Start loop for the focused agent
        if(i <= N-1):
            for j in range(i+1, N): #Compare to the another compared agent
                # error = x[:, i] - x[:, j] #Distance between 2 agents
                agent_i = x[:, i]
                agent_j = x[:, j]
                agent_i_theta = getTheta(agent_i[0],agent_i[1],agent_i[2])
                agent_j_theta = getTheta(agent_j[0],agent_j[1],agent_j[2])
                agent_i_phi = getPhi(agent_i[0],agent_i[1])
                agent_j_phi = getPhi(agent_j[0],agent_j[1])
                arc_dist = haversine(agent_i_theta, agent_i_phi, agent_j_theta, agent_j_phi)
                # errorScalar = np.sqrt((error[0]*error[0] + error[1]*error[1] + error[2]*error[2]))
                CBF_drone_data.append(arc_dist)
                h = arc_dist - safety_radius
                Ag[countAgents, (3*i, (3*i+1),(3*i+2))] = -2*arc_dist
                Ag[countAgents, (3*j, (3*j+1),(3*j+2))] = 2*arc_dist
                # bg[countAgents] = barrier_gain*np.power(h, 3)
                if h > 0:
                    bg[countAgents] = barrier_gain*np.power(h, 3)
                else:
                    bg[countAgents] = unsafe_barrier_gain*np.power(h, 3)
                
                countAgents += 1
        errorCenter = x[:, i] - center #Distance between an agent and an obstacle
                
        # hObs = (errorObs[0]*errorObs[0] + errorObs[1]*errorObs[1] + errorObs[2]*errorObs[2]) - np.power(safety_radius, 2) #Distance between 2 drones
        errorCenterScalar = np.sqrt((errorCenter[0]*errorCenter[0] + errorCenter[1]*errorCenter[1] + errorCenter[2]*errorCenter[2]))
        CBF_center_data.append(errorCenterScalar)
        hCenter = errorCenterScalar - (radius)
        if hCenter < 0:
            exceed_barrier = hCenter
        # print(math.sqrt((errorObs[0]*errorObs[0] + errorObs[1]*errorObs[1] + errorObs[2]*errorObs[2])))
        # print(hObs)
        ACenter[0, (3*i, (3*i+1),(3*i+2))] = -2*errorCenter
        # Aob[countObstacles, (3*i, (3*i+1),(3*i+2))] = np.array([0,1])
                
        # bob[countObstacles] = barrier_gain*np.power(hObs, 3)
        if hCenter > 0:
            bCenter[0] = barrier_gain*np.power(hCenter, 3)
        else:
            bCenter[0] = unsafe_barrier_gain*np.power(hCenter, 3)
        # bob[countObstacles] = 0.4*barrier_gain*(errorObsScalar - safety_radius/2)**3
        # bob[countObstacles] = barrier_gain*(errorObsScalar - safety_radius/2)**3
        

        if (obstacles.size != 0):
            for k in range(obSize): #Compare to the another compared agent
                errorObs = x[:, i] - obstacles[:, k] #Distance between an agent and an obstacle
                
                # hObs = (errorObs[0]*errorObs[0] + errorObs[1]*errorObs[1] + errorObs[2]*errorObs[2]) - np.power(safety_radius, 2) #Distance between 2 drones
                errorObsScalar = np.sqrt((errorObs[0]*errorObs[0] + errorObs[1]*errorObs[1] + errorObs[2]*errorObs[2]))
                # CBF_center_data.append(errorObsScalar)
                hObs = errorObsScalar - (obstacles_safety_radii[k])
                if hObs < 0:
                    exceed_barrier = hObs
                # print(math.sqrt((errorObs[0]*errorObs[0] + errorObs[1]*errorObs[1] + errorObs[2]*errorObs[2])))
                # print(hObs)
                Aob[countObstacles, (3*i, (3*i+1),(3*i+2))] = -2*errorObs
                # Aob[countObstacles, (3*i, (3*i+1),(3*i+2))] = np.array([0,1])
                
                # bob[countObstacles] = barrier_gain*np.power(hObs, 3)
                if hObs > 0:
                    bob[countObstacles] = barrier_gain*np.power(hObs, 3)
                else:
                    bob[countObstacles] = unsafe_barrier_gain*np.power(hObs, 3)
                # bob[countObstacles] = 0.4*barrier_gain*(errorObsScalar - safety_radius/2)**3
                # bob[countObstacles] = barrier_gain*(errorObsScalar - safety_radius/2)**3
                countObstacles += 1

    # print("countObstacles")
    # print(countObstacles)

    # print("Ag")
    # print(Ag)
    # print("Size")
    # print(Ag.shape)
    # print("Aob")
    # print(Aob[63])
    # print("Size")
    # print(Aob.shape)

    if (obstacles.size != 0):
        A = np.concatenate((Ag, Aob))
        b = np.concatenate((bg, bob))
        # print("A")
        # print(A[91])
        # print("Size")
        # print(A.shape)
    else:
        A = Ag
        b = bg
    
    # print(A)
    # print(b)
    # Threshold control inputs before QP
    norms = np.linalg.norm(dxi, 2, 0)
    idxs_to_normalize = (norms > magnitude_limit)
    dxi[:, idxs_to_normalize] *= magnitude_limit/norms[idxs_to_normalize]

    f = -2*np.reshape(dxi, 3*N, order='F')
    result = qp(H, matrix(f), matrix(A), matrix(b))['x']
    
    return np.reshape(result, (3, -1), order='F'),exceed_barrier





# points = np.array([[0,2.4,20], [0, 0, -20], [20, 0, 0],
#                    [0, 20, 0], [0, -20, 0], [-20, 0, 0]])


fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_zlim(-20, 20)
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

# sort vertices (optional, helpful for plotting)
# sv.sort_vertices_of_regions()
# while True:
region_plot = True
for i in range(300):
    if i == 0:
        scaledPoints = points
    else:    
        scaledPoints = np.array([])
        for p in points:
            scaledPoint = getScaledPoint(p)
            # if i == 83:
            #     print("point : ", p)
            #     print("scaled point : ", scaledPoint)
            if scaledPoints.size == 0:
                scaledPoints = np.hstack((scaledPoints, scaledPoint))
            else:    
                scaledPoints = np.vstack((scaledPoints, scaledPoint))    
    
    # if i%3 == 0:
    #     points = scaledPoints

    sv = SphericalVoronoi(scaledPoints, radius, center)
    sv.sort_vertices_of_regions()
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b')
        
    regionCenters = []
    # initial_velos = []
    # j = 0
    pCounter = 0
    for region in sv.regions:
        
        region_vertices = [sv.vertices[j] for j in region]
        region_center = np.mean(region_vertices, axis=0)
        scaled_center = getScaledPoint(region_center)
        # dis_x = scaled_center[0] - points[i][0]
        # dis_y = scaled_center[1] - points[i][1]
        # dis_z = scaled_center[2] - points[i][2]
        # displacement = getDisplacement(dis_x,dis_y,dis_z)
        # initial_velo_x = (dis_x*velo)/displacement
        # initial_velo_y = (dis_y*velo)/displacement
        # initial_velo_z = (dis_z*velo)/displacement
        # initial_velos.append([initial_velo_x,initial_velo_y,initial_velo_z])
        regionCenters.append(scaled_center)

        # plot region
        
        # start = sv.vertices[region[0]]/radius
        # end = sv.vertices[region[-1]]/radius
        # region_slerp = geometric_slerp(start, end, t_vals)*radius
        # ax.plot(region_slerp[..., 0], region_slerp[..., 1], region_slerp[..., 2], c='g')
        # if region_plot:
        for region_i in range(len(region)):
            start = sv.vertices[region[region_i]]/radius
            end = sv.vertices[region[region_i-1]]/radius
            region_slerp = geometric_slerp(start, end, t_vals)*radius
            ax.plot(region_slerp[..., 0], region_slerp[..., 1], region_slerp[..., 2], c='g')
            # region_plot = False

        # ax.plot([points[pCounter][0],scaled_center[0]],[points[pCounter][1],scaled_center[1]],[points[pCounter][2],scaled_center[2]],color = 'k', linestyle='dashed')
        # start=np.array([0,0,1])
        # end= np.array([0,1,0])
        # slerp = geometric_slerp(start, end, t_vals)

        slerp = geometric_slerp(scaledPoints[pCounter]/radius, scaled_center/radius, t_vals)*radius
        ax.plot(slerp[...,0], slerp[...,1], slerp[...,2], c='k')

        pCounter +=1
        ax.scatter(scaled_center[0],scaled_center[1],scaled_center[2],c='r')
    new_points = []
    for regionCounter in range(len(regionCenters)):
        start = scaledPoints[regionCounter]
        end = regionCenters[regionCounter]
        startLat = getTheta(start[0],start[1],start[2])
        startLong = getPhi(start[0],start[1])
        goalLat = getTheta(end[0],end[1],end[2])
        goalLong = getPhi(end[0],end[1])   
        # rDiff = haversine(startLat,startLong,goalLat,goalLong)
        rDiff = end-start
        thetaDiff = goalLat - startLat #x
        phiDiff = goalLong - startLong #y
        # zDiff = np.arccos((np.cos(startLong)*np.cos(goalLong))+(np.sin(startLong)*np.sin(goalLong)*np.cos(thetaDiff))) #z
        
        # omega = np.sqrt(thetaDiff**2 + phiDiff**2 + zDiff**2)/2
        

        tangential = haversine(startLat,startLong,goalLat,goalLong)
        # omega = 2 * np.arctan2(np.sqrt(h), np.sqrt(1 - h))

        # tangential = omegah * radius

        velocity_x = tangential * np.cos(goalLat) * np.cos(goalLong)
        velocity_y = tangential * np.cos(goalLat) * np.sin(goalLong)
        velocity_z = tangential * np.sin(goalLat)

        # scalarDiff = np.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2 + (end[2]-start[2])**2)

        # vx = tangential*(end[0]-start[0])/scalarDiff
        # vy = tangential*(end[1]-start[1])/scalarDiff
        # vz = tangential*(end[2]-start[2])/scalarDiff

        # xDiff = (rDiff[0]*np.sin(startLat)*np.cos(startLong))+(start[0]*thetaDiff*np.cos(startLat)*np.cos(startLong))-(start[0]*np.sin(startLat)*phiDiff*np.sin(startLong))
        # yDiff = (rDiff[1]*np.sin(startLat)*np.sin(startLong))+(start[1]*thetaDiff*np.cos(startLat)*np.sin(startLong))-(start[1]*np.sin(startLat)*phiDiff*np.cos(startLong))
        # zDiff = (rDiff[2]*np.cos(startLat))-(start[2]*thetaDiff*np.sin(startLat))
        # determinator = np.sqrt((xDiff**2)+(yDiff**2)+(zDiff**2))
        # velocity_x = xDiff*velo/determinator
        # velocity_y = yDiff*velo/determinator
        # velocity_z = zDiff*velo/determinator
        nP = np.array([scaledPoints[regionCounter][0]+velocity_x,scaledPoints[regionCounter][1]+velocity_y,scaledPoints[regionCounter][2]+velocity_z])
        errorCenter = nP - center #Distance between an agent and an obstacle         
        errorCenterScalar = np.sqrt((errorCenter[0]*errorCenter[0] + errorCenter[1]*errorCenter[1] + errorCenter[2]*errorCenter[2]))
        CBF_center_data.append(errorCenterScalar)
        
        newPosition = getScaledPoint([scaledPoints[regionCounter][0]+velocity_x,scaledPoints[regionCounter][1]+velocity_y,scaledPoints[regionCounter][2]+velocity_z])
        new_points.append(newPosition)
        
        print(regionCounter)
        print(start)
        print(end)
        print(velocity_x)
        print(velocity_y)
        print(velocity_z)
    points = np.array(new_points)
        
    # regionCenters = [[0,0,20],[0,0,-20],[0,20,0],[0,-20,0],[20,0,0],[-20,0,0]]
    
    # for c in regionCenters:
    #     ax.scatter(c[0],c[1],c[2],c='r')
    
    # positions = np.transpose(np.array(points))[:3][:]
    # goals = np.transpose(np.array(regionCenters))[:3][:]
    # ob1 = np.array([0, -20, 0])
    # ob2 = np.array([0, 20, 0])
    # ob3 = np.array([20, 0, 0])
    # ob4 = np.array([-20, 0, 0])
    # obstacles = np.transpose(np.array([center,ob1,ob2,ob3,ob4]))[:3][:]
    # obstacles = np.transpose(np.array([[20,0,0],[0,20,0],[0,-20,0],[-20,0,0]]))[:3][:]
    # obstacles_safety_radii = [radius,5,5,5,5]
    # obstacles_safety_radii = [3,3,3,3]
    # dxi = si_position_controller(positions,goals)
    # dxi, exceedOrNot = barrier(dxi, positions, obstacles, safety_radius,obstacles_safety_radii)
    # if (exceedOrNot != 0):
    #     print("exceed or not: ", exceedOrNot)
    #     break

    u,v=np.mgrid[0:2*np.pi:20j, 0:np.pi:20j]
    x=np.cos(u)*np.sin(v)*(radius) + center[0]
    y=np.sin(u)*np.sin(v)*(radius) + center[1]
    z=np.cos(v)*(radius) + center[2]
    ax.plot_wireframe(x, y, z, color='lightskyblue', linewidth=0.5)

    now = time.time()
    
    # if i == 0:
    #     plt.show()
    # else:
    plt.savefig('/home/robolab/python-spherical/result/FIG_'+'{:05}'.format(i)+'.png')
    # plt.show()
        
    ax.clear()
    ax.set_zlim(-20, 20)
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    writer.writerow(CBF_drone_data)
    writer1.writerow(CBF_center_data)
    CBF_drone_data.clear()
    CBF_center_data.clear()


