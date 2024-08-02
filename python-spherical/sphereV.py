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

trajectories = []
f3 = open('/home/robolab/python-spherical/trajectories.csv', 'w')
writer3 = csv.writer(f3)
CBF_drone_data = []
f = open('/home/robolab/python-spherical/cbf_drone_error.csv', 'w')
writer = csv.writer(f)
CBF_center_data = []
f1 = open('/home/robolab/python-spherical/cbf_center_error.csv', 'w')
writer1 = csv.writer(f1)
CBF_obs_data = []
f2 = open('/home/robolab/python-spherical/cbf_obs_error.csv', 'w')
writer2 = csv.writer(f2)

radius = 20 
center = np.array([0, 0, 0])
t_vals = np.linspace(0, 1, 2000)
velo = 0.1
safety_radius = 3
options['show_progress'] = False
# options['reltol'] = 1e-6
# options['abstol'] = 1e-6

def getArcFrom2Circle(r):
    return np.multiply(np.arcsin(np.divide(r,2*radius)), 4*radius)

def haversine(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude from degrees to radians
    # lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlat += -2*np.pi if (dlat>np.pi) else 0
    dlat = abs(dlat)
    # dlat = lat2 - lat1
    dlon = lon2 - lon1
    dlon += -2*np.pi if (dlon>np.pi) else 0
    dlon = abs(dlon)
    # a = (1-np.cos(dlat)+(np.cos(lat1)*np.cos(lat2)*np.cos(dlon)))/2
    # a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    # c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    # c = 2 * np.arcsin(np.sqrt(a))
    c = np.arccos(np.cos(lat1)*np.cos(lat2)*np.cos(dlon)+np.sin(lat1)*np.sin(lat2))
    
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

obs = np.array([[20,0,0],[0,20,0],[0,-20,0],[-20,0,0],[0,0,20],[0,0,-20]])
points = np.array([])

while len(points) < 8:
    rand_xyz = np.random.uniform(-20, 20, size=3)
    scaled_rand_xyz = getScaledPoint(rand_xyz)
    valid = 1
    for ob in obs:
        distance = ob - scaled_rand_xyz
        distanceScalar = np.sqrt(distance[0]**2 + distance[1]**2 + distance[2]**2)
        if distanceScalar < 5:
            valid = 0
        if valid == 0:
            break
    if valid != 0 and len(points) != 0:
        for point in points:
            distance = point - scaled_rand_xyz
            distanceScalar = np.sqrt(distance[0]**2 + distance[1]**2 + distance[2]**2)
            if distanceScalar < 5:
                valid = 0
            if valid == 0:
                break
    if valid == 1:
        # scaled_rand_xyz = scaled_rand_xyz * 1.0125
        if points.size == 0:
            points = np.hstack((points, scaled_rand_xyz))
        # points = np.hstack((points, np.array([scaledX,scaledY,scaledZ])))
        else:    
            points = np.vstack((points, scaled_rand_xyz))
# ([[20,0,0],[0,20,0],[0,-20,0],[-20,0,0],[0,0,20],[0,0,-20]])
# rand_xyz = [
#     [11.547,11.547,11.547],
#     [11.547,11.547,-11.547],
#     [11.547,-11.547,11.547],
#     [11.547,-11.547,-11.547],
#     [-11.547,11.547,11.547],
#     [-11.547,11.547,-11.547],
#     [-11.547,-11.547,11.547],
#     [-11.547,-11.547,-11.547]
# ]

# for p in points:
#     # displacement = getDisplacement(p[0],p[1],p[2])
#     # converter = np.divide(radius,displacement)
#     # scaledX = p[0]*converter
#     # scaledY = p[1]*converter
#     # scaledZ = p[2]*converter
#     scaledPoint = getScaledPoint(p)
#     if points.size == 0:
#         points = np.hstack((points, scaledPoint))
#         # points = np.hstack((points, np.array([scaledX,scaledY,scaledZ])))
#     else:    
#         points = np.vstack((points, scaledPoint))
#         # points = np.vstack((points, np.array([scaledX,scaledY,scaledZ])))   




# points = np.array([[20,20,20],[-20,-20,20],[-20,20,20],[20,-20,20],[-10,-10,20]])
def getGreatCircleDistance(R,r):
    return 2 * np.arcsin(r/(2*R)) * R


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

def barrier(dxi, x, scaledX, obstacles, safety_radius, obstacles_safety_radii, barrier_gain=1, unsafe_barrier_gain = 1e6, magnitude_limit=velo):
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
    AImagineCenter = np.zeros((N, 3*N))
    bImagineCenter = np.zeros(N)
    exceed_barrier = 0
    countAgents = 0
    countObstacles = 0
    for i in range(N): #Start loop for the focused agent
        if(i <= N-1):
            for j in range(i+1, N): #Compare to the another compared agent
                error = x[:, i] - x[:, j] #Distance between 2 agents
                agent_i = scaledX[:, i]
                agent_j = scaledX[:, j]
                agent_i_theta = getThetaScaled(agent_i[2])
                agent_j_theta = getThetaScaled(agent_j[2])
                agent_i_phi = getPhiScaled(agent_i[0],agent_i[1])
                agent_j_phi = getPhiScaled(agent_j[0],agent_j[1])
                arc_dist = haversine(agent_i_theta, agent_i_phi, agent_j_theta, agent_j_phi)
                errorScalar = np.sqrt((error[0]*error[0] + error[1]*error[1] + error[2]*error[2]))
                # CBF_drone_data.append(errorScalar)
                CBF_drone_data.append(errorScalar)
                # h=errorScalar-safety_radius
                h = arc_dist - getGreatCircleDistance(radius, safety_radius)
                Ag[countAgents, (3*i, (3*i+1),(3*i+2))] = -2*error
                Ag[countAgents, (3*j, (3*j+1),(3*j+2))] = 2*error
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
        # hCenter = errorCenterScalar - (radius)
        hCenter = errorCenterScalar - (radius)
        if hCenter < 0:
            exceed_barrier = hCenter
        # print(math.sqrt((errorObs[0]*errorObs[0] + errorObs[1]*errorObs[1] + errorObs[2]*errorObs[2])))
        # print(hObs)
        ACenter[0, (3*i, (3*i+1),(3*i+2))] = -2*errorCenter
        # ACenter[i, (3*i, (3*i+1),(3*i+2))] = -2*errorCenter

        # Aob[countObstacles, (3*i, (3*i+1),(3*i+2))] = np.array([0,1])
                
        # bob[countObstacles] = barrier_gain*np.power(hObs, 3)
        if hCenter > 0:
            bCenter[i] = barrier_gain*np.power(hCenter, 3)
        else:
            bCenter[i] = unsafe_barrier_gain*np.power(hCenter, 3)
        # bob[countObstacles] = 0.4*barrier_gain*(errorObsScalar - safety_radius/2)**3
        # bob[countObstacles] = barrier_gain*(errorObsScalar - safety_radius/2)**3
            
        errorImagineCenter = x[:, i] - (scaledX[:,i]*2)
        errorImagineCenterScalar = np.sqrt((errorImagineCenter[0]*errorImagineCenter[0] + errorImagineCenter[1]*errorImagineCenter[1] + errorImagineCenter[2]*errorImagineCenter[2]))
        hImagineCenter = errorImagineCenterScalar - (radius - 0.1)
        AImagineCenter[i, (3*i, (3*i+1),(3*i+2))] = -2*errorImagineCenter
        if hImagineCenter > 0:
            bImagineCenter[i] = barrier_gain*np.power(hImagineCenter, 3)
        else:
            bImagineCenter[i] = unsafe_barrier_gain*np.power(hImagineCenter, 3)

        if (obstacles.size != 0):
            for k in range(obSize): #Compare to the another compared agent
                errorObs = x[:, i] - obstacles[:, k] #Distance between an agent and an obstacle
                agent_i = scaledX[:, i]
                obstacle_j = obstacles[:, k]
                agent_i_theta = getThetaScaled(agent_i[2])
                obstacle_j_theta = getThetaScaled(obstacle_j[2])
                agent_i_phi = getPhiScaled(agent_i[0],agent_i[1])
                obstacle_j_phi = getPhiScaled(obstacle_j[0],obstacle_j[1])
                arc_dist = haversine(agent_i_theta, agent_i_phi, obstacle_j_theta, obstacle_j_phi)
                # errorScalar = np.sqrt((error[0]*error[0] + error[1]*error[1] + error[2]*error[2]))
                # CBF_drone_data.append(errorScalar)
                errorObsScalar = np.sqrt((errorObs[0]*errorObs[0] + errorObs[1]*errorObs[1] + errorObs[2]*errorObs[2]))

                CBF_obs_data.append(errorObsScalar)
                # h=errorScalar-safety_radius
                hObs = arc_dist - getGreatCircleDistance(radius, obstacles_safety_radii[k])
                # hObs = (errorObs[0]*errorObs[0] + errorObs[1]*errorObs[1] + errorObs[2]*errorObs[2]) - np.power(safety_radius, 2) #Distance between 2 drones
                # CBF_obs_data.append(errorObsScalar)
                # CBF_center_data.append(errorObsScalar)
                # hObs = errorObsScalar - obstacles_safety_radii[k]
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
    # A = np.concatenate((np.concatenate((Ag, Aob)),ACenter))
    # b = np.concatenate((np.concatenate((bg, bob)),bCenter))
                
    # A = np.concatenate((ACenter, AImagineCenter))
    # b = np.concatenate((bCenter, bImagineCenter))
    # A = np.concatenate((A, Ag))
    # b = np.concatenate((b, bg))
    A = np.concatenate((ACenter, Ag))
    b = np.concatenate((bCenter, bg))
    # A = np.concatenate((A, AImagineCenter))
    # b = np.concatenate((b, bImagineCenter))
    if (obstacles.size != 0):
        A = np.concatenate((A, Aob))
        b = np.concatenate((b, bob))
        # print("A")
        # print(A[91])
        # print("Size")
        # print(A.shape)
    # else:
    #     A = Ag
    #     b = bg
    
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

allObs = []

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

# sort vertices (optional, helpful for plotting)
# sv.sort_vertices_of_regions()
# while True:
region_plot = True
for i in range(300):
    print(i)
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
    # if i == 0:
    #     points += 0.05
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
            if (start != end).any():
                region_slerp = geometric_slerp(start, end, t_vals)*radius
                ax.plot(region_slerp[..., 0], region_slerp[..., 1], region_slerp[..., 2], c='g')
            # region_plot = False

        # ax.plot([points[pCounter][0],scaled_center[0]],[points[pCounter][1],scaled_center[1]],[points[pCounter][2],scaled_center[2]],color = 'k', linestyle='dashed')
        # start=np.array([0,0,1])
        # end= np.array([0,1,0])
        # slerp = geometric_slerp(start, end, t_vals)
        agentStart = scaledPoints[pCounter]/radius
        agentGoal = scaled_center/radius
        if (agentStart != agentGoal).any():
            slerp = geometric_slerp(agentStart, agentGoal, t_vals)*radius
            ax.plot(slerp[...,0], slerp[...,1], slerp[...,2], c='k')
        
        pCounter +=1
        ax.scatter(scaled_center[0],scaled_center[1],scaled_center[2],c='r')
        
                    
    # regionCenters = [[0,0,20],[0,0,-20],[0,20,0],[0,-20,0],[20,0,0],[-20,0,0]]
    
    # for c in regionCenters:
    #     ax.scatter(c[0],c[1],c[2],c='r')
    for iterP,point in enumerate(points):
        trajectories.append(point[0])
        trajectories.append(point[1])
        trajectories.append(point[2])
        trajectories.append(regionCenters[iterP][0])
        trajectories.append(regionCenters[iterP][1])
        trajectories.append(regionCenters[iterP][2])
    positions = np.transpose(np.array(points))[:3][:]
    goals = np.transpose(np.array(regionCenters))[:3][:]
    scaledPositions = np.transpose(np.array(scaledPoints))[:3][:]
    # ob1 = np.array([0, -20, 0])
    # ob2 = np.array([0, 20, 0])
    # ob3 = np.array([20, 0, 0])
    # ob4 = np.array([-20, 0, 0])
    # obstacles = np.transpose(np.array([center,ob1,ob2,ob3,ob4]))[:3][:]


    obstacles = np.transpose(obs)[:3][:]
    # obs = np.array([])
    # obstacles = np.array([])
    # obstacles_safety_radii = [radius,5,5,5,5]
    obstacles_safety_radii = [5,5,5,5,5,5]
    # obstacles_safety_radii=np.array([])
    dxi = si_position_controller(positions,goals)
    dxi, exceedOrNot = barrier(dxi, positions, scaledPositions, obstacles, safety_radius,obstacles_safety_radii)
    # if (exceedOrNot != 0):
    #     print("exceed or not: ", exceedOrNot)
    #     break
    # for ob in obs:
    #     u,v=np.mgrid[0:2*np.pi:10j, 0:np.pi:10j]
    #     x=np.cos(u)*np.sin(v)*(5) + ob[0]
    #     y=np.sin(u)*np.sin(v)*(5) + ob[1]
    #     z=np.cos(v)*(5) + ob[2]
    #     ax.plot_wireframe(x, y, z, color='g', linewidth=0.5)
    

    new_points = []
    pointCounter = 0
    for point in points:
        distanceToGoal = regionCenters[pointCounter]-point
        scalarDistanceToGoal = getDisplacement(distanceToGoal[0],distanceToGoal[1],distanceToGoal[2])
        speed = getDisplacement(dxi[0][pointCounter],dxi[1][pointCounter],dxi[2][pointCounter])
        if  scalarDistanceToGoal<=speed :
            new_speed = scalarDistanceToGoal/2
            x_velo = ((dxi[0][pointCounter]*new_speed)/speed)
            y_velo = ((dxi[1][pointCounter]*new_speed)/speed)
            z_velo = ((dxi[2][pointCounter]*new_speed)/speed)
            new_point = [point[0]+x_velo, point[1]+y_velo, point[2]+z_velo]
            new_points.append(new_point)
            # new_points.append(regionCenters[pointCounter])
        else:
            new_point = [point[0]+dxi[0][pointCounter], point[1]+dxi[1][pointCounter],point[2]+dxi[2][pointCounter]]
            # new_point = getScaledPoint([point[0]+dxi[0][pointCounter], point[1]+dxi[1][pointCounter],point[2]+dxi[2][pointCounter]])
            new_points.append(new_point)
        pointCounter +=1
    points = np.array(new_points)

    u,v=np.mgrid[0:2*np.pi:20j, 0:np.pi:20j]
    x=np.cos(u)*np.sin(v)*(radius) + center[0]
    y=np.sin(u)*np.sin(v)*(radius) + center[1]
    z=np.cos(v)*(radius) + center[2]
    ax.plot_wireframe(x, y, z, color='lightskyblue', linewidth=0.5)

    for o in allObs:
        ax.plot(o[:, 0], o[:, 1], o[:, 2], 'g', alpha = 0.5)

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
    minOb = [min(CBF_obs_data)]
    writer.writerow([min(CBF_drone_data)])
    writer1.writerow(CBF_center_data)
    writer2.writerow(minOb)
    writer3.writerow(trajectories)
    CBF_drone_data.clear()
    CBF_center_data.clear()
    CBF_obs_data.clear()
    trajectories.clear()

