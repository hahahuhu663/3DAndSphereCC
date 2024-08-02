import math
import time
from scipy.spatial import ConvexHull
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from tess import Container
import numpy as np
import csv
from scipy.special import comb
from cvxopt import matrix, sparse
from cvxopt.solvers import qp, options
import warnings
warnings.filterwarnings("error")
from scipy.optimize import fsolve

obstacles = [
            [15, 5, 5],
            [15, -5, 5],
            [15, 0, 15],
            [-15, -5, 5],
            [-15, 5, 5],
            [-15, 0, 15],
            [5, 15, 5],
            [-5, 15, 5],
            [0, 15, 15],
            [5, -15, 5],
            [-5, -15, 5],
            [0, -15, 15],
            [0, 0, 5],
            [0, 0, 15],
            [0, 0, 25],
            [0, 0, 35],
        ]
drone = [
        [7,7,0],
        [0,7,0],
        [-7,7,0],
        [-7,0,0],
        [7,0,0],
        [7,-7,0],
        [0,-7,0],
        [-7,-7,0]
         ]

colorList = ['k','y','g','c','b','m','r','w']

def si_position_controller(xi, positions, x_velocity_gain=1, y_velocity_gain=1, z_velocity_gain=1, velocity_magnitude_limit=1):

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

def callSolve(Ox,Oy,Oz,Px,Py,Pz):
    def func(x):
        return ((((Px-Ox)*x))/6.6)**4 + ((((Py-Oy)*x))/6.6)**4 + ((((Pz-Oz)*x))/6.6)**4 - 1
    # root = fsolve(func, 0.5)
    root = 0
    try:
        root = fsolve(func, 0.5)
    except RuntimeWarning:
        root = fsolve(func, 2)
    return [(Ox + (Px-Ox)*root),(Oy + (Py-Oy)*root),(Oz + (Pz-Oz)*root)]

def barrier(dxi, x, obstacles, safety_radius, barrier_gain=1, unsafe_barrier_gain = 1e6, magnitude_limit=1):
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
    if (obstacles.any()):
        obSize = obstacles.shape[1]
        Aob = np.zeros((N*obSize, 3*N))
        bob = np.zeros(N*obSize)
    wierd_radius = 6
    radius_counter = 0
    countAgents = 0
    countObstacles = 0
    for i in range(N): #Start loop for the focused agent
        if(i <= N-1):
            for j in range(i+1, N): #Compare to the another compared agent
                error = x[:, i] - x[:, j] #Distance between 2 agents
                # h = (error[0]*error[0] + error[1]*error[1] + error[2]*error[2]) - np.power(safety_radius, 2) #Distance between 2 drones
                errorScalar = math.sqrt((error[0]*error[0] + error[1]*error[1] + error[2]*error[2]))
                h = errorScalar - safety_radius
                # h = errorScalar - wierd_radius
                # data5[i].append(errorScalar)
                # data3.append(errorScalar) #Record distance between 2 agents to CSV
                Ag[countAgents, (3*i, (3*i+1),(3*i+2))] = -2*error
                Ag[countAgents, (3*j, (3*j+1),(3*j+2))] = 2*error
                # bg[countAgents] = barrier_gain*np.power(h, 3)
                if h > 0:
                    bg[countAgents] = barrier_gain*np.power(h, 3)
                else:
                    bg[countAgents] = unsafe_barrier_gain*np.power(h, 3)
                
                countAgents += 1

        if (obstacles.any()):
            for k in range(obSize): #Compare to the another compared agent
                barrierEdge = callSolve(obstacles[:, k][0],obstacles[:, k][1],obstacles[:, k][2],
                          x[:, i][0],x[:, i][1],x[:, i][2])

                errorObs = x[:, i] - obstacles[:, k] #Distance between an agent and an obstacle
                
                # hObs = (errorObs[0]*errorObs[0] + errorObs[1]*errorObs[1] + errorObs[2]*errorObs[2]) - np.power(safety_radius, 2) #Distance between 2 drones
                errorObsScalar = math.sqrt((errorObs[0]*errorObs[0] + errorObs[1]*errorObs[1] + errorObs[2]*errorObs[2]))
                barrierScalar = math.sqrt((barrierEdge[0]-obstacles[:, k][0])**2 + (barrierEdge[1]-obstacles[:, k][1])**2 + (barrierEdge[2]-obstacles[:, k][2])**2)
                # hObs = errorObsScalar - safety_radius
                hObs = errorObsScalar - barrierScalar
                # data6[i].append(errorObsScalar)
                # if i == 0:
                #     data7.append(errorObsScalar)
                #     data7.append(barrierScalar)
                # data4.append(errorObsScalar) 
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

    if (obstacles.any()):
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
    
    return np.reshape(result, (3, -1), order='F')

def getLineLen(x_len, y_len, z_len):
    return math.sqrt((x_len**2) + (y_len**2) + (z_len**2))

def getDestination(i, initials, destinations):
    # popSize = len(initials)
    # new_velo = []
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
    #Write data
    # data2.append(line_len)
    # data.append(x_initial)
    # data.append(y_initial)
    # data.append(z_initial)
    # data.append(x_destination)
    # data.append(y_destination)
    # data.append(z_destination)
    return [x_destination, y_destination, z_destination]

def createVoronoiContainer(positions, limit, isPeriodic):
    return Container(positions, limits=limit, periodic=isPeriodic)

def plotVoronoi(ax, vertices, faceVertices, centroid, position, colorIndex):
    # poly3d = [[vertices[faceVertices[ix][iy]] for iy in range(len(faceVertices[ix]))] for ix in range(len(faceVertices))]
    # ax.add_collection3d(Poly3DCollection(poly3d,edgecolors='k',facecolors=color,linewidths=1, alpha=0.2))
    poly3d = [[vertices[faceVertices[ix][iy]] for iy in range(len(faceVertices[ix]))] for ix in range(len(faceVertices))]
    ax.add_collection3d(Poly3DCollection(poly3d,edgecolors='k',facecolors=colorList[colorIndex],linewidths=1, alpha=0.2))
    ax.scatter(centroid[0], centroid[1], centroid[2], c='r', s=30)
    ax.scatter(position[0], position[1], position[2], c='b', s=30)
    ax.plot([position[0],centroid[0]],[position[1],centroid[1]],[position[2],centroid[2]],color = 'k', linestyle='dashed')

def calculateVoronoi(container, positions, radius, hasDensityFunction, plotSphere):
    # elasped_time = time.time() - dataStartTime
    i = 0
    # j = positions.len
    goals_x = []
    goals_y = []
    goals_z = []
    # scatter = []
    now = time.time()
    # if hasDensityFunction:
    #     sample_size = 1000
    #     deviation = 1
    #     target = [0,0,20]
    #     x_unit = np.random.normal(target[0], deviation, sample_size) 
    #     y_unit = np.random.normal(target[1], deviation, sample_size)
    #     z_unit = np.random.normal(target[2], deviation, sample_size)
    #     # density_scatter.append(x_unit)
    #     # density_scatter.append(y_unit)
    #     # density_scatter.append(z_unit)
    #     point_samples = np.vstack((np.array(x_unit), np.array(y_unit), np.array(z_unit))).T

    # start = time.time()
    for v in container:
        # vertices = v.vertices()
        # print(vertices)
        centroid = v.centroid()
        # faceVertices = v.face_vertices()
        # if hasDensityFunction:
            # print("generating density function")
            # target = [((elasped_time//5)%30)-15, ((elasped_time//5)%30)-15, ((elasped_time//5)%30)] #center of density function
            # centroid, x_density, y_density, z_density = density_function(vertices,target, centroid)
            # centroid, point_samples = density_function(vertices, faceVertices, centroid, point_samples)
            # centroid = density_function(vertices, faceVertices, centroid, point_samples)

        # plotVoronoi(ax, vertices, faceVertices, centroid, positions[i], i)
        # if plotSphere:
        #     sphere(positions[i], radius, ax)

        destinations =  getDestination(i, positions, centroid)
        goals_x.append(destinations[0])
        goals_y.append(destinations[1])
        goals_z.append(destinations[2])

        i += 1
        # j -= 1   
    # print("Used time for Voronoi:" + str(time.time() - start))
    
    # plt.savefig('/home/robolab/plot_result/cbf_cvt/FIG_'+str(now)+'.png') 
    # fig.canvas.draw()
    # fig.canvas.flush_events()
    # plt.show()
    # resetAxes()
    return goals_x, goals_y, goals_z

def calculateBarrier(positions, obstacles, radius, goals_x, goals_y, goals_z):
    # print(np.array(obstacles))
    positionForCbf = np.transpose(np.array(positions))
    
    # print("positions")
    # print(positions)
    # print("positions for cbf")
    # print(positionForCbf)
    # print("obstacles")
    # print(obstacles)
    # print("obstacles for cbf")
    # print(obstaclesForCbf)
    goals = np.array([goals_x, goals_y, goals_z])
    dxi = si_position_controller(positionForCbf[:3][:], goals[:3][:])
    # startTimeBarrier = time.time()
    if(obstacles):
        # for obstacle in obstacles:
        #     ax.scatter(obstacle[0], obstacle[1], obstacle[2], c='g', s=30, marker="s")
        transposed_obstacles = np.transpose(np.array(obstacles))
        dxi = barrier(dxi, positionForCbf[:3][:], transposed_obstacles[:3][:], radius)
    else:
        dxi = barrier(dxi, positionForCbf[:3][:], np.array([]), radius)

    # print("Time usage for CBF :" + str(time.time()-startTimeBarrier))

    return dxi

def calculateVoronoiAndBarrier(container, positions, obstacles=None, radius = 0, hasDensityFunction=False, plotSphere=False):
    goals_x, goals_y, goals_z = calculateVoronoi(container, positions, radius, hasDensityFunction, plotSphere)
    
    # for position in positions:
    #     data6.append([])
    #     data5.append([])
    
    # print(goals_x, goals_y, goals_z)
    return calculateBarrier(positions, obstacles, radius, goals_x, goals_y, goals_z)

voronoiLimit = [(-20,-20, 0),(20,20,40)]
for i in range(300):
    c = createVoronoiContainer(drone, voronoiLimit, False)
    dxi = calculateVoronoiAndBarrier(c, drone, obstacles, 5.5, False, True)
    drone[0][0] = drone[0][0] + dxi[0][0] 
    drone[0][1] = drone[0][1] + dxi[1][0]
    drone[0][2] = drone[0][2] + dxi[2][0]

    drone[1][0] = drone[1][0] + dxi[0][1] 
    drone[1][1] = drone[1][1] + dxi[1][1]
    drone[1][2] = drone[1][2] + dxi[2][1]

    drone[2][0] = drone[2][0] + dxi[0][2] 
    drone[2][1] = drone[2][1] + dxi[1][2]
    drone[2][2] = drone[2][2] + dxi[2][2]

    drone[3][0] = drone[3][0] + dxi[0][3] 
    drone[3][1] = drone[3][1] + dxi[1][3]
    drone[3][2] = drone[3][2] + dxi[2][3]

    drone[4][0] = drone[4][0] + dxi[0][4] 
    drone[4][1] = drone[4][1] + dxi[1][4]
    drone[4][2] = drone[4][2] + dxi[2][4]

    drone[5][0] = drone[5][0] + dxi[0][5] 
    drone[5][1] = drone[5][1] + dxi[1][5]
    drone[5][2] = drone[5][2] + dxi[2][5]

    drone[6][0] = drone[6][0] + dxi[0][6] 
    drone[6][1] = drone[6][1] + dxi[1][6]
    drone[6][2] = drone[6][2] + dxi[2][6]

    drone[7][0] = drone[7][0] + dxi[0][7] 
    drone[7][1] = drone[7][1] + dxi[1][7]
    drone[7][2] = drone[7][2] + dxi[2][7]