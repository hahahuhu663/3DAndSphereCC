#!/usr/bin/env python
import os
import sys
import tempfile
import rospy
import math
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from geometry_msgs.msg import Twist
from mpl_toolkits.mplot3d import Axes3D
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

hz = 60

model1 = GetModelStateRequest()
model1.model_name = 'uav1'

model2 = GetModelStateRequest()
model2.model_name = 'uav2'

model3 = GetModelStateRequest()
model3.model_name = 'uav3'

# model4 = GetModelStateRequest()
# model4.model_name = 'uav4'
#
# model5 = GetModelStateRequest()
# model5.model_name = 'uav5'
#
# model6 = GetModelStateRequest()
# model6.model_name = 'uav6'
#
# model7 = GetModelStateRequest()
# model7.model_name = 'uav7'
#
# model8 = GetModelStateRequest()
# model8.model_name = 'uav8'

f3 = open('/home/robolab/plot_result/error3drones.csv', 'w')
writer3 = csv.writer(f3)
data3 = []
# f = open('/home/robolab/plot_result_2/csv_file.csv', 'w')
# writer = csv.writer(f)
# data = []
# f2 = open('/home/robolab/plot_result/trajectories.csv', 'w')
# writer2 = csv.writer(f)
# data2 = []

def sphere(o, r, axes):
    u,v=np.mgrid[0:2*np.pi:10j, 0:np.pi:10j]
    x=np.cos(u)*np.sin(v)*r + o[0]
    y=np.sin(u)*np.sin(v)*r + o[1]
    z=np.cos(v)*r + o[2]
    axes.plot_wireframe(x, y, z, color='lightskyblue', linewidth=0.5)

def si_position_controller(xi, positions, x_velocity_gain=1, y_velocity_gain=1, z_velocity_gain=1, velocity_magnitude_limit=0.5):

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
    dxi[0][:] = x_velocity_gain*(positions[0][:]-xi[0][:])
    dxi[1][:] = y_velocity_gain*(positions[1][:]-xi[1][:])
    dxi[2][:] = z_velocity_gain*(positions[2][:]-xi[2][:])
    # Threshold magnitude
    norms = np.linalg.norm(dxi, axis=0)
    idxs = np.where(norms > velocity_magnitude_limit)
    if norms[idxs].size != 0:
        dxi[:, idxs] *= velocity_magnitude_limit/norms[idxs]
    return dxi

def barrier(dxi, x, safety_radius=3, barrier_gain=1, unsafe_barrier_gain=1, magnitude_limit=0.6):
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
    N = dxi.shape[1]
    num_constraints = int(comb(N, 2))

    A = np.zeros((num_constraints, 3*N))
    b = np.zeros(num_constraints)
    H = sparse(matrix(2*np.identity(3*N)))
    print("H")
    print(H)
    # H = sparse(matrix(3*np.identity(3*N)))

    count = 0
    for i in range(N-1):
        for j in range(i+1, N):
            error = x[:, i] - x[:, j]
            data3.append(math.sqrt(np.power(error[0],2) + np.power(error[1],2) + np.power(error[2],2)))
            h = (error[0]*error[0] + error[1]*error[1] + error[2]*error[2]) - np.power(safety_radius, 2)
            # h = math.sqrt((error[0]*error[0] + error[1]*error[1] + error[2]*error[2])) - safety_radius
            A[count, (3*i, (3*i+1),(3*i+2))] = -2*error
            A[count, (3*j, (3*j+1),((3*j+2)))] = 2*error

            if h >= 0:
                b[count] = barrier_gain*np.power(h, 3)
            else:
                b[count] = unsafe_barrier_gain*np.power(h, 3)

            count += 1
    # Threshold control inputs before QP
    norms = np.linalg.norm(dxi, 2, 0)
    # print("NORMS")
    # print(norms)
    idxs_to_normalize = (norms > magnitude_limit)
    dxi[:, idxs_to_normalize] *= magnitude_limit/norms[idxs_to_normalize]

    f = -2*np.reshape(dxi, 3*N, order='F')
    result = qp(H, matrix(f), matrix(A), matrix(b))['x']
    print(result)

    return np.reshape(result, (3, -1), order='F')

# def si_position_controller(xi, positions, x_velocity_gain=1, y_velocity_gain=1, velocity_magnitude_limit=0.15):
#
#     """
#     xi: 2xN numpy array (of single-integrator states of the robots)
#     points: 2xN numpy array (of desired points each robot should achieve)
#     -> 2xN numpy array (of single-integrator control inputs)
#     """
#     #Check user input types
#     assert isinstance(xi, np.ndarray), "In the si_position_controller function created by the create_si_position_controller function, the single-integrator robot states (xi) must be a numpy array. Recieved type %r." % type(xi).__name__
#     assert isinstance(positions, np.ndarray), "In the si_position_controller function created by the create_si_position_controller function, the robot goal points (positions) must be a numpy array. Recieved type %r." % type(positions).__name__
#     #Check user input ranges/sizes
#     assert xi.shape[0] == 2, "In the si_position_controller function created by the create_si_position_controller function, the dimension of the single-integrator robot states (xi) must be 2 ([x;y]). Recieved dimension %r." % xi.shape[0]
#     assert positions.shape[0] == 2, "In the si_position_controller function created by the create_si_position_controller function, the dimension of the robot goal points (positions) must be 2 ([x_goal;y_goal]). Recieved dimension %r." % positions.shape[0]
#     assert xi.shape[1] == positions.shape[1], "In the si_position_controller function created by the create_si_position_controller function, the number of single-integrator robot states (xi) must be equal to the number of robot goal points (positions). Recieved a single integrator current position input array of size %r x %r and desired position array of size %r x %r." % (xi.shape[0], xi.shape[1], positions.shape[0], positions.shape[1])
#     _,N = np.shape(xi)
#     dxi = np.zeros((2, N))
#     # Calculate control input
#     dxi[0][:] = x_velocity_gain*(positions[0][:]-xi[0][:])
#     dxi[1][:] = y_velocity_gain*(positions[1][:]-xi[1][:])
#     # Threshold magnitude
#     norms = np.linalg.norm(dxi, axis=0)
#     idxs = np.where(norms > velocity_magnitude_limit)
#     if norms[idxs].size != 0:
#         dxi[:, idxs] *= velocity_magnitude_limit/norms[idxs]
#     return dxi
#
# def barrier(dxi, x, safety_radius=0.17, barrier_gain=100, magnitude_limit=0.2):
#     #Check user input types
#     assert isinstance(dxi, np.ndarray), "In the function created by the create_single_integrator_barrier_certificate2 function, the single-integrator robot velocity command (dxi) must be a numpy array. Recieved type %r." % type(dxi).__name__
#     assert isinstance(x, np.ndarray), "In the function created by the create_single_integrator_barrier_certificate2 function, the robot states (x) must be a numpy array. Recieved type %r." % type(x).__name__
#
#     #Check user input ranges/sizes
#     assert x.shape[0] == 2, "In the function created by the create_single_integrator_barrier_certificate2 function, the dimension of the single integrator robot states (x) must be 2 ([x;y]). Recieved dimension %r." % x.shape[0]
#     assert dxi.shape[0] == 2, "In the function created by the create_single_integrator_barrier_certificate2 function, the dimension of the robot single integrator velocity command (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % dxi.shape[0]
#     assert x.shape[1] == dxi.shape[1], "In the function created by the create_single_integrator_barrier_certificate2 function, the number of robot states (x) must be equal to the number of robot single integrator velocity commands (dxi). Recieved a current robot pose input array (x) of size %r x %r and single integrator velocity array (dxi) of size %r x %r." % (x.shape[0], x.shape[1], dxi.shape[0], dxi.shape[1])
#
#
#     # Initialize some variables for computational savings
#     N = dxi.shape[1]
#     num_constraints = int(comb(N, 2))
#     A = np.zeros((num_constraints, 2*N))
#     b = np.zeros(num_constraints)
#     H = sparse(matrix(2*np.identity(2*N)))
#
#     count = 0
#     for i in range(N-1):
#         for j in range(i+1, N):
#             error = x[:, i] - x[:, j]
#             h = (error[0]*error[0] + error[1]*error[1]) - np.power(safety_radius, 2)
#
#             A[count, (2*i, (2*i+1))] = -2*error
#             A[count, (2*j, (2*j+1))] = 2*error
#             if h >= 0:
#                 b[count] = barrier_gain*np.power(h, 3)
#             else:
#                 b[count] = unsafe_barrier_gain*np.power(h, 3)
#
#             count += 1
#     # Threshold control inputs before QP
#     norms = np.linalg.norm(dxi, 2, 0)
#     idxs_to_normalize = (norms > magnitude_limit)
#     dxi[:, idxs_to_normalize] *= magnitude_limit/norms[idxs_to_normalize]
#
#     f = -2*np.reshape(dxi, 2*N, order='F')
#     result = qp(H, matrix(f), matrix(A), matrix(b))['x']
#
#     return np.reshape(result, (2, -1), order='F')

def getLineLen(x_len, y_len, z_len):
    return math.sqrt((x_len**2) + (y_len**2) + (z_len**2))

def getVelocity(i, initials, destinations, speed = 20):
    popSize = len(initials)
    new_velo = []
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
    # data.append(line_len)
    # data.append('')
    # data.append(x_initial)
    # data.append(y_initial)
    # data.append(z_initial)
    if((line_len/(hz/2))*2 < (speed/(hz/2))):
        speed = line_len/(hz/2)
    z_velo = ((z_destination - z_initial)/line_len)*speed
    y_velo = ((y_destination - y_initial)/line_len)*speed
    x_velo = ((x_destination - x_initial)/line_len)*speed
    new_velo = [x_velo, y_velo, z_velo]
    return new_velo

class Follower:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.ax.set_zlim(0, 12)
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)
        self.cmd_vel_pub01 = rospy.Publisher('uav1/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub02 = rospy.Publisher('uav2/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub03 = rospy.Publisher('uav3/cmd_vel',Twist, queue_size=1)
        # self.cmd_vel_pub04 = rospy.Publisher('uav4/cmd_vel',Twist, queue_size=1)
        # self.cmd_vel_pub05 = rospy.Publisher('uav5/cmd_vel',Twist, queue_size=1)
        # self.cmd_vel_pub06 = rospy.Publisher('uav6/cmd_vel',Twist, queue_size=1)
        # self.cmd_vel_pub07 = rospy.Publisher('uav7/cmd_vel',Twist, queue_size=1)
        # self.cmd_vel_pub08 = rospy.Publisher('uav8/cmd_vel',Twist, queue_size=1)
        self.twist1 = Twist()
        self.twist2 = Twist()
        self.twist3 = Twist()
        # self.twist4 = Twist()
        # self.twist5 = Twist()
        # self.twist6 = Twist()
        # self.twist7 = Twist()
        # self.twist8 = Twist()

        self.pose1 = []
        self.pose2 = []
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        a = self.move

    def move(self):
        self.cmd_vel_pub01.publish(self.twist1)
        self.cmd_vel_pub02.publish(self.twist2)
        self.cmd_vel_pub03.publish(self.twist3)
        # self.cmd_vel_pub04.publish(self.twist4)
        # self.cmd_vel_pub05.publish(self.twist5)
        # self.cmd_vel_pub06.publish(self.twist6)
        # self.cmd_vel_pub07.publish(self.twist7)
        # self.cmd_vel_pub08.publish(self.twist8)

    def getPose(self):
        get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        pose1 = get_model_srv(model1).pose.position
        pose2 = get_model_srv(model2).pose.position
        pose3 = get_model_srv(model3).pose.position
        # pose4 = get_model_srv(model4).pose.position
        # pose5 = get_model_srv(model5).pose.position
        # pose6 = get_model_srv(model6).pose.position
        # pose7 = get_model_srv(model7).pose.position
        # pose8 = get_model_srv(model8).pose.position
        self.pose1 = [pose1.x, pose1.y, pose1.z]
        self.pose2 = [pose2.x, pose2.y, pose2.z]
        self.pose3 = [pose3.x, pose3.y, pose3.z]
        # self.pose4 = [pose4.x, pose4.y, pose4.z]
        # self.pose5 = [pose5.x, pose5.y, pose5.z]
        # self.pose6 = [pose6.x, pose6.y, pose6.z]
        # self.pose7 = [pose7.x, pose7.y, pose7.z]
        # self.pose8 = [pose8.x, pose8.y, pose8.z]

    def resetAxes(self):
        self.ax.clear()
        self.ax.set_zlim(0, 12)
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)

    def plotVoronoi(self, vertices, faceVertices, centroid, position, color):
        poly3d = [[vertices[faceVertices[ix][iy]] for iy in range(len(faceVertices[ix]))] for ix in range(len(faceVertices))]
        self.ax.add_collection3d(Poly3DCollection(poly3d,edgecolors='k',facecolors=color,linewidths=1, alpha=0.2))
        self.ax.scatter(centroid[0], centroid[1], centroid[2], c='r', s=30)
        self.ax.scatter(position[0], position[1], position[2], c='b', s=30)
        self.ax.plot([position[0],centroid[0]],[position[1],centroid[1]],[position[2],centroid[2]],color = 'k', linestyle='dashed')


    def calculateVelo(self):
        o_position = [self.pose1, self.pose2, self.pose3]
        for p in o_position:
            self.ax.scatter(p[0], p[1], p[2], c='b', s=30)
            sphere(p,3, self.ax)
        position = np.transpose(np.array(o_position))
        dxi = si_position_controller(position[:3][:], (np.mat('0 0 -5 ; 5 -5 0; 5 5 5'))[:3][:])
        dxi = barrier(dxi, position[:3][:])
        # print(self.pose1)


        self.twist1.linear.x = dxi[0][0]
        self.twist1.linear.y = dxi[1][0]
        self.twist1.linear.z = dxi[2][0]
        # if (self.pose1[2] < 2):
        #     self.twist1.linear.z = 0.3
        # elif (self.pose1[2] > 2):
        #     self.twist1.linear.z = -0.3
        # else:
            # self.twist1.linear.z = 0
        self.twist2.linear.x = dxi[0][1]
        self.twist2.linear.y = dxi[1][1]
        self.twist2.linear.z = dxi[2][1]
        self.twist3.linear.x = dxi[0][2]
        self.twist3.linear.y = dxi[1][2]
        self.twist3.linear.z = dxi[2][2]
        # if (self.pose2[2] < 2):
        #     self.twist2.linear.z = 0.3
        # elif (self.pose2[2] > 2):
        #     self.twist2.linear.z = -0.3
        # else:
        #     self.twist2.linear.z = 0
        # # positions = [self.pose1, self.pose2, self.pose3, self.pose4, self.pose5, self.pose6, self.pose7, self.pose8]
        # c = Container(positions, limits=[(-20,-20, 0),(20,20,40)], periodic=False)
        # i = 0
        # j = 7
        # colorList = ['k','y','g','c','b','m','r','w']
        # for v in c:
        #     vertices = v.vertices()
        #     faceVertices = v.face_vertices()
        #     self.plotVoronoi(vertices, faceVertices, v.centroid(), positions[i], colorList[i])
        #     new_velo =  getVelocity(i, positions, v.centroid())
        #     if (i == 0):
        #         self.twist1.linear.x = new_velo[0]
        #         self.twist1.linear.y = new_velo[1]
        #         self.twist1.linear.z = new_velo[2]
        #     elif (i == 1):
        #         self.twist2.linear.x = new_velo[0]
        #         self.twist2.linear.y = new_velo[1]
        #         self.twist2.linear.z = new_velo[2]
        #     elif (i == 2):
        #         self.twist3.linear.x = new_velo[0]
        #         self.twist3.linear.y = new_velo[1]
        #         self.twist3.linear.z = new_velo[2]
        #     elif (i == 3):
        #         self.twist4.linear.x = new_velo[0]
        #         self.twist4.linear.y = new_velo[1]
        #         self.twist4.linear.z = new_velo[2]
        #     elif (i == 4):
        #         self.twist5.linear.x = new_velo[0]
        #         self.twist5.linear.y = new_velo[1]
        #         self.twist5.linear.z = new_velo[2]
        #     elif (i == 5):
        #         self.twist6.linear.x = new_velo[0]
        #         self.twist6.linear.y = new_velo[1]
        #         self.twist6.linear.z = new_velo[2]
        #     elif (i == 6):
        #         self.twist7.linear.x = new_velo[0]
        #         self.twist7.linear.y = new_velo[1]
        #         self.twist7.linear.z = new_velo[2]
        #     elif (i == 7):
        #         self.twist8.linear.x = new_velo[0]
        #         self.twist8.linear.y = new_velo[1]
        #         self.twist8.linear.z = new_velo[2]
        #     else:
        #         sys.exit("velocity loop error")
        #     # plt.show()
        #     i += 1
        #     j -= 1
        now = rospy.get_rostime()
        plt.savefig('/home/robolab/plot_result/result_3_drones/FIG_'+str(now)+'.png')
        # writer.writerow(data)
        # data.clear()
        self.resetAxes()

        writer3.writerow([np.min(data3)])
        data3.clear()

    def callback(self, data):
        while not rospy.is_shutdown():
            rate = rospy.Rate(hz)
            self.getPose()
            self.calculateVelo()
            self.move()
            rate.sleep()
        f3.close()

def listener():
    rospy.init_node('follower')
    rospy.wait_for_service('/gazebo/get_model_state')
    follower = Follower()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        f.close()
        pass
