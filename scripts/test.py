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

hz = 60

model1 = GetModelStateRequest()
model1.model_name = 'uav1'

model2 = GetModelStateRequest()
model2.model_name = 'uav2'

model3 = GetModelStateRequest()
model3.model_name = 'uav3'

model4 = GetModelStateRequest()
model4.model_name = 'uav4'

model5 = GetModelStateRequest()
model5.model_name = 'uav5'

model6 = GetModelStateRequest()
model6.model_name = 'uav6'

model7 = GetModelStateRequest()
model7.model_name = 'uav7'

model8 = GetModelStateRequest()
model8.model_name = 'uav8'

f = open('/home/robolab/plot_result/csv_file.csv', 'w')
writer = csv.writer(f)
data = []
# f2 = open('/home/robolab/plot_result/trajectories.csv', 'w')
# writer2 = csv.writer(f)
# data2 = []

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
    data.append(line_len)
    data.append('')
    data.append(x_initial)
    data.append(y_initial)
    data.append(z_initial)
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
        self.ax.set_zlim(0, 40)
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        self.cmd_vel_pub01 = rospy.Publisher('uav1/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub02 = rospy.Publisher('uav2/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub03 = rospy.Publisher('uav3/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub04 = rospy.Publisher('uav4/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub05 = rospy.Publisher('uav5/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub06 = rospy.Publisher('uav6/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub07 = rospy.Publisher('uav7/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub08 = rospy.Publisher('uav8/cmd_vel',Twist, queue_size=1)
        self.twist1 = Twist()
        self.twist2 = Twist()
        self.twist3 = Twist()
        self.twist4 = Twist()
        self.twist5 = Twist()
        self.twist6 = Twist()
        self.twist7 = Twist()
        self.twist8 = Twist()

        self.pose1 = []
        self.pose2 = []
        self.pose3 = []
        self.pose4 = []
        self.pose5 = []
        self.pose6 = []
        self.pose7 = []
        self.pose8 = []

        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        a = self.move

    def move(self):
        self.cmd_vel_pub01.publish(self.twist1)
        self.cmd_vel_pub02.publish(self.twist2)
        self.cmd_vel_pub03.publish(self.twist3)
        self.cmd_vel_pub04.publish(self.twist4)
        self.cmd_vel_pub05.publish(self.twist5)
        self.cmd_vel_pub06.publish(self.twist6)
        self.cmd_vel_pub07.publish(self.twist7)
        self.cmd_vel_pub08.publish(self.twist8)

    def getPose(self):
        get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        pose1 = get_model_srv(model1).pose.position
        pose2 = get_model_srv(model2).pose.position
        pose3 = get_model_srv(model3).pose.position
        pose4 = get_model_srv(model4).pose.position
        pose5 = get_model_srv(model5).pose.position
        pose6 = get_model_srv(model6).pose.position
        pose7 = get_model_srv(model7).pose.position
        pose8 = get_model_srv(model8).pose.position
        self.pose1 = [pose1.x, pose1.y, pose1.z]
        self.pose2 = [pose2.x, pose2.y, pose2.z]
        self.pose3 = [pose3.x, pose3.y, pose3.z]
        self.pose4 = [pose4.x, pose4.y, pose4.z]
        self.pose5 = [pose5.x, pose5.y, pose5.z]
        self.pose6 = [pose6.x, pose6.y, pose6.z]
        self.pose7 = [pose7.x, pose7.y, pose7.z]
        self.pose8 = [pose8.x, pose8.y, pose8.z]

    def resetAxes(self):
        self.ax.clear()
        self.ax.set_zlim(0, 40)
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)

    def plotVoronoi(self, vertices, faceVertices, centroid, position, color):
        poly3d = [[vertices[faceVertices[ix][iy]] for iy in range(len(faceVertices[ix]))] for ix in range(len(faceVertices))]
        self.ax.add_collection3d(Poly3DCollection(poly3d,edgecolors='k',facecolors=color,linewidths=1, alpha=0.2))
        self.ax.scatter(centroid[0], centroid[1], centroid[2], c='r', s=30)
        self.ax.scatter(position[0], position[1], position[2], c='b', s=30)
        self.ax.plot([position[0],centroid[0]],[position[1],centroid[1]],[position[2],centroid[2]],color = 'k', linestyle='dashed')

    def calculateVelo(self):
        positions = [self.pose1, self.pose2, self.pose3, self.pose4, self.pose5, self.pose6, self.pose7, self.pose8]
        c = Container(positions, limits=[(-20,-20, 0),(20,20,40)], periodic=False)
        i = 0
        j = 7
        colorList = ['k','y','g','c','b','m','r','w']
        for v in c:
            vertices = v.vertices()
            faceVertices = v.face_vertices()
            self.plotVoronoi(vertices, faceVertices, v.centroid(), positions[i], colorList[i])
            new_velo =  getVelocity(i, positions, v.centroid())
            if (i == 0):
                self.twist1.linear.x = new_velo[0]
                self.twist1.linear.y = new_velo[1]
                self.twist1.linear.z = new_velo[2]
            elif (i == 1):
                self.twist2.linear.x = new_velo[0]
                self.twist2.linear.y = new_velo[1]
                self.twist2.linear.z = new_velo[2]
            elif (i == 2):
                self.twist3.linear.x = new_velo[0]
                self.twist3.linear.y = new_velo[1]
                self.twist3.linear.z = new_velo[2]
            elif (i == 3):
                self.twist4.linear.x = new_velo[0]
                self.twist4.linear.y = new_velo[1]
                self.twist4.linear.z = new_velo[2]
            elif (i == 4):
                self.twist5.linear.x = new_velo[0]
                self.twist5.linear.y = new_velo[1]
                self.twist5.linear.z = new_velo[2]
            elif (i == 5):
                self.twist6.linear.x = new_velo[0]
                self.twist6.linear.y = new_velo[1]
                self.twist6.linear.z = new_velo[2]
            elif (i == 6):
                self.twist7.linear.x = new_velo[0]
                self.twist7.linear.y = new_velo[1]
                self.twist7.linear.z = new_velo[2]
            elif (i == 7):
                self.twist8.linear.x = new_velo[0]
                self.twist8.linear.y = new_velo[1]
                self.twist8.linear.z = new_velo[2]
            else:
                sys.exit("velocity loop error")
            # plt.show()
            i += 1
            j -= 1
        now = rospy.get_rostime()
        plt.savefig('/home/robolab/plot_result/result/FIG_'+str(now)+'.png')
        writer.writerow(data)
        data.clear()
        self.resetAxes()

    def callback(self, data):
        while not rospy.is_shutdown():
            rate = rospy.Rate(hz)
            self.getPose()
            self.calculateVelo()
            self.move()
            rate.sleep()
        f.close()

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
