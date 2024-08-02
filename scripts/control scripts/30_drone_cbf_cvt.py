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

#import function from other files
from voronoiAndBarrier import *

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

model9 = GetModelStateRequest()
model9.model_name = 'uav9'

model10 = GetModelStateRequest()
model10.model_name = 'uav10'

model11 = GetModelStateRequest()
model11.model_name = 'uav11'

model12 = GetModelStateRequest()
model12.model_name = 'uav12'

model13 = GetModelStateRequest()
model13.model_name = 'uav13'

model14 = GetModelStateRequest()
model14.model_name = 'uav14'

model15 = GetModelStateRequest()
model15.model_name = 'uav15'

model16 = GetModelStateRequest()
model16.model_name = 'uav16'

model17 = GetModelStateRequest()
model17.model_name = 'uav17'

model18 = GetModelStateRequest()
model18.model_name = 'uav18'

model19 = GetModelStateRequest()
model19.model_name = 'uav19'

model20 = GetModelStateRequest()
model20.model_name = 'uav20'

model21 = GetModelStateRequest()
model21.model_name = 'uav21'

model22 = GetModelStateRequest()
model22.model_name = 'uav22'

model23 = GetModelStateRequest()
model23.model_name = 'uav23'

model24 = GetModelStateRequest()
model24.model_name = 'uav24'

model25 = GetModelStateRequest()
model25.model_name = 'uav25'

model26 = GetModelStateRequest()
model26.model_name = 'uav26'

model27 = GetModelStateRequest()
model27.model_name = 'uav27'

model28 = GetModelStateRequest()
model28.model_name = 'uav28'

model29 = GetModelStateRequest()
model29.model_name = 'uav29'

model30 = GetModelStateRequest()
model30.model_name = 'uav30'

model_box_1 = GetModelStateRequest()
model_box_1.model_name = 'box_1'

model_box_2 = GetModelStateRequest()
model_box_2.model_name = 'box_2'

model_box_3 = GetModelStateRequest()
model_box_3.model_name = 'box_3'

model_box_4 = GetModelStateRequest()
model_box_4.model_name = 'box_4'

model_box_5 = GetModelStateRequest()
model_box_5.model_name = 'box_5'

model_box_6 = GetModelStateRequest()
model_box_6.model_name = 'box_6'

model_box_7 = GetModelStateRequest()
model_box_7.model_name = 'box_7'

model_box_8 = GetModelStateRequest()
model_box_8.model_name = 'box_8'

class Follower:
    def __init__(self):
        #Set publisher twist for each agent
        self.cmd_vel_pub01 = rospy.Publisher('uav1/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub02 = rospy.Publisher('uav2/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub03 = rospy.Publisher('uav3/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub04 = rospy.Publisher('uav4/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub05 = rospy.Publisher('uav5/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub06 = rospy.Publisher('uav6/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub07 = rospy.Publisher('uav7/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub08 = rospy.Publisher('uav8/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub09 = rospy.Publisher('uav9/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub10 = rospy.Publisher('uav10/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub11 = rospy.Publisher('uav11/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub12 = rospy.Publisher('uav12/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub13 = rospy.Publisher('uav13/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub14 = rospy.Publisher('uav14/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub15 = rospy.Publisher('uav15/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub16 = rospy.Publisher('uav16/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub17 = rospy.Publisher('uav17/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub18 = rospy.Publisher('uav18/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub19 = rospy.Publisher('uav19/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub20 = rospy.Publisher('uav20/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub21 = rospy.Publisher('uav21/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub22 = rospy.Publisher('uav22/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub23 = rospy.Publisher('uav23/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub24 = rospy.Publisher('uav24/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub25 = rospy.Publisher('uav25/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub26 = rospy.Publisher('uav26/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub27 = rospy.Publisher('uav27/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub28 = rospy.Publisher('uav28/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub29 = rospy.Publisher('uav29/cmd_vel',Twist, queue_size=1)
        self.cmd_vel_pub30 = rospy.Publisher('uav30/cmd_vel',Twist, queue_size=1)
        self.twist1 = Twist()
        self.twist2 = Twist()
        self.twist3 = Twist()
        self.twist4 = Twist()
        self.twist5 = Twist()
        self.twist6 = Twist()
        self.twist7 = Twist()
        self.twist8 = Twist()
        self.twist9 = Twist()
        self.twist10 = Twist()
        self.twist11 = Twist()
        self.twist12 = Twist()
        self.twist13 = Twist()
        self.twist14 = Twist()
        self.twist15 = Twist()
        self.twist16 = Twist()
        self.twist17 = Twist()
        self.twist18 = Twist()
        self.twist19 = Twist()
        self.twist20 = Twist()
        self.twist21 = Twist()
        self.twist22 = Twist()
        self.twist23 = Twist()
        self.twist24 = Twist()
        self.twist25 = Twist()
        self.twist26 = Twist()
        self.twist27 = Twist()
        self.twist28 = Twist()
        self.twist29 = Twist()
        self.twist30 = Twist()
        self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        pose_box_1 = self.get_model_srv(model_box_1).pose.position
        pose_box_2 = self.get_model_srv(model_box_2).pose.position
        pose_box_3 = self.get_model_srv(model_box_3).pose.position
        pose_box_4 = self.get_model_srv(model_box_4).pose.position
        pose_box_5 = self.get_model_srv(model_box_5).pose.position
        pose_box_6 = self.get_model_srv(model_box_6).pose.position
        pose_box_7 = self.get_model_srv(model_box_7).pose.position
        pose_box_8 = self.get_model_srv(model_box_8).pose.position
        self.pose_box_1 = [pose_box_1.x, pose_box_1.y, pose_box_1.z]
        self.pose_box_2 = [pose_box_2.x, pose_box_2.y, pose_box_2.z]
        self.pose_box_3 = [pose_box_3.x, pose_box_3.y, pose_box_3.z]
        self.pose_box_4 = [pose_box_4.x, pose_box_4.y, pose_box_4.z]
        self.pose_box_5 = [pose_box_5.x, pose_box_5.y, pose_box_5.z]
        self.pose_box_6 = [pose_box_6.x, pose_box_6.y, pose_box_6.z]
        self.pose_box_7 = [pose_box_7.x, pose_box_7.y, pose_box_7.z]
        self.pose_box_8 = [pose_box_8.x, pose_box_8.y, pose_box_8.z]
        self.obstacles = [self.pose_box_1, self.pose_box_2, self.pose_box_3, self.pose_box_4, self.pose_box_5, self.pose_box_6, self.pose_box_7, self.pose_box_8]
        #safety radius for barrier
        self.safety_radius = 5.8
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
        self.cmd_vel_pub09.publish(self.twist9)
        self.cmd_vel_pub10.publish(self.twist10)
        self.cmd_vel_pub11.publish(self.twist11)
        self.cmd_vel_pub12.publish(self.twist12)
        self.cmd_vel_pub13.publish(self.twist13)
        self.cmd_vel_pub14.publish(self.twist14)
        self.cmd_vel_pub15.publish(self.twist15)
        self.cmd_vel_pub16.publish(self.twist16)
        self.cmd_vel_pub17.publish(self.twist17)
        self.cmd_vel_pub18.publish(self.twist18)
        self.cmd_vel_pub19.publish(self.twist19)
        self.cmd_vel_pub20.publish(self.twist20)
        self.cmd_vel_pub21.publish(self.twist21)
        self.cmd_vel_pub22.publish(self.twist22)
        self.cmd_vel_pub23.publish(self.twist23)
        self.cmd_vel_pub24.publish(self.twist24)
        self.cmd_vel_pub25.publish(self.twist25)
        self.cmd_vel_pub26.publish(self.twist26)
        self.cmd_vel_pub27.publish(self.twist27)
        self.cmd_vel_pub28.publish(self.twist28)
        self.cmd_vel_pub29.publish(self.twist29)
        self.cmd_vel_pub30.publish(self.twist30)

    def getPose(self): #Get postion of agents
        pose1 = self.get_model_srv(model1).pose.position
        pose2 = self.get_model_srv(model2).pose.position
        pose3 = self.get_model_srv(model3).pose.position
        pose4 = self.get_model_srv(model4).pose.position
        pose5 = self.get_model_srv(model5).pose.position
        pose6 = self.get_model_srv(model6).pose.position
        pose7 = self.get_model_srv(model7).pose.position
        pose8 = self.get_model_srv(model8).pose.position
        pose9 = self.get_model_srv(model9).pose.position
        pose10 = self.get_model_srv(model10).pose.position
        pose11 = self.get_model_srv(model11).pose.position
        pose12 = self.get_model_srv(model12).pose.position
        pose13 = self.get_model_srv(model13).pose.position
        pose14 = self.get_model_srv(model14).pose.position
        pose15 = self.get_model_srv(model15).pose.position
        pose16 = self.get_model_srv(model16).pose.position
        pose17 = self.get_model_srv(model17).pose.position
        pose18 = self.get_model_srv(model18).pose.position
        pose19 = self.get_model_srv(model19).pose.position
        pose20 = self.get_model_srv(model20).pose.position
        pose21 = self.get_model_srv(model21).pose.position
        pose22 = self.get_model_srv(model22).pose.position
        pose23 = self.get_model_srv(model23).pose.position
        pose24 = self.get_model_srv(model24).pose.position
        pose25 = self.get_model_srv(model25).pose.position
        pose26 = self.get_model_srv(model26).pose.position
        pose27 = self.get_model_srv(model27).pose.position
        pose28 = self.get_model_srv(model28).pose.position
        pose29 = self.get_model_srv(model29).pose.position
        pose30 = self.get_model_srv(model30).pose.position
        self.pose1 = [pose1.x, pose1.y, pose1.z]
        self.pose2 = [pose2.x, pose2.y, pose2.z]
        self.pose3 = [pose3.x, pose3.y, pose3.z]
        self.pose4 = [pose4.x, pose4.y, pose4.z]
        self.pose5 = [pose5.x, pose5.y, pose5.z]
        self.pose6 = [pose6.x, pose6.y, pose6.z]
        self.pose7 = [pose7.x, pose7.y, pose7.z]
        self.pose8 = [pose8.x, pose8.y, pose8.z]
        self.pose9 = [pose9.x, pose9.y, pose9.z]
        self.pose10 = [pose10.x, pose10.y, pose10.z]
        self.pose11 = [pose11.x, pose11.y, pose11.z]
        self.pose12 = [pose12.x, pose12.y, pose12.z]
        self.pose13 = [pose13.x, pose13.y, pose13.z]
        self.pose14 = [pose14.x, pose14.y, pose14.z]
        self.pose15 = [pose15.x, pose15.y, pose15.z]
        self.pose16 = [pose16.x, pose16.y, pose16.z] 
        self.pose17 = [pose17.x, pose17.y, pose17.z]
        self.pose18 = [pose18.x, pose18.y, pose18.z]
        self.pose19 = [pose19.x, pose19.y, pose19.z]
        self.pose20 = [pose20.x, pose20.y, pose20.z]
        self.pose21 = [pose21.x, pose21.y, pose21.z]
        self.pose22 = [pose22.x, pose22.y, pose22.z]
        self.pose23 = [pose23.x, pose23.y, pose23.z]
        self.pose24 = [pose24.x, pose24.y, pose24.z]
        self.pose25 = [pose25.x, pose25.y, pose25.z]
        self.pose26 = [pose26.x, pose26.y, pose26.z] 
        self.pose27 = [pose27.x, pose27.y, pose27.z]
        self.pose28 = [pose28.x, pose28.y, pose28.z]
        self.pose29 = [pose29.x, pose29.y, pose29.z]
        self.pose30 = [pose30.x, pose30.y, pose30.z]
        

    def calculateVelo(self):
        # positions_old = [self.pose1, self.pose2, self.pose3, self.pose4, self.pose5, self.pose6, self.pose7, self.pose8]
        positions = [self.pose1, self.pose2, self.pose3, self.pose4, self.pose5, self.pose6, self.pose7, self.pose8, self.pose9, self.pose10, 
                     self.pose11, self.pose12, self.pose13, self.pose14, self.pose15, self.pose16, self.pose17, self.pose18, self.pose19, self.pose20,
                     self.pose21, self.pose22, self.pose23, self.pose24, self.pose25, self.pose26, self.pose27, self.pose28, self.pose29, self.pose30,]
        # positions=[]
        voronoiLimit = [(-20,-20, 0),(20,20,40)]
        # start = time.time()
        # for i in range(len(positions_old)): #prevent limit exceed due to the performance issue
        #     x = positions_old[i][0]
        #     y = positions_old[i][1]
        #     z = positions_old[i][2]
        #     # print(str(x) + ", " + str(y) + ", " + str(z))
        #     if (x < voronoiLimit[0][0]):
        #         x = voronoiLimit[0][0]+0.0001
        #     elif (x > voronoiLimit[1][0]):
        #         x = voronoiLimit[1][0]-0.0001

        #     if (y < voronoiLimit[0][1]):
        #         y = voronoiLimit[0][1]+0.0001
        #     elif (y > voronoiLimit[1][1]):
        #         y = voronoiLimit[1][1]-0.0001

        #     if (z < voronoiLimit[0][2]):
        #         z = voronoiLimit[0][2]+0.0001
        #     elif (z > voronoiLimit[1][2]):
        #         z = voronoiLimit[1][2]-0.0001   
        #     positions.append([x,y,z])
        # print("Used time for prevent limit exceed:" + str(time.time() - start))
        # print("Position:")
        # print(positions)
        # print(positions_old)

        # obstacles = [self.pose_box_1, self.pose_box_2, self.pose_box_3, self.pose_box_4, self.pose_box_5, self.pose_box_6, self.pose_box_7, self.pose_box_8]
        
        c = createVoronoiContainer(positions, voronoiLimit, False)
        
        dxi = calculateVoronoiAndBarrier(c, positions, self.obstacles, self.safety_radius, False, True)
        
        # print(dxi)
        self.twist1.linear.x = dxi[0][0]
        self.twist1.linear.y = dxi[1][0]
        self.twist1.linear.z = dxi[2][0]
        self.twist2.linear.x = dxi[0][1]
        self.twist2.linear.y = dxi[1][1]
        self.twist2.linear.z = dxi[2][1]
        self.twist3.linear.x = dxi[0][2]
        self.twist3.linear.y = dxi[1][2]
        self.twist3.linear.z = dxi[2][2]
        self.twist4.linear.x = dxi[0][3]
        self.twist4.linear.y = dxi[1][3]
        self.twist4.linear.z = dxi[2][3]
        self.twist5.linear.x = dxi[0][4]
        self.twist5.linear.y = dxi[1][4]
        self.twist5.linear.z = dxi[2][4]
        self.twist6.linear.x = dxi[0][5]
        self.twist6.linear.y = dxi[1][5]
        self.twist6.linear.z = dxi[2][5]
        self.twist7.linear.x = dxi[0][6]
        self.twist7.linear.y = dxi[1][6]
        self.twist7.linear.z = dxi[2][6]
        self.twist8.linear.x = dxi[0][7]
        self.twist8.linear.y = dxi[1][7]
        self.twist8.linear.z = dxi[2][7]
        self.twist9.linear.x = dxi[0][8]
        self.twist9.linear.y = dxi[1][8]
        self.twist9.linear.z = dxi[2][8]
        self.twist10.linear.x = dxi[0][9]
        self.twist10.linear.y = dxi[1][9]
        self.twist10.linear.z = dxi[2][9]
        self.twist11.linear.x = dxi[0][10]
        self.twist11.linear.y = dxi[1][10]
        self.twist11.linear.z = dxi[2][10]
        self.twist12.linear.x = dxi[0][11]
        self.twist12.linear.y = dxi[1][11]
        self.twist12.linear.z = dxi[2][11]
        self.twist13.linear.x = dxi[0][12]
        self.twist13.linear.y = dxi[1][12]
        self.twist13.linear.z = dxi[2][12]
        self.twist14.linear.x = dxi[0][13]
        self.twist14.linear.y = dxi[1][13]
        self.twist14.linear.z = dxi[2][13]
        self.twist15.linear.x = dxi[0][14]
        self.twist15.linear.y = dxi[1][14]
        self.twist15.linear.z = dxi[2][14]
        self.twist16.linear.x = dxi[0][15]
        self.twist16.linear.y = dxi[1][15]
        self.twist16.linear.z = dxi[2][15]
        self.twist17.linear.x = dxi[0][16]
        self.twist17.linear.y = dxi[1][16]
        self.twist17.linear.z = dxi[2][16]
        self.twist18.linear.x = dxi[0][17]
        self.twist18.linear.y = dxi[1][17]
        self.twist18.linear.z = dxi[2][17]
        self.twist19.linear.x = dxi[0][18]
        self.twist19.linear.y = dxi[1][18]
        self.twist19.linear.z = dxi[2][18]
        self.twist20.linear.x = dxi[0][19]
        self.twist20.linear.y = dxi[1][19]
        self.twist20.linear.z = dxi[2][19]
        self.twist21.linear.x = dxi[0][20]
        self.twist21.linear.y = dxi[1][20]
        self.twist21.linear.z = dxi[2][20]
        self.twist22.linear.x = dxi[0][21]
        self.twist22.linear.y = dxi[1][21]
        self.twist22.linear.z = dxi[2][21]
        self.twist23.linear.x = dxi[0][22]
        self.twist23.linear.y = dxi[1][22]
        self.twist23.linear.z = dxi[2][22]
        self.twist24.linear.x = dxi[0][23]
        self.twist24.linear.y = dxi[1][23]
        self.twist24.linear.z = dxi[2][23]
        self.twist25.linear.x = dxi[0][24]
        self.twist25.linear.y = dxi[1][24]
        self.twist25.linear.z = dxi[2][24]
        self.twist26.linear.x = dxi[0][25]
        self.twist26.linear.y = dxi[1][25]
        self.twist26.linear.z = dxi[2][25]
        self.twist27.linear.x = dxi[0][26]
        self.twist27.linear.y = dxi[1][26]
        self.twist27.linear.z = dxi[2][26]
        self.twist28.linear.x = dxi[0][27]
        self.twist28.linear.y = dxi[1][27]
        self.twist28.linear.z = dxi[2][27]
        self.twist29.linear.x = dxi[0][28]
        self.twist29.linear.y = dxi[1][28]
        self.twist29.linear.z = dxi[2][28]
        self.twist30.linear.x = dxi[0][29]
        self.twist30.linear.y = dxi[1][29]
        self.twist30.linear.z = dxi[2][29]

        writeFile()
        

    def callback(self, data):
        while not rospy.is_shutdown():
            rate = rospy.Rate(hz)
            self.getPose()
            self.calculateVelo()
            self.move()
            rate.sleep()
        f.close()
        f2.close()
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
