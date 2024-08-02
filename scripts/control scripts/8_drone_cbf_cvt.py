#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from geometry_msgs.msg import Twist
# import matplotlib
# matplotlib.use('Agg')
# matplotlib.use('SVG')

#import function from other files
# from voronoiAndBarrierV2 import *
from voronoiAndBarrier import *


# hz = 60

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

# model_box_1 = GetModelStateRequest()
# model_box_1.model_name = 'box_1'

# model_box_2 = GetModelStateRequest()
# model_box_2.model_name = 'box_2'

# model_box_3 = GetModelStateRequest()
# model_box_3.model_name = 'box_3'

# model_box_4 = GetModelStateRequest()
# model_box_4.model_name = 'box_4'

# model_box_5 = GetModelStateRequest()
# model_box_5.model_name = 'box_5'

# model_box_6 = GetModelStateRequest()
# model_box_6.model_name = 'box_6'

# model_box_7 = GetModelStateRequest()
# model_box_7.model_name = 'box_7'

# model_box_8 = GetModelStateRequest()
# model_box_8.model_name = 'box_8'

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
        self.twist1 = Twist()
        self.twist2 = Twist()
        self.twist3 = Twist()
        self.twist4 = Twist()
        self.twist5 = Twist()
        self.twist6 = Twist()
        self.twist7 = Twist()
        self.twist8 = Twist()
        self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # pose_box_1 = self.get_model_srv(model_box_1).pose.position
        # pose_box_2 = self.get_model_srv(model_box_2).pose.position
        # pose_box_3 = self.get_model_srv(model_box_3).pose.position
        # pose_box_4 = self.get_model_srv(model_box_4).pose.position
        # pose_box_5 = self.get_model_srv(model_box_5).pose.position
        # pose_box_6 = self.get_model_srv(model_box_6).pose.position
        # pose_box_7 = self.get_model_srv(model_box_7).pose.position
        # pose_box_8 = self.get_model_srv(model_box_8).pose.position
        # self.pose_box_1 = [pose_box_1.x, pose_box_1.y, pose_box_1.z]
        # self.pose_box_2 = [pose_box_2.x, pose_box_2.y, pose_box_2.z]
        # self.pose_box_3 = [pose_box_3.x, pose_box_3.y, pose_box_3.z]
        # self.pose_box_4 = [pose_box_4.x, pose_box_4.y, pose_box_4.z]
        # self.pose_box_5 = [pose_box_5.x, pose_box_5.y, pose_box_5.z]
        # self.pose_box_6 = [pose_box_6.x, pose_box_6.y, pose_box_6.z]
        # self.pose_box_7 = [pose_box_7.x, pose_box_7.y, pose_box_7.z]
        # self.pose_box_8 = [pose_box_8.x, pose_box_8.y, pose_box_8.z]
        # self.obstacles = [self.pose_box_1, self.pose_box_2, self.pose_box_3, self.pose_box_4, self.pose_box_5, self.pose_box_6, self.pose_box_7, self.pose_box_8]
        #safety radius for barrier
        self.obstacles = [
            [-10, -10, 2.5],
            [-10, -10, 7.5],
            [-10, -10, 12.5],
            [-10, -10, 17.5],
            [-10, -10, 22.5],
            [-10, -10, 27.5],
            [-10, -10, 32.5],
            [-10, -10, 37.5],
            [12.5, 12.5, 2.5],
            [12.5, 12.5, 7.5],
            [7.5, 7.5, 2.5],
            [7.5, 7.5, 7.5],
            [7.5, 12.5, 2.5],
            [7.5, 12.5, 7.5],
            [12.5, 7.5, 2.5],
            [12.5, 7.5, 7.5],
            [10, 10, 12.5],
            [-7, -7, 2.5],
            [10, -10, 2.5],
        ]
        self.safety_radius = 5.5
        self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        # a = self.move

    def move(self):
        # print(self.twist1)
        self.cmd_vel_pub01.publish(self.twist1)
        self.cmd_vel_pub02.publish(self.twist2)
        self.cmd_vel_pub03.publish(self.twist3)
        self.cmd_vel_pub04.publish(self.twist4)
        self.cmd_vel_pub05.publish(self.twist5)
        self.cmd_vel_pub06.publish(self.twist6)
        self.cmd_vel_pub07.publish(self.twist7)
        self.cmd_vel_pub08.publish(self.twist8)

    def getPose(self): #Get postion of agents
        pose1 = self.get_model_srv(model1).pose.position
        pose2 = self.get_model_srv(model2).pose.position
        pose3 = self.get_model_srv(model3).pose.position
        pose4 = self.get_model_srv(model4).pose.position
        pose5 = self.get_model_srv(model5).pose.position
        pose6 = self.get_model_srv(model6).pose.position
        pose7 = self.get_model_srv(model7).pose.position
        pose8 = self.get_model_srv(model8).pose.position
        self.pose1 = [pose1.x, pose1.y, pose1.z]
        self.pose2 = [pose2.x, pose2.y, pose2.z]
        self.pose3 = [pose3.x, pose3.y, pose3.z]
        self.pose4 = [pose4.x, pose4.y, pose4.z]
        self.pose5 = [pose5.x, pose5.y, pose5.z]
        self.pose6 = [pose6.x, pose6.y, pose6.z]
        self.pose7 = [pose7.x, pose7.y, pose7.z]
        self.pose8 = [pose8.x, pose8.y, pose8.z]
        

    def calculateVelo(self):
        # positions_old = [self.pose1, self.pose2, self.pose3, self.pose4, self.pose5, self.pose6, self.pose7, self.pose8]
        positions = [self.pose1, self.pose2, self.pose3, self.pose4, self.pose5, self.pose6, self.pose7, self.pose8]
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
        # dxi = calculateVoronoiAndBarrier(c, positions, None, self.safety_radius, True, True)
        
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

        writeFile()
        

    def callback(self, data):
        i = 0
        while not rospy.is_shutdown():
            startTime = time.time()
            # rate = rospy.Rate(hz)
            self.getPose()
            self.calculateVelo()
            self.move()
            # if (i == 0):
            #     i += 1
            #     showPlot()
            # rate.sleep()
            print("time taken per refresh")
            print(time.time() - startTime)
            print("REFRESHED!!!")
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
