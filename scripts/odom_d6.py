#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
rospy.init_node('odom_d6')
odom_pub = rospy.Publisher("odom_d6", Odometry, queue_size=60)
rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
odom = Odometry()
#header = Header()
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"
model = GetModelStateRequest()
model.model_name = 'uav6'

r = rospy.Rate(60)
while not rospy.is_shutdown():
	result = get_model_srv(model)
	odom.pose.pose = result.pose
	odom.twist.twist = result.twist
	odom.header.stamp = rospy.Time.now()
	odom_pub.publish(odom)
	r.sleep()
