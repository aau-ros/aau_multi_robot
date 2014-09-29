#!/usr/bin/python
import os
import argparse
import rospkg
import rospy
import time
import ConfigParser
import random
import socket
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from adhoc_communication.srv import ChangeMCMembership

publishers = [];
position_x = [];
position_y = [];

rospy.loginfo("Inited Positions and Publishers")
rospy.init_node("test_route_repair")


looks = []; # 0 = N ; 1 = E ; 2 = S ; 3 = W
#TMP Size of map  100x100
width = 25
height = 25

num_robots = rospy.get_param('~num_robots');
distance_to_connect = rospy.get_param('~distance');

for num_robot in range(0,num_robots):
	full_topic = "robot_%i/base_pose_ground_truth"%num_robot
	publishers.append(rospy.Publisher(full_topic,Odometry,queue_size=10,latch=True))
	x = random.randint(0,width)
	y = random.randint(0,height)
	position_x.append(x)
	position_y.append(y)
	looks.append(random.randrange(0,4))

while not rospy.is_shutdown():
	rospy.sleep(1.)
	#rospy.loginfo("start")
#Change positions randomply:
#50% forward, 20%Left 20%Right 10%Back
	for num_robot in range(0,num_robots):
		val = random.randint(0,10)
		#rospy.loginfo("--%i--"%val)
		if val > 3: #Forward
			if looks[num_robot] == 0:
				position_x[num_robot] = position_x[num_robot] + 1
			elif looks[num_robot] == 1:
				position_y[num_robot] = position_y[num_robot] + 1
			elif looks[num_robot] == 2:
				position_x[num_robot] = position_x[num_robot] - 1
			elif looks[num_robot] == 3:
				position_y[num_robot] = position_y[num_robot] - 1
		elif val > 2:#Left
			looks[num_robot] = looks[num_robot] - 1
			if looks[num_robot] < 0:
				looks[num_robot] = 3
		elif val > 1:#Right
			looks[num_robot] = looks[num_robot] + 1
			if looks[num_robot] > 3:
				looks[num_robot] = 0
		else: #Back
			if looks[num_robot] == 0:
				position_x[num_robot] = position_x[num_robot] - 1
			elif looks[num_robot] == 1:
				position_y[num_robot] = position_y[num_robot] - 1
			elif looks[num_robot] == 2:
				position_x[num_robot] = position_x[num_robot] + 1
			elif looks[num_robot] == 3:
				position_y[num_robot] = position_y[num_robot] + 1
		if position_x[num_robot] < 1:
			looks[num_robot] = 0
		if position_x[num_robot] > height:
			looks[num_robot] = 3

		if position_y[num_robot] < 0:
			looks[num_robot] = 1
		if position_y[num_robot] > width:
			looks[num_robot] = 4


	for num_robot in range(0,num_robots):
		msg = Odometry()
		msg.header.stamp = rospy.Time.now()
		msg.pose.pose.position = Point(position_x[num_robot], position_y[num_robot], 0)
		msg.header.frame_id = str(looks[num_robot])
		publishers[num_robot].publish(msg)

	for cur_robot in range(0,num_robots):
			for compare_robot in range(0,num_robots):
				if cur_robot == compare_robot:
					continue
				else:
					dx = position_x[cur_robot] - position_x[compare_robot]
					dy = position_y[cur_robot] - position_y[compare_robot]
					dis = math.sqrt(dx*dx+dy*dy)
					#rospy.loginfo("dx:%i"%dx)
					#rospy.loginfo("dy:%i"%dy)
					#rospy.loginfo("dis:%i"%dis)
					if dis < distance_to_connect:
						name = "/robot_%i/adhoc_communication/join_mc_group"%cur_robot
						group_name = "robot_"+str(compare_robot)
						rospy.loginfo("%s"%group_name)
						rospy.loginfo(" is calling %s to join group"%name)
						join_service = rospy.ServiceProxy(name, ChangeMCMembership)
						res = join_service(group_name,1)
						#rospy.loginfo("result:%i"%res)
					#join mc groud
