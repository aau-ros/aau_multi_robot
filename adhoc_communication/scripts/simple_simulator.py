#!/usr/bin/python
#import os
#import argparse
#import rospkg
import rospy
#import time
#import ConfigParser
import random
#import socket
#import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from adhoc_communication.srv import ChangeMCMembership

publishers = [];
position_x = [];
position_y = [];

rospy.loginfo("Inited Positions and Publishers")
rospy.init_node("test_route_repair", log_level=rospy.DEBUG)


num_orientations = 4 # N => 0, E => 1, S => 2, O => 3
width = 25
height = 25

num_robots = rospy.get_param('~num_robots');
distance_to_connect = rospy.get_param('~distance');

robots = []

random.seed(0)

class Robot(object):
    id = 0

    def __init__(self, x, y, o):
        self.x = x
        self.y = y
        self.orientation = o
        self.id = Robot.id

        self.travel_distance = 0
        self.distance_to_travel = 0
        
        self.p_straight = 0.30
        self.p_left = 0.20
        self.p_right = 0.20
        self.p_back = 0.1
        #self.p_stall = 0.2

        full_topic = "robot_%i/base_pose_ground_truth" %self.id
        self.publisher = rospy.Publisher(full_topic,Odometry,queue_size=10,latch=True)
        Robot.id += 1


    def publish(self):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position = Point(self.x, self.y, 0)
        msg.header.frame_id = str(self.orientation)
        self.publisher.publish(msg)


    def move(self):
        #rospy.logdebug("Robot %d previous position: x=%d y=%d o=%d" %(self.id,
                                                                      #self.x,
                                                                      #self.y,
                                                                      #self.orientation))
        # move robot in th according direction
        if self.orientation == 0:
            self.y += 1
        elif self.orientation == 1:
            self.x += 1
        elif self.orientation == 2:
            self.y -= 1
        elif self.orientation == 3:
            self.x -= 1

        # make sure robot does not leave map
        if self.x < 1:
            self.orientation = 0
            self.x = 1
        if self.x > height:
            self.orientation = 3
            self.x = height
        if self.y < 1:
            self.orientation = 1
            self.y = 1
        if self.y > width:
            self.orintation = 4
            self.y = width
        rospy.logdebug("Robot %d new position     : x=%d y=%d o=%d     Distance left: %d" %(self.id,
                                                                      self.x,
                                                                      self.y,
                                                                      self.orientation,
                                                                      self.distance_to_travel))





    def step(self):
        if self.distance_to_travel <= 0:
            # determine new orientation and travel distance
            r = random.random()
            if r <= self.p_straight:
                # orientate straight, do nothing
                #rospy.logwarn("Straight")
                pass 
            elif r <= self.p_straight + self.p_left:
                #rospy.logwarn("Left")
                # orientate left
                self.orientation = (self.orientation - 1) % num_orientations
            elif r <= self.p_straight + self.p_left + self.p_right:
                #rospy.logwarn("Right")
                # orientate right
                self.orientation = (self.orientation + 1) % num_orientations
            elif r <= self.p_straight + self.p_left + self.p_right + self.p_back:
                # orientate back
                #rospy.logwarn("Back")
                self.orientation = (self.orientation + 2) % num_orientations
            else:
                #rospy.logwarn("Stall")
                return
            self.distance_to_travel = random.randint(0,5)
            rospy.loginfo("Traveling %d steps in direction %d." %(self.distance_to_travel, self.orientation))
        
        # travel
        self.move()
        self.distance_to_travel -= 1





for robot in range(0,num_robots):
    x_initial = width / 2 - (num_robots / 2) + robot
    y_initial = 1
    robots.append(Robot(x_initial, y_initial,0))








while not rospy.is_shutdown():
    rospy.sleep(.1)

    for robot in robots:
        robot.step()
        
    for cur_robot in range(0,num_robots):
        for compare_robot in range(0,num_robots):
            if cur_robot == compare_robot:
                continue
            else:
                #dx = position_x[cur_robot] - position_x[compare_robot]
                #dy = position_y[cur_robot] - position_y[compare_robot]
                #dis = math.sqrt(dx*dx+dy*dy)
                #rospy.loginfo("dx:%i"%dx)
                #rospy.loginfo("dy:%i"%dy)
                #rospy.loginfo("dis:%i"%dis)
                robot_name = "robot_%d" %cur_robot
                name = "/robot_%i/adhoc_communication/join_mc_group"%cur_robot
                group_name = "mc_robot_"+str(compare_robot)
                rospy.logdebug("Robot %s trying to join group '%s'" %(name,group_name))
                try:
                    join_service = rospy.ServiceProxy(name, ChangeMCMembership)
                    res = join_service(group_name,1)
                except rospy.service.ServiceException:
                    rospy.logwarn("Robot %s cannot join group %s." %(robot_name,group_name))
