#!/usr/bin/env python
import rospy
from array import array
from  nav_msgs.msg import OccupancyGrid
from  std_msgs.msg import String
import operator

class MapListener:
	def __init__(self,topic):
		self.sub_topic = topic
		rospy.Subscriber(self.sub_topic,OccupancyGrid,self.callback)
	def callback(self,data):
		_frame_id = data.header.frame_id
		rospy.loginfo(_frame_id)


if __name__ == '__main__':
	rospy.loginfo("started my node")
	rospy.init_node('map_listener_simple',anonymous=True)
	listener = MapListener("map")
	rospy.spin()
