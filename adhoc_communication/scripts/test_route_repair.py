#!/usr/bin/python
import os
import argparse
import rospkg
import rospy
import time
import ConfigParser
import random
import socket

parser = argparse.ArgumentParser(description="Create and run multi-robot simulation.")
parser.add_argument("-n", "--number-of-robots", dest="number_of_robots", type=int,
                    help="Number of robots to simulate", metavar="N", default=[2],
                   nargs=1)
parser.add_argument("-d", "--distance", metavar="D", default=[10],
                    dest="distance", nargs=1, help="The name of the 'time'-directory for the log path.")

args, unknown = parser.parse_known_args()


if len(unknown) > 0:
    print "error: unrecognized arguments: %s" %unknown
    print "Please check the new syntax of this script.\n"
    parser.print_help()
    exit(0)

rospack = rospkg.RosPack()
package_path = rospack.get_path("adhoc_communication")
launch_file = "test_route_repair.launch"
launch_path = os.path.join(package_path, "launch", launch_file)
num_robots = args.number_of_robots[0]
distance = args.distance[0]

robot_macs = []
for num_robot in range(0,num_robots):
    robot_macs.append("02:%02d:00:00:00:00" %(num_robot + 1))


date = time.strftime("%y-%m-%d", time.localtime())
ttime = time.strftime("%H-%M-%S", time.localtime())
log_path = os.path.join(rospack.get_path("multi_robot_analyzer"), "logs", date, ttime)


fh_launch_file = open(launch_path,"w")
fh_launch_file.truncate()

fh_launch_file.write("<?xml version=\"1.0\"?>\n")
fh_launch_file.write("<launch>\n")
fh_launch_file.write("<node pkg=\"adhoc_communication\" name=\"simple_simulator\" type=\"simple_simulator.py\" output=\"screen\" > \n")
fh_launch_file.write("\t<param name=\"num_robots\" value=\"%i\" /> \n"%num_robots)
fh_launch_file.write("\t<param name=\"distance\" value=\"%i\" /> \n"%distance)
fh_launch_file.write("</node> \n")
for num_robot in range(0,num_robots):
	fh_launch_file.write("\t<group ns=\"robot_%i\">\n"% num_robot)
	fh_launch_file.write("\t\t<include file=\"$(find adhoc_communication)/launch/adhoc_communication.launch\">\n")
	fh_launch_file.write("\t\t\t<arg name=\"use_sim_time\" value=\"false\" />\n")
	fh_launch_file.write("\t\t\t<arg name=\"num_of_robots\" value=\"%s\" />\n" % args.number_of_robots[0])
	fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
	fh_launch_file.write("\t\t\t<arg name=\"emulate\" value=\"false\" />\n") 
	fh_launch_file.write("\t\t\t<arg name=\"hostname\" value=\"robot_%i\" />\n"%num_robot) 
	fh_launch_file.write("\t\t\t<arg name=\"interface\" value=\"lo\" />\n")
	fh_launch_file.write("\t\t\t<arg name=\"simulation_mode\" value=\"true\" />\n") 
	fh_launch_file.write("\t\t\t<arg name=\"mac\" value=\"%s\" />\n" %robot_macs[num_robot])
	fh_launch_file.write("\t\t</include>\n")
	fh_launch_file.write("\t</group>\n")
fh_launch_file.write("</launch>")
fh_launch_file.close()
os.system("roslaunch adhoc_communication test_route_repair.launch")


