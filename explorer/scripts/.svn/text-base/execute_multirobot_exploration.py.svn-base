#!/usr/bin/python
import os
import argparse
import rospkg
import time
import ConfigParser
import socket

import get_interfaces

parser = argparse.ArgumentParser(description="Create and run multi-robot exploration.")

parser.add_argument("-t", "--time", metavar="HH-MM-SS", default=[time.strftime("%H-%M-%S", time.localtime())],
                    dest="time", nargs=1, help="The name of the 'time'-directory for the log path.")
parser.add_argument("-d", "--date", metavar="yy-mm-dd", default=[time.strftime("%y-%m-%d", time.localtime())],
                    dest="date", nargs=1, help="The name of the 'time'-directory for the log path.")
parser.add_argument("-i", "--sim-id", default=[1], nargs=1, metavar="ID",
                    help="The ID of the execution run", dest="sim_id")
parser.add_argument("--exp-strategy",dest="exploration_strategy",type=int,nargs="?",default=7,
		    help="The strategy to be used for exploration.",choices=range(1,8))
parser.add_argument("--screen-output",dest="screen_output",default=False,help="Set ROS output to screen.",
		    action="store_true")
parser.add_argument("-b",dest="bag",type=bool,nargs="?",default=True,
		    help="Record the run with rosbag")



args, unknown = parser.parse_known_args()



if args.screen_output:
    output_type = "screen"
else:
    output_type = "log"

rospack = rospkg.RosPack()
launch_file = "now.launch";
sim_id = args.sim_id[0]

interface = ""
interfaces = get_interfaces.get_network_interfaces()
#print [str(ni) for ni in interfaces]
for ni in interfaces:
	if(ni.name.startswith("w")):
		interface=ni.name
	#print ni.name

package_path = rospack.get_path("explorer")
launch_file = "launch_exploration.launch";
launch_path = os.path.join(package_path, "launch", launch_file)

t = time.localtime()
log_path = os.path.join(rospack.get_path("multi_robot_analyzer"), "logs", args.date[0], args.time[0], str(sim_id))
fh_launch_file = open(launch_path,"w")
fh_launch_file.truncate()
print ("Writing File headers")
fh_launch_file.write("<?xml version=\"1.0\"?>\n")
fh_launch_file.write("<launch>\n")


fh_launch_file.write("\t<include file=\"$(find explorer)/launch/exploration.launch\">")
fh_launch_file.write("\t\t<arg name=\"log_path\" value=\"%s\" />\n" % log_path)
fh_launch_file.write("\t\t<arg name=\"frontier_selection\" value=\"%s\" />" %args.exploration_strategy)
fh_launch_file.write("\t</include>\n")

fh_launch_file.write("\t\t<include file=\"$(find map_merger)/launch/map_merger.launch\">\n")
fh_launch_file.write("\t\t\t<arg name=\"output\" value=\"%s\" />" %output_type)
fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
fh_launch_file.write("\t\t\t<arg name=\"use_sim_time\" value=\"false\" />\n")
fh_launch_file.write("\t\t</include>\n")

fh_launch_file.write("\t\t<include file=\"$(find connection_manager)/launch/connection_manager.launch\">\n")
fh_launch_file.write("\t\t\t<arg name=\"output\" value=\"%s\" />" %output_type)
fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
fh_launch_file.write("\t\t\t<arg name=\"use_sim_time\" value=\"false\" />\n")
fh_launch_file.write("\t\t</include>\n")

fh_launch_file.write("\t\t<include file=\"$(find adhoc_communication)/launch/adhoc_communication.launch\">\n")
fh_launch_file.write("\t\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
fh_launch_file.write("\t\t\t<arg name=\"use_sim_time\" value=\"false\" />\n")
#fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
fh_launch_file.write("\t\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
fh_launch_file.write("\t\t\t<arg name=\"interface\" value=\"%s\" />\n" %interface)
fh_launch_file.write("\t\t</include>\n")

if args.bag == True:
	fh_launch_file.write("\t\t<node name=\"rosbag\" type=\"start_rosbag.sh\" pkg=\"explorer\" args=\"%s/rosbag.bag\"/>"%log_path)
fh_launch_file.write("</launch>\n")
fh_launch_file.close()

print("start_time_run_%s" %sim_id, "%s" %time.ctime())
os.system("roslaunch explorer launch_exploration.launch")




