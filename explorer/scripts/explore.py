#!/usr/bin/python
import os
import argparse
import rospkg
import time
import ConfigParser
import socket

parser = argparse.ArgumentParser(description="Create and run multi-robot exploration.")

parser.add_argument("-t", "--time", metavar="HH-MM-SS", default=[time.strftime("%H-%M-%S", time.localtime())],
                    dest="time", nargs=1, help="The name of the 'time'-directory for the log path.")
parser.add_argument("-d", "--date", metavar="yy-mm-dd", default=[time.strftime("%y-%m-%d", time.localtime())],
                    dest="date", nargs=1, help="The name of the 'time'-directory for the log path.")
parser.add_argument("-i", "--sim-id", default=[1], nargs=1, metavar="ID",
                    help="The ID of the execution run", dest="sim_id")
parser.add_argument("--exp-strategy",dest="exploration_strategy",type=int,nargs="?",default=1,
		    help="The strategy to be used for exploration.",choices=range(0,7))
parser.add_argument("--screen-output",dest="screen_output",default=False,help="Set ROS output to screen.",
		    action="store_true")
parser.add_argument("--interface",dest='interface',nargs='?',type=str,default='wlan0')

args, unknown = parser.parse_known_args()


if os.environ.get('ROBOT_PLATFORM') == "pioneer3at":
    os.system("sudo chmod a+rw /dev/ttyACM0")
    os.system("sudo chmod a+rw /dev/ttyUSB0")


if args.screen_output:
    output_type = "screen"
else:
    output_type = "log"

rospack = rospkg.RosPack()
sim_id = args.sim_id[0]

package_path = rospack.get_path("explorer")
launch_file = "auto_generated_launch_exploration.launch";
launch_path = os.path.join(package_path, "launch", launch_file)

t = time.localtime()
log_path = os.path.join(rospack.get_path("multi_robot_analyzer"), "logs", args.date[0], args.time[0], str(sim_id))
fh_launch_file = open(launch_path,"w")
fh_launch_file.truncate()
print ("Writing File headers")
fh_launch_file.write("<?xml version=\"1.0\"?>\n")
fh_launch_file.write("<launch>\n")


fh_launch_file.write("\t<include file=\"$(find explorer)/launch/simple_navigation_prerequisites_hydro_$(env ROBOT_PLATFORM).launch\">\n")
fh_launch_file.write("\t\t<arg name=\"log_path\" value=\"%s\" />\n" % log_path)
fh_launch_file.write("\t</include>\n\n")

fh_launch_file.write("\t<include file=\"$(find explorer)/launch/simple_navigation_$(env ROBOT_PLATFORM).launch\">\n")
fh_launch_file.write("\t\t<arg name=\"frontier_selection\" value=\"%s\" />" %args.exploration_strategy)
fh_launch_file.write("\t\t<arg name=\"log_path\" value=\"%s\" />\n" % log_path)
fh_launch_file.write("\t</include>\n\n")

fh_launch_file.write("\t<include file=\"$(find adhoc_communication)/launch/adhoc_communication.launch\">\n")
fh_launch_file.write("\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
fh_launch_file.write("\t\t<arg name=\"interface\" value=\"%s\" />\n" %args.interface)
fh_launch_file.write("\t</include>\n\n")

fh_launch_file.write("\t<include file=\"$(find connection_manager)/launch/connection_manager.launch\">\n")
fh_launch_file.write("\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
fh_launch_file.write("\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
fh_launch_file.write("\t</include>\n\n")

fh_launch_file.write("\t<include file=\"$(find map_merger)/launch/map_merger.launch\">\n")
fh_launch_file.write("\t\t<arg name=\"output\" value=\"%s\" />\n" %output_type)
fh_launch_file.write("\t\t<arg name=\"log_path\" value=\"%s\" />\n" %log_path)
fh_launch_file.write("\t\t<arg name=\"use_sim_time\" value=\"false\" />\n")
fh_launch_file.write("\t</include>\n\n")


fh_launch_file.write("</launch>\n")
fh_launch_file.close()

print("start_time_run_%s" %sim_id, "%s" %time.ctime())
os.system("roslaunch explorer %s" %launch_file)
