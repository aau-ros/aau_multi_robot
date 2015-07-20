/*
When the charge is complete a message would be published.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "battery_simulation/Voltage.h"
using namespace std;
bool home = false;
int battery = 0;

void callback(const std_msgs::Empty::ConstPtr &msg) {
    ROS_INFO("Starting to recharge the robot");
    home = true;
}

void battery_callback(const battery_simulation::Voltage &msg) {
    battery = msg.percent;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "charge_signal");
    std_msgs::Empty charge_msg;
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    double charge_time, full_charge_time;
    ros::Publisher charge = nh.advertise<std_msgs::Empty>("charge_complete", 1000);
    ros::Subscriber sub = nh.subscribe("going_to_recharge", 1, callback);
    ros::Subscriber sub2 = nh.subscribe("battery_state", 1, battery_callback);
    nh.getParam("ChargeSignal/energy/charge_time",full_charge_time);

	while ( ros::ok() ) {

        if(home){
            charge_time = full_charge_time * (100-battery)/100 * 60*60;
            ROS_INFO("Chargeing Time: %f ",charge_time);
            ros::Duration(charge_time).sleep();
            charge.publish(charge_msg);
            ROS_INFO("Sending recharge signal");
            home = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
	}
	return 0;
}
