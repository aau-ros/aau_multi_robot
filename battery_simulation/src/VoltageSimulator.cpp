/*
Voltage simulation node
Here we are publishing the battery voltage values.
When the battery voltage is to low we don't decrease the value
because the robot would be shut down already.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

#define VOLTAGE_MAX 13.5
#define VOLTAGE_MIN 11.0

using namespace std;

std_msgs::Float64 battery_state;

// Callback message when charging is done.

void charge_complete_callback(const std_msgs::Empty::ConstPtr &msg) {
    ROS_INFO("charging done.");
	battery_state.data = VOLTAGE_MAX;
}
int main(int argc, char** argv) {
	ROS_INFO("Voltage simulator launching...");
	ros::init(argc, argv, "voltage_simulate");
	ros::NodeHandle n;
	ros::Publisher voltage_pub = n.advertise<std_msgs::Float64>("battery_state", 1000);
	ros::Subscriber charge_sub = n.subscribe("charge_complete", 1000, charge_complete_callback);
	ros::Rate loop_rate(0.5);
	battery_state.data = VOLTAGE_MAX;
	while ( ros::ok() ) {
        voltage_pub.publish(battery_state);
        // Decrease by a value until we are at the cut off voltage
		if (battery_state.data > VOLTAGE_MIN)
			battery_state.data += -0.01;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
