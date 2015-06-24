/*
Battery simulation node
Calculates the percentage of the charging state
of the battery with a simple linear model.
*/
#include <ros/ros.h>
#include "battery_simulation/Voltage.h"
#include <std_msgs/Int32.h>

#define VOLTAGE_MAX 13.5
#define VOLTAGE_MIN 11.0

using namespace std;
double battery_voltage = VOLTAGE_MAX;

std_msgs::Int32 battery_state_per;


// Callback message
void battery_state_callback(const battery_simulation::Voltage::ConstPtr& msg)
{
    battery_voltage = msg->voltage;
}

int main(int argc, char** argv) {
    ROS_INFO("Battery simulator launching...");
    ros::init(argc, argv, "Battery_simulate");
	ros::NodeHandle n;

    ros::Publisher battery_pub = n.advertise<std_msgs::Int32>("battery_state_per", 1000);
    ros::Subscriber voltage_sub = n.subscribe("battery_state", 1000, battery_state_callback);

	ros::Rate loop_rate(0.5);

    float diff = VOLTAGE_MAX - VOLTAGE_MIN;

	while ( ros::ok() ) {

        // linear function for the battery_state calculation

        battery_state_per.data = (int) ((battery_voltage - VOLTAGE_MIN)/ (diff) *100);
        battery_pub.publish(battery_state_per);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
