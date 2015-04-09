/*
Voltage simulation node
Publishes values corresponding to a simulated battery from a given
max to a given cut off.
Once the cut off is reached, cease decreasing and wait for a 'charge
complete' signal to be published before beginning from the start.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

#define VOLTAGE_MAX 12.8
#define VOLTAGE_MIN 11.0

using namespace std;

std_msgs::Float64 battery_state;
/*
Message indicating charging is complete.
*/
void charge_complete_callback(const std_msgs::Empty::ConstPtr &msg) {
	ROS_INFO("Got signal we are charged.");
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
		ROS_INFO("Publishing voltage %f...", battery_state.data);
		voltage_pub.publish(battery_state);
		// Decrease by 0.1 every second if we're not at cut off
		if (battery_state.data > VOLTAGE_MIN)
			battery_state.data += -0.01;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
