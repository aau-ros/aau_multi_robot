/*
When the charge is complete a message would be published.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
using namespace std;
int main(int argc, char** argv) {

	ros::init(argc, argv, "charge_signal");
    std_msgs::Empty charge_msg;
    ros::NodeHandle nh;
    ros::Publisher charge = nh.advertise<std_msgs::Empty>("charge_complete", 1000);



    ros::Rate loop_rate(0.003);

	while ( ros::ok() ) {
        charge.publish(charge_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
