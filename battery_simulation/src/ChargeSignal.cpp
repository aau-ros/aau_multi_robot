/*
When the charge is complete a message would be published.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
using namespace std;
bool home = false;

void callback(const std_msgs::Empty::ConstPtr &msg) {
    ROS_INFO("Starting to recharge the robot");
    home = true;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "charge_signal");
    std_msgs::Empty charge_msg;
    ros::NodeHandle nh;
    ros::Publisher charge = nh.advertise<std_msgs::Empty>("charge_complete", 1000);
    ros::Subscriber sub = nh.subscribe("going_to_recharge", 1000, callback);

	while ( ros::ok() ) {
        if(home){
            ros::Duration(20).sleep();
            charge.publish(charge_msg);
            ROS_INFO("Sending recharge signal");
            home = false;
        }

        ros::spinOnce();
	}
	return 0;
}
