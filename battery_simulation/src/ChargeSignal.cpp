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
    ros::Rate loop_rate(1);
    double charge_time;
    ros::Publisher charge = nh.advertise<std_msgs::Empty>("charge_complete", 1000);
    ros::Subscriber sub = nh.subscribe("going_to_recharge", 1, callback);
    nh.getParam("ChargeSignal/energy/charge_time",charge_time);

	while ( ros::ok() ) {

        if(home){
            ROS_INFO("Chargeing Time: %f ",charge_time*60*60);
            ros::Duration(charge_time*60*60).sleep();
            charge.publish(charge_msg);
            ROS_INFO("Sending recharge signal");
            home = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
	}
	return 0;
}
