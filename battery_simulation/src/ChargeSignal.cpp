/*
When the robot arrives the home base for recharging we starting the recharge process.
The recharge time is a parameter from the parameter file.
When the charge is complete a message would be published.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "battery_simulation/Charge.h"
#include "battery_simulation/Battery.h"
using namespace std;
bool home = false;
int battery = 0;
double charge_time, full_charge_time;

void callback(const std_msgs::Empty::ConstPtr &msg) {
    ROS_INFO("Starting to recharge the robot");
    charge_time = full_charge_time * (100-battery)/100 * 60*60;
    home = true;
}

void battery_callback(const battery_simulation::Battery &msg) {
    battery = msg.percent;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "charge_signal");
    battery_simulation::Charge charge_msg;
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    ros::Publisher charge = nh.advertise<battery_simulation::Charge>("charging", 1000);
    ros::Subscriber sub = nh.subscribe("going_to_recharge", 1, callback);
    ros::Subscriber sub2 = nh.subscribe("battery_state", 1, battery_callback);
    nh.getParam("ChargeSignal/energy/charge_time",full_charge_time);

	while ( ros::ok() ) {

        if(home){
            ROS_INFO("Charging time: %f ",charge_time);
            charge_msg.remaining_time = charge_time;
            if(charge_time <= 0)
                home = false;
            charge_time -= 1;
            charge.publish(charge_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
	}
	return 0;
}
