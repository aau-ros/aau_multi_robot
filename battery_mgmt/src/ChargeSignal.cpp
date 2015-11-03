/*
When the robot arrives the home base for recharging we starting the recharge process.
The recharge time is a parameter from the parameter file.
When the charge is complete a message would be published.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "battery_mgmt/Charge.h"
#include "battery_mgmt/Battery.h"
using namespace std;
bool home = false;
int battery = 0;
double charge_time, full_charge_time;
double rate = 0.5; // Hz

/**
 * Get the charging state for the turtlebot
 * This information is read from the diagnostic array which is provided by the diagnostic aggregator
 */
void turtlebot_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
    for( size_t status_i = 0; status_i < msg->status.size(); ++status_i )
    {
        if( msg->status[status_i].name.compare("/Power System/Battery") == 0)
        {
            for (size_t value_i = 0; value_i < msg->status[status_i].values.size(); ++value_i)
            {
                // TODO: Get remaining charging time and publish it
                if( msg->status[status_i].values[value_i].key.compare("Charging State") == 0 )
                {
                    if(msg->status[status_i].values[value_i].value.c_str() == "Not Charging")
                    {

                    }
                }
            }
        }
    }
}

void callback(const std_msgs::Empty::ConstPtr &msg) {
    ROS_INFO("Starting to recharge the robot");
    charge_time = full_charge_time * (100-battery)/100 * 60*60;
    home = true;
}

void battery_callback(const battery_mgmt::Battery &msg) {
    battery = msg.percent;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "charge_signal");
    battery_mgmt::Charge charge_msg;
    ros::NodeHandle nh;
    ros::Rate loop_rate(rate);
    ros::Publisher charge = nh.advertise<battery_mgmt::Charge>("charging", 1);
    ros::Subscriber sub = nh.subscribe("going_to_recharge", 1, callback);
    ros::Subscriber sub2 = nh.subscribe("battery_state", 1, battery_callback);
    nh.getParam("ChargeSignal/energy/charge_time",full_charge_time);

	while ( ros::ok() ) {
        // charging at docking station
        if(home && charge_time > 0){
            charge_time -= (1/rate);
            if(charge_time <= 0)
                home = false;
            ROS_INFO("Charging time: %f ",charge_time);
            charge_msg.remaining_time = charge_time;
        }
        // discharging
        else{
            charge_time = full_charge_time * (100-battery)/100 * 60*60;
            charge_msg.remaining_time = charge_time;
        }
        charge.publish(charge_msg);

        ros::spinOnce();
        loop_rate.sleep();
	}
	return 0;
}
