/*
Battery simulation node
Calculates the percentage of the charging state
of the battery with a simple linear model.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "battery_simulation/Voltage.h"
#include "geometry_msgs/Twist.h"


using namespace std;
int battery_charge;
int moving_consumption;
int standing_consumption;
double actual_charge;
double x_linear, z_angular;

battery_simulation::Voltage battery_state;

// Callback message
void charge_complete_callback(const std_msgs::Empty::ConstPtr &msg) {
    //When the charging is done we reset the battery state to 100%.
    ROS_INFO("charging done.");
    battery_state.percent = 100;
    battery_state.recharge = true;
    actual_charge = battery_charge;
}
void cmd_callback(const geometry_msgs::Twist &msg){
    //get the velocity and rotation values
    x_linear = msg.linear.x;
    z_angular = msg.angular.z;
}

int main(int argc, char** argv) {
    ROS_INFO("Battery simulator launching...");
    ros::init(argc, argv, "Battery_simulate");
	ros::NodeHandle n;

    battery_state.recharge = false;
    battery_state.percent = 100;

    double rate = 0.5;
    double temp;

    ros::Publisher battery_pub = n.advertise<battery_simulation::Voltage>("battery_state", 1000);
    ros::Subscriber charge_sub = n.subscribe("charge_complete", 1000, charge_complete_callback);
    ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1000, cmd_callback);

    n.getParam("ChargeSignal/energy/battery_charge",battery_charge);
    n.getParam("ChargeSignal/energy/moving_consumption",moving_consumption);
    n.getParam("ChargeSignal/energy/standing_consumption",standing_consumption);

    actual_charge = battery_charge;
    ros::Rate loop_rate(rate);

	while ( ros::ok() ) {

        battery_pub.publish(battery_state);
        /*
        In the cmd_vel message there are only two important parameters, the x-linear value and the z-angular value.
        When they are zero the robots stands still. If the x-value is unequal to zero then the robot moves forward or
        backward and if the z-value is unequal to zero the robot rotates.
        When the robot stands still it consumes less energy but when it moves it consumes more energy.
        */
        if(x_linear == 0 && z_angular == 0){
            if (battery_state.percent > 0){
                temp = actual_charge - standing_consumption / (rate * 3600 );
                actual_charge = temp;
            }
            }else{
            if (battery_state.percent > 0)
                temp = actual_charge - moving_consumption / (rate * 3600 );
                actual_charge = temp;
        }
        battery_state.percent = (actual_charge * 100) / battery_charge ;

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
