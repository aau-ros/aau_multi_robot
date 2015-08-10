/*
Battery simulation node
Calculates the percentage of the charging state
of the battery with a simple linear model.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "battery_simulation/Battery.h"
#include "battery_simulation/Charge.h"
#include "geometry_msgs/Twist.h"
#include "explorer/Speed.h"


using namespace std;
int battery_charge;
int moving_consumption;
int standing_consumption;
double actual_charge;
double x_linear, z_angular;
double moving_percentage;
double avg_speed;
double full_charge_time;
bool charging;

battery_simulation::Battery battery_state;

void charge_starting_callback(const std_msgs::Empty::ConstPtr &msg) {
    charging = true;
}

void charging_callback(const battery_simulation::Charge::ConstPtr &msg) {
    if(msg->remaining_time <= 0)
    {
        ROS_INFO("Charging done");
        battery_state.percent = 100;
        battery_state.remaining_distance = avg_speed * moving_percentage * (float)battery_charge/moving_consumption*3600;
        actual_charge = battery_charge;
        charging = false;
    }
}

void cmd_callback(const geometry_msgs::Twist &msg){
    //get the velocity and rotation values
    x_linear = msg.linear.x;
    z_angular = msg.angular.z;
}

void speed_callback(const explorer::Speed &msg){
    avg_speed = msg.avg_speed;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Battery_simulate");
	ros::NodeHandle n;
    ROS_INFO("Battery simulator launching...");

    battery_state.percent = 100;
    battery_state.remaining_distance = avg_speed * (float)battery_charge/moving_consumption*3600;

    double rate = 0.5; // Hz
    double moving_time;
    double standing_time;

    bool debugShown = false;

    standing_time = 0;
    moving_time = 0;
    charging = false;

    ros::Publisher battery_pub = n.advertise<battery_simulation::Battery>("battery_state", 1);
    ros::Subscriber charge1_sub = n.subscribe("going_to_recharge", 1, charge_starting_callback);
    ros::Subscriber charge2_sub = n.subscribe("charging", 1000, charging_callback);
    ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1000, cmd_callback);
    ros::Subscriber speed_sub = n.subscribe("avg_speed", 1000, speed_callback);

    n.getParam("BatteryState/energy/battery_charge",battery_charge);
    n.getParam("BatteryState/energy/moving_consumption",moving_consumption);
    n.getParam("BatteryState/energy/standing_consumption",standing_consumption);
    n.getParam("BatteryState/energy/charge_time",full_charge_time);

    actual_charge = battery_charge;
    ros::Rate loop_rate(rate);

	while ( ros::ok() ) {

        battery_pub.publish(battery_state);

        // only use energy if robot is not recharging
        if(charging == false)
        {
            /*
            In the cmd_vel message there are only two important parameters, the x-linear value and the z-angular value.
            When they are zero the robots stands still. If the x-value is unequal to zero then the robot moves forward or
            backward and if the z-value is unequal to zero the robot rotates.
            When the robot stands still it consumes less energy and when it moves it consumes more energy.
            */
            if(x_linear == 0 && z_angular == 0){
                if (battery_state.percent > 0){
                    actual_charge -= standing_consumption / (rate * 3600 );
                }
                standing_time += 1/rate;
            }else{
                if (battery_state.percent > 0){
                    actual_charge -= moving_consumption / (rate * 3600 );
                }
                moving_time += 1/rate;
            }

            // percentage of battery charge remaining
            battery_state.percent = (actual_charge * 100) / battery_charge;

            /* The remaining distance depends on the percentage of time the robot is moving.
             * In the beginning this calculation is very inaccurate. We set the remaining distance to a high value
             * to avoid that the robot returns too early to the home point for recharging.
             */
            if(avg_speed <= 0.25){
                avg_speed = 0.25;
            }
            if(moving_time < 30){
                moving_percentage = 0.25;
            }
            else{
                moving_percentage = moving_time / (standing_time+moving_time);
            }

            //ROS_ERROR("remaining_distance = %.2f/100 * %.2f * %d/%d*3600", battery_state.percent, avg_speed, battery_charge, moving_consumption);
            // remaining distance the robot can still travel in meters
            //                                                                            max time the robot can travel with full battery
            battery_state.remaining_distance = battery_state.percent/100.0 * avg_speed * (float)battery_charge/moving_consumption*3600;

            int bs = battery_state.percent;
            if((bs % 10) == 0)
            {
                if (debugShown == false)
                {
                    ROS_ERROR("Battery: %d%%  ---  remaining distance: %.2fm", bs, battery_state.remaining_distance);
                    debugShown = true;
                }
            }
            else
                debugShown = false;
        }

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
