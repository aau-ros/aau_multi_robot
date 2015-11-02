/*
Battery simulation node
Calculates the percentage of the charging state
of the battery with a simple linear model.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "battery_mgmt/Battery.h"
#include "battery_mgmt/Charge.h"
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

battery_mgmt::Battery battery_state;

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
                if( msg->status[status_i].values[value_i].key.compare("Percent") == 0 )
                {
                    battery_charge = (int) ::atof(msg->status[status_i].values[value_i].value.c_str());
                }
            }
        }
    }
}

void charge_starting_callback(const std_msgs::Empty::ConstPtr &msg) {
    charging = true;
}

void charging_callback(const battery_mgmt::Charge::ConstPtr &msg) {
    if(msg->remaining_time <= 0)
    {
        ROS_INFO("Charging done");
        battery_state.percent = 100;
        battery_state.remaining_distance = avg_speed * (float)battery_charge/moving_consumption*3600;
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

    // If the average speed is very low, there is probably something wrong
    if(msg.avg_speed > 0.25){
        avg_speed = msg.avg_speed;
    }
    else{
        avg_speed = 0.25;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Battery_simulate");
	ros::NodeHandle n;
    ROS_INFO("Battery simulator launching...");

    avg_speed = 0.25;

    n.getParam("BatteryState/energy/battery_charge",battery_charge);
    n.getParam("BatteryState/energy/moving_consumption",moving_consumption);
    n.getParam("BatteryState/energy/standing_consumption",standing_consumption);
    n.getParam("BatteryState/energy/charge_time",full_charge_time);

    battery_state.percent = 100;
    battery_state.remaining_distance = avg_speed * (float)battery_charge/moving_consumption*3600;

    double rate = 0.5; // Hz
    double moving_time;
    double standing_time;

    bool debugShown = false;
    bool simulation = false;

    standing_time = 0;
    moving_time = 0;
    charging = false;

    ros::Publisher battery_pub = n.advertise<battery_mgmt::Battery>("battery_state", 1);
    ros::Subscriber charge1_sub = n.subscribe("going_to_recharge", 1, charge_starting_callback);
    ros::Subscriber charge2_sub = n.subscribe("charging", 1000, charging_callback);
    ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1000, cmd_callback);
    ros::Subscriber speed_sub = n.subscribe("avg_speed", 1000, speed_callback);

    // get robot platform
    std::string env_var;
    ros::get_environment_variable(env_var, "ROBOT_PLATFORM");
    ROS_INFO("Environment variable: %s",env_var.c_str());

    // check if we are on a real robot or if we are in simulation
    if( env_var.compare("turtlebot") == 0)
    {
        //sub3 = nh.subscribe("diagnostics_agg",1000,&Explorer::turtlebot_callback,this);
    }
    else if(env_var.compare("pioneer3dx") == 0 || env_var.compare("pioneer3at") == 0)
    {
        // todo: read voltage and convert to percentage
    }
    else
    {
        simulation = true;
    }

    actual_charge = battery_charge;
    ros::Rate loop_rate(rate);

	while ( ros::ok() ) {

        battery_pub.publish(battery_state);

        // only use energy if robot is not recharging
        if(charging == false)
        {
            /*
             * In the cmd_vel message there are only two important parameters, the x-linear value and the z-angular value.
             * When they are zero the robots stands still. If the x-value is unequal to zero then the robot moves forward or
             * backward and if the z-value is unequal to zero the robot rotates.
             * When the robot stands still it consumes less energy and when it moves it consumes more energy.
             */
            if(battery_state.percent > 0){
                if(x_linear == 0 && z_angular == 0){
                    actual_charge -= standing_consumption / (rate * 3600 );
                }
                else{
                    actual_charge -= moving_consumption / (rate * 3600 );
                }
            }

            // percentage of battery charge remaining
            battery_state.percent = (actual_charge * 100) / battery_charge;

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
