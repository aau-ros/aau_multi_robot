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
double battery_charge;
int moving_consumption;
int standing_consumption;
double actual_charge;
double x_linear, z_angular;
double avg_speed;
bool charging;
double moving_time;
double standing_time;
double moving_perc;
double standing_perc;
double full_charge_time;
double remaining_charge_time;

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
                    battery_charge = ::atof(msg->status[status_i].values[value_i].value.c_str());
                }
            }
        }
    }
}

void charge_starting_callback(const std_msgs::Empty::ConstPtr &msg) {
    charging = true;
}

void charging_callback(const battery_mgmt::Charge::ConstPtr &msg) {
    remaining_charge_time = msg->remaining_time;
    if(remaining_charge_time <= 0)
    {
        ROS_INFO("Charging done");
        battery_state.percent = 100;
        battery_state.remaining_distance = avg_speed * battery_charge/moving_consumption*3600;
        battery_state.remaining_time = battery_charge*3600 * (moving_perc/moving_consumption + standing_perc/standing_consumption);
        actual_charge = battery_charge;
        charging = false;
    }
}

/*
 * In the cmd_vel message there are only two important parameters, the x-linear value and the z-angular value.
 * When they are zero the robots stands still. If the x-value is unequal to zero then the robot moves forward or
 * backward and if the z-value is unequal to zero the robot rotates.
 * When the robot stands still it consumes less energy and when it moves it consumes more energy.
 */
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
    moving_time = 0;
    standing_time = 0;
    moving_perc = 0.5;
    standing_perc = 0.5;
    remaining_charge_time = 0;

    double rate = 0.5; // Hz

    bool debugShown = false;
    bool simulation = false;

    charging = false;

    n.getParam("BatteryState/energy/battery_charge",battery_charge);
    n.getParam("BatteryState/energy/moving_consumption",moving_consumption);
    n.getParam("BatteryState/energy/standing_consumption",standing_consumption);
    n.getParam("BatteryState/energy/charge_time",full_charge_time);

    battery_state.percent = 100;
    battery_state.remaining_distance = avg_speed * battery_charge/moving_consumption*3600;
    battery_state.remaining_time = battery_charge*3600 * (moving_perc/moving_consumption + standing_perc/standing_consumption);

    ros::Publisher battery_pub = n.advertise<battery_mgmt::Battery>("battery_state", 1);
    ros::Subscriber charge1_sub = n.subscribe("going_to_recharge", 1, charge_starting_callback);
    ros::Subscriber charge2_sub = n.subscribe("charging", 1, charging_callback);
    ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1, cmd_callback);
    ros::Subscriber speed_sub = n.subscribe("avg_speed", 1, speed_callback);

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

        // increase battery charge (don't count time)
        if(charging){
            actual_charge = (1 - remaining_charge_time/full_charge_time/3600) * battery_charge;
        }

        // decrease battery charge and compute standing and moving times
        else{
            // robot is standing
            if(x_linear == 0 && z_angular == 0){
                actual_charge -= standing_consumption / (rate * 3600 );
                standing_time += 1/rate;
            }
            // robot is moving
            else{
                actual_charge -= moving_consumption / (rate * 3600 );
                moving_time += 1/rate;
            }

            // calculate percentages of standing and moving times
            moving_perc = moving_time / (moving_time + standing_time);
            standing_perc = standing_time / (standing_time + moving_time);
            if(moving_perc < 0.5){
                moving_perc = 0.5;
                standing_perc = 0.5;
            }
        }

        // percentage of battery charge remaining
        battery_state.percent = (actual_charge * 100) / battery_charge;
        if(battery_state.percent < 0)
            battery_state.percent = 0;
        if(battery_state.percent > 100)
            battery_state.percent = 100;

        // remaining distance the robot can still travel in meters
        battery_state.remaining_distance = avg_speed * actual_charge/moving_consumption*3600;

        // remaining time the robot can still work in seconds
        battery_state.remaining_time = actual_charge*3600 * (moving_perc/moving_consumption + standing_perc/standing_consumption);

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

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
