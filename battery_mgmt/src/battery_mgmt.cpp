#include <ros/ros.h>
#include <battery_mgmt.h>

#include <std_msgs/Empty.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "battery_mgmt/Battery.h"
#include "battery_mgmt/Charge.h"
#include "geometry_msgs/Twist.h"
#include "explorer/Speed.h"

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "battery_mgmt/Charge.h"
#include "battery_mgmt/Battery.h"

using namespace std;

battery_mgmt::battery_mgmt()
{
    // read parameters
    nh.getParam("battery_mgmt/avg_speed", speed_avg);
    nh.getParam("battery_mgmt/charge_time", time_charge_full);
    nh.getParam("battery_mgmt/moving_consumption", consumption_moving);
    nh.getParam("battery_mgmt/standing_consumption", consumption_standing);
    nh.getParam("battery_mgmt/battery_charge", charge_max);


    // initialize private variables
    charge = charge_max;
    speed_linear = 0;
    speed_angular = 0;
    time_moving = 0;
    time_standing = 0;
    perc_moving = 0.5
    perc_standing = 0.5;

    // initialize message
    state.charging = false;
    state.soc = 100;
    state.remaining_time_charge = 0;
    state.remaining_time_run = charge_max * 3600 * (perc_moving / consumption_moving + perc_standing / consumption_standing);
    state.remaining_distance = speed_avg * charge_max / consumption_moving * 3600;


    bool debugShown = false;

    // get robot platform
//     std::string platform;
//     ros::get_environment_variable(platform, "ROBOT_PLATFORM");
//     ROS_INFO("Robot platform: %s",platform.c_str());
//     if(platform.compare("turtlebot") == 0){
//         // todo: subscribe to diagnostics topic
//         //sub3 = nh.subscribe("diagnostics_agg",1000,&Explorer::turtlebot_callback,this);
//     }
//     else if(platform.compare("pioneer3dx") == 0 || platform.compare("pioneer3at") == 0){
//         // todo: read voltage and convert to percentage
//     }
//     else{
//         // default: we are simulating
//     }

    // advertise topics
    pub_battery = nh.advertise<battery_mgmt::battery>("battery_state", 1);

    // subscribe to topics
    sub_charge = nh.subscribe("going_to_recharge", 1, charge_callback);
    sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmd_vel_callback);
    sub_speed = nh.subscribe("avg_speed", 1, speed_callback);
}

void battery_mgmt::charge_callback(const std_msgs::Empty::ConstPtr &msg)
{
    charging = true;
    ROS_ERROR("starting to charge");
    ROS_INFO("Starting to recharge the robot");
    charge_time = full_charge_time * (100-battery)/100 * 60*60;
    home = true;
    ROS_ERROR("starting to charge");
}

void battery_mgmt::cmd_vel_callback(const geometry_msgs::Twist &msg)
{
    speed_linear = msg.linear.x;
    speed_angular = msg.angular.z;
}

void battery_mgmt::speed_callback(const explorer::Speed &msg){

    // If the average speed is very low, there is probably something wrong
    if(msg.avg_speed > 0.25){
        speed_avg = msg.avg_speed;
    }
    else{
        speed_avg = 0.25;
    }
}

/**
 * Get the charging state for the turtlebot
 * This information is read from the diagnostic array which is provided by the diagnostic aggregator
 */
// void battery_mgmt::turtlebot_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
// {
//     for( size_t status_i = 0; status_i < msg->status.size(); ++status_i )
//     {
//         if( msg->status[status_i].name.compare("/Power System/Battery") == 0)
//         {
//             for (size_t value_i = 0; value_i < msg->status[status_i].values.size(); ++value_i)
//             {
//                 if( msg->status[status_i].values[value_i].key.compare("Percent") == 0 )
//                 {
//                     battery_charge = ::atof(msg->status[status_i].values[value_i].value.c_str());
//                 }
//
//                 if( msg->status[status_i].values[value_i].key.compare("Charging State") == 0 )
//                 {
//                     if(msg->status[status_i].values[value_i].value.c_str() == "Not Charging")
//                     {
//
//                     }
//                 }
//             }
//         }
//     }
// }

// void charging_callback(const battery_mgmt::Charge::ConstPtr &msg) {
//     remaining_charge_time = msg->remaining_time;
//     if(charging && remaining_charge_time <= 0)
//     {
//         ROS_ERROR("stop charging");
//         ROS_INFO("Charging done");
//         battery_state.percent = 100;
//         battery_state.remaining_distance = avg_speed * battery_charge/moving_consumption*3600;
//         battery_state.remaining_time = battery_charge*3600 * (moving_perc/moving_consumption + standing_perc/standing_consumption);
//         actual_charge = battery_charge;
//         charging = false;
//     }
// }