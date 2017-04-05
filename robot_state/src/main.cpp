#include <ros/ros.h>
#include <ros/console.h>
#include <robot_state/GetRobotState.h>

bool get_robot_state_callback(robot_state::GetRobotState::Request &req, robot_state::GetRobotState::Response &res);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_state");
    ros::NodeHandle nh;
    
    // Frequency of loop
    double rate = 0.5; // Hz
    ros::Rate loop_rate(rate);

    ros::ServiceServer ss_get_robot_state = nh.advertiseService("robot_state/get_robot_state", get_robot_state_callback);

    while(ros::ok()){
    
        // get updates from subscriptions
        ros::spinOnce();

        // sleep for 1/rate seconds
        loop_rate.sleep();
    }

    return 0;
}

bool get_robot_state_callback(robot_state::GetRobotState::Request &req, robot_state::GetRobotState::Response &res) {
    ROS_ERROR("YYYYYYYYYYYYYYYYYYYEES");
    return false;
}
