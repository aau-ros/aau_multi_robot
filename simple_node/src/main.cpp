#include <ros/ros.h>
#include <ros/console.h>
#include <robot_state/robot_state_management.h>
#include <robot_state/GetRobotState.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_node");

    // Change debugging level to "debug" (i.e., print all ROS_* messages)
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged(); //TODO allow user to set log level for testing not to display errors (if possible...)
    }

    ros::Time::init();
    ros::Duration(5).sleep();

    ros::NodeHandle nh;
    ros::ServiceClient get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");

    robot_state::GetRobotState get_srv_msg;
    ROS_ERROR("%d", get_robot_state_sc.call(get_srv_msg));

    ROS_INFO("here!!!!!");    

    RobotStateApi r;

//    ROS_INFO("%d", r.getRobotState2());

    ROS_INFO("Entering main loop");
    double rate = 10; // Hz
    ros::Time::init(); // necessary to use ros::Rate, or we get an error during testing
    ros::Rate loop_rate(rate);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep(); // sleep for 1/rate seconds
    }
    
    ROS_INFO("Shutting down node");
    ros::waitForShutdown();
        
    return 0;
}
