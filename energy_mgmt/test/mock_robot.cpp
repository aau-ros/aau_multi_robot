#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "energy_mgmt");
    ros::Time::init();

    ROS_INFO("This is a mock node");
    
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }
    
    while(ros::ok()){
        ros::spinOnce();
    }
    ros::shutdown();
        
    return 0;
}
