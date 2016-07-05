#include <ros/ros.h>
//#include <ros/console.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "measure");
    ros::NodeHandle nh;

    // subscribe to voltage topic


    // Frequency of loop
    double rate = 0.5; // Hz
    ros::Rate loop_rate(rate);

    while(ros::ok()){
        // get updates from subscriptions
        ros::spinOnce();

        // sleep for 1/rate seconds
        loop_rate.sleep();

        ROS_INFO("test");
    }

    return 0;
}
