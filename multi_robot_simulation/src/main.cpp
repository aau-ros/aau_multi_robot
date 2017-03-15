#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");

    double rate = 0.5; // Hz
    ros::Rate loop_rate(rate);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
