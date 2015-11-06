#include <ros/ros.h>
#include <ros/console.h>
#include <docking_coordination.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");
    docking_coordination dc;
    //ros::Rate loop_rate(0.5); // Hz

    ros::spin();
    //while(ros::ok()){
    //    //dc.update_llh();
    //    ros::spinOnce();
    //    loop_rate.sleep();
    //}

    return 0;
}