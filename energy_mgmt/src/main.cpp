#include <ros/ros.h>
#include <ros/console.h>
#include <battery.h>
#include <docking.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");

    // instantiate classes
    battery bat;
    docking doc;

    // Frequency of loop
    double rate = 0.5; // Hz
    ros::Rate loop_rate(rate);

    while(ros::ok()){
        // get updates from subscriptions
        ros::spinOnce();

        // compute new battery state
        bat.compute();

        // output battery state to console
        bat.output();

        // publish battery state
        bat.publish();

        // send broadcast message with positions of all (known) docking stations


        // sleep for 1/rate seconds
        loop_rate.sleep();
    }

    return 0;
}
