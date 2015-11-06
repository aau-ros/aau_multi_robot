#ifndef BATTERY_MGMT_H
#define BATTERY_MGMT_H

//#include "battery_mgmt/Battery.h"
//#include "battery_mgmt/Charge.h"

class battery_mgmt
{
public:
    /**
     * Constructor
     */
    battery_mgmt();



private:
    /**
     * The node handle.
     */
    ros::NodeHandle nh;

    /**
     * Subscribers for the required topics.
     */
    ros::Subscriber sub_charge, sub_cmd_vel, sub_speed;

    /**
     * Publishers.
     */
    ros::Publisher pub_battery;


    battery_mgmt::battery state;

    double charge, charge_max;

    int consumption_moving, consumption_standing;

    double speed_linear, speed_angular, speed_avg;

    double time_moving, time_standing;

    double perc_moving, perc_standing;

    double time_charge_full;

    void charge_callback(const std_msgs::Empty::ConstPtr &msg);

    /**
     * In the cmd_vel message there are only two important parameters, the x-linear value and the z-angular value.
     * When they are zero the robots stands still. If the x-value is unequal to zero then the robot moves forward or
     * backward and if the z-value is unequal to zero the robot rotates.
     * When the robot stands still it consumes less energy and when it moves it consumes more energy.
     */
    void cmd_vel_callback(const geometry_msgs::Twist &msg);

};

#endif  /* BATTERY_MGMT_H */
