#ifndef COMPUTER_H
#define COMPUTER_H

#include <ros/ros.h>
#include <explorer/battery_state.h>
#include <explorer/Speed.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/mutex.hpp>
#include <robot_state/robot_state_management.h>

class BatteryStateUpdater : public RobotStateHandler
{
public:
    BatteryStateUpdater(explorer::battery_state *b);
    void execute(InitializingState *r) override;
    void initializeBatteryState();
    void updateBatteryState();


private:
    // Parameters
    double speed_avg_init; //TODO speed_avg maybe is not a good name
    double power_charging;              // W (i.e, watt)
    double power_per_speed;             // W/(m/s)
    double power_moving_fixed_cost;     // W/(m/s)
    double power_sonar;                 // W
    double power_laser;                 // W
    double power_microcontroller;       // W
    double power_basic_computations;    // W
    double power_advanced_computations; // W
    double max_speed_linear;            // m/s
    double maximum_traveling_distance;  // m/s

    double last_pose_x, last_pose_y;
    double traveled_distance;
    double last_traveled_distance;
    double total_traveled_distance;
    double speed_avg;
    double speed_linear;
    double speed_angular;
    boost::mutex mutex_traveled_distance;

    explorer::battery_state *b;

    ros::Subscriber avg_speed_sub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber pose_sub;

    void loadParameters();
    void initializeVariables();
    void subscribeToTopics();

    void substractEnergyRequiredForSensing() {};
    void substractEnergyRequiredForBasicComputations() {};
    void substractEnergyRequiredForAdvancedComputations() {};
    void substractEnergyRequiredForKeepingRobotAlive() {};
    void rechargeBattery();

    void avgSpeedCallback(const explorer::Speed &msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);
    void cmdVelCallback(const geometry_msgs::Twist &msg);
};

#endif // COMPUTER_H
