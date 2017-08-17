#ifndef COMPUTER_H
#define COMPUTER_H

#include <ros/ros.h>
#include <explorer/battery_state.h>
#include <explorer/Speed.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <robot_state/robot_state_management.h>
#include <utilities/time_manager.h>
#include "robot_state_manager_interface.h"

//class BatteryStateUpdater : public RobotStateHandler
class BatteryStateUpdater
{
public:
    BatteryStateUpdater(explorer::battery_state *b);
    void setTimeManager(TimeManagerInterface *time_manager);
    void setRobotStateManager(RobotStateManagerInterface *robot_state_manager);
    void logMetadata();
    void createLogDirectory();
    void updateBatteryState();
//    void handle(InitializingState *state) override;
//    void handle(ChoosingActionState *state) override;
//    void handle(ComputingNextGoalState *state) override;

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
    double last_traveled_distance;
    double total_traveled_distance;
    double speed_avg;
    double speed_linear;
    double speed_angular;
    double elapsed_time;
    double time_last_update;
    double prev_consumed_energy_A;
    boost::mutex mutex_traveled_distance;

    std::string log_path;
    std::string info_file, battery_state_filename;
    std::fstream fs_info, battery_state_fs;
    std::string robot_name;
    std::string robot_prefix;

    explorer::battery_state *b;
    RobotStateManagerInterface *robot_state_manager;
    TimeManagerInterface *time_manager;

    ros::Subscriber avg_speed_sub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber pose_sub;

    void loadParameters();
    void initializeBatteryState();
    void initializeVariables();
    void subscribeToTopics();

    void computeElapsedTime();
    void updateRemainingUsableDistanceAndRunningTime();
    void substractEnergyRequiredForSensing();
    void substractEnergyRequiredForBasicComputations();
    void substractEnergyRequiredForAdvancedComputations();
    void substractEnergyRequiredForKeepingRobotAlive();
    void substractEnergyRequiredForLocomotion();
    void subtractTraveledDistance();
    void rechargeBattery();

    void avgSpeedCallback(const explorer::Speed &msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);
    void cmdVelCallback(const geometry_msgs::Twist &msg);
};

#endif // COMPUTER_H
