#ifndef BATTERY_SIMULATE_H
#define BATTERY_SIMULATE_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>
//#include <energy_mgmt/battery_state.h>
#include <explorer/battery_state.h>
#include <explorer/Speed.h>
#include "std_msgs/Float32.h"
#include <adhoc_communication/EmRobot.h>
#include <sstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <stdlib.h>
#include <fstream>
#include "time_manager_interface.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/thread/mutex.hpp>

#include <robot_state/robot_state.h>
#include <robot_state/GetRobotState.h>
#include <robot_state/SetRobotState.h>

#include "robot_state_manager.h" //TODO this requires to link in CMakeLists.txt... better use a pointer? https://stackoverflow.com/questions/12466055/field-has-incomplete-type-error

#define SSTR(x) static_cast<std::ostringstream &>((std::ostringstream() << std::dec << x)).str()

class RobotStateManager;

class battery_simulate
{
public:
    /**
     * Constructor.
     */
    battery_simulate();

    /**
     * Compute the battery state.
     */
    void compute();

    void log();

    /**
     * Publishes a message containing the battery state.
     */
    void publish();

    /**
     * The battery state.
     * This includes:
     *  - bool charging
     *  - float32 soc
     *  - float32 remaining_time_charge
     *  - float32 remaining_time_run
     *  - float32 remaining_distance
     */
    explorer::battery_state state;
    
    void run();

    void createLogDirectory();
    void createLogFiles();
    
    /*************************
     ** Debugging functions **
     *************************/
    void setTimeManager(TimeManagerInterface *time_manager);
    double last_time_secs();
    void set_last_time();
    void initializeSimulationTime();
    double getElapsedTime();
    void spinOnce();
    bool initializing;
    double getConsumedEnergyA();
    double getConsumedEnergyB();
    double getMaximumTravelingDistance();
    double getTotalTraveledDistance();
    double _f1, _f2, _f3, _f4, _f5, _f6;
    double getRemainingDistance();

private:
    /**
     * The node handle.
     */
    ros::NodeHandle nh;

    /**
     * Subscribers for the required topics.
     */
    ros::Subscriber sub_charge, sub_cmd_vel, sub_speed, sub_soc, sub_time, sub_robot_state, pose_sub;

    /**
     * Publishers.
     */
    ros::Publisher pub_battery, pub_full_battery_info;

    /**
     * Battery charge in Wh.
     * Charge is the current battery charge, charge_max is the charge if the battery is fully charged.
     */
    double charge, charge_max;

    /**
     * The power consumption of the robot.
     */
    double power_sonar, power_laser, power_basic_computations, power_advanced_computations, power_microcontroller, power_moving_fixed_cost, power_per_speed, power_charging;
    
    double power_idle;

    /**
     * Speed of the robot.
     * The values are received from subscribed topics.
     */
    double speed_linear, speed_angular, speed_avg, speed_avg_init;

    /**
     * Total time the robot has been standing and moving.
     * This excludes the times while the robot is recharging.
     */
    double time_moving, time_standing;

    /**
     * Percentage of time the robot has been standing and moving.
     */
    double perc_moving, perc_standing;

    /**
     * Whether or not the soc output has been shown.
     * This is necessary so that each value is shown only once.
     */
    bool output_shown;

    /**
     * TODO
     * NEW, do we need this???
     */
    double total_time;

    /**
     * The last time that the computations where carried out.
     */
    ros::Time time_last;

    /**
     * Callback that is triggered when the robot should start recharging.
     */
    void cb_charge(const std_msgs::Empty::ConstPtr &msg);

    /**
     * Callback that is used to determine if robot is standing or moving.
     * In the cmd_vel message there are only two important parameters, the x-linear value and the z-angular value.
     * When they are zero the robots stands still. If the x-value is unequal to zero then the robot moves forward or
     * backward and if the z-value is unequal to zero the robot rotates.
     * When the robot stands still it consumes less energy and when it moves it consumes more energy.
     */
    void cb_cmd_vel(const geometry_msgs::Twist &msg);

    /**
     * Callback that is used to get the average speed of the robot.
     */
    void cb_speed(const explorer::Speed &msg);

    /**
     * Callback that is used to get the robot's state of charge.
     */
    void cb_soc(const std_msgs::Float32::ConstPtr& msg);

    /**
     * TODO: do we need this?
     * Callback to get maximum time.
     */
    void totalTime(const std_msgs::Float32::ConstPtr& msg);
    
    // Maximum speed of the robot
    double max_speed_linear;
    
    double maximum_traveling_distance, traveled_distance, consumed_energy_A, consumed_energy_B;
    
    ros::Publisher pub_charging_completed;
    ros::Subscriber sub_robot;
    
    
   enum state_t
    {
        exploring,  // the robot is computing which is the next frontier to be
                    // explored

        going_charging,  // the robot has the right to occupy a DS to recharge

        charging,  // the robot is charging at a DS

        finished,  // the robot has finished the exploration

        fully_charged,  // the robot has recently finished a charging process; notice
                        // that the robot is in this state even if it is not really
                        // fully charged (since just after a couple of seconds after
                        // the end of the recharging process the robot has already
                        // lost some battery energy, since it consumes power even
                        // when it stays still

        stuck,

        in_queue,  // the robot is in a queue, waiting for a DS to be vacant

        auctioning,  // auctioning: the robot has started an auction; notice that if
                     // the robot is aprticipating to an auction that it was not
                     // started by it, its state is not equal to auctioning!!!
                     
        auctioning_2,

        going_in_queue,  // the robot is moving near a DS to later put itself in
                         // in_queue state

        going_checking_vacancy,  // the robot is moving near a DS to check if it
                                 // vacant, so that it can occupy it and start
                                 // recharging

        checking_vacancy,  // the robot is currently checking if the DS is vacant,
                           // i.e., it is waiting information from the other robots
                           // about the state of the DS

        moving_to_frontier_before_going_charging,  // TODO hmm...

        moving_to_frontier,  // the robot has selected the next frontier to be
                             // reached, and it is moving toward it
        leaving_ds,          // the robot was recharging, but another robot stopped
        dead,
        moving_away_from_ds,
        auctioning_3,
        stopped,
        exploring_for_graph_navigation
    };
    
    void cb_robot(const adhoc_communication::EmRobot::ConstPtr &msg);
    
    float mass;
    bool advanced_computations_bool;
    
    std::string log_path;
    std::string info_file, battery_state_filename;
    std::fstream fs_info, battery_state_fs;
    std::string robot_name;
    std::string robot_prefix;
    int robot_id;
    bool idle_mode;
    ros::Time sim_time_start;
    ros::WallTime wall_time_start;
    bool do_not_consume_battery;
    TimeManagerInterface *time_manager;
    double elapsed_time;
    unsigned int counter_moving_to_frontier;
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);
    double pose_x, pose_y, last_x, last_y;
    boost::mutex mutex_traveled_distance;
    double last_traveled_distance, total_traveled_distance;
    double prev_consumed_energy_A;
    
    ros::ServiceClient set_robot_state_sc;
    ros::ServiceClient get_robot_state_sc;
    
    void rechargeBattery(double time_diff_sec);
    bool isRobotMoving();
    
    RobotStateManager robot_state_manager;
    
};


#endif //BATTERY_SIMULATE_H
