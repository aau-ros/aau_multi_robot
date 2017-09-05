#ifndef CONCRETE_BID_COMPUTER_H
#define CONCRETE_BID_COMPUTER_H

#include <limits>
#include <mutex>
#include <fstream>
#include <boost/filesystem.hpp>
#include <adhoc_communication/ExpFrontierElement.h>
#include <adhoc_communication/EmDockingStation.h>
#include <adhoc_communication/ExpFrontier.h>
#include <adhoc_communication/EmRobot.h>
#include <explorer/battery_state.h>
#include <robot_state/robot_state_management.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "bid_computer.h"
#include <utilities/data_logger.h>
#include <std_srvs/SetBool.h>

struct ds_t
{
    int id;
    double x, y; // coordinates of the DS in the /map frame
    double world_x, world_y; // coordinates of the DS in the /world frame (i.e., in case of a simulation, in the reference system of the simulator)
    bool vacant;
    double timestamp;
};

enum simple_state_t
{
    active,
    idle
};

struct robot_t
{
    int id;
    bool active;
    double x, y, home_world_x, home_world_y;
    int selected_ds;
};

class ConcreteBidComputer : public BidComputer {
public:
    ConcreteBidComputer();
    double getBid() override;
    void updateLlh() override;
    void processMessages() override;
    void logMetadata() override;

private:
    double w1, w2, w3, w4, w5;
    double l1, l2, l3, l4, l5;
    double llh;
    double origin_absolute_x, origin_absolute_y;
    bool optimal_ds_is_set, next_optimal_ds_set;
    std::vector<adhoc_communication::ExpFrontierElement> jobs, next_jobs;
    double optimal_ds_x, optimal_ds_y;
    double next_optimal_ds_x, next_optimal_ds_y; 
    double robot_x, robot_y, next_robot_x, next_robot_y;
    std::vector<ds_t> ds;
    std::vector<robot_t> robots;
    ros::Subscriber sub_battery, sub_robots, sub_jobs, sub_docking_stations, sub_new_optimal_ds, pose_sub;
    adhoc_communication::EmDockingStation::ConstPtr em_docking_station_msg;
    std::mutex robot_mutex, ds_mutex, message_mutex;
    explorer::battery_state::ConstPtr battery, next_battery;
    std::string log_path, robot_prefix;
    DataLogger *data_logger, *data_logger2;
    ros::ServiceServer set_l5_ss;

    void loadParameters();
    void initializeVariables();
    void subscribeToTopics();
    void update_l1();
    unsigned int countVacantDss();
    unsigned int countActiveRobots();
    void update_l2();
    void update_l3();
    void countJobsAndCloseJobs(unsigned int &num_jobs, unsigned int &num_jobs_close);
    void update_l4();
    double distanceRobotOptimalDs();
    double distanceOptimalDsClosestFrontier();
    double distance_from_robot(double x, double y);
    double distance(double start_x, double start_y, double goal_x, double goal_y);
    void cb_battery(const explorer::battery_state::ConstPtr &msg);
    void cb_robots(const adhoc_communication::EmRobot::ConstPtr &msg);
    void cb_jobs(const adhoc_communication::ExpFrontier::ConstPtr &msg);
    void cb_docking_stations(const adhoc_communication::EmDockingStation::ConstPtr &msg);
    void newOptimalDsCallback(const adhoc_communication::EmDockingStation::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);
    bool isActiveState(robot_state::robot_state_t state);
    void abs_to_rel(double absolute_x, double absolute_y, double *relative_x, double *relative_y);
    bool set_l5_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
};

#endif // CONCRETE_BID_COMPUTER_H
