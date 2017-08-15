#ifndef CONCRETE_BID_COMPUTER_H
#define CONCRETE_BID_COMPUTER_H

#include <limits>
#include <mutex>
#include <adhoc_communication/ExpFrontierElement.h>
#include <adhoc_communication/EmDockingStation.h>
#include <adhoc_communication/ExpFrontier.h>
#include <adhoc_communication/EmRobot.h>
#include <explorer/battery_state.h>
#include <robot_state/robot_state_management.h>
#include "bid_computer.h"

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
    double getBid();
    void processMessages();

private:
    double w1, w2, w3, w4;
    double l1, l2, l3, l4;
    double llh;
    double num_robots;
    double origin_absolute_x, origin_absolute_y;
    bool recompute_llh;
    bool optimal_ds_is_set;
    std::vector<adhoc_communication::ExpFrontierElement> jobs, next_jobs;
    double optimal_ds_x, optimal_ds_y;
    double robot_x, robot_y;
    std::vector<ds_t> ds;
    std::vector<robot_t> robots;
    double conservative_maximum_distance_one_way;
    explorer::battery_state battery;
    double next_remaining_distance, maximum_travelling_distance;
    ros::Subscriber sub_battery, sub_robots, sub_jobs, sub_docking_stations;
    adhoc_communication::EmDockingStation::ConstPtr em_docking_station_msg;
    explorer::battery_state::ConstPtr next_battery;
    std::mutex robot_mutex, ds_mutex, message_mutex;

    void update_l1();
    unsigned int countVacantDss();
    unsigned int countActiveRobots();
    void update_l2();
    void update_l3();
    void update_l4();
    double distance_from_robot(double x, double y);
    double distance(double start_x, double start_y, double goal_x, double goal_y);
    void cb_battery(const explorer::battery_state::ConstPtr &msg);
    void cb_robots(const adhoc_communication::EmRobot::ConstPtr &msg);
    void cb_jobs(const adhoc_communication::ExpFrontier::ConstPtr &msg);
    void cb_docking_stations(const adhoc_communication::EmDockingStation::ConstPtr &msg);
    bool isActiveState(state_t state);
    void abs_to_rel(double absolute_x, double absolute_y, double *relative_x, double *relative_y);
};

#endif // CONCRETE_BID_COMPUTER_H
