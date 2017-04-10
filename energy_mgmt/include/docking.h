#ifndef DOCKING_H
#define DOCKING_H

#include <ros/ros.h>
#include <navfn/navfn_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <adhoc_communication/MmListOfPoints.h>
#include <adhoc_communication/ExpFrontier.h>
#include <adhoc_communication/EmAuction.h>
#include <adhoc_communication/EmDockingStation.h>
#include <adhoc_communication/EmRobot.h>
#include <adhoc_communication/SendEmAuction.h>
#include <adhoc_communication/SendEmDockingStation.h>
#include <adhoc_communication/SendEmRobot.h>
#include <adhoc_communication/MmListOfPoints.h>
//#include <adhoc_communication/SendMmPoint.h>
#include <map_merger/TransformPoint.h>
#include <energy_mgmt/battery_state.h>
#include <energy_mgmt/AuctionResult.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <explorer/RobotPosition.h>
#include <explorer/DistanceFromRobot.h>
#include <sstream>
#include <math.h>
#include <stdlib.h>

#define PI 3.14159265

#define SSTR(x) static_cast<std::ostringstream &>((std::ostringstream() << std::dec << x)).str()

#define AUCTION_TIMEOUT 5
#define FORCED_AUCTION_END_TIMEOUT (AUCTION_TIMEOUT + 2)
#define AUCTION_RESCHEDULING_TIME (AUCTION_TIMEOUT * 3)
#define FIDUCIAL_LASER_RANGE 10 //TODO meters???
#define DS_SELECTION_POLICY 0
#define MAX_DISTANCE 50
#define V 5
#define OPP_ONLY_TWO_DS false

using namespace std;

class docking
{
  public:
    /**
     * Constructor.
     */
    docking();

    /**
     * @brief Compute the optimal docking station for the robot
     *
     * Given a docking station selected policy, this function computes the dockign station that is currently the optimal one (according to the given policy) for recharging.
     */
    void compute_optimal_ds();

    void update_robot_state();

    void map_info();
    
    /**
     * @brief Check if there are docking stations close enough to the robot to be considered discovered 
     *
     * For each docking station D, the robot checks if the distance between it and D is less than a certain value: if it so, the docking station D can be considered discovered (i.e., it can be used to recharge from now on).
     */
    void discover_docking_stations();

  private:
    /**
     * The node handle.
     */
    ros::NodeHandle nh;

    /**
     * Service client for sending an auction.
     */
    ros::ServiceClient sc_send_auction, sc_send_docking_station, sc_send_robot;

    /**
     * Subscribers for the required topics.
     */
    ros::Subscriber sub_battery, sub_robots, sub_jobs, sub_docking_stations, sub_auction_starting, sub_auction_reply;

    /**
     * Callbacks for the subscribed topics.
     */
    void cb_battery(const energy_mgmt::battery_state::ConstPtr &msg);
    void cb_robots(const adhoc_communication::EmRobot::ConstPtr &msg);
    void cb_jobs(const adhoc_communication::ExpFrontier::ConstPtr &msg);
    void cb_docking_stations(const adhoc_communication::EmDockingStation::ConstPtr &msg);
    void cb_new_auction(const adhoc_communication::EmAuction::ConstPtr &msg);

    /**
     * Get the likelihood value as a linear combination of the values l1 to l4.
     * @return double: The likelihood value.
     */
    double get_llh();

    /**
     * Update the likelihood value l1.
     */
    void update_l1();

    /**
     * Update the likelihood value l2.
     */
    void update_l2();

    /**
     * Update the likelihood value l3.
     */
    void update_l3();

    /**
     * Update the likelihood value l4.
     */
    void update_l4();

    /**
     * Send an auction to a multicast group.
     * @param string multicast_group: The multicast group to send the auction to.
     * @param adhoc_communication::EmAuction auction: The auction that will be sent.
     * @param string topic: The topic name which the auction will be published in.
     * @return bool: Success of transmission.
     */
    bool auction_send_multicast(string multicast_group, adhoc_communication::EmAuction auction, string topic);

    /**
     * Compute the length of the trajectory from the robots current position to a given goal.
     * @param double goal_x: The x-coordinate of the goal (in meters).
     * @param double goal_y: The y-coordinate of the goal (in meters).
     * @param bool euclidean: Whether or not to use euclidean distance. If it is left to default (i.e. euclidean=false),
     * then the actual path is calculated using Dijkstra's algorithm.
     * @return double: The distance between the robots current position and the goal (in meters).
     */
    double distance_from_robot(double goal_x, double goal_y, bool euclidean = false);

    /**
     * Compute the length of the trajectory from a given start to a given goal.
     * @param double start_x: The x-coordinate of the start (in meters).
     * @param double start_y: The y-coordinate of the start (in meters).
     * @param double goal_x: The x-coordinate of the goal (in meters).
     * @param double goal_y: The y-coordinate of the goal (in meters).
     * @param bool euclidean: Whether or not to use euclidean distance. If it is left to default (i.e. euclidean=false),
     * then the acutal path is calculated using Dijkstra's algorithm.
     * @return double: The distance between the robots current position and the goal (in meters).
     */
    double distance(double start_x, double start_y, double goal_x, double goal_y, bool euclidean = false);

    /**
     * Navigation function and costmap for calculating paths.
     */
    // navfn::NavfnROS nav;
    costmap_2d::Costmap2DROS *costmap;

    /**
     * Distance until which jobs are still considered close by (in meters).
     */
    double distance_close;

    /**
     * The coordinate frame used for calculating path lengths.
     */
    string move_base_frame;

    /**
     * Name and ID of the robot.
     */
    string robot_name, robot_prefix;
    int robot_id;

    /**
     * ID of the last auction.
     */
    int auction_id;

    /**
     * A vector of all robots with their current state.
     */
    int num_robots;  // number of robots is known in simulations

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
        leaving_ds           // the robot was recharging, but another robot stopped
    };

    state_t robot_state;

    enum simple_state_t
    {
        active,
        idle
    };

    struct robot_t
    {
        int id;
        simple_state_t state;
        double x, y;
        int target_ds;
    };
    robot_t robot;
    vector<robot_t> robots;

    /**
     * A vector of all docking stations with coordinates and vacancy.
     */
    struct ds_t
    {
        int id;
        double x;
        double y;
        bool vacant;
    };
    vector<ds_t> ds;
    vector<ds_t> undiscovered_ds;

    /**
     * The battery state containing time needed to fully charge the battery and time left until battery depletion.
     */
    energy_mgmt::battery_state battery;

    /**
     * A vector of all currently available jobs (e.g. frontiers for exploration).
     */
    struct job_t
    {
        int id;
        double x;
        double y;
    };
    vector<job_t> jobs;

    /**
     * Likelihood values for going recharging. A linear combination of the values is used in the auctions.
     */
    double l1, l2, l3, l4;

    /**
     * The weights for the weighted sum of the likelihood values l1,...,l4.
     */
    double w1, w2, w3, w4;

    // F
    ros::Publisher pub_ds, pub_new_target_ds;
    bool test;
    ds_t *best_ds;
    ros::Subscriber sub_robot_position, sub_auction_winner_adhoc;
    ros::ServiceServer ss_send_docking_station;
    bool foo(adhoc_communication::SendEmDockingStation::Request &req,
             adhoc_communication::SendEmDockingStation::Response &res);
    ros::Publisher pub_adhoc_new_best_ds;
    ros::Subscriber sub_adhoc_new_best_ds, sub_all_points, sub_recharge;
    ros::ServiceClient sc_trasform;

    void timerCallback(const ros::TimerEvent &);

    ros::Timer timer_restart_auction, timer_finish_auction, timer2;

    struct auction_bid_t
    {
        int robot_id;
        float bid;
    };
    vector<auction_bid_t> auction_bids;

    ros::Subscriber sub_vacant_docking_station, sub_charging_completed, sub_translate, sub_vacant_ds, sub_occupied_ds,
        sub_check_vacancy;

    void cb_charging_completed(const std_msgs::Empty &msg);
    void cb_translate(const adhoc_communication::EmDockingStation::ConstPtr &msg);

    void timer_callback_schedure_auction_restarting(const ros::TimerEvent &event);

    void abs_to_rel(double absolute_x, double absolute_y, double *relative_x, double *relative_y);
    void rel_to_abs(double relative_x, double relative_y, double *absolute_x, double *absolute_y);

    double origin_absolute_x, origin_absolute_y;

    bool optimal_ds_computed_once;

    /**
     * @brief Preload_docking_stations from a file
     *
     * Since robots have no fiducial sensors to detect the presence of docking stations, we need to preload them from a
     *file that stores the positions of all the docking stations in the environment. In particular, the idea is to store
     *in this file the coordinates of each docking station with respect to an ideally global and fixed reference system;
     *this of course requires that we have to specify somewhere also the original positiol of the robot in this global
     *reference system (i.e., we have to specify how the local reference systems of the robots are collocated in this
     *global reference system); this is done within the launch files.
     *
     * To simulate a real scenario where the docking stations should be discovered during the exploration and not been
     *already known at the beginning, we store these docking station in a seperate vector from the one that stores the
     *already discovered docking station.
     *
     * More precisely, we load the position of each docking station and we store it in a vector A; during its
     *exploration, the robot checks (with a certain frequency) if there is a docking stations in the neighborhood, i.e.,
     *given that it knows the positions of all the dockgin stations and it can compute at each instant its position in
     *the global reference system, it can checks if the distance between it an a given docking station D is less than a
     *certain value, so that we can simulated the range of a possible fiducial sensor to detect the docking station. If
     *the distance is less than this "fake range", we move the docking station D from vector A of the currently unknown
     *docking station position to vector B that keeps track of the docking stations that were alraedy discovered up to
     *this instant.
     */
    void preload_docking_stations();

    void vacant_ds_callback(const std_msgs::Empty::ConstPtr &);
    void occupied_ds_callback(const std_msgs::Empty::ConstPtr &);

    struct auction_t
    {
        int robot_id;
        int auction_id;
    };

    vector<auction_t> auctions;

    void cb_auction_reply(const adhoc_communication::EmAuction::ConstPtr &);

    bool managing_auction;
    int participating_to_auction;

    void check_vacancy_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg);

    bool going_to_ds, going_to_check_if_ds_is_free, need_to_charge, charging_completed, going_charging_bool;

    std::vector<ros::Timer> timers;

    void end_auction_participation_timer_callback(const ros::TimerEvent &event);

    ros::Publisher pub_lost_own_auction, pub_won_auction, pub_lost_other_robot_auction, pub_auction_result,
        pub_moving_along_path;
    bool lost_own_auction, auction_winner, lost_other_robot_auction, update_state_required;
    void abort_charging_callback(const std_msgs::Empty &msg);

    ros::Subscriber sub_robot_pose, sub_robot;
    ros::ServiceClient sc_robot_pose, sc_distance_from_robot;

    ds_t *next_optimal_ds, *target_ds, *next_target_ds;

    void cb_robot(const adhoc_communication::EmRobot::ConstPtr &msg);

    double llh;

    string my_prefix, my_node;

    void initLogpath();

    std::string log_path;

    void start_new_auction();

    bool num_ds;

    void set_target_ds_vacant(bool vacant);

    void compute_MST(int graph[V][V]);

    int minKey(int key[], bool mstSet[]);

    int printMST(int parent[], int n, int graph[V][V]);

    /**
     * @brief Search for a path connecting the given nodes in an undirected weighted tree
     *
     * The function checks if it is possible to find a path from node 'start' to node 'end' in the (possibly) undirected
     *tree 'tree': if a path is found, the nodes that compose the path are stored in 'path', i.e., 'path' contains the
     *sequence of nodes that form the path going from 'start' to 'end' ('start' and 'end' are the first and last node,
     *respectively); if such path is found, the function returns \c true, otherwise it returns \c false.
     *
     * \p tree is a weighted adiacency matrix: \p tree[i][j] is different from 0 if (and only if) there is an edge
     *connecting node 'i' and node 'j'. Notice however that the weights are not considered by the function (it takes a
     *weighted tree just because in the class it is used to find a path on a minimum spanning tree).
     *
     * @param tree the undirected tree where to search for a path, represented with an adiacency matrix
     * @param start index of the starting node of the path
     * @param end index of the ending node of the path
     * @param path the sequence of nodes composing the path (if a path exists) //TODO list of indexes of DSs???
     * @return \c true if a path was found, \c false otherwise
     */
    bool find_path(std::vector<std::vector<int> > tree, int start, int target, std::vector<int> &path);

    /**
     * @brief Auxiliary function to find a path in an undirected weigthed tree
     *
     * This function is called by find_path() to search for a path in a tree. It uses a depth-first search approach to
     *find the path, with recursive calls: it checks if there is a node (possibly more than one) connected to 'start' in
     *'tree', and when such a node is found, it recursively calls itself on 'tree' passing as parameter 'start' the node
     *just found. The recursion successfully ends if an edge from 'start' to 'end' is found.
     *
     * Since the tree can be undirected, the function needs to specify when it recursively calls itself the node of the
     *tree that is the current start of the path, to avoid that the recursive call will "backtrack" by selecting this
     *node.
     *
     * \p tree is a weighted adiacency matrix: \p tree[i][j] is different from 0 if (and only if) there is an edge
     *connecting node 'i' and node 'j'. Notice however that the weights are not considered by the function (it takes a
     *weighted tree just because in the class it is used to find a path on a minimum spanning tree).
     *
     * @param tree the undirected tree where to search for a path, represented with ad adiacency matrix
     * @param start index of the starting node of the path
     * @param end index of the ending node of the path
     * @param path the path going from 'target' to 'start' (i.e., it is the reversed path w.r.t. the path that is
     *             actually requested)
     * @param prev_node index of the node on which the caller of this function was positioned when perfoming the
     *                  recursive call
     * @return \c true if a path was found, \c false otherwise
     */
    bool find_path_aux(std::vector<std::vector<int> > tree, int start, int target, std::vector<int> &path,
                       int prev_node);

    int mst[V][V];

    bool moving_along_path;

    int ds_selection_policy;

    bool compute_closest_ds();

    enum ds_state_t
    {
        vacant,
        occupied,
        assigned
    };

    void robot_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void adhoc_ds(const adhoc_communication::EmDockingStation::ConstPtr &msg);
    void points(const adhoc_communication::MmListOfPoints::ConstPtr &msg);
    void cb_recharge(const std_msgs::Empty &msg);
    void cb_auction_result(const adhoc_communication::EmAuction::ConstPtr &msg);

    std::string csv_file;
    std::fstream fs_csv;
    
    ros::Time time_start;
    
    bool started_own_auction;
    

};

#endif /* DOCKING_H */
