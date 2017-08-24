#include <ExplorationPlanner.h>
#include "ros/ros.h"
#include "hungarian.h"
#include "munkres.h"
#include "boost_matrix.h"
#include <ros/console.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <navfn/navfn_ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <string.h>
#include <stdlib.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/GridCells.h>
#include <adhoc_communication/ExpCluster.h>
#include <adhoc_communication/ExpClusterElement.h>
#include <adhoc_communication/ExpAuction.h>
#include <adhoc_communication/ExpFrontier.h>
#include <adhoc_communication/ExpFrontierElement.h>
#include <adhoc_communication/SendExpFrontier.h>
#include <adhoc_communication/SendExpFrontier2.h>
#include <adhoc_communication/SendExpAuction.h>
#include <adhoc_communication/SendExpCluster.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
//#include <explorer/Frontier.h> //<simple_navigation/Frontier.h>>
//#include <map_merger/pointFromOtherRobot.h>
#include <adhoc_communication/MmListOfPoints.h>
#include <map_merger/TransformPoint.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <math.h>
#include "explorer/Distance.h"

#define MAX_DISTANCE 2000           // max distance to starting point
#define MAX_GOAL_RANGE 5.0 //0.7          // min distance between frontiers during search [meters]
#define MINIMAL_FRONTIER_RANGE 0.7  // min distance between frontiers during selection [meters]
#define INNER_DISTANCE 10 //8            // radius around backoff goal point without obstacles [cells]
#define MAX_NEIGHBOR_DIST 1         // max distance between frontiers within a cluster
#define CLUSTER_MERGING_DIST 0.8    // merge clusters that are closer toghether than this distance
#define CLOSE_FRONTIER_RANGE 11     // distance within which frontiers are selected clock wise (for left most frontier strategy) [meters]

#define APPROACH 0
//#define QUICK_SELECTION
#define ALL_LOG_LEVEL false
#define LOG_MUTEX false

#include <typeinfo>
#define SHOW(a) std::cout << #a << ": " << (a) << std::endl
#define VALIDITY_INTERVAL 600 //s

int limit_search = 10000;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wformat"

using namespace explorationPlanner;

template <typename T>
    std::string NumberToString ( T Number )
    {
        std::ostringstream ss;
        ss << Number;
        return ss.str();
    }

ExplorationPlanner::ExplorationPlanner(int robot_id, bool robot_prefix_empty, std::string robot_name_parameter) : costmap_ros_(0), occupancy_grid_array_(0), exploration_trans_array_(0), obstacle_trans_array_(0), frontier_map_array_(0), is_goal_array_(0), map_width_(0), map_height_(0), num_map_cells_(0), initialized_(false), last_mode_(FRONTIER_EXPLORE), p_alpha_(0), p_dist_for_goal_reached_(1), p_goal_angle_penalty_(0), p_min_frontier_size_(0), p_min_obstacle_dist_(0), p_plan_in_unknown_(true), p_same_frontier_dist_(0), p_use_inflated_obs_(false), previous_goal_(0), inflated(0), lethal(0), free(0), threshold_free(127), threshold_inflated(252), threshold_lethal(253),frontier_id_count(0), exploration_travel_path_global(0), exploration_travel_path_global_meters(0), cluster_id(0), initialized_planner(false), auction_is_running(false), auction_start(false), auction_finished(true), start_thr_auction(false), auction_id_number(1), next_auction_position_x(0), next_auction_position_y(0), other_robots_position_x(0), other_robots_position_y(0), number_of_completed_auctions(0), number_of_uncompleted_auctions(0), first_run(true), first_negotiation_run(true), robot_prefix_empty_param(false){

    _f2 = 100.0;
    my_selected_frontier = new frontier_t;
    winner_of_auction = true;
    my_error_counter = 0;
    optimal_ds_x = 0;
    optimal_ds_y = 0;
    num_ds = -1;
//    recompute_ds_graph = false;
    optimal_ds_set = false;
    retrying_searching_frontiers = 0;
    received_scan = false;
    errors = 0;
    optimal_ds_set = false;
    robot_x = 0, robot_y = 0;
    test_mode = false;
//    update_distances_index = 0;
    erased = false;
    next_optimal_ds_id = -1;
    _f1 = -1.0;
    use_theta = false;

    trajectory_strategy = "euclidean";
    robot_prefix_empty_param = robot_prefix_empty;

    robot_last_x = robotPose.getOrigin().getX();
    robot_last_y = robotPose.getOrigin().getY();

    responded_t init_responded;
    init_responded.auction_number = 0;
    init_responded.robot_number = 0;
    robots_already_responded.push_back(init_responded);

    auction_running = false;
    std::stringstream robot_number;
    robot_number << robot_id;

    std::string prefix = "/robot_";
    std::string robo_name = prefix.append(robot_number.str());

    if(robot_prefix_empty_param == true)
    {
        // NO SIMULATION*
        robo_name = "";
        robot_str = robot_name_parameter;
    }

    
    std::string sendFrontier_msgs = robo_name +"/adhoc_communication/send_frontier";
    std::string sendAuction_msgs  = robo_name +"/adhoc_communication/send_auction";

    ros::NodeHandle tmp;
    nh_service = &tmp;

    ROS_DEBUG("Sending frontier: '%s'     SendAuction: '%s'", sendFrontier_msgs.c_str(), sendAuction_msgs.c_str());

    ssendFrontier = nh_service->serviceClient<adhoc_communication::SendExpFrontier>(sendFrontier_msgs);
    ssendFrontier_2 = nh_service->serviceClient<adhoc_communication::SendExpFrontier2>(sendFrontier_msgs);
    ssendAuction = nh_service->serviceClient<adhoc_communication::SendExpAuction>(sendAuction_msgs);


//    pub_negotion_first = nh_negotiation_first.advertise <adhoc_communication::Frontier> ("negotiation_list_first", 10000);

//    pub_frontiers = nh_frontier.advertise <adhoc_communication::Frontier> ("frontiers", 10000);
//    pub_visited_frontiers = nh_visited_frontier.advertise <adhoc_communication::Frontier> ("visited_frontiers", 10000);
    pub_frontiers = nh_frontier.advertise <adhoc_communication::ExpFrontier> ("frontiers", 10000);

    ros::NodeHandle nh2;
    ds_with_EOs_pub = nh2.advertise<adhoc_communication::EmDockingStation>("ds_with_EOs", 10000);

    pub_visited_frontiers = nh_visited_frontier.advertise <adhoc_communication::ExpFrontier> ("visited_frontiers", 10000);

    pub_visited_frontiers_points = nh_visited_Point.advertise <visualization_msgs::MarkerArray> ("visitedfrontierPoints", 2000, true);

    pub_Point = nh_Point.advertise < geometry_msgs::PointStamped> ("goalPoint", 100, true);
    pub_frontiers_points = nh_frontiers_points.advertise <visualization_msgs::MarkerArray> ("frontierPoints", 2000, true);

//    pub_auctioning_status = nh_auction_status.advertise<adhoc_communication::AuctionStatus> ("auctionStatus", 1000);
//    pub_auctioning_first = nh_auction_first.advertise<adhoc_communication::Auction> ("auction_first", 1000);

//    std::string frontier_sub = robo_name+"/frontiers";
//    std::string visited_frontiers_sub = robo_name+"/visited_frontiers";
//    std::string auction_sub = robo_name+"/auction";
//    std::string all_positions_sub = robo_name+"/all_positions";
//    ROS_INFO("Subscribing to: ");
//    ROS_INFO("%s  %s  %s  %s", frontier_sub.c_str(), visited_frontiers_sub.c_str(), auction_sub.c_str(), all_positions_sub.c_str());


//    sub_control = nh_control.subscribe("/control", 10000, &ExplorationPlanner::controlCallback, this);
    sub_frontiers = nh_frontier.subscribe(robo_name+"/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
    sub_visited_frontiers = nh_visited_frontier.subscribe(robo_name+"/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
    sub_negotioation = nh_negotiation.subscribe(robo_name+"/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);
    sub_auctioning = nh_auction.subscribe(robo_name+"/auction", 1000, &ExplorationPlanner::auctionCallback, this);
    sub_position = nh_position.subscribe(robo_name+"/all_positions", 1000, &ExplorationPlanner::positionCallback, this);
    
    sub_negotiation_2 = nh_negotiation.subscribe(robo_name+"/send_frontier_for_coordinated_exploration", 10000, &ExplorationPlanner::my_negotiationCallback, this);
    sub_next_goal = nh_negotiation.subscribe(robo_name+"/send_next_robot_goal", 1000, &ExplorationPlanner::robot_next_goal_callback, this);
    sub_reply_negotiation = nh_negotiation.subscribe(robo_name+"/reply_to_frontier", 10000, &ExplorationPlanner::my_replyToNegotiationCallback, this);
    
    sub_new_optimal_ds = nh.subscribe(robo_name+"/explorer/new_optimal_ds", 10000, &ExplorationPlanner::new_optimal_ds_callback, this);
    
    sub_new_ds_on_graph = nh.subscribe("new_ds_on_graph", 10000, &ExplorationPlanner::new_ds_on_graph_callback, this);
    sub_ds_count = nh.subscribe("ds_count", 10, &ExplorationPlanner::ds_count_callback, this);
    
    publish_goal_ds_for_path_navigation = nh.advertise<adhoc_communication::EmDockingStation>("goal_ds_for_path_navigation", 10);
    
    received_robot_info = false;
    ros::NodeHandle nh;
    //sc_distance_frontier_robot = nh.serviceClient<explorer::Distance>("energy_mgmt/distance_on_graph", true);
    sub_this_robot = nh.subscribe("this_robot", 10, &ExplorationPlanner::this_robot_callback, this);

    srand((unsigned)time(0));
    
    time_start = ros::Time::now();

    ros::NodeHandle nh_tilde("~");
    if(!nh_tilde.getParam("num_robots", num_robots))
        ROS_FATAL("param not found");
    
}

void ExplorationPlanner::this_robot_callback(const adhoc_communication::EmRobot::ConstPtr &msg) {
    if(!received_robot_info) {
        robot_home_world_x = msg.get()->home_world_x;
        robot_home_world_y = msg.get()->home_world_y;
    }
    received_robot_info = true;
}

void ExplorationPlanner::setRobotConfig(int name, double robot_home_position_x, double robot_home_position_y, std::string frame)
{
    robot_name = name;
    robot_home_x = robot_home_position_x;
    robot_home_y = robot_home_position_y;
    move_base_frame = frame;
}

void ExplorationPlanner::home_position_(const geometry_msgs::PointStamped::ConstPtr& msg) {
	home_position_x_ = msg.get()->point.x;
	home_position_y_ = msg.get()->point.y;
}

void ExplorationPlanner::positionCallback(const adhoc_communication::MmListOfPoints::ConstPtr& msg)
{
    //ROS_DEBUG("Position Callback !!!!!");
    position_mutex.lock();

    other_robots_positions.positions.clear();
    for(unsigned int i = 0; i < msg.get()->positions.size(); i++)
    {
        other_robots_positions.positions.push_back(msg.get()->positions.at(i));
    }
    //ROS_INFO("positions size: %u", msg.get()->positions.size());
    position_mutex.unlock();
}

bool ExplorationPlanner::my_quick_check_efficiency_of_goal(double available_distance, frontier_t * frontier)
{
    double total_distance_eu;
    double x = frontier->x_coordinate;
    double y = frontier->y_coordinate;
    
    //check euclidean distances
    if(optimal_ds_set)
        total_distance_eu = euclidean_distance(x, y, robot_x, robot_y) + euclidean_distance(x, y, optimal_ds_x, optimal_ds_y);
    else
        total_distance_eu = euclidean_distance(x, y, robot_x, robot_y) + euclidean_distance(x, y, robot_home_x, robot_home_y);
    //ROS_INFO("Euclidean distance to frontier and then to optimal DS: %.2f",total_distance);
    return total_distance_eu < available_distance;
}

bool ExplorationPlanner::my_check_efficiency_of_goal(double available_distance, frontier_t * frontier)
{
    double distance, distance_eu;
    double total_distance, total_distance_eu;
    double x, y;
//    if(frontier->smart_goal_set) {
//        x = frontier->smart_x_coordinate;
//        y = frontier->smart_y_coordinate;
//    } else {
        x = frontier->x_coordinate;
        y = frontier->y_coordinate;
//    }
    
    //check euclidean distances
    total_distance_eu = euclidean_distance(x, y, robot_x, robot_y);
    if(optimal_ds_set)
        total_distance_eu += euclidean_distance(x, y, optimal_ds_x, optimal_ds_y);
    else
        total_distance_eu += euclidean_distance(x, y, robot_home_x, robot_home_y);
    if(total_distance_eu > available_distance)
        return false;
    
    // distance to robot
    total_distance = trajectory_plan_meters(x, y, robot_x, robot_y);
    if(total_distance < 0) {
        // if the distance between robot and target is less than a certain small value, consider the target reachable... this is necessary because sometimes goals too close to the robot are considered unreachable, which is a problem when the robot is starting the exploration, since it very often (almost every time) selects as first goal its starting position
        if(euclidean_distance(x, y, robot_x, robot_y) < 2) 
            distance = euclidean_distance(x, y, robot_x, robot_y); 
        else {
            ROS_WARN("Failed to compute distance! frontier at (%.1f, %.1f)", x, y);
//            total_distance = fallback_distance_computation(x, y);
            if(errors == 0)
                my_error_counter++;
            errors++;
            return false;
        }
    }
    frontier->my_distance_to_robot = total_distance;
    
    // distance from frontier to optimal ds
    double target_x, target_y;
    if(optimal_ds_set) {
        target_x = optimal_ds_x;
        target_y = optimal_ds_y;
    } else {
        target_x = robot_home_x;
        target_y = robot_home_y;
    }
    distance = trajectory_plan_meters(target_x, target_y, x, y);
    if(distance < 0) {
        if(euclidean_distance(target_x, target_y, x, y) < 2)
            distance = euclidean_distance(target_x, target_y, x, y);
        else {
            ROS_WARN("Failed to compute distance! (%.2f, %.2f), %d", target_x, target_y, optimal_ds_set);
//            if(optimal_ds_set)
//                distance = fallback_distance_computation(x, y, robot_home_x, robot_home_y);
//            else
//                distance = fallback_distance_computation(x, y, optimal_ds_x, optimal_ds_y);
            if(errors == 0)
                my_error_counter++;
            errors++;
            return false;
        }
    }
    frontier->my_distance_to_optimal_ds = distance;
    
    total_distance += distance;

    ROS_INFO("Distance to frontier and then DS: %.2f",total_distance);
    return available_distance > total_distance;

}

bool ExplorationPlanner::existFrontiers() {
    return frontiers.size() > 0 ? true : false;
}

bool ExplorationPlanner::existReachableFrontiersWithDsGraphNavigation(double max_available_distance, bool *error) {
    ROS_INFO("existReachableFrontiersWithDsGraphNavigation");
    ROS_DEBUG("frontiers.size(): %u", frontiers.size());  
    bool found_reachable_frontier = false;
    
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);

    
    
//    bool exit = false;
//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
//    for(unsigned int i=0; i < frontiers.size() && !exit; i++)
//        for(unsigned int j=0; j < ds_list.size() && !exit; j++) {
//            double total_distance;
//            double x_f = frontiers[i].x_coordinate;
//            double y_f = frontiers[i].y_coordinate;
//            double x_ds = ds_list[j].x;
//            double y_ds = ds_list[j].y;
//            
//            //check euclidean distances
//            total_distance = euclidean_distance(x_ds, y_ds, x_f, y_f);
//            if(total_distance * 2 > max_available_distance) //todo reduce the value from the one used by existFrontiersReachableWithFullBattery
//                continue;
//            
//            // distance DS-frontier
//            total_distance = simplifiedDistanceFromDs(j, i);
//            if(total_distance < 0){
//                ROS_WARN("Failed to compute distance! (%.2f, %.2f), (%.2f, %.2f)", x_f, y_f, x_ds, y_ds);
////                total_distance = fallback_distance_computation(x_f, y_f, x_ds, y_ds) * 2;
////                if(errors == 0)
////                    my_error_counter++;
////                errors++;
//                *error = true;
//            }
//            if(total_distance * 2 < max_available_distance) {
//                found_reachable_frontier = true;
//                exit = true;
//            }
//        }
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    return found_reachable_frontier;
    


//    double min_dist = std::numeric_limits<int>::max();
//    ds_t *min_ds = NULL;
//    int retry = 0;
//    // search for the closest DS with EOs
////    while (min_ds == NULL && retry < 5) {
//    while (min_ds == NULL) {
//        for (unsigned int i = 0; i < ds_list.size(); i++)
//        {
//            for (unsigned int j = 0; j < frontiers.size(); j++)
//            {
//                // check if the DS has EOs
////                double dist = distance(ds_list.at(i).x, ds_list.at(i).y, frontiers.at(j).x_coordinate, frontiers.at(j).y_coordinate);
//                double dist = simplifiedDistanceFromDs(i, j);
//                if (dist < 0) {
//                    ROS_WARN("Distance computation failed");
//                    continue;
//                }
//                if (dist * 2 < max_available_distance)
//                {
//                    // the DS has EOS
//                    found_reachable_frontier = true;
//                    
//                    // check if this DS is the closest DS with EOs
//                    double dist2 = distance_from_robot(ds_list.at(i).x, ds_list.at(i).y);
//                    if (dist2 < 0) {
//                        ROS_WARN("Distance computation failed");
//                        continue;
//                    }
//                    if (dist2 < min_dist)
//                    {
//                        min_dist = dist2;
//                        min_ds = &ds_list.at(i);
//                    }
//                    
//                    break;
//                }
//            }
//        }
////        retry++;
////        if(min_ds == NULL)
////            ros::Duration(3).sleep();
//    }
//    min_ds_for_path_traversal = min_ds;
    
    
    
    double min_dist = std::numeric_limits<int>::max();
    ds_t *min_ds = NULL;
    int retry = 0;
    // search for the closest DS with EOs
//    while (min_ds == NULL && retry < 5) {
//    while (min_ds == NULL) 
    {
    
        mutex_ds.lock();
        unsigned int ds_size = ds_list.size();
        mutex_ds.unlock();
        
        unsigned int index = rand() % ds_size; // the index used to access ds_list; we do in this way to (more or less) randomly select a DS with EOs
    
        for (unsigned int count = 0; count < ds_size && min_ds == NULL; count++)   // to loop through all ds_list
        {
            for (unsigned int j = 0; j < frontiers.size(); j++)
            {
                // check if the DS has EOs
//                double dist = distance(ds_list.at(i).x, ds_list.at(i).y, frontiers.at(j).x_coordinate, frontiers.at(j).y_coordinate);

                if(index < 0 || index >= ds_size)
                    ROS_FATAL("orrible values");
                    
//                if(2 * euclidean_distance(ds_list.at(index).x, ds_list.at(index).y, frontiers.at(j).x_coordinate, frontiers.at(j).y_coordinate) > max_available_distance)
//                    continue;

                double dist = simplifiedDistanceFromDs(index, j);
                if (dist < 0) {
                    ROS_WARN("Distance computation failed");
                    continue;
                }
                if (dist * 2 < max_available_distance)
                {
                    // the DS has EOS
                    found_reachable_frontier = true;
                    
//                    // check if this DS is the closest DS with EOs
//                    double dist2 = distance_from_robot(ds_list.at(index).x, ds_list.at(index).y);
//                    if (dist2 < 0) {
//                        ROS_WARN("Distance computation failed");
//                        continue;
//                    }
//                    if (dist2 < min_dist)
//                    {
//                        min_dist = dist2;
                        mutex_ds.lock();
                        min_ds = &ds_list.at(index);
                        mutex_ds.unlock();
//                    }
                    
                    break;
                }
            }
            
            index++;
            if(index == ds_size)
                index = 0;

        }
//        retry++;
//        if(min_ds == NULL)
//            ros::Duration(3).sleep();
    }
    min_ds_for_path_traversal = min_ds;
    
    
    
    release_mutex(&store_frontier_mutex, __FUNCTION__);
    return found_reachable_frontier;
}

bool ExplorationPlanner::recomputeGoal() {
    ROS_INFO("my_error_counter: %d", my_error_counter);
    ROS_INFO("retrying_searching_frontiers: %d", retrying_searching_frontiers);
    return ( (retrying_searching_frontiers > 0 && retrying_searching_frontiers <= 3) || (my_error_counter > 0 && my_error_counter <= 5) ) ? true : false;
}

void ExplorationPlanner::logRemainingFrontiers(std::string csv_file) {
    fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs_csv << "### Frontiers (in Stage coordinates) ###";
    
    fs_csv << "# Remaining frontiers";
    for(unsigned int i=0; i < frontiers.size(); i++)
        fs_csv << frontiers.at(i).x_coordinate + robot_home_world_x << "," << frontiers.at(i).y_coordinate + robot_home_world_y << std::endl;
        
    fs_csv << "# Unreachable frontiers";
    for(unsigned int i=0; i < unreachable_frontiers.size(); i++)
        fs_csv << unreachable_frontiers.at(i).x_coordinate + robot_home_world_x << "," << unreachable_frontiers.at(i).y_coordinate + robot_home_world_y << std::endl;
        
    fs_csv.close();
}

bool ExplorationPlanner::existFrontiersReachableWithFullBattery(float max_available_distance, bool *error) {
    ROS_INFO("existFrontiersReachableWithFullBattery");
    
    if(!optimal_ds_set)
        return false;
        
    if(ds_list.size() == 0)
        ROS_FATAL("ds_list.size() == 0, which should be impossible here, or nothing will work...");
    
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    unsigned int ds_index = -1;
    mutex_ds.lock();
    
        
    for(unsigned int i=0; i < ds_list.size(); i++)
        if(ds_list.at(i).id == optimal_ds_id) {
            ds_index = i;
            break;
        }
    if(ds_index < 0 || ds_index > 100)
        ROS_FATAL("orrible values");
        
    mutex_ds.unlock();

    // distance to next frontier
    for (int i = 0; i < frontiers.size(); i++) {
        double distance;
        
        //check euclidean distances
//        distance = euclidean_distance(optimal_ds_x, optimal_ds_y, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
//        if(distance * 2 > max_available_distance)
//            continue;
        
        distance = simplifiedDistanceFromDs(ds_index, i);
        if(distance < 0){
            ROS_WARN("Failed to compute distance!");
            *error = true;
            continue;
        }
        if(distance * 2 < max_available_distance) // 0.9 just for safety, since the robot has to leave the DS and compute the next frontier... moreover it helps against continuous recharging at the current DS from docking... 
        {
            release_mutex(&store_frontier_mutex, __FUNCTION__);
            return true;
        }
    }
    release_mutex(&store_frontier_mutex, __FUNCTION__);
    return false;
                
}

void ExplorationPlanner::new_optimal_ds_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg) {
    ROS_INFO("Storing new optimal DS position");
    acquire_mutex(&mutex_optimal_ds, __FUNCTION__);
    next_optimal_ds_id = msg.get()->id;
    next_optimal_ds_x = msg.get()->x;
    next_optimal_ds_y = msg.get()->y;
    release_mutex(&mutex_optimal_ds, __FUNCTION__);
}

void ExplorationPlanner::updateOptimalDs() {
    if(next_optimal_ds_id >= 0)
        setOptimalDs(next_optimal_ds_id, next_optimal_ds_x, next_optimal_ds_y);
}

void ExplorationPlanner::robot_next_goal_callback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{
    ROS_INFO("Received next goal (frontier) of robot %d: (%.2f, %.2f)", (int) msg.get()->frontier_element[0].detected_by_robot, msg.get()->frontier_element[0].x_coordinate, msg.get()->frontier_element[0].y_coordinate);
      
    //ROS_INFO("Transform frontier coordinates");
        
    std::stringstream robot_number;
    robot_number << msg.get()->frontier_element[0].detected_by_robot;
    std::string robo_name, prefix = "robot_";
    robo_name = prefix.append(robot_number.str());

    //store_frontier_mutex.lock();

    //std::string service_topic = robo_name2.append("/map_merger/transformPoint"); // FIXME for real scenario!!! robot_might not be used here
    std::string service_topic = "map_merger/transformPoint";

//              ROS_INFO("Robo name: %s   Service to subscribe to: %s", robo_name.c_str(), service_topic.c_str());
    
    client = nh_transform.serviceClient<map_merger::TransformPoint>(service_topic);
    ros::service::waitForService(service_topic, ros::Duration(3).sleep());
    if(!ros::service::exists(service_topic, true))
        return;

    service_message.request.point.x = msg.get()->frontier_element[0].x_coordinate;
    service_message.request.point.y = msg.get()->frontier_element[0].y_coordinate;
    service_message.request.point.src_robot_id = msg.get()->frontier_element[0].detected_by_robot;
    //ROS_DEBUG("Robot name:  %s", service_message.request.point.src_robot.c_str());
    
    //ROS_ERROR("Old x: %f   y: %f", msg.get()->frontier_element[0].x_coordinate, msg.get()->frontier_element[0].y_coordinate);

    //ROS_ERROR("calling client");
    if(client.call(service_message))
        ; //ROS_ERROR("New x: %.1f   y: %.1f", service_message.response.point.x, service_message.response.point.y);
    else {
        ROS_ERROR("call to translate FAILED!");
        return;
    }   
    
//    bool found = false;
//    for(int i=0; i<last_robot_auctioned_frontier_list.size() && !found; i++)
//        if(last_robot_auctioned_frontier_list.at(i).detected_by_robot == msg.get()->frontier_element[0].detected_by_robot) {
//           last_robot_auctioned_frontier_list.at(i).x_coordinate = service_message.response.point.x;
//           last_robot_auctioned_frontier_list.at(i).y_coordinate = service_message.response.point.y;
//           found = true;
//        }
//    if(!found)
    {
        frontier_t new_goal;
        new_goal.detected_by_robot = msg.get()->frontier_element[0].detected_by_robot;
        new_goal.x_coordinate = msg.get()->frontier_element[0].x_coordinate;
        new_goal.y_coordinate = msg.get()->frontier_element[0].y_coordinate;
        new_goal.timestamp = ros::Time::now();
        mutex_last_robot_auctioned_frontier_list.lock();
        last_robot_auctioned_frontier_list.push_back(new_goal);
        mutex_last_robot_auctioned_frontier_list.unlock();
    }
}

void ExplorationPlanner::my_replyToNegotiationCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg) {
    if(fabs(frontier_under_negotiation.x_coordinate - msg.get()->frontier_element[0].x_coordinate) < 0.1 && fabs(frontier_under_negotiation.y_coordinate - msg.get()->frontier_element[0].y_coordinate) < 0.1) {
        ROS_DEBUG("%f vs %f", my_bid, msg.get()->frontier_element[0].bid);
        if(my_bid > msg.get()->frontier_element[0].bid ) //they are cost, so the higher, the worse
            winner_of_auction = false;
    }
    else {
        ROS_INFO("received reply for a frontier not currently under auction by this robot: ignoring");
    }    
}

void ExplorationPlanner::clean_frontiers_under_auction() {
    frontiers_under_auction.clear();
}

double ExplorationPlanner::distance_from_robot(double x, double y) {
    if(costmap_ros_ == NULL) {
        ROS_DEBUG("Costmap is not ready yet: cannot compute the distance between target and robot");
        return -1;   
    }
    //return trajectory_plan(x, y) * costmap_ros_->getCostmap()->getResolution(); //return a (very very rough, since it is computed as resolution * number_of_grid_cells_between_the_two_points) distance in meters
    double dist = trajectory_plan_meters(x, y);
    if(dist < 0)
        ROS_WARN("failed dist");
    return dist;
        
}

double ExplorationPlanner::distance(double x1, double y1, double x2, double y2) {
    if(costmap_ros_ == NULL) {
        ROS_DEBUG("Costmap is not ready yet: cannot compute the distance between given points");
        return -1;   
    }
    //return trajectory_plan(x1, y1, x2, y2) * costmap_ros_->getCostmap()->getResolution(); //return a (very very rough) distance in meters
    double dist = trajectory_plan_meters(x1, y1, x2, y2); //return a (very very rough) distance in meters
    if(dist<0)
        ROS_WARN("failed distance");
    return dist;
}

bool ExplorationPlanner::getRobotPose(tf::Stamped < tf::Pose > &robotPose) { //F returns position in meters or in cells? maybe cells...
    if(costmap_ros_ == NULL) {
        ROS_DEBUG("Costmap is not ready yet: cannot get robot pose");
        return false;   
    }
    return costmap_global_ros_->getRobotPose(robotPose);
}

void ExplorationPlanner::new_ds_on_graph_callback(const adhoc_communication::EmDockingStation msg) {
    //ROS_ERROR("RECEVIED!");
    
    if(num_ds <= 0) {       
        num_ds = msg.total_number_of_ds;
        //ds_list.resize(num_ds);
//        for (int i = 0; i < num_ds; i++)
//        {
//            std::vector<float> temp_f;
//            for (int j = 0; j < num_ds; j++) {
//                temp_f.push_back(-1);
//            }
////            ds_graph.push_back(temp_f);
//        }
    }
    
    ds_t ds;
    ds.x = msg.x;
    ds.y = msg.y;
    ds.id = msg.id;
    
    mutex_ds.lock();      
    ds.has_EOs = true; 
    ds_list.push_back(ds);

    mutex_ds.unlock();
}

void ExplorationPlanner::ds_count_callback(const std_msgs::Int32 msg) {
    ROS_ERROR("count received! %d", msg.data);
    if(num_ds <= 0) {       
        num_ds = msg.data;
        //ds_list.resize(num_ds);
        for (int i = 0; i < num_ds; i++)
        {
            std::vector<float> temp_f;
            for (int j = 0; j < num_ds; j++) {
                temp_f.push_back(-1);
            }
//            ds_graph.push_back(temp_f);
        }
    }
}

bool ExplorationPlanner::isFrontierReached(int point) {

	tf::Stamped < tf::Pose > robotPose;
	if (!costmap_ros_->getRobotPose(robotPose)) {
		ROS_WARN("[isFrontierReached]: Failed to get RobotPose");
	}
	geometry_msgs::PoseStamped robotPoseMsg;
	tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

	unsigned int fx, fy;
	double wfx, wfy;
	costmap_.indexToCells(point, fx, fy);
	costmap_.mapToWorld(fx, fy, wfx, wfy);

	double dx = robotPoseMsg.pose.position.x - wfx;
	double dy = robotPoseMsg.pose.position.y - wfy;

	if ((dx * dx) + (dy * dy)
			< (p_dist_for_goal_reached_ * p_dist_for_goal_reached_)) {
		//ROS_DEBUG("[isFrontierReached]: frontier is within the squared range of distance: %f m", p_dist_for_goal_reached_);
		return true;
	}

	return false;

}

double ExplorationPlanner::euclidean_distance(float x1, float y1, float x2, float y2) {
    return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

/* Try to acquire (lock) the mutex passed as argument */
void ExplorationPlanner::acquire_mutex(boost::mutex *mutex, std::string function_name) {
    ROS_DEBUG_COND_NAMED(ALL_LOG_LEVEL || LOG_MUTEX, "locks", "Function '%s' is trying to acquire mutex...", function_name.c_str());
    mutex->lock();
    ROS_DEBUG_COND_NAMED(ALL_LOG_LEVEL || LOG_MUTEX, "locks", "%s acquired by function '%s'", "Mutex", function_name.c_str());
}

/* Release (unlock) the mutex passed as argument */
void ExplorationPlanner::release_mutex(boost::mutex *mutex, std::string function_name) {
    mutex->unlock();
    ROS_DEBUG_COND_NAMED(ALL_LOG_LEVEL || LOG_MUTEX, "locks", "%s released by function '%s'", "Mutex", function_name.c_str());
}

bool ExplorationPlanner::storeFrontier_without_locking(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id)
{
    frontier_t new_frontier;

    if(robot_prefix_empty_param == true)
    {
        ROS_DEBUG("Storing Frontier ID: %d   Robot: %s", id, detected_by_robot_str.c_str());
        if(id != -1)
        {
            new_frontier.id = id;
        }else
        {
            new_frontier.id = frontier_id_count++;
        }
        new_frontier.detected_by_robot_str = detected_by_robot_str;
        new_frontier.detected_by_robot = detected_by_robot;
        new_frontier.x_coordinate = x;
        new_frontier.y_coordinate = y;
//        smartGoalBackoff(x, y, costmap_global_ros_, &new_frontier.smart_x_coordinate, &new_frontier.smart_y_coordinate);
        
        //F
        new_frontier.cluster_id = -1;
        discovered_new_frontier = true;
        frontiers.push_back(new_frontier);
    }else
    {
        if(detected_by_robot != robot_name)
        {
            new_frontier.id = id;
        }
        else
        {
           new_frontier.id = (robot_name * 10000) + frontier_id_count++;
        }

        new_frontier.detected_by_robot = detected_by_robot;
        new_frontier.detected_by_robot_str = detected_by_robot_str;
        new_frontier.x_coordinate = x;
        new_frontier.y_coordinate = y;
        
        //F
        new_frontier.cluster_id = -1;
        discovered_new_frontier = true;
        frontiers.push_back(new_frontier);
    }

    return true;
}

void ExplorationPlanner::set_auction_timeout(int timeout) {
    auction_timeout = timeout;
}

void ExplorationPlanner::add_to_sorted_fontiers_list_if_convinient(frontier_t frontier)
{
    for(unsigned int i=0; i < frontiers_under_auction.size(); i++)
        if(frontier.id == frontiers_under_auction.at(i).id) {
            ROS_INFO("This frontier is targetted by another robot: ignore it");
            return;
        }
    
    unsigned int k;
    bool inserted = false;
    for(k=0; k<sorted_frontiers.size() && !inserted ; k++)
        if(frontier.cost < sorted_frontiers.at(k).cost) {
            sorted_frontiers.insert(sorted_frontiers.begin() + k, frontier);
            if(sorted_frontiers.size() > limit_search*3 + 1)
                ROS_ERROR("somethign went wrong...");
            else if(sorted_frontiers.size() == limit_search*3 + 1) // times 3 to keep into account possible failures in the distance computation
                sorted_frontiers.erase(sorted_frontiers.begin() + sorted_frontiers.size() - 1);
            inserted = true;
            }
            
    if(!inserted && k < limit_search*3 - 2) //TODO do we need the -2?
        sorted_frontiers.push_back(frontier);
        
    if(sorted_frontiers.size() > limit_search*3)
        ROS_ERROR("somethign went wrong...");
}

bool ExplorationPlanner::home_is_reachable(double available_distance) {
    double dist = trajectory_plan_meters(robot_home_x, robot_home_y);
    if(dist < 0)
        return false;
    return available_distance > dist;
}

std::vector<frontier_t> ExplorationPlanner::getFrontierList() {
    return frontiers;
}

void ExplorationPlanner::setOptimalDs(unsigned int id, double x, double y) {
    acquire_mutex(&mutex_optimal_ds, __FUNCTION__);
    optimal_ds_id = id;
    optimal_ds_x = x;
    optimal_ds_y = y;
    if(!optimal_ds_set)
        ROS_INFO("optimal ds found");
    optimal_ds_set = true;
    release_mutex(&mutex_optimal_ds, __FUNCTION__);
}

bool ExplorationPlanner::updateRobotPose()
{
    int tries = 0;
    acquire_mutex(&costmap_mutex, __FUNCTION__);
    while (!costmap_ros_->getRobotPose(robotPose))
    {
        ROS_ERROR("Failed to get RobotPose");
        if(tries < 5) {
            tries++;
            ros::Duration(2).sleep();
        }
        else {
            release_mutex(&costmap_mutex, __FUNCTION__);
            return false;
        }
    }
    
    robot_x = robotPose.getOrigin().getX();
    robot_y = robotPose.getOrigin().getY();
    
    release_mutex(&costmap_mutex, __FUNCTION__);
    return true;
}

void ExplorationPlanner::sendListDssWithEos() {
    ROS_FATAL("MISSING");
    for(unsigned int i=0; i < ds_list.size(); i++) {
        adhoc_communication::EmDockingStation msg;
        msg.id = ds_list.at(i).id;
        msg.has_EOs = ds_list.at(i).has_EOs;
        ds_with_EOs_pub.publish(msg);
    }
}

double ExplorationPlanner::computeTheta(double frontier_x, double frontier_y) {
    ROS_INFO("%.1f, %.1f, %.1f, %.1f, %.1f, %.1f", robot_last_y, robot_y, robot_last_x, robot_x, frontier_y, frontier_x);
    double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
    double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
    if(_f1 < 0) {
        _f1 = 1/M_PI * (M_PI - fabs(fabs(theta_s - theta_g) - M_PI));
        _f2 = frontier_x;
        _f3 = frontier_y;
    }
    else {
        _f4 = 1/M_PI * (M_PI - fabs(fabs(theta_s - theta_g) - M_PI));
        _f5 = frontier_x;
        _f6 = frontier_y;
    }
    return 1/M_PI * (M_PI - fabs(fabs(theta_s - theta_g) - M_PI)); //it seems complex, but it's just to keep into account the sign of the angle (which would be lost with just the fabs calls)
}
