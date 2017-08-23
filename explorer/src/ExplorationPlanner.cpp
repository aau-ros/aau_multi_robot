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

std::vector< std::vector<int> > array_to_matrix(int* m, int rows, int cols) {
    int i,j;
    std::vector< std::vector<int> > r;
    r.resize(rows, std::vector<int>(cols, 0));

    for(i=0;i<rows;i++)
    {
        for(j=0;j<cols;j++)
            r[i][j] = m[i*cols+j];
    }
    return r;
}

bool sortCluster(const ExplorationPlanner::cluster_t &lhs, const ExplorationPlanner::cluster_t &rhs)
{
    if(lhs.cluster_element.size() > 1 && rhs.cluster_element.size() > 1)
    {
        return lhs.cluster_element.front().dist_to_robot < rhs.cluster_element.front().dist_to_robot;
    }
    return false; //F
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
    
    //ROS_ERROR("%s", sub_new_ds_on_graph.getTopic().c_str());
    //ROS_ERROR("%s", sub_ds_count.getTopic().c_str());
    
    

    // TODO
//    sub_robot = nh_robot.subscribe(robo_name+"/adhoc_communication/new_robot", 1000, &ExplorationPlanner::new_robot_callback, this);


//    adhoc_communication::Auction auction_init_status_msg;
//    auction_init_status_msg.start_auction = false;
//    auction_init_status_msg.auction_finished = true;
//    auction_init_status_msg.auction_status_message = true;
//    sendToMulticast("mc_", auction_init_status_msg, "auction");
//
//    adhoc_communication::Auction auction_init_msg;
//    auction_init_msg.start_auction = false;
//    auction_init_msg.auction_finished = true;
//    auction_init_msg.auction_status_message = false;
//    sendToMulticast("mc_", auction_init_msg, "auction");



//    if(robot_id == 1)
//    {
//        sub_frontiers = nh_frontier.subscribe("/robot_1/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
//        sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_1/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
//        sub_negotioation_first = nh_negotiation_first.subscribe("/robot_0/negotiation_list_first", 10000, &ExplorationPlanner::negotiationCallback, this);
//        sub_auctioning_first = nh_auction_first.subscribe("/robot_0/auction_first", 1000, &ExplorationPlanner::auctionCallback, this);
//        sub_position = nh_position.subscribe("/robot_1/all_positions", 1000, &ExplorationPlanner::positionCallback, this);
//        sub_auctioning_status = nh_auction_status.subscribe("/robot_0/auctionStatus", 1000, &ExplorationPlanner::auctionStatusCallback, this);
//    }else if(robot_id == 0)
//    {
//        sub_frontiers = nh_frontier.subscribe("/robot_0/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
//        sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_0/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
//        sub_negotioation_first = nh_negotiation_first.subscribe("/robot_1/negotiation_list_first", 10000, &ExplorationPlanner::negotiationCallback, this);
//        sub_auctioning_first = nh_auction_first.subscribe("/robot_1/auction_first", 1000, &ExplorationPlanner::auctionCallback, this);
//        sub_position = nh_position.subscribe("/robot_0/all_positions", 1000, &ExplorationPlanner::positionCallback, this);
//        sub_auctioning_status = nh_auction_status.subscribe("/robot_1/auctionStatus", 1000, &ExplorationPlanner::auctionStatusCallback, this);
//    }

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

void ExplorationPlanner::initialize_planner(std::string name,
		costmap_2d::Costmap2DROS *costmap, costmap_2d::Costmap2DROS *costmap_global, DistanceComputerInterface *distance_computer) {

    ROS_INFO("Initializing the planner");

	//copy the pointed costmap to be available in ExplorationPlanner

	this->costmap_ros_ = costmap;
        this->costmap_global_ros_ = costmap_global;

        if(initialized_planner == false)
        {
                nav.initialize("navigation_path", costmap_global_ros_);
                initialized_planner = true;
//                this->distance_computer = distance_computer;
        }
    int dim = costmap_global_ros_->getCostmap()->getSizeInCellsY() * costmap_global_ros_->getCostmap()->getSizeInCellsY();
    float dim_meters = costmap_global_ros_->getCostmap()->getSizeInMetersY() * costmap_global_ros_->getCostmap()->getSizeInMetersY();
    //ROS_ERROR("%d, %.0f, %.0f", dim, dim_meters, (float) dim * 0.05 * 0.05 );
	//Occupancy_grid_array is updated here
	this->setupMapData();

	last_mode_ = FRONTIER_EXPLORE;
	this->initialized_ = true;
	
	//ROS_ERROR("Initialized");

	/*
	 * reset all counter variables, used to count the number of according blocks
	 * within the occupancy grid.
	 */
	unknown = 0, free = 0, lethal = 0, inflated = 0;

}

bool ExplorationPlanner::transformToOwnCoordinates_frontiers()
{
    ROS_INFO("Transform frontier coordinates");

    //store_frontier_mutex.lock();
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    
    //ROS_ERROR("Transform frontier coordinates - lock acquired");

    for(int i = 0; i < frontiers.size(); i++)
    {
        bool same_robot = false;
        if(robot_prefix_empty_param == true)
        {
            if(frontiers.at(i).detected_by_robot_str.compare(robot_str) == 0)
                same_robot = true;
//                ROS_ERROR("Same Robot Detected");
        }else
        {
            if(frontiers.at(i).detected_by_robot == robot_name)
                same_robot = true;
        }
        if(same_robot == false)
        {
//            ROS_ERROR("Same robot is false");
            bool transform_flag = false;
            for(int j=0; j < transformedPointsFromOtherRobot_frontiers.size(); j++)
            {
                if(robot_prefix_empty_param == 0)
                {
                    if(transformedPointsFromOtherRobot_frontiers.at(j).id == frontiers.at(i).id && frontiers.at(i).detected_by_robot_str.compare(transformedPointsFromOtherRobot_frontiers.at(j).robot_str)== 0)
                    {
                        transform_flag = true;
                        break;
                    }
                }else
                {
                    if(transformedPointsFromOtherRobot_frontiers.at(j).id == frontiers.at(i).id)
                    {
                        transform_flag = true;
                        break;
                    }
                }
            }

            if(transform_flag != true)
            {
                std::string robo_name, robo_name2;

                if(robot_prefix_empty_param == false)
                {
                    std::stringstream robot_number;
                    robot_number << frontiers.at(i).detected_by_robot;

                    std::string prefix = "robot_";
                    robo_name = prefix.append(robot_number.str());
                    ROS_DEBUG("Robots name is        %s", robo_name.c_str());

                    std::stringstream robot_number2;
                    robot_number2 << robot_name;

                    std::string prefix2 = "/robot_";
                    robo_name2 = prefix2.append(robot_number2.str());
                }
                else
                {
//                    ROS_ERROR("Get Robot Name ... ");
//                    robo_name = lookupRobotName(frontiers.at(i).detected_by_robot);
//                    robo_name2 = lookupRobotName(robot_name);
                    robo_name = frontiers.at(i).detected_by_robot_str;
                    robo_name2 = robot_str;
                    ROS_DEBUG("Robot: %s   transforms from robot: %s", robo_name2.c_str(), robo_name.c_str());
                }



                std::string service_topic = robo_name2.append("/map_merger/transformPoint"); // FIXME for real scenario!!! robot_might not be used here

//              ROS_INFO("Robo name: %s   Service to subscribe to: %s", robo_name.c_str(), service_topic.c_str());

                client = nh_transform.serviceClient<map_merger::TransformPoint>(service_topic);
                
                //F
                ros::service::waitForService(service_topic, ros::Duration(3).sleep());
                if(!ros::service::exists(service_topic, true))
                    return false;

                service_message.request.point.x = frontiers.at(i).x_coordinate;
                service_message.request.point.y = frontiers.at(i).y_coordinate;
                service_message.request.point.src_robot = robo_name;

                ROS_DEBUG("Robot name:  %s", service_message.request.point.src_robot.c_str());
                ROS_DEBUG("Old x: %f   y: %f", frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);

                //ROS_ERROR("calling client");
                if(client.call(service_message))
                {
                    frontiers.at(i).x_coordinate = service_message.response.point.x;
                    frontiers.at(i).y_coordinate = service_message.response.point.y;

                    transform_point_t transform_point;
                    transform_point.id = frontiers.at(i).id;

                    if(robot_prefix_empty_param == true)
                    {
                        transform_point.robot_str = frontiers.at(i).detected_by_robot_str;
                    }
                    transformedPointsFromOtherRobot_frontiers.push_back(transform_point);
                    ROS_DEBUG("New x: %.1f   y: %.1f",service_message.response.point.x, service_message.response.point.y);
                }
                else
                    ; //ROS_ERROR("FAILED!");
            }
        }
    }
    release_mutex(&store_frontier_mutex, __FUNCTION__);
    ROS_INFO("Transform frontier coordinates DONE");
    return true;
}

bool ExplorationPlanner::transformToOwnCoordinates_visited_frontiers()
{
    ROS_INFO("Transform Visited Frontier Coordinates");

    for(int i = 0; i < visited_frontiers.size(); i++)
    {
        bool same_robot = false;
        if(robot_prefix_empty_param == true)
        {
            if(visited_frontiers.at(i).detected_by_robot_str.compare(robot_str) == 0)
                same_robot = true;
//                ROS_ERROR("Same Robot Detected");
        }else
        {
            if(visited_frontiers.at(i).detected_by_robot == robot_name)
                same_robot = true;
        }
        if(same_robot == false)
        {
            bool transform_flag = false;
            for(int j=0; j < transformedPointsFromOtherRobot_visited_frontiers.size(); j++)
            {
                if(robot_prefix_empty_param == 0)
                {
                    if(transformedPointsFromOtherRobot_visited_frontiers.at(j).id == visited_frontiers.at(i).id && visited_frontiers.at(i).detected_by_robot_str.compare(transformedPointsFromOtherRobot_visited_frontiers.at(j).robot_str)== 0)
                    {
                        transform_flag = true;
                        break;
                    }
                }else
                {
                    if(transformedPointsFromOtherRobot_visited_frontiers.at(j).id == visited_frontiers.at(i).id)
                    {
                        transform_flag = true;
                        break;
                    }
                }
            }

            if(transform_flag != true)
            {

                std::string robo_name, robo_name2;

                if(robot_prefix_empty_param == false)
                {
                    std::stringstream robot_number;
                    robot_number << visited_frontiers.at(i).detected_by_robot;

                    std::string prefix = "robot_";
                    robo_name = prefix.append(robot_number.str());
                    ROS_DEBUG("Robots name is        %s", robo_name.c_str());

                    std::stringstream robot_number2;
                    robot_number2 << robot_name;

                    std::string prefix2 = "/robot_";
                    robo_name2 = prefix2.append(robot_number2.str());
                }
                else
                {
//                    robo_name = lookupRobotName(visited_frontiers.at(i).detected_by_robot);
//                    robo_name2 = lookupRobotName(robot_name);

                    robo_name = visited_frontiers.at(i).detected_by_robot_str;
                    robo_name2 = robot_str;
                     ROS_DEBUG("Robot: %s   transforms from robot: %s", robo_name2.c_str(), robo_name.c_str());
                }


                std::string service_topic = robo_name2.append("/map_merger/transformPoint"); // FIXME for real scenario!!! robot_ might not be used here

                client = nh_transform.serviceClient<map_merger::TransformPoint>(service_topic);
                
                //F
                ros::service::waitForService(service_topic, ros::Duration(3).sleep());
                if(!ros::service::exists(service_topic, true))
                    return false;

                service_message.request.point.x = visited_frontiers.at(i).x_coordinate;
                service_message.request.point.y = visited_frontiers.at(i).y_coordinate;
                service_message.request.point.src_robot = robo_name;

                ROS_DEBUG("Old visited x: %f   y: %f", visited_frontiers.at(i).x_coordinate, visited_frontiers.at(i).y_coordinate);

                ROS_ERROR("calling");
                if(client.call(service_message))
                {
                    visited_frontiers.at(i).x_coordinate = service_message.response.point.x;
                    visited_frontiers.at(i).y_coordinate = service_message.response.point.y;

                    transform_point_t transform_point;
                    transform_point.id = visited_frontiers.at(i).id;

                    if(robot_prefix_empty_param == true)
                    {
                        transform_point.robot_str = visited_frontiers.at(i).detected_by_robot_str;
                    }
                    transformedPointsFromOtherRobot_visited_frontiers.push_back(transform_point);
                    ROS_DEBUG("New visited x: %.1f   y: %.1f",service_message.response.point.x, service_message.response.point.y);
                }
                else
                    ; //ROS_ERROR("FAILED!!!!");
            }
        }
    }
    ROS_INFO("Transform visited frontier coordinates DONE");
    return true;
}

/**
 * Compute the length of the trajectory from the robots current position to a given target
 * and store it in the global variable exploration_travel_path_global
 */
void ExplorationPlanner::trajectory_plan_store(double target_x, double target_y)
{
    double distance = trajectory_plan_meters(target_x, target_y);

    if(distance >= 0) {
        //F
        //exploration_travel_path_global += distance;
        //exploration_travel_path_global_meters += distance * costmap_ros_->getCostmap()->getResolution();
        exploration_travel_path_global_meters += distance;
    }
    else {
        ROS_ERROR("Failed to compute and store distance!");
        //ROS_ERROR("Failed to compute and store distance! Using euclidean distance as approximation..."); //TODO it could fail due to a failure of getRobotPose
        //exploration_travel_path_global_meters += euclidean_distance(target_x, target_y);
    }
}

double ExplorationPlanner::trajectory_plan_meters(double target_x, double target_y)
{
    if (!costmap_global_ros_->getRobotPose(robotPose))
    {
        ROS_ERROR("Failed to get RobotPose");
        return -1;
    }
    //ROS_ERROR("%f", costmap_global_ros_->getCostmap()->getResolution());
    double dist = trajectory_plan_meters(robotPose.getOrigin().getX(), robotPose.getOrigin().getY(), target_x, target_y);
    if(dist < 0) {
        ROS_WARN("failed distance");
    }
    return dist;
}

/**
 * Compute the length of the trajectory from a given start to a given target in meters
 */
double ExplorationPlanner::trajectory_plan_meters(double start_x, double start_y, double target_x, double target_y)
{
    ROS_INFO("trajectory_plan_meters");

    geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;
    double distance;

    std::vector<double> backoffGoal, backoffGoal2;
    bool backoff_flag = smartGoalBackoff(target_x,target_y, costmap_global_ros_, &backoffGoal);
    bool backoff_flag2 = smartGoalBackoff(start_x,start_y, costmap_global_ros_, &backoffGoal2);

    startPointSimulated.header.seq = start_point_simulated_message++;	// increase the sequence number
    startPointSimulated.header.stamp = ros::Time::now();
    startPointSimulated.header.frame_id = move_base_frame;
    if(backoff_flag2 == true)
    {
        startPointSimulated.pose.position.x = backoffGoal2.at(0);
        startPointSimulated.pose.position.y = backoffGoal2.at(1);
    }
    else
    {
        startPointSimulated.pose.position.x = start_x;
        startPointSimulated.pose.position.y = start_y;
    }
    startPointSimulated.pose.position.z = 0;
    startPointSimulated.pose.orientation.x = 0;
    startPointSimulated.pose.orientation.y = 0;
    startPointSimulated.pose.orientation.z = 0;
    startPointSimulated.pose.orientation.w = 1;

    goalPointSimulated.header.seq = goal_point_simulated_message++;	// increase the sequence number
    goalPointSimulated.header.stamp = ros::Time::now();
    goalPointSimulated.header.frame_id = move_base_frame;
    if(backoff_flag == true)
    {
        goalPointSimulated.pose.position.x = backoffGoal.at(0);
        goalPointSimulated.pose.position.y = backoffGoal.at(1);
    }
    else
    {
        goalPointSimulated.pose.position.x = target_x;
        goalPointSimulated.pose.position.y = target_y;
    }
    goalPointSimulated.pose.position.z = 0;
    goalPointSimulated.pose.orientation.x = 0;
    goalPointSimulated.pose.orientation.y = 0;
    goalPointSimulated.pose.orientation.z = 0;
    goalPointSimulated.pose.orientation.w = 1;

    std::vector<geometry_msgs::PoseStamped> global_plan;

    //acquire_mutex(&costmap_mutex, __FUNCTION__);
//    ROS_INFO("computing path");   
        
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_global_ros_->getCostmap()->getMutex()));
        
    bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);
//    distance_computer->computeDistance(1,2,3,4); //TODO use this for execution and testing...
    
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> unlock(*(costmap_global_ros_->getCostmap()->getMutex()));
//    ROS_INFO("path computed");
    //release_mutex(&costmap_mutex, __FUNCTION__);
    
    //ROS_ERROR("%d", successful);
    //ros::Duration(2).sleep();
    
    if(successful == true)
    {
//        ROS_ERROR("Path from (%f, %f) to (%f, %f)", startPointSimulated.pose.position.x, startPointSimulated.pose.position.y, goalPointSimulated.pose.position.x, goalPointSimulated.pose.position.y);
//        //distance =  global_plan.size();
//        for(int i=0; i < global_plan.size(); i++)
//            ROS_ERROR("(%f, %f)", global_plan[i].pose.position.x, global_plan[i].pose.position.y);
        
        distance = 0;
        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
        geometry_msgs::PoseStamped prev_point = (*it);
        it++;
        for(; it != global_plan.end(); it++) {
//            distance += euclidean_distance(prev_point.pose.position.x, prev_point.pose.position.y, (*it).pose.position.x, (*it).pose.position.y);
            distance += sqrt( (prev_point.pose.position.x - (*it).pose.position.x) * (prev_point.pose.position.x - (*it).pose.position.x) + (prev_point.pose.position.y - (*it).pose.position.y) * (prev_point.pose.position.y - (*it).pose.position.y) ); //* costmap_global_ros_->getCostmap()->getResolution();
            prev_point = (*it);
        }
        
        //ROS_ERROR("%f", distance);
        
//        for(int i=0; i < ds_list.size(); i++) {
//            if( fabs(target_x - ds_list[i].x) <= 0.3 && fabs(target_y - ds_list[i].y) <= 0.3)
//                ROS_ERROR("ok %.1f, %.1f", target_x, target_y);
//        }
        
//       ROS_INFO("end trajectory_plan_meters successfully");
        return distance;
    }
    else
    {
        // It should never happen that trajectory_plan_meters is called on a goal that is not reachable...
//        double wx, wy;
//        unsigned int mx, my;
//        costmap_global_ros_->getCostmap()->worldToMap(wx, wy, mx, my);
//        double world_x = (mx - costmap_global_ros_->getCostmap()->getSizeInCellsX() / 2) * costmap_global_ros_->getCostmap()->getResolution();
//        double world_y = (my - costmap_global_ros_->getCostmap()->getSizeInCellsY() / 2) * costmap_global_ros_->getCostmap()->getResolution();
        ROS_WARN("makePlan() failed for goal (%.1f, %.1f) from start (%.1f, %.1f) (Stage coord.s); returning -1...", goalPointSimulated.pose.position.x + robot_home_world_x,  goalPointSimulated.pose.position.y + robot_home_world_y, startPointSimulated.pose.position.x + robot_home_world_x, startPointSimulated.pose.position.y + robot_home_world_y);
        
//        ROS_ERROR("makePlan() failed for goal (%.1f, %.1f) from start (%.1f, %.1f) (Stage coord.s); returning -1...", goalPointSimulated.pose.position.x,  goalPointSimulated.pose.position.y, startPointSimulated.pose.position.x, startPointSimulated.pose.position.y);
        return -1;
    }
}

double ExplorationPlanner::simplifiedDistanceFromDs(unsigned int ds_index, unsigned int frontier_index) {
    
    acquire_mutex(&mutex_erase_frontier, __FUNCTION__);
    mutex_ds.lock();
    if(frontiers.size() <= frontier_index || ds_list.size() <= ds_index) {
        ROS_FATAL("invalid index in simplifiedDistanceFromDs()");
        ROS_FATAL("frontiers: size: %u, index: %u", frontiers.size(), frontier_index);
        ROS_FATAL("ds: size: %u; index: %u", ds_list.size(), ds_index);
    }
    mutex_ds.unlock();    
    
    double distance;
    if(ds_index >= frontiers.at(frontier_index).list_distance_from_ds.size() || frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) < 0)
    {
        double ds_x, ds_y, f_x, f_y;
        ds_x = ds_list.at(ds_index).x;
        ds_y = ds_list.at(ds_index).y;
        f_x = frontiers.at(frontier_index).x_coordinate;
        f_y = frontiers.at(frontier_index).y_coordinate;
        distance = trajectory_plan_meters(ds_x, ds_y, f_x, f_y); //TODO allow user to select if in this case we should use the real distance or not
    } 
    else
        distance = frontiers.at(frontier_index).list_distance_from_ds.at(ds_index);
    
    release_mutex(&mutex_erase_frontier, __FUNCTION__);
    return distance;
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

void ExplorationPlanner::printFrontiers()
{
    for(int i = 0; i < frontiers.size(); i++)
    {
        if(robot_prefix_empty_param == true)
        {
            ROS_INFO("Frontier %d:   x: %f   y: %f   robot: %s", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot_str.c_str());
        }else
        {
            ROS_INFO("Frontier %d:   x: %f   y: %f", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
        }
    }
}

bool ExplorationPlanner::storeFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id)
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
        new_frontier.x_coordinate = x;
        new_frontier.y_coordinate = y;
        
        //F
        new_frontier.cluster_id = -1;

        //store_frontier_mutex.lock();
        acquire_mutex(&store_frontier_mutex, __FUNCTION__);
        discovered_new_frontier = true;
        frontiers.push_back(new_frontier);
        release_mutex(&store_frontier_mutex, __FUNCTION__);
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
        new_frontier.x_coordinate = x;
        new_frontier.y_coordinate = y;
        
        //F
        new_frontier.cluster_id = -1;

        //store_frontier_mutex.lock();
        acquire_mutex(&store_frontier_mutex, __FUNCTION__);
        discovered_new_frontier = true;
        frontiers.push_back(new_frontier);
        release_mutex(&store_frontier_mutex, __FUNCTION__);
    }

    return true;
}

bool ExplorationPlanner::removeStoredFrontier(int id, std::string detected_by_robot_str)
{
    for(int i= 0; i < frontiers.size(); i++)
    {
        if(robot_prefix_empty_param == true)
        {
            ROS_DEBUG("Removing frontier with id '%d' detected by robot '%s'", frontiers.at(i).id, frontiers.at(i).detected_by_robot_str.c_str());
            if(frontiers.at(i).id == id && frontiers.at(i).detected_by_robot_str.compare(detected_by_robot_str) == 0)
            {
                ROS_DEBUG("Removing Frontier ID: %d  at position: %d  of Robot: %s", frontiers.at(i).id, i, frontiers.at(i).detected_by_robot_str.c_str());
                
                //store_frontier_mutex.lock();
                acquire_mutex(&store_frontier_mutex, __FUNCTION__);
                acquire_mutex(&mutex_erase_frontier, __FUNCTION__);
                frontiers.erase(frontiers.begin()+i);
//                if(i > 0)
//                {
//                    i --;
//                }
                erased = true;
                release_mutex(&store_frontier_mutex, __FUNCTION__);
                release_mutex(&mutex_erase_frontier, __FUNCTION__);
                
                //break; //FIXME ... only a test
            }
        }
        else
        {
            if(frontiers.at(i).id == id)
            {
            
                //store_frontier_mutex.lock();
                acquire_mutex(&store_frontier_mutex, __FUNCTION__);
                acquire_mutex(&mutex_erase_frontier, __FUNCTION__);
                erased = true;
                frontiers.erase(frontiers.begin()+i);
                if(i > 0)
                {
                    i --;
                }
                release_mutex(&store_frontier_mutex, __FUNCTION__);
                release_mutex(&mutex_erase_frontier, __FUNCTION__);
                break;
            }
        }
    }
    return true;
}

bool ExplorationPlanner::storeVisitedFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id)
{

    frontier_t visited_frontier;


    if(robot_prefix_empty_param == true)
    {
        ROS_DEBUG("Storing Visited Frontier ID: %d   Robot: %s", visited_frontier_id_count, detected_by_robot_str.c_str());
        if(id != -1)
        {
            visited_frontier.id = id;
        }else
        {
            visited_frontier.id = visited_frontier_id_count++;
        }

        visited_frontier.detected_by_robot_str = detected_by_robot_str;
        visited_frontier.x_coordinate = x;
        visited_frontier.y_coordinate = y;

        store_visited_mutex.lock();
        visited_frontiers.push_back(visited_frontier);
        store_visited_mutex.unlock();

    }else
    {
         if(detected_by_robot != robot_name)
        {
            visited_frontier.id = id;
        }
        else
        {
           visited_frontier.id = (robot_name * 10000) + visited_frontier_id_count++;
        }

        //ROS_ERROR("STORING %f, %f", x, y);
        visited_frontier.detected_by_robot = detected_by_robot;
        visited_frontier.x_coordinate = x;
        visited_frontier.y_coordinate = y;

        store_visited_mutex.lock();
        visited_frontiers.push_back(visited_frontier);
        store_visited_mutex.unlock();
    }

    bool break_flag = false;
    for(int i = 0; i < clusters.size(); i++)
    {
        for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
        {
            if(clusters.at(i).cluster_element.at(j).id == id)
            {
                ROS_DEBUG("Set cluster unreachable count to 0");
                clusters.at(i).unreachable_frontier_count = 0;
                break_flag = true;
                break;
            }
        }
        if(break_flag == true)
        {
            break;
        }
    }

    return true;
}

bool ExplorationPlanner::removeVisitedFrontier(int id, std::string detected_by_robot_str)
{
    for(int i= 0; i< visited_frontiers.size(); i++)
    {
        if(robot_prefix_empty_param == true)
        {

            if(visited_frontiers.at(i).id == id && visited_frontiers.at(i).detected_by_robot_str.compare(detected_by_robot_str) == 0)
            {
                ROS_INFO("Removing Visited Frontier ID: %d  at position: %d  of Robot: %s", visited_frontiers.at(i).id, i, visited_frontiers.at(i).detected_by_robot_str.c_str());
                store_visited_mutex.lock();
                visited_frontiers.erase(visited_frontiers.begin()+i);
//                if(i > 0)
//                {
//                    i --;
//                }
                store_visited_mutex.unlock();
                break;
            }
        }else
        {
            if(visited_frontiers.at(i).id == id)
            {
                store_visited_mutex.lock();
                visited_frontiers.erase(visited_frontiers.begin()+i);
                if(i > 0)
                {
                    i --;
                }
                store_visited_mutex.unlock();
                break;
            }
        }
    }
    return true;
}

bool ExplorationPlanner::storeUnreachableFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id)
{
    frontier_t unreachable_frontier;

    if(robot_prefix_empty_param == true)
    {
        ROS_DEBUG("Storing Unreachable Frontier ID: %d   Robot: %s", unreachable_frontier_id_count, detected_by_robot_str.c_str());

        if(id != -1)
        {
            unreachable_frontier.id = id;
        }else
        {
            unreachable_frontier.id = unreachable_frontier_id_count++;
        }

        unreachable_frontier.detected_by_robot_str = detected_by_robot_str;
        unreachable_frontier.x_coordinate = x;
        unreachable_frontier.y_coordinate = y;

        unreachable_frontiers.push_back(unreachable_frontier);
        //my_unreachable_frontiers.push_back(unreachable_frontier);

    }else
    {
        if(detected_by_robot != robot_name)
        {
            unreachable_frontier.id = id;
        }
        else
        {
           unreachable_frontier.id = (robot_name * 10000) + unreachable_frontier_id_count++;
        }

        unreachable_frontier.detected_by_robot = detected_by_robot;
        unreachable_frontier.x_coordinate = x;
        unreachable_frontier.y_coordinate = y;

        //frontiers.push_back(unreachable_frontier);
        unreachable_frontiers.push_back(unreachable_frontier); //F
        //my_unreachable_frontiers.push_back(unreachable_frontier);

    }

    bool break_flag = false;
    for(int i = 0; i < clusters.size(); i++)
    {
        for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
        {
            if(clusters.at(i).cluster_element.at(j).id == id)
            {
                ROS_WARN("Increasing cluster unreachable count");
                clusters.at(i).unreachable_frontier_count++;
                break_flag = true;
                break;
            }
        }
        if(break_flag == true)
        {
            break;
        }
    }

    return true;

}

bool ExplorationPlanner::sendToMulticast(std::string multi_cast_group, adhoc_communication::ExpFrontier frontier_to_send, std::string topic)
{
//    ROS_INFO("SENDING frontier id: %ld    detected_by: %ld", frontier_to_send.id ,frontier_to_send.detected_by_robot);
    adhoc_communication::SendExpFrontier service_frontier; // create request of type any+

    std::stringstream robot_number;
    robot_number << robot_name;
    std::string prefix = "robot_";
    std::string robo_name = prefix.append(robot_number.str());

    if(robot_prefix_empty_param == true)
    {
        robo_name = robot_str;
//        robo_name = lookupRobotName(robot_name);
    }
    std::string destination_name = multi_cast_group + robo_name; //for multicast
//    std::string destination_name = robo_name; // unicast

    ROS_INFO("sending to multicast group '%s' on topic: '%s'",destination_name.c_str(), topic.c_str());
    service_frontier.request.dst_robot = destination_name;
    service_frontier.request.frontier = frontier_to_send;
    service_frontier.request.topic = topic;

    if (ssendFrontier.call(service_frontier))
    {
            ROS_DEBUG("Successfully called service  sendToMulticast");

            if(service_frontier.response.status)
            {
                    ROS_DEBUG("adhoc comm returned successful transmission");
                    return true;
            }
            else
            {
                ROS_DEBUG("Failed to send to multicast group %s!",destination_name.c_str());
                return false;
            }
    }
    else
    {
     ROS_WARN("Failed to call service sendToMulticast [%s]",ssendFrontier.getService().c_str());
     return false;
    }
}

bool ExplorationPlanner::sendToMulticastAuction(std::string multi_cast_group, adhoc_communication::ExpAuction auction_to_send, std::string topic)
{
    adhoc_communication::SendExpAuction service_auction; // create request of type any+

    std::stringstream robot_number;
    robot_number << robot_name;
    std::string prefix = "robot_";
    std::string robo_name = prefix.append(robot_number.str());

    if(robot_prefix_empty_param == true)
    {
//        robo_name = lookupRobotName(robot_name);
        robo_name = robot_str;
    }

    std::string destination_name = multi_cast_group + robo_name; //for multicast
//    std::string destination_name = robo_name; // unicast

    ROS_INFO("sending auction to multicast group '%s' on topic '%s'",destination_name.c_str(), topic.c_str());
    service_auction.request.dst_robot = destination_name;
    service_auction.request.auction = auction_to_send;
    service_auction.request.topic = topic;

    if (ssendAuction.call(service_auction))
    {
            ROS_DEBUG("Successfully called service sendToMulticast");

            if(service_auction.response.status)
            {
                    ROS_DEBUG("Auction was multicasted successfully.");
                    return true;
            }
            else
            {
                ROS_WARN("Failed to send auction to mutlicast group %s!",destination_name.c_str());
                return false;
            }
    }
    else
    {
     ROS_WARN("Failed to call service sendToMulticastAuction [%s]",ssendAuction.getService().c_str());
     return false;
    }
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

void ExplorationPlanner::frontierCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{
    // TODO: Allow only frontiers that are not too close to walls
    //ROS_ERROR("frontierCallback, %u", msg.get()->frontier_element.size());

    adhoc_communication::ExpFrontierElement frontier_element;
    for(unsigned int i = 0; i < msg.get()->frontier_element.size(); i++)
    {
        frontier_element = msg.get()->frontier_element.at(i);
        bool result = true;
        for (unsigned int j = 0; j < frontiers.size(); j++)
        {
            if(robot_prefix_empty_param == true)
            {
                //ROS_ERROR("FrontierCallback ... ");
                if(frontiers.at(j).detected_by_robot_str.compare(frontier_element.detected_by_robot_str) == 0 && frontier_element.id == frontiers.at(j).id)
                {
                    ROS_ERROR("Same Detected ...");
                    result = false;
                    break;
                }
            }
            else
            {
                if(frontier_element.detected_by_robot == robot_name)
                {
                    //ROS_ERROR("This frontier was detected by this robot: ignoring");
                    result = false;
                    break;
                }
                else if (frontier_element.id == frontiers.at(j).id)
                {
                    //ROS_ERROR("This frontier has been already received: ignoring");
                    result = false;
                    break;
                }
            }
        }
        if (result == true)
        {

            if(robot_prefix_empty_param == true)
            {
                //ROS_ERROR("Received New Frontier with ID: %ld  Robot: %s", frontier_element.id, frontier_element.detected_by_robot_str.c_str());
                storeFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, frontier_element.detected_by_robot_str, frontier_element.id);
            }
            else
            {
                //ROS_ERROR("Received New Frontier of Robot %ld with ID %ld", frontier_element.detected_by_robot, frontier_element.id);
                if(frontier_element.detected_by_robot != robot_name)
                {
                    storeFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, "", frontier_element.id);
                }
            }
        }
    }

}

void ExplorationPlanner::visited_frontierCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{
    adhoc_communication::ExpFrontierElement frontier_element;
    for(unsigned int i = 0; i < msg.get()->frontier_element.size(); i++)
    {
        frontier_element = msg.get()->frontier_element.at(i);

        bool result = true;
        for (unsigned int j = 0; j < visited_frontiers.size(); j++)
        {
            if(frontier_element.detected_by_robot == robot_name)
            {
                result = false;
                break;
            }
            else
            {
                if (frontier_element.id == visited_frontiers.at(j).id)
                {
                    result = false;
                    break;
                }
            }
        }
        if (result == true)
        {
            ROS_DEBUG("Received New Visited Frontier of Robot %d with ID %d", frontier_element.detected_by_robot, frontier_element.id);
            if(robot_prefix_empty_param == true)
            {
                ROS_DEBUG("Storing Visited Frontier ID: %d  Robot: %s", frontier_element.id, frontier_element.detected_by_robot_str.c_str());
                storeVisitedFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, frontier_element.detected_by_robot_str, frontier_element.id);
            }else
            {
                if(frontier_element.detected_by_robot != robot_name)
                {
                    storeVisitedFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, "", frontier_element.id);
                }
            }
        }
    }
}

bool ExplorationPlanner::publish_frontier_list()
{
//    ROS_INFO("PUBLISHING FRONTIER LIST");

    publish_subscribe_mutex.lock();

    adhoc_communication::ExpFrontier frontier_msg;
    //ROS_ERROR("%u", frontiers.size());
    for(unsigned int i = 0; i<frontiers.size(); i++)
    {
        /* Send only frontiers detected by this robot (to avoid creating very huge message, which would require a lot of time to be completely analyzed) */
        if(frontiers.at(i).detected_by_robot != robot_name)
            continue;
            
        adhoc_communication::ExpFrontierElement frontier_element;

        frontier_element.id = frontiers.at(i).id;
        frontier_element.detected_by_robot = frontiers.at(i).detected_by_robot;
        frontier_element.detected_by_robot_str = frontiers.at(i).detected_by_robot_str;
        frontier_element.robot_home_position_x = frontiers.at(i).robot_home_x;
        frontier_element.robot_home_position_y = frontiers.at(i).robot_home_y;
        frontier_element.x_coordinate = frontiers.at(i).x_coordinate;
        frontier_element.y_coordinate = frontiers.at(i).y_coordinate;

        frontier_msg.frontier_element.push_back(frontier_element);
    }

    pub_frontiers.publish(frontier_msg);
    //sendToMulticast("mc_",frontier_msg, "frontiers");

    publish_subscribe_mutex.unlock();
}

bool ExplorationPlanner::publish_visited_frontier_list()
{
//    ROS_INFO("PUBLISHING VISITED FRONTIER LIST");

    publish_subscribe_mutex.lock();

    adhoc_communication::ExpFrontier visited_frontier_msg;

    for(unsigned int i = 0; i<visited_frontiers.size(); i++)
    {
        adhoc_communication::ExpFrontierElement visited_frontier_element;
        visited_frontier_element.id = visited_frontiers.at(i).id;
        visited_frontier_element.detected_by_robot = visited_frontiers.at(i).detected_by_robot;
        visited_frontier_element.detected_by_robot_str = visited_frontiers.at(i).detected_by_robot_str;
        visited_frontier_element.robot_home_position_x = visited_frontiers.at(i).robot_home_x;
        visited_frontier_element.robot_home_position_y = visited_frontiers.at(i).robot_home_y;
        visited_frontier_element.x_coordinate = visited_frontiers.at(i).x_coordinate;
        visited_frontier_element.y_coordinate = visited_frontiers.at(i).y_coordinate;
        
        //ROS_ERROR("PUBLISHING %f, %f", visited_frontier_element.x_coordinate, visited_frontier_element.y_coordinate);

        visited_frontier_msg.frontier_element.push_back(visited_frontier_element);
    }

    pub_visited_frontiers.publish(visited_frontier_msg);
    //sendToMulticast("mc_",visited_frontier_msg, "visited_frontiers");

    publish_subscribe_mutex.unlock();
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

void ExplorationPlanner::clearVisitedFrontiers()
{
//    ROS_INFO("Clear Visited");

    /* visited_frontier.at(0) is the Home point. Do not compare
     * with this point. Otherwise this algorithm deletes it, a new
     * frontier close to the home point is found which is then again
     * deleted since it is to close to the home point. This would not
     * have any impact on the exploration, but in simulation mode
     * (simulation = true) the frontier_ID is steadily up counted.
     * This is not necessary!
     */
    std::vector<int> goals_to_clear;

    for (unsigned int i = 1; i < visited_frontiers.size(); i++)
    {
        for (unsigned int j = 0; j < frontiers.size(); j++)
	{
            double diff_x = visited_frontiers.at(i).x_coordinate - frontiers.at(j).x_coordinate;
            double diff_y = visited_frontiers.at(i).y_coordinate - frontiers.at(j).y_coordinate;

            if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE) {
                if(robot_prefix_empty_param == true)
                {
                    removeStoredFrontier(frontiers.at(j).id, frontiers.at(j).detected_by_robot_str);
                    if(j > 0)
                    {
                        j--;
                    }
                }else
                {
                    removeStoredFrontier(frontiers.at(j).id, "");
                    if(j > 0)
                    {
                        j--;
                    }
//                    goals_to_clear.push_back(frontiers.at(j).id);
                }
                break;
            }
        }
    }

//    for(int i= 0; i< goals_to_clear.size(); i++)
//    {
//        removeStoredFrontier(goals_to_clear.at(i), goals_to_clear.);
//    }
}

void ExplorationPlanner::clearUnreachableFrontiers()
{
//    ROS_INFO("Clear UNREACHABLE");

    /* visited_frontier.at(0) is the Home point. Do not compare
     * with this point. Otherwise this algorithm deletes it, a new
     * frontier close to the home point is found which is then again
     * deleted since it is to close to the home point. This would not
     * have any impact on the exploration, but in simulation mode
     * (simulation = true) the frontier_ID is steadily up counted.
     * This is not necessary!
     */
    std::vector<int> goals_to_clear;

    for (unsigned int i = 1; i < unreachable_frontiers.size(); i++)
    {
        for (unsigned int j = 0; j < frontiers.size(); j++)
	{
            double diff_x = unreachable_frontiers.at(i).x_coordinate - frontiers.at(j).x_coordinate;
            double diff_y = unreachable_frontiers.at(i).y_coordinate - frontiers.at(j).y_coordinate;

            if (fabs(diff_x) <= 0.2 && fabs(diff_y) <= 0.2) {
//                goals_to_clear.push_back(frontiers.at(j).id);
                if(robot_prefix_empty_param == true)
                {
                    removeStoredFrontier(frontiers.at(j).id, frontiers.at(j).detected_by_robot_str);
                    if(j > 0)
                    {
                        j--;
                    }
                }else
                {
                    removeStoredFrontier(frontiers.at(j).id, "");
                    if(j > 0)
                    {
                        j--;
                    }
    //                    goals_to_clear.push_back(frontiers.at(j).id);
                }
                break;
            }
        }
    }

//    for(int i= 0; i< goals_to_clear.size(); i++)
//    {
//        removeStoredFrontier(goals_to_clear.at(i));
//    }
}

void ExplorationPlanner::clearSeenFrontiers(costmap_2d::Costmap2DROS *global_costmap)
{
//    ROS_INFO("Clear Seen");
    unsigned int mx, my, point;
    std::vector<int> neighbours, goals_to_clear;

//    this->costmap_global_ros_ = global_costmap;
//    costmap_global_ros_->getCostmapCopy(costmap_global_);
//    costmap_global_ros_->getCostmap();

//    ROS_INFO("Map origin  x: %f    y: %f", global_costmap->getCostmap()->getOriginX(), global_costmap->getCostmap()->getOriginY());
//    ROS_INFO("Map size    x: %d    y: %d", global_costmap->getCostmap()->getSizeInCellsX(), global_costmap->getCostmap()->getSizeInCellsY());
    if(frontiers.size() > 1)
    {
        for(unsigned int i = 0; i < frontiers.size(); i++)
        {
            unsigned int new_mx, new_my;
            bool unknown_found = false;
            bool obstacle_found = false;
            bool freespace_found = false;

            mx = 0;
            my = 0;
            
            //ROS_ERROR("%.2f", costmap_ros_->getCostmap()->getResolution());
           // ROS_INFO("frontier x: %f   y: %f", frontiers.at(i).x_coordinate,frontiers.at(i).y_coordinate);
            if(!global_costmap->getCostmap()->worldToMap(frontiers.at(i).x_coordinate,frontiers.at(i).y_coordinate,mx,my))
            {
                ROS_ERROR("Cannot convert coordinates successfully.");
                continue;
            }
//            ROS_INFO("Map coordinates mx: %d  my: %d",mx,my);

            neighbours = getMapNeighbours(mx, my, 6);

//            ROS_INFO("Neighbours: %u", neighbours.size());
            for (unsigned int j = 0; j < neighbours.size()/2; j++)
            {


//                ROS_INFO("Get Neighbour %d and %d",j*2, j*2+1);
                new_mx = neighbours.at(j*2);
                new_my = neighbours.at(j*2+1);
//                ROS_INFO("got access");


//                ROS_INFO("Calculating at position x: %d    y: %d", new_mx, new_my);

                //F
                unsigned char cost;
                //cost = global_costmap->getCostmap()->getCost(new_mx, new_my);
                cost = getCost(global_costmap, new_mx, new_my);
                
                
//                ROS_INFO("x position: %d       y position: %d", new_mx, new_my);
//                ROS_INFO("Got Cost");
                if(cost == costmap_2d::NO_INFORMATION)
                {
                    unknown_found = true;
                }
                else if(cost == costmap_2d::FREE_SPACE)
                {
                    freespace_found = true;
                }
                else if(cost == costmap_2d::LETHAL_OBSTACLE)
                {
                    obstacle_found = true;
                }
//                ROS_INFO("Done");
            }

            if(unknown_found == false || obstacle_found == true || freespace_found == false)
            {

//                goals_to_clear.push_back(frontiers.at(i).id);
                if(robot_prefix_empty_param == true)
                {
                    removeStoredFrontier(frontiers.at(i).id, frontiers.at(i).detected_by_robot_str);
                    if(i > 0)
                    {
                        i--;
                    }
                }else
                {
                    removeStoredFrontier(frontiers.at(i).id, "");
                    if(i > 0)
                    {
                        i--;
                    }
                }
                seen_frontier_list.push_back(frontiers.at(i));

            }
        }
    }
    
    ROS_DEBUG("frontiers.size() after cleaning: %u", frontiers.size());
}

/**
 * Find a backoff point in the neighborhood of (x,y) that is surrounded by only free space cells.
 * The backoff point must be clear of obstacles up to a distance of INNER_DISTANCE.
 * The neighborhood of (x,y) consists of 40 points surrounding (x,y).
 */
bool ExplorationPlanner::smartGoalBackoff(double x, double y, costmap_2d::Costmap2DROS *global_costmap, std::vector<double> *new_goal)
{
    unsigned int mx, my, new_mx, new_my, inner_mx, inner_my;
    double wx, wy;
    std::vector<int> neighbours, inner_neighbours;

    if(!global_costmap->getCostmap()->worldToMap(x,y,mx,my))
    {
        ROS_ERROR("Cannot convert coordinates successfully.");
        //ROS_ERROR("%f, %f", x, y);
        //ROS_ERROR("%f, %f", global_costmap->getCostmap()->getSizeInMetersX(), global_costmap->getCostmap()->getSizeInMetersY());
        return false;
    }
    //ROS_DEBUG("Map coordinates mx: %d  my: %d",mx,my);
    //ROS_ERROR("%f, %f", global_costmap->getCostmap()->getSizeInMetersX(), global_costmap->getCostmap()->getSizeInMetersY());


    neighbours = getMapNeighbours(mx, my, 40);
    //ROS_DEBUG("Got neighbours");
    for (unsigned int j = 0; j< neighbours.size()/2; j++)
    {
        // ROS_DEBUG("Get neighbours %d and %d",j*2,j*2+1); // bad place for debug output
        new_mx = neighbours.at(j*2);
        new_my = neighbours.at(j*2+1);

        //F
        unsigned char cost;
        //cost = global_costmap->getCostmap()->getCost(new_mx, new_my);
        cost = getCost(global_costmap, new_mx, new_my);

        if( cost == costmap_2d::FREE_SPACE)
        {
            bool back_off_goal_found = true;

            inner_neighbours = getMapNeighbours(new_mx, new_my, INNER_DISTANCE);
            for (unsigned int i = 0; i< inner_neighbours.size()/2; i++)
            {
                // ROS_DEBUG("Get inner neighbours %d and %d",i*2,i*2+1); // bad place for debug output!
                inner_mx = inner_neighbours.at(i*2);
                inner_my = inner_neighbours.at(i*2+1);

                //F
                unsigned char inner_cost;
                //inner_cost = global_costmap->getCostmap()->getCost(inner_mx, inner_my);
                inner_cost = getCost(global_costmap, inner_mx, inner_my);
                
                if( inner_cost != costmap_2d::FREE_SPACE)
                {
                    back_off_goal_found = false;
                    break;
                }
            }

            if(back_off_goal_found == true)
            {
                global_costmap->getCostmap()->mapToWorld(new_mx, new_my, wx, wy);
                new_goal->push_back(wx);
                new_goal->push_back(wy);
                return true;
            }
        }
    }
    return false;
}

std::vector<int> ExplorationPlanner::getMapNeighbours(unsigned int point_x, unsigned int point_y, int distance)
{
    std::vector<int> neighbours;

    for(int i = 0; i< distance; i++)
    {
        neighbours.push_back(point_x+i+1);
        neighbours.push_back(point_y);

        neighbours.push_back(point_x-i-1);
        neighbours.push_back(point_y);

        neighbours.push_back(point_x);
        neighbours.push_back(point_y+i+1);

        neighbours.push_back(point_x);
        neighbours.push_back(point_y-i-1);
    }
    return neighbours;
}

/*
 * searches the occupancy grid for frontier cells and merges them into one target point per frontier.
 * The returned frontiers are in world coordinates.
 */
void ExplorationPlanner::findFrontiers()
{

    ROS_INFO("Find Frontiers");
    allFrontiers.clear();
    int select_frontier = 1;
    std::vector<double> final_goal,start_points;

    /*
     * check for all cells in the occupancy grid whether
     * or not they are frontier cells. If a possible frontier is found, true is
     * returned
     */
    int new_frontier_point = 0;
    for (unsigned int i = 0; i < num_map_cells_; i++)
    {
        int new_frontier_point = isFrontier_2(i);
        if (new_frontier_point != 0)
        {
            /*
             * If isFrontier() returns true, the point which is checked to be a frontier
             * is indeed a frontier.
             */

            /*
             * Push back adds data x to a vector.
             * If a frontier was found, the position of the frontier is stored
             * in the allFrontiers vector.
             */
            allFrontiers.push_back(new_frontier_point);
        }
    }
    ROS_INFO("Found %u frontier cells which are transformed into frontiers points. Starting transformation...", allFrontiers.size());

    /*
     * Iterate over all frontiers. The frontiers stored in allFrontiers are
     * already REAL frontiers and can be approached by the robot to get new
     * informations of the environment.
     * To limit the amount of frontiers and only pick those which are valuable to
     * drive there. The rest of the frontiers are stored in a goal buffer which contain all
     * frontiers within the map. Additionally check whether or not a newly found
     * frontier has already been added to the list. If it is already in the list, do
     * not make a new entry with the coordinates of the frontier!
     */
     
     //F
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    
    // If there are no free cells in the map, get rid of all the previously stored frontiers, since for sure they are no more valid frontiers (or there should be some free cells in the map)
    if(allFrontiers.size() == 0)
        frontiers.clear();
    
    for (unsigned int i = 0; i < allFrontiers.size(); ++i)
    {
        geometry_msgs::PoseStamped finalFrontier;
        double wx, wy, wx2, wy2, wx3, wy3;
        unsigned int mx, my, mx2, my2, mx3, my3;
        bool result;

//        acquire_mutex(&costmap_mutex, __FUNCTION__);
        costmap_ros_->getCostmap()->indexToCells(allFrontiers.at(i), mx, my);
        costmap_ros_->getCostmap()->mapToWorld(mx, my, wx, wy);
//        release_mutex(&costmap_mutex, __FUNCTION__);

        //ROS_INFO("index: %d   map_x: %d   map_y: %d   world_x: %f   world_y: %f", allFrontiers.at(i), mx, my, wx, wy);

        /*
         * Check neighboring frontiers and only remember them if they are further
         * then a distance of MAX_GOAL_RANGE away from each other, discard otherwise.
         */
        if(select_frontier == 1)
        {
            result = true;
            
            for (unsigned int j = 0; j < frontiers.size(); j++)
            {
                if (fabs(wx - frontiers.at(j).x_coordinate) <= MAX_GOAL_RANGE && fabs(wy - frontiers.at(j).y_coordinate) <= MAX_GOAL_RANGE)
                {
                    result = false;
                    break;
                }
            }
            
            //F
//            if(result)
//                for (unsigned int j = 0; j < visited_frontiers.size(); j++)
//                {
//                    if (fabs(wx - visited_frontiers.at(j).x_coordinate) <= MAX_GOAL_RANGE && fabs(wy - visited_frontiers.at(j).y_coordinate) <= MAX_GOAL_RANGE)
//                    {
//                        result = false;
//                        break;
//                    }
//                }
            if(result)
                for (unsigned int j = 0; j < unreachable_frontiers.size(); j++)
                {
                    if (fabs(wx - unreachable_frontiers.at(j).x_coordinate) <= MAX_GOAL_RANGE && fabs(wy - unreachable_frontiers.at(j).y_coordinate) <= MAX_GOAL_RANGE)
                    {
                        result = false;
                        break;
                    }
                }
            //end_F
            
            if (result == true)
            {
                storeFrontier_without_locking(wx,wy,robot_name,robot_str,-1);
            }  
            
        }
        else if(select_frontier == 2)
        {
            std::vector<int> neighbour_index;

            for (unsigned int j = 0; j < allFrontiers.size(); ++j)
            {
            
//                acquire_mutex(&costmap_mutex, __FUNCTION__);
                costmap_ros_->getCostmap()->indexToCells(allFrontiers[j], mx2, my2);
                costmap_ros_->getCostmap()->mapToWorld(mx2, my2, wx2, wy2);
//                release_mutex(&costmap_mutex, __FUNCTION__);

                if (fabs(wx - wx2) <= MINIMAL_FRONTIER_RANGE && fabs(wy - wy2) <= MINIMAL_FRONTIER_RANGE && fabs(wx - wx2) != 0 && fabs(wy - wy2) != 0)
                {
                    neighbour_index.push_back(allFrontiers[j]);
                }
            }


            for (unsigned int n = 0; n < neighbour_index.size(); ++n)
            {
//                acquire_mutex(&costmap_mutex, __FUNCTION__);
                costmap_ros_->getCostmap()->indexToCells(neighbour_index[n], mx2, my2);
                costmap_ros_->getCostmap()->mapToWorld(mx2, my2, wx2, wy2);
//                release_mutex(&costmap_mutex, __FUNCTION__);

                while(true)
                {
                    bool end_point_found = true;
                    for (unsigned int k = 0; k < allFrontiers.size(); ++k)
                    {
//                        acquire_mutex(&costmap_mutex, __FUNCTION__);
                        costmap_ros_->getCostmap()->indexToCells(allFrontiers[k], mx3, my3);
                        costmap_ros_->getCostmap()->mapToWorld(mx3, my3, wx3, wy3);
//                        release_mutex(&costmap_mutex, __FUNCTION__);

                        if (fabs(wx2 - wx3) <= MINIMAL_FRONTIER_RANGE && fabs(wy2 - wy3) <= MINIMAL_FRONTIER_RANGE && wx2 != wx3 && wy2 != wy3 && wx != wx3 && wy != wy3)
                        {
                            wx  = wx2;
                            wy  = wy2;
                            wx2 = wx3;
                            wy2 = wy3;
                            end_point_found = false;
                        }
                    }
                    if(end_point_found == true)
                    {
                        start_points.push_back(wx2);
                        start_points.push_back(wy2);

                        break;
                    }
                }
            }
            goal_buffer_x.push_back(start_points.at(0)+(start_points.at(2)-start_points.at(0))/2);
            goal_buffer_y.push_back(start_points.at(1)+(start_points.at(3)-start_points.at(1))/2);
        }
    }
  
    if(!received_scan && allFrontiers.size() == 0) {
        ROS_ERROR("Apparently no laser scan has been received yet: retying later...");
        retrying_searching_frontiers++;
    }
    else {
        retrying_searching_frontiers = 0;
        received_scan = true;   
    }
    
    //F
    release_mutex(&store_frontier_mutex, __FUNCTION__);
    ROS_INFO("Size of all frontiers in the list: %u", frontiers.size());
}

bool ExplorationPlanner::reachable_target(double x, double y) {
    if(costmap_ros_ == NULL) {
        ROS_DEBUG("Costmap is not ready yet: cannot check reachability of the target");
        return false;
    }
    
    geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;

    startPointSimulated.header.seq = 1; //TODO correct??
    startPointSimulated.header.stamp = ros::Time::now();
    startPointSimulated.header.frame_id = move_base_frame;
    startPointSimulated.pose.position.x = robotPose.getOrigin().getX();
    startPointSimulated.pose.position.y = robotPose.getOrigin().getY();
    startPointSimulated.pose.position.z = 0;
    startPointSimulated.pose.orientation.x = 0;
    startPointSimulated.pose.orientation.y = 0;
    startPointSimulated.pose.orientation.z = 0;
    startPointSimulated.pose.orientation.w = 1;

    goalPointSimulated.header.seq = 1; //TODO correct??
    goalPointSimulated.header.stamp = ros::Time::now();
    goalPointSimulated.header.frame_id = move_base_frame;  
    goalPointSimulated.pose.position.x = x;
    goalPointSimulated.pose.position.y = y;
    goalPointSimulated.pose.position.z = 0;
    goalPointSimulated.pose.orientation.x = 0;
    goalPointSimulated.pose.orientation.y = 0;
    goalPointSimulated.pose.orientation.z = 0;
    goalPointSimulated.pose.orientation.w = 1;

    std::vector<geometry_msgs::PoseStamped> global_plan;

    //acquire_mutex(&costmap_mutex, __FUNCTION__);
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_global_ros_->getCostmap()->getMutex()));
    bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> unlock(*(costmap_global_ros_->getCostmap()->getMutex()));
    //release_mutex(&costmap_mutex, __FUNCTION__);
    
    if(successful == true) {
        double final_x, final_y;
        double acc = 1;
        final_x = global_plan.at(global_plan.size()-1).pose.position.x;
        final_y = global_plan.at(global_plan.size()-1).pose.position.y;
        
        //for(int i =0; i < global_plan.size(); i++)
        //    ROS_ERROR("point of plan: (%f, %f)", global_plan.at(i).pose.position.x, global_plan.at(i).pose.position.y);
        
        //ROS_ERROR("finals: (%f, %f) - target: (%f, %f)", final_x, final_y, x, y);
        
        if(final_x > x) {
            if(final_x - x > acc)
                return false;
            else if(x - final_x > acc)
                return false;
        }
        if(final_y > y) {
            if(final_y - y > acc)
                return false;
            else if(y - final_y > acc)
                return false;
        }
        return true;
    }
    else {
//        ROS_ERROR("the target is for the moment unreachable");
        // The target is (at least at the moment) unreachable, in the sense that there is not a path to it. Notice that it's not possible to understand if there was a failure in the computation of the path caused by the global planner or if a path does not really exists.
        return false;
   }
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

bool ExplorationPlanner::compute_and_publish_ds_path(double max_available_distance, int *result) {
    //just compute the goal ds for the moment //TODO complete
    ROS_DEBUG("Searching path on DS graph; %u frontiers exist", frontiers.size());
    double min_dist = std::numeric_limits<int>::max();
    ds_t *min_ds = NULL;
    int retry = 0;
    
    // "safety" initialization (if the caller gets -1 as a value, something went wrong)
    *result = -1;
    
    // search for the closest DS with EOs
//    while (min_ds == NULL && retry < 5) {
//        for (unsigned int i = 0; i < ds_list.size(); i++)
//        {
//            for (unsigned int j = 0; j < frontiers.size(); j++)
//            {
//                // check if the DS has EOs
//                double dist = distance(ds_list.at(i).x, ds_list.at(i).y, frontiers.at(j).x_coordinate, frontiers.at(j).y_coordinate);
//                if (dist < 0) {
//                    ROS_WARN("Distance computation failed");
//                    continue;
//                }
//                if (dist * 2 < max_available_distance)
//                {
//                    // the DS has EOS: check if it is the closest DS with EOs
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
//        retry++;
//        if(min_ds == NULL)
//            ros::Duration(3).sleep();
//    }

    min_ds = min_ds_for_path_traversal;
    
    if (min_ds == NULL) {
        ROS_INFO("min_ds == NULL");
        *result = 1;
        return false;
        // this could happen if distance() always fails... //TODO(IMPORTANT) what happen if I return and the explorer node needs to reach a frontier?
    }

    // compute closest DS
    min_dist = std::numeric_limits<int>::max();
    ds_t *closest_ds = NULL;
    retry = 0;
    while(closest_ds == NULL && retry < 10) {
        for (unsigned int i = 0; i < ds_list.size(); i++)
        {
            double dist = distance_from_robot(ds_list.at(i).x, ds_list.at(i).y);
            if (dist < 0) {
                ROS_WARN("Distance computation failed");
                continue;
            }

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_ds = &ds_list.at(i);
            }
        }
        retry++;
        if(closest_ds == NULL)
            ros::Duration(3).sleep();
    }
    if(closest_ds == NULL)  {
        ROS_INFO("closest_ds == NULL");
        *result = 2;
        return false;
    }
    
    if(closest_ds->id == min_ds->id) {
        ROS_INFO("closest_ds->id == min_ds->id");
        *result = 3;
//        return false; //DO NOT RETURN!!! we can still find a path...
    }

    adhoc_communication::EmDockingStation msg;
    msg.id = min_ds->id;
    ROS_DEBUG("publishing path...");
    publish_goal_ds_for_path_navigation.publish(msg);
    
    ROS_DEBUG("Finished compute_and_publish_ds_path()");
    *result = 0;
    return true;
    
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

bool ExplorationPlanner::my_negotiate()
{
    winner_of_auction = true;
    
#ifndef QUICK_SELECTION
     
    frontiers_under_auction.push_back(*my_selected_frontier);

    ROS_INFO("Publish fronter for negotiation");
    adhoc_communication::ExpFrontier negotiation_list;
    adhoc_communication::ExpFrontierElement negotiation_element;
    //negotiation_element.detected_by_robot_str = 
    negotiation_element.detected_by_robot = robot_name;
//    negotiation_element.detected_by_robot = my_selected_frontier->detected_by_robot;
    negotiation_element.x_coordinate = my_selected_frontier->x_coordinate;
    negotiation_element.y_coordinate = my_selected_frontier->y_coordinate;
    negotiation_element.id = my_selected_frontier->id;
        
    negotiation_list.frontier_element.push_back(negotiation_element);

    my_sendToMulticast("mc_", negotiation_list, "send_frontier_for_coordinated_exploration");

#endif

    first_negotiation_run = false;
}

bool ExplorationPlanner::my_sendToMulticast(std::string multi_cast_group, adhoc_communication::ExpFrontier frontier_list, std::string topic)
{    
    //ROS_ERROR("SENDING frontier id: %ld", frontier_list.frontier_element[0].id);
    adhoc_communication::SendExpFrontier service_frontier; // create request of type any+

    std::stringstream robot_number;
    robot_number << robot_name;
    std::string prefix = "robot_";
    std::string robo_name = prefix.append(robot_number.str());

    if(robot_prefix_empty_param == true)
    {
        robo_name = robot_str;
    }
    std::string destination_name = multi_cast_group + robo_name; //for multicast
    ROS_INFO("sending to multicast group '%s' on topic: '%s'",destination_name.c_str(), topic.c_str());
    service_frontier.request.dst_robot = destination_name;
    service_frontier.request.frontier = frontier_list;
    service_frontier.request.topic = topic;

    if (ssendFrontier.call(service_frontier))
    {
            ROS_DEBUG("Successfully called service  sendToMulticast");

            if(service_frontier.response.status)
            {
                    ROS_DEBUG("adhoc comm returned successful transmission");
                    return true;
            }
            else
            {
                ROS_DEBUG("Failed to send to multicast group %s!",destination_name.c_str());
                return false;
            }
    }
    else
    {
     ROS_ERROR("Failed to call service sendToMulticast [%s]",ssendFrontier_2.getService().c_str());
     return false;
    }
}

void ExplorationPlanner::my_negotiationCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{
    ROS_DEBUG("Received frontier (%.2f, %.2f) by robot %d", msg.get()->frontier_element[0].x_coordinate,  msg.get()->frontier_element[0].y_coordinate, msg.get()->frontier_element[0].detected_by_robot);

    double robot_x, robot_y;
    
    //ROS_INFO("Transform frontier coordinates");
        
    std::stringstream robot_number;
    robot_number << msg.get()->frontier_element[0].detected_by_robot;
    std::string robo_name, prefix = "robot_";
    robo_name = prefix.append(robot_number.str());

    //std::string service_topic = robo_name2.append("/map_merger/transformPoint"); // FIXME for real scenario!!! robot_might not be used here
    std::string service_topic = "map_merger/transformPoint";

//              ROS_INFO("Robo name: %s   Service to subscribe to: %s", robo_name.c_str(), service_topic.c_str());
    
    client = nh_transform.serviceClient<map_merger::TransformPoint>(service_topic);
    ros::service::waitForService(service_topic, ros::Duration(3).sleep());
    if(!ros::service::exists(service_topic, true)) {
        ROS_FATAL("service to translate coordinates does not exist!!!");
        return;
    }

    service_message.request.point.x = msg.get()->frontier_element[0].x_coordinate;
    service_message.request.point.y = msg.get()->frontier_element[0].y_coordinate;
    service_message.request.point.src_robot = robo_name;   
    //ROS_DEBUG("Robot name:  %s", service_message.request.point.src_robot.c_str());
    
    //ROS_ERROR("Old x: %f   y: %f", msg.get()->frontier_element[0].x_coordinate, msg.get()->frontier_element[0].y_coordinate);

    ROS_INFO("calling map_merger service to translate coordinates");
    if(client.call(service_message))
        ; //ROS_ERROR("New x: %.1f   y: %.1f", service_message.response.point.x, service_message.response.point.y);
    else {
        ROS_ERROR("FAILED call to translate coordinates for frontier negotiation!");
        return;   
    }
    
    ROS_DEBUG("Old x: %f   y: %f", msg.get()->frontier_element[0].x_coordinate, msg.get()->frontier_element[0].y_coordinate);
    ROS_DEBUG("New x: %f   y: %f", service_message.response.point.x, service_message.response.point.y);

    //acquire_mutex(&store_frontier_mutex, __FUNCTION__); //TODO maybe we need a mutex, but it causes deadlocks...
//    int index = -1;
//    for(int i=0; i<frontiers.size(); i++) //TODO inefficient (and the robot could be unable to send the frontier in time...)
//        if( fabs(frontiers.at(i).x_coordinate - service_message.response.point.x) < 1.0 && fabs(frontiers.at(i).y_coordinate - service_message.response.point.y) < 1.0 ) { //TODO correct?
//            index = i;
//            break;
//        }
//        
//    if(index < 0) {
////        ROS_INFO("robot doesn't know the auctioned frontier: ignoring it");
//        ROS_INFO("robot does NOT know the auctioned frontier");
////        return;
//    } else {
//        ROS_INFO("robot knows the auctioned frontier");
//    }
    
    frontier_t new_frontier;
    new_frontier.x_coordinate = service_message.response.point.x;
    new_frontier.y_coordinate = service_message.response.point.y;
    
    if(!my_check_efficiency_of_goal(available_distance_for_reply, &new_frontier)) {
        ROS_INFO("this frontier is currently not reachable by the robot: do not reply");
        return;
    }

    ROS_INFO("Replying to frontier auction");
//    double cost = frontier_cost(&frontiers.at(index));
    double cost = frontier_cost(&new_frontier);
    //release_mutex(&store_frontier_mutex, __FUNCTION__);
   
    //ROS_ERROR("%d + %d + %d + %f", d_g, d_gb, d_gbe, theta);
    
    adhoc_communication::ExpFrontier negotiation_list;
    adhoc_communication::ExpFrontierElement negotiation_element;
    //negotiation_element.detected_by_robot = my_selected_frontier->detected_by_robot;
    negotiation_element.x_coordinate = msg.get()->frontier_element[0].x_coordinate;
    negotiation_element.y_coordinate = msg.get()->frontier_element[0].y_coordinate;
    //negotiation_element.id = my_selected_frontier->id;
    negotiation_element.bid = cost;
    negotiation_list.frontier_element.push_back(negotiation_element);
    
    my_sendToMulticast("mc_", negotiation_list, "reply_to_frontier");
    
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

//TODO(IMPORTANT) safety coefficients
bool ExplorationPlanner::my_determine_goal_staying_alive(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id, bool energy_above_th, int w1, int w2, int w3, int w4)
{
    errors = 0;
    ros::Time start_time;
    selection_time = 0;
    number_of_frontiers = 0;
    frontier_selected = false;
    start_time = ros::Time::now();
    winner_of_auction = true;
    this->available_distance = available_distance;

    if (frontiers.size() <= 0 && clusters.size() <= 0)
    {
        ROS_INFO("No frontier/cluster available");
        return false;
    }
    
    if (available_distance <=0 ) {
        ROS_INFO("available_distance is negative");
        return false;
    } 

    sorted_frontiers.clear();
    
    //TODO move to a separate function that is called by explorer, since in case of error (when my_... is recalled by itself), this code otherwise is re-executed every time...
    ROS_DEBUG("frontiers size: %u", frontiers.size());
    if(APPROACH == 0)
        my_sort_cost_0(energy_above_th, w1, w2, w3, w4);
    else {
        ROS_ERROR("Invalid approach!");
        sorted_frontiers = frontiers;
    }
    
    sort_time = (ros::Time::now() - start_time).toSec();
    start_time = ros::Time::now();
    number_of_frontiers = frontiers.size();
    
    ROS_DEBUG("sorted_frontiers size: %u", sorted_frontiers.size());
    if(sorted_frontiers.size() == 0) {
        my_error_counter = 0;
        return false;
    }

    unsigned int skipped = 0;
    if(frontier_selected || APPROACH == 0) {
        frontier_selected = false;
        for(unsigned int i=0; i < sorted_frontiers.size(); i++)
        {
            if(APPROACH == 0)
                if(!my_check_efficiency_of_goal(available_distance, &sorted_frontiers.at(i))) {
                    ROS_INFO("frontier currentl unreachable: skipping");
                    continue;
                }

            my_selected_frontier = &sorted_frontiers.at(i);

            if(!test_mode) {

                //start auction
                my_bid = sorted_frontiers.at(i).cost;

                if(i == sorted_frontiers.size()-1 && (skipped + 1) >= num_robots)
                    ROS_INFO("this is the only frontier for the robot: no auctioning");
                else 
                {
                    ROS_INFO("start frontier negotiation");
                    frontier_under_negotiation.x_coordinate = my_selected_frontier->x_coordinate;
                    frontier_under_negotiation.y_coordinate = my_selected_frontier->y_coordinate;
                    my_negotiate();
    //         
                    for(int j = 0; j < auction_timeout; j++) {
                        ros::Duration(1).sleep();
                        ros::spinOnce();
                    }
                    
                    if(!winner_of_auction) {
                        ROS_INFO("frontier under auction: skip");
                        skipped++;
                        continue;
                    }
                }
                
                frontiers_under_auction.clear();
                
                adhoc_communication::ExpFrontier negotiation_list;
                adhoc_communication::ExpFrontierElement negotiation_element;
                negotiation_element.detected_by_robot = robot_name;
                negotiation_element.x_coordinate = my_selected_frontier->x_coordinate;
                negotiation_element.y_coordinate = my_selected_frontier->y_coordinate;
                negotiation_element.id = my_selected_frontier->id;
                    
                negotiation_list.frontier_element.push_back(negotiation_element);
            
                my_sendToMulticast("mc_", negotiation_list, "send_next_robot_goal");
            }
            
            ROS_INFO("selected goal: %.2f, %.2f", my_selected_frontier->x_coordinate, my_selected_frontier->y_coordinate);
            final_goal->push_back(my_selected_frontier->x_coordinate);
            final_goal->push_back(my_selected_frontier->y_coordinate);            
            final_goal->push_back(my_selected_frontier->detected_by_robot);
            final_goal->push_back(my_selected_frontier->id);
            
            // DEBUGGING
            final_goal->push_back(my_selected_frontier->_theta);

            robot_str_name->push_back(robot_name_str); 

            my_error_counter = 0;
            frontier_selected = true;
                      
            break;
        }
    }

    selection_time = (ros::Time::now() - start_time).toSec();    
    
    my_error_counter = 0;
    return frontier_selected;

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

void ExplorationPlanner::visualize_Frontiers()
{
        visualization_msgs::MarkerArray markerArray;

        for (int i = 0; i < frontiers.size(); i++)
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = move_base_frame;
            marker.header.stamp = ros::Time::now();
            marker.header.seq = frontier_seq_number++;
            marker.ns = "my_namespace";
            marker.id = frontier_seq_number;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(2); //TODO //F
            marker.pose.position.x = frontiers.at(i).x_coordinate;
            marker.pose.position.y = frontiers.at(i).y_coordinate;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.a = 1.0;

            if(frontiers.at(i).detected_by_robot == robot_name)
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            else if(frontiers.at(i).detected_by_robot == 1)
            {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            else if(frontiers.at(i).detected_by_robot == 2)
            {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            else
            {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }

            markerArray.markers.push_back(marker);
        }
        
        for(int j=0; j < ds_list.size(); j++) {
            visualization_msgs::Marker marker;

            marker.header.frame_id = move_base_frame;
            marker.header.stamp = ros::Time::now();
            marker.header.seq = frontier_seq_number++;
            marker.ns = "my_namespace";
            marker.id = frontier_seq_number;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(2); //TODO //F
            marker.pose.position.x = ds_list.at(j).x;
            marker.pose.position.y = ds_list.at(j).y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 0.1;

            marker.color.a = 1.0;

            marker.color.r = 0.5;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            markerArray.markers.push_back(marker);
        }
        
        
        

        pub_frontiers_points.publish <visualization_msgs::MarkerArray>(markerArray);
}

void ExplorationPlanner::visualize_visited_Frontiers()
{
        visualization_msgs::MarkerArray markerArray;

        for (int i = 0; i < visited_frontiers.size(); i++)
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = move_base_frame;
            marker.header.stamp = ros::Time::now();
            marker.header.seq = i+1;
            marker.ns = "my_namespace";
            marker.id = i+1;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = visited_frontiers.at(i).x_coordinate;
            marker.pose.position.y = visited_frontiers.at(i).y_coordinate;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.a = 1.0;

            if(visited_frontiers.at(i).detected_by_robot == robot_name)
            {
                marker.color.a = 0.2;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 1)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 2)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 3)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.5;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 4)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 0.5;
                marker.color.b = 1.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 5)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 0.5;
            }
            if(visited_frontiers.at(i).detected_by_robot == 6)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.5;
                marker.color.g = 0.0;
                marker.color.b = 0.5;
            }
            if(visited_frontiers.at(i).detected_by_robot == 7)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
            }
            if(visited_frontiers.at(i).detected_by_robot == 8)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 9)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
            }
            if(visited_frontiers.at(i).detected_by_robot == 10)
            {
                marker.color.a = 0.2;
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
            }
            markerArray.markers.push_back(marker);
        }

        pub_visited_frontiers_points.publish <visualization_msgs::MarkerArray>(markerArray);
}

void ExplorationPlanner::setupMapData() {

	if ((this->map_width_ != costmap_ros_->getCostmap()->getSizeInCellsX())
			|| (this->map_height_ != costmap_ros_->getCostmap()->getSizeInCellsY())) {
		this->deleteMapData();

		map_width_ = costmap_ros_->getCostmap()->getSizeInCellsX();
		map_height_ = costmap_ros_->getCostmap()->getSizeInCellsY();
        num_map_cells_ = map_width_ * map_height_;

		ROS_INFO("Costmap size in cells: width:%d   height: %d   map_cells:%d",map_width_,map_height_,num_map_cells_);

		// initialize exploration_trans_array_, obstacle_trans_array_, goalMap and frontier_map_array_
//		exploration_trans_array_ = new unsigned int[num_map_cells_];
//		obstacle_trans_array_ = new unsigned int[num_map_cells_];
//		is_goal_array_ = new bool[num_map_cells_];
//		frontier_map_array_ = new int[num_map_cells_];
//		clearFrontiers();
//		resetMaps();
	}


	occupancy_grid_array_ = costmap_ros_->getCostmap()->getCharMap();

	for (unsigned int i = 0; i < num_map_cells_; i++) {

		countCostMapBlocks(i);

//	  if(occupancy_grid_array_[i] == costmap_2d::NO_INFORMATION || occupancy_grid_array_[i] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || occupancy_grid_array_[i] == costmap_2d::LETHAL_OBSTACLE || occupancy_grid_array_[i] == costmap_2d::FREE_SPACE)
//	  {
//	  }else
//	  {
//		  ROS_DEBUG("%d",(int)occupancy_grid_array_[i]);
//		  //occupancy_grid_array_[i] = '0'; // write 0 to the grid!
//	  }
	}
	ROS_INFO("--------------------- Iterate through Costmap --------------------");
	ROS_INFO("Free: %d  Inflated: %d  Lethal: %d  unknown: %d rest: %d", free,
			inflated, lethal, unknown,
			num_map_cells_ - (free + inflated + lethal + unknown));
	ROS_INFO("------------------------------------------------------------------");
        free_space = free;
}

int ExplorationPlanner::isFrontier(int point) {


	if (isFree(point)) {

		/*
		 * The point is either a obstacle or a point with not enough
		 * information about
		 * Therefore, check if the point is surrounded by other NO_INFORMATION
		 * points. Then further space to explore is found ---> frontier
		 */
		//int Neighbours = 0;
		//int points[((int)pow(8,Neighbours+1))+8]; // NEIGHBOURS points, each containing 8 adjacent points
		int no_inf_count = 0;
//		int inscribed_count = 0;
		int adjacent_points[16];
		/*
		 * Now take one point and lookup all adjacent points (surrounding points).
		 * The variable adjacentPoints contains all neighboring point (up, right, left ...)
		 */

		/*
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[0]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[1]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[2]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[3]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[4]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[5]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[6]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[7]);
		 */

		//histogram[(int)occupancy_grid_array_[point]]++;

		if ((int) occupancy_grid_array_[point] == costmap_2d::FREE_SPACE)//<= threshold_free)
		{
			getAdjacentPoints(point, adjacent_points);
			for (int i = 0; i < 16; i++) // length of adjacent_points array
			{
                if (adjacent_points[i] < 0)
                {
                    continue;
                }
				if (occupancy_grid_array_[adjacent_points[i]] == costmap_2d::NO_INFORMATION) {
					no_inf_count++;
//                                        ROS_DEBUG("No information found!");
				}
                                else if (occupancy_grid_array_[adjacent_points[i]] == costmap_2d::LETHAL_OBSTACLE) {
					/*
					 * Do not break here ... In some scenarios it may happen that unknown and free space
					 * form a border, but if just a small corridor is available there, it would not be
					 * detected as frontier, since even one neighboring block is an inflated block.
					 * Therefore do nothing, if even one unknown block is a neighbor of an free space
					 * block, then it is a frontier!
					 */

					//inscribed_count++;
//                                        ROS_DEBUG("Obstacle found");
					return(0);
				}
			}

			/*
			 * Count the number of lethal obstacle (unknown space also included
			 * here). If an inflated obstacle was detected ... the point is not
			 * a possible frontier and should no longer be considered.
			 * Otherwise, when all surronding blocks --> 64+7 are unknown just the
			 * one in the middle is free space, then we found a frontier!!
			 * On every found frontier, true is returned
			 */
			if (no_inf_count > 0)
			{
				/*
				 * Above a adjacent Point is taken and compared with NO_INFORMATION.
				 * If the point contains no information, the next surrounding point from that point
				 * is taken and again compared with NO_INFORMATION. If there are at least
				 * two points with no information surrounding a no information point, then return true!
				 * (at least two points with no information means that there is sufficient space
				 * with not enough information about)
				 */

                            //return(backoff(point));
                           return(point);
			}
		}
	}
	return(0);
}

int ExplorationPlanner::isFrontier_2(int point) {


	if (isFree(point)) {

		/*
		 * The point is either a obstacle or a point with not enough
		 * information about
		 * Therefore, check if the point is surrounded by other NO_INFORMATION
		 * points. Then further space to explore is found ---> frontier
		 */
		//int Neighbours = 0;
		//int points[((int)pow(8,Neighbours+1))+8]; // NEIGHBOURS points, each containing 8 adjacent points
		int no_inf_count = 0;
//		int inscribed_count = 0;
		int adjacent_points[16];
		/*
		 * Now take one point and lookup all adjacent points (surrounding points).
		 * The variable adjacentPoints contains all neighboring point (up, right, left ...)
		 */

		/*
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[0]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[1]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[2]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[3]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[4]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[5]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[6]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[7]);
		 */

		//histogram[(int)occupancy_grid_array_[point]]++;

		if ((int) occupancy_grid_array_[point] == costmap_2d::FREE_SPACE)//<= threshold_free)
		{
			getAdjacentPoints(point, adjacent_points);
			for (int i = 0; i < 16; i++) // length of adjacent_points array
			{
                if (adjacent_points[i] < 0)
                {
                    continue;
                }
				if (occupancy_grid_array_[adjacent_points[i]] == costmap_2d::NO_INFORMATION) {
					no_inf_count++;
//                                        ROS_DEBUG("No information found!");
				}
                                else if (occupancy_grid_array_[adjacent_points[i]] == costmap_2d::LETHAL_OBSTACLE) {
					/*
					 * Do not break here ... In some scenarios it may happen that unknown and free space
					 * form a border, but if just a small corridor is available there, it would not be
					 * detected as frontier, since even one neighboring block is an inflated block.
					 * Therefore do nothing, if even one unknown block is a neighbor of an free space
					 * block, then it is a frontier!
					 */

					//inscribed_count++;
//                                        ROS_DEBUG("Obstacle found");
					return(0);
				}
			}

			/*
			 * Count the number of lethal obstacle (unknown space also included
			 * here). If an inflated obstacle was detected ... the point is not
			 * a possible frontier and should no longer be considered.
			 * Otherwise, when all surronding blocks --> 64+7 are unknown just the
			 * one in the middle is free space, then we found a frontier!!
			 * On every found frontier, true is returned
			 */
			if (no_inf_count > 0)
			{
				/*
				 * Above a adjacent Point is taken and compared with NO_INFORMATION.
				 * If the point contains no information, the next surrounding point from that point
				 * is taken and again compared with NO_INFORMATION. If there are at least
				 * two points with no information surrounding a no information point, then return true!
				 * (at least two points with no information means that there is sufficient space
				 * with not enough information about)
				 */

                            //return(backoff(point));
                           return(point);
			}
		}
	}
	return(0);
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

inline void ExplorationPlanner::getAdjacentPoints(int point, int points[]) {

	/*
	 * Get all surrounding neighbors and the neighbors of those.
	 */

	/*
	 * 		ooo ooo
	 *
	 */

	/*
	 points[0] = left(point);
	 points[1] = right(point);
	 points[2] = left(points[0]);
	 points[3] = right(points[1]);
	 points[4] = left(points[2]);
	 points[5] = right(points[3]);
	 points[6] = left(points[4]);
	 points[7] = right(points[5]);
	 */

	/*
	 *            o
	 *          o   o
	 *            o
	 *
	 */

        /*
	points[0] = left(point);
	points[1] = up(point);
	points[2] = right(point);
	points[3] = down(point);
        */

	/*
	 *          o o o
	 *          o   o
	 *          o o o
	 *
	 */


	 points[0] = left(point);
	 points[1] = up(point);
	 points[2] = right(point);
	 points[3] = down(point);
	 points[4] = upleft(point);
	 points[5] = upright(point);
	 points[6] = downright(point);
	 points[7] = downleft(point);

	/*
	 *        *   *   *
	 *          o o o
	 *        * o   o *
	 *          o o o
	 *        *   *   *
	 */

	 points[8] =  left(points[0]);
	 points[9] =  up(points[1]);
	 points[10] = right(points[2]);
	 points[11] = down(points[3]);
	 points[12] = upleft(points[4]);
	 points[13] = upright(points[5]);
	 points[14] = downright(points[6]);
	 points[15] = downleft(points[7]);


	/*
	 *        * + * + *
	 *        + o o o +
	 *        * o   o *
	 *        + o o o +
	 *        * + * + *
	 */
	/*
	 points[16] = up(points[4]);
	 points[17] = up(points[5]);
	 points[18] = right(points[5]);
	 points[19] = right(points[6]);
	 points[20] = down(points[6]);
	 points[21] = down(points[7]);
	 points[22] = left(points[7]);
	 points[23] = left(points[4]);
	 */

	/*
	 *      #     #     #
	 *        *   *   *
	 *          o o o
	 *      # * o   o * #
	 *          o o o
	 *        *   *   *
	 *      #     #     #
	 */
	/*
	 points[16] = left(points[8]);
	 points[17] = up(points[9]);
	 points[18] = right(points[10]);
	 points[19] = down(points[11]);
	 points[20] = upleft(points[12]);
	 points[21] = upright(points[13]);
	 points[22] = downright(points[14]);
	 points[23] = downleft(points[15]);
	 */
}

bool ExplorationPlanner::countCostMapBlocks(int point) {

       if (occupancy_grid_array_[point] == costmap_2d::NO_INFORMATION) {
//		ROS_DEBUG("[isFree] NO_INFORMATION");
		unknown++;
		return true;
	} else if ((int) occupancy_grid_array_[point] == costmap_2d::FREE_SPACE) {
		free++;
//		ROS_DEBUG("[isFree] FREE SPACE FOUND");
		return true;
	}

	else if ((int) occupancy_grid_array_[point] == costmap_2d::LETHAL_OBSTACLE) {
		lethal++;
//		ROS_DEBUG("[isFree] LETHAL OBSTACLE FOUND");
		return true;
	} else if ((int) occupancy_grid_array_[point] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
		inflated++;
//		ROS_DEBUG("[isFree] INSCRIBED INFLATED OBSTACLE FOUND");
		return true;
	} else
        {
		int undefined = (unsigned char) occupancy_grid_array_[point];
		// An obstacle was found. This is no free space, so a FALSE is returned
//		ROS_DEBUG("undefined value: %d", undefined);
		return true;
	}
	return false;
}

bool ExplorationPlanner::isFree(int point) {
	if (isValid(point)) {

		return true;
	} else {
		ROS_ERROR("Point is not valid! Reason: negative number ");
	}
	return false;
}

inline bool ExplorationPlanner::isValid(int point) {
	return (point >= 0);
}

void ExplorationPlanner::clearFrontiers() {
	std::fill_n(frontier_map_array_, num_map_cells_, 0);
}

inline int ExplorationPlanner::left(int point) {
	// only go left if no index error and if current point is not already on the left boundary
	if ((point % map_width_ != 0)) {
		return point - 1;
	}
	return -1;
}

inline int ExplorationPlanner::upleft(int point) {
	if ((point % map_width_ != 0) && (point >= (int) map_width_)) {
		return point - map_width_ - 1;
	}
	return -1;

}

inline int ExplorationPlanner::up(int point) {
	if (point >= (int) map_width_) {
		return point - map_width_;
	}
	return -1;
}

inline int ExplorationPlanner::upright(int point) {
	if ((point >= (int) map_width_) && ((point + 1) % (int) map_width_ != 0)) {
		return point - map_width_ + 1;
	}
	return -1;
}

inline int ExplorationPlanner::right(int point) {
	if ((point + 1) % map_width_ != 0) {
		return point + 1;
	}
	return -1;

}

inline int ExplorationPlanner::downright(int point) {
	if (((point + 1) % map_width_ != 0)
			&& ((point / map_width_) < (map_width_ - 1))) {
		return point + map_width_ + 1;
	}
	return -1;

}

inline int ExplorationPlanner::down(int point) {
	if ((point / map_width_) < (map_width_ - 1)) {
		return point + map_width_;
	}
	return -1;
}

inline int ExplorationPlanner::downleft(int point) {
	if (((point / map_width_) < (map_width_ - 1))
			&& (point % map_width_ != 0)) {
		return point + map_width_ - 1;
	}
	return -1;
}

unsigned char ExplorationPlanner::getCost(costmap_2d::Costmap2DROS *costmap, unsigned int cell_x, unsigned int cell_y) {
    if(cell_x >= costmap->getCostmap()->getSizeInCellsX() || cell_y >= costmap->getCostmap()->getSizeInCellsY() ) {
        //ROS_ERROR("Try to get the cost of a cell outside the costmap: returning LETHAL_OBSTACLE...");
        return costmap_2d::LETHAL_OBSTACLE;
    }
    return costmap->getCostmap()->getCost(cell_x, cell_y);
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

void ExplorationPlanner::my_sort_cost_0(bool energy_above_th, int w1, int w2, int w3, int w4)
{
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    
    my_energy_above_th = energy_above_th;
    this->w1 = w1;
    this->w2 = w2;
    this->w3 = w3;
    this->w4 = w4;
    
    if(frontiers.size() > 0)
    {     
        for(unsigned int j = 0; j < frontiers.size(); j++)
        {
            if(!my_quick_check_efficiency_of_goal(this->available_distance, &frontiers.at(j)))
                continue;
            // calculate cost function
            frontiers.at(j).cost = frontier_cost_0(&frontiers.at(j));
            add_to_sorted_fontiers_list_if_convinient(frontiers.at(j));        
        }
        robot_last_x = robot_x;
        robot_last_y = robot_y;
        use_theta = true;
    }
    else
    {
        ROS_INFO("Sorting not possible, no frontiers available");
    }
    
    release_mutex(&store_frontier_mutex, __FUNCTION__);
    ROS_INFO("finished my_sort_cost_0");
  
}

bool ExplorationPlanner::home_is_reachable(double available_distance) {
    double dist = trajectory_plan_meters(robot_home_x, robot_home_y);
    if(dist < 0)
        return false;
    return available_distance > dist;
}

double ExplorationPlanner::frontier_cost(frontier_t *frontier) {
    return frontier_cost_0(frontier);
}

double ExplorationPlanner::frontier_cost_0(frontier_t *frontier) {

    // frontier position
    double frontier_x = frontier->x_coordinate;
    double frontier_y = frontier->y_coordinate;

    // calculate d_g
    double d_g = euclidean_distance(frontier_x, frontier_y, robot_x, robot_y);

    // calculate d_gbe
    double d_gb;
    if(optimal_ds_set)
        d_gb = euclidean_distance(frontier_x, frontier_y, optimal_ds_x, optimal_ds_y);
    else
        d_gb = euclidean_distance(frontier_x, frontier_y, robot_home_x, robot_home_y);
    double d_gbe;
    if(my_energy_above_th)
    {
        d_gbe = -d_gb;
    }
    else
    {
        d_gbe = d_gb;
    }
    
    // calculate d_r
    double d_r = 0;
    ros::Time time_now = ros::Time::now();
    mutex_last_robot_auctioned_frontier_list.lock();
    for(unsigned int i=0; i<last_robot_auctioned_frontier_list.size(); i++) {
    
        // remove auctioned frontiers that are too old //TODO should be better doing this in another place... but it may be inefficient
        if(time_now - last_robot_auctioned_frontier_list.at(i).timestamp > ros::Duration(VALIDITY_INTERVAL)) {
            ROS_INFO("expired");
            last_robot_auctioned_frontier_list.erase(last_robot_auctioned_frontier_list.begin() + i);
        }
        else {
            double distance = euclidean_distance(frontier_x, frontier_y, last_robot_auctioned_frontier_list.at(i).x_coordinate, last_robot_auctioned_frontier_list.at(i).y_coordinate);
            if(distance < 0)
                continue;
            if(distance < d_r || d_r == 0) 
                d_r = distance;        
        }
    }
    mutex_last_robot_auctioned_frontier_list.unlock();
    d_r = -d_r;    

    // calculate theta
    double theta;
    if(use_theta)
        theta = computeTheta(frontier_x, frontier_y);
    else
        theta = 0;
    //DEBUGGING
    frontier->_theta = theta;

    // calculate cost function
    return w1 * d_g + w2 * d_gbe + w3 * d_r + w4 * theta;
 
}

std::vector<frontier_t> ExplorationPlanner::getFrontierList() {
    return frontiers;
}

void ExplorationPlanner::addDistance(double x1, double y1, double x2, double y2, double distance) {
    std::vector<double> d;
    d.push_back(x1);
    d.push_back(y1);
    d.push_back(x2);
    d.push_back(y2);
    d.push_back(distance);
    distance_list.push_back(d);
}

void ExplorationPlanner::setRobotPosition(double x, double y) {
    robot_last_x = robot_x;
    robot_x = x;
    robot_last_y = robot_y;
    robot_y = y;
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

void ExplorationPlanner::updateDistances(double max_available_distance, bool use_heuristic) {
    for(unsigned int i=0; i< ds_list.size(); i++)
        ds_list.at(i).has_EOs = false;

    for(unsigned int frontier_index = frontiers.size() - 1; frontier_index >= 0; frontier_index--) { //start from the bottom not to penalize the newest frontiers
        for(unsigned int ds_index=0; ds_index < ds_list.size(); ds_index++) {
            acquire_mutex(&mutex_erase_frontier, __FUNCTION__);
            
            // in case meanwhile some frontiers have been deleted
            if(frontier_index >= frontiers.size()) {
                release_mutex(&mutex_erase_frontier, __FUNCTION__);
                return;
            }
            
            while(frontiers.at(frontier_index).list_distance_from_ds.size() < ds_list.size())
                frontiers.at(frontier_index).list_distance_from_ds.push_back(-1);

            if(frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) > 0 && use_heuristic)
                ;
            else if(frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) > 0 && frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) < max_available_distance)
                ;
            else {
                double distance;
                distance = euclidean_distance(ds_list.at(ds_index).x, ds_list.at(ds_index).y, frontiers.at(frontier_index).x_coordinate, frontiers.at(frontier_index).y_coordinate);
                if(distance*2 > max_available_distance)
                    frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) = distance;
                else {
                     distance = trajectory_plan_meters(ds_list.at(ds_index).x, ds_list.at(ds_index).y, frontiers.at(frontier_index).x_coordinate, frontiers.at(frontier_index).y_coordinate);
                    if(distance < 0)
                        ;
                    else
                        frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) = distance;
                }
            }

            if (frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) > 0 && (frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) * 2) < max_available_distance)
                ds_list.at(ds_index).has_EOs = true;

            release_mutex(&mutex_erase_frontier, __FUNCTION__);
        }
    }
    
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
