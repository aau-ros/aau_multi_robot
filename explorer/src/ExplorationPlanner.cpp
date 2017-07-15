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
#define MAX_GOAL_RANGE 3.0 //0.7          // min distance between frontiers during search [meters]
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

//TODO
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wformat"

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int dist[], unsigned long size, bool sptSet[])
{
   // Initialize min value
   int min = INT_MAX, min_index = -1;
  
   for (int v = 0; v < size; v++)
     if (sptSet[v] == false && dist[v] <= min)
         min = dist[v], min_index = v;
  
   return min_index;
}
  
// A utility function to print the constructed distance array
void printSolution(int dist[], unsigned long size)
{
   printf("Vertex   Distance from Source\n");
   for (int i = 0; i < size; i++)
      printf("%d \t\t %d\n", i, dist[i]);
}
  
// Funtion that implements Dijkstra's shortest path algorithm
// from a given source node to a given destination node
// for a graph represented using adjacency matrix representation
float dijkstra(std::vector <std::vector<float> > graph, int src, int dest)
{   
    unsigned long V = graph.size();
     int dist[V];     // The output array.  dist[i] will hold the shortest
                      // distance from src to i
  
     bool sptSet[V]; // sptSet[i] will true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized
  
     // Initialize all distances as INFINITE and stpSet[] as false
     for (int i = 0; i < V; i++)
        dist[i] = INT_MAX, sptSet[i] = false;
  
     // Distance of source vertex from itself is always 0
     dist[src] = 0;
  
     // Find shortest path for all vertices
     for (int count = 0; count < V-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
       int u = minDistance(dist, V, sptSet);
       
       if(u < 0)
         ROS_ERROR("Unconnected components!!!");
  
       // Mark the picked vertex as processed
       sptSet[u] = true;
  
       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < V; v++) {
  
         // Update dist[v] only if is not in sptSet, there is an edge from 
         // u to v, and total weight of path from src to  v through u is 
         // smaller than current value of dist[v]
         if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX 
                                       && dist[u]+graph[u][v] < dist[v])
            dist[v] = dist[u] + graph[u][v];
            
         if(u == dest)
            break;
       
       }
     }
  
     // print the constructed distance array
     //printSolution(dist, V);
     return (float) dist[dest];
}

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

    my_selected_frontier = new frontier_t;
    winner_of_auction = true;
    my_error_counter = 0;
    optimal_ds_x = 0;
    optimal_ds_y = 0;
    num_ds = -1;
    recompute_ds_graph = false;
    optimal_ds_set = false;
    retrying_searching_frontiers = 0;
    received_scan = false;
    errors = 0;
    optimal_ds_set = false;
    robot_x = 0, robot_y = 0;

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
        /*NO SIMULATION*/
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
    pub_visited_frontiers = nh_visited_frontier.advertise <adhoc_communication::ExpFrontier> ("visited_frontiers", 10000);

    pub_visited_frontiers_points = nh_visited_Point.advertise <visualization_msgs::MarkerArray> ("visitedfrontierPoints", 2000, true);

    pub_Point = nh_Point.advertise < geometry_msgs::PointStamped> ("goalPoint", 100, true);
    pub_frontiers_points = nh_frontiers_points.advertise <visualization_msgs::MarkerArray> ("frontierPoints", 2000, true);

//    pub_auctioning_status = nh_auction_status.advertise<adhoc_communication::AuctionStatus> ("auctionStatus", 1000);
//    pub_auctioning_first = nh_auction_first.advertise<adhoc_communication::Auction> ("auction_first", 1000);

    pub_clusters = nh_cluster.advertise<geometry_msgs::PolygonStamped>("clusters", 2000, true);

    pub_cluster_grid_0 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_0", 2000, true);
    pub_cluster_grid_1 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_1", 2000, true);
    pub_cluster_grid_2 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_2", 2000, true);
    pub_cluster_grid_3 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_3", 2000, true);
    pub_cluster_grid_4 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_4", 2000, true);
    pub_cluster_grid_5 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_5", 2000, true);
    pub_cluster_grid_6 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_6", 2000, true);
    pub_cluster_grid_7 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_7", 2000, true);
    pub_cluster_grid_8 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_8", 2000, true);
    pub_cluster_grid_9 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_9", 2000, true);
    pub_cluster_grid_10 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_10", 2000, true);
    pub_cluster_grid_11 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_11", 2000, true);
    pub_cluster_grid_12 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_12", 2000, true);
    pub_cluster_grid_13 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_13", 2000, true);
    pub_cluster_grid_14 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_14", 2000, true);
    pub_cluster_grid_15 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_15", 2000, true);
    pub_cluster_grid_16 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_16", 2000, true);
    pub_cluster_grid_17 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_17", 2000, true);
    pub_cluster_grid_18 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_18", 2000, true);
    pub_cluster_grid_19 = nh_cluster_grid.advertise<nav_msgs::GridCells>("cluster_grid_19", 2000, true);

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
    
}

void ExplorationPlanner::this_robot_callback(const adhoc_communication::EmRobot::ConstPtr &msg) {
    if(!received_robot_info) {
        robot_home_world_x = msg.get()->home_world_x;
        robot_home_world_y = msg.get()->home_world_y;
    }
    received_robot_info = true;
}

void ExplorationPlanner::Callbacks()
{
    ros::Rate r(10);
    while(ros::ok())
    {
//        publish_subscribe_mutex.lock();

        if(robot_name == 1) //TODO
        {
                sub_frontiers = nh_frontier.subscribe("/robot_0/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
                sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_0/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
                sub_negotioation = nh_negotiation.subscribe("/robot_0/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);
                sub_auctioning = nh_auction.subscribe("/robot_1/auction", 1000, &ExplorationPlanner::auctionCallback, this);

//                sub_frontiers = nh_frontier.subscribe("/robot_2/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
//                sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_2/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
//                sub_negotioation = nh_negotiation.subscribe("/robot_2/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);
//
//                sub_frontiers = nh_frontier.subscribe("/robot_3/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
//                sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_3/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
//                sub_negotioation = nh_negotiation.subscribe("/robot_3/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);

        }
        else if(robot_name == 0)
        {
                sub_frontiers = nh_frontier.subscribe("/robot_1/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
                sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_1/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
                sub_negotioation = nh_negotiation.subscribe("/robot_1/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);
                sub_auctioning = nh_auction.subscribe("/robot_0/auction", 1000, &ExplorationPlanner::auctionCallback, this);
//                sub_frontiers = nh_frontier.subscribe("/robot_2/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
//                sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_2/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
//                sub_negotioation = nh_negotiation.subscribe("/robot_2/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);
//
//                sub_frontiers = nh_frontier.subscribe("/robot_3/frontiers", 10000, &ExplorationPlanner::frontierCallback, this);
//                sub_visited_frontiers = nh_visited_frontier.subscribe("/robot_3/visited_frontiers", 10000, &ExplorationPlanner::visited_frontierCallback, this);
//                sub_negotioation = nh_negotiation.subscribe("/robot_3/negotiation_list", 10000, &ExplorationPlanner::negotiationCallback, this);

        }
//        publish_subscribe_mutex.unlock();

        r.sleep();
//        ros::spinOnce();
    }
}

void ExplorationPlanner::initialize_planner(std::string name,
		costmap_2d::Costmap2DROS *costmap, costmap_2d::Costmap2DROS *costmap_global) {

    ROS_INFO("Initializing the planner");

	//copy the pointed costmap to be available in ExplorationPlanner

	this->costmap_ros_ = costmap;
        this->costmap_global_ros_ = costmap_global;

        if(initialized_planner == false)
        {
                nav.initialize("navigation_path", costmap_global_ros_);
                initialized_planner = true;
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

void ExplorationPlanner::clusterFrontiers()
{

//    ROS_INFO("Clustering frontiers");

    int strategy = 1;
    /*
     * Strategy:
     * 1 ... merge clusters close together
     * 2 ... merge clusters based on model
     */

    bool cluster_found_flag = false, same_id = false;
    
    //store_frontier_mutex.lock();
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);

    for(unsigned int i = 0; i < frontiers.size(); i++)
    {
        ROS_DEBUG("Frontier at: %u   and cluster size: %u",i, clusters.size());
        
        //F
        /* If the frontier has been already inserted in a cluster, move to next frontier */
        if(frontiers.at(i).cluster_id >= 0)
            continue;
        
        cluster_found_flag = false;
        bool frontier_used = false;
        same_id = false;

        for(unsigned int j = 0; j < clusters.size(); j++)
        {
            ROS_DEBUG("cluster %u contains %u elements", j, clusters.at(j).cluster_element.size());
            for(unsigned int n = 0; n < clusters.at(j).cluster_element.size(); n++)
            {
               ROS_DEBUG("accessing cluster %u   and element: %d", j, n);

               if(fabs(frontiers.at(i).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate) < MAX_NEIGHBOR_DIST && fabs(frontiers.at(i).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate) < MAX_NEIGHBOR_DIST)
               {
                   for(unsigned int m = 0; m < clusters.at(j).cluster_element.size(); m++)
                   {
                      ROS_DEBUG("checking id %d with element id: %d",frontiers.at(i).id,clusters.at(j).cluster_element.at(m).id);

                      if(robot_prefix_empty_param == true)
                      {
                          if(frontiers.at(i).id == clusters.at(j).cluster_element.at(m).id && frontiers.at(i).detected_by_robot_str.compare(clusters.at(j).cluster_element.at(m).detected_by_robot_str) == 0)
                          {
                              ROS_DEBUG("SAME ID FOUND !!!!!!!");
                              frontier_used = true;
                              same_id = true;
                              break;
                          }
                      }else
                      {
                          if(frontiers.at(i).id == clusters.at(j).cluster_element.at(m).id)
                          {
                              ROS_DEBUG("SAME ID FOUND !!!!!!!");
                              frontier_used = true;
                              same_id = true;
                              break;
                          }
                      }


                   }
                   if(same_id == false)
                   {
                      cluster_found_flag = true;
                   }
               }
               if(same_id == true || cluster_found_flag == true)
               {
                   break;
               }
            }
            if(same_id == true)
            {
                break;
            }else
            {
                if(cluster_found_flag == true)
                {
                    ROS_DEBUG("Frontier: %d attached", frontiers.at(i).id);
                    clusters.at(j).cluster_element.push_back(frontiers.at(i));
                    
                    //F
                    frontiers.at(i).cluster_id = clusters.at(j).id;
                    
                    frontier_used = true;
                    break;
                }
            }
        }
        if(cluster_found_flag == false && same_id == false)
        {
            ROS_DEBUG("ADD CLUSTER");
            cluster_t cluster_new;
            cluster_new.cluster_element.push_back(frontiers.at(i));
            cluster_new.id = (robot_name * 10000) + cluster_id++;
            
            //F
            frontiers.at(i).cluster_id = cluster_new.id;

            cluster_mutex.lock();
            clusters.push_back(cluster_new);
            cluster_mutex.unlock();

            ROS_DEBUG("Frontier: %d in new cluster", frontiers.at(i).id);
            frontier_used = true;
        }
        if(frontier_used == false)
        {
            ROS_WARN("Frontier: %d not used", frontiers.at(i).id);
        }
    }

    release_mutex(&store_frontier_mutex, __FUNCTION__);

    /*
     * To finish the process finally check whether a cluster is close to another one.
     * If so, merge them.
     */
    bool run_without_merging = false;

    if(strategy == 1)
    {
        while(run_without_merging == false)
        {
            bool merge_clusters = false;
            for(unsigned int i = 0; i < clusters.size(); i++)
            {
                for(unsigned int m = 0; m < clusters.at(i).cluster_element.size(); m ++)
                {
                    if(clusters.at(i).cluster_element.size() > 1)
                    {
                        for(unsigned int j = 0; j < clusters.size(); j++)
                        {
                            if(clusters.at(i).id != clusters.at(j).id)
                            {
                                if(clusters.at(j).cluster_element.size() > 1)
                                {
                                    for(unsigned int n = 0; n < clusters.at(j).cluster_element.size(); n++)
                                    {
                                        if(fabs(clusters.at(i).cluster_element.at(m).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate) < CLUSTER_MERGING_DIST && fabs(clusters.at(i).cluster_element.at(m).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate) < CLUSTER_MERGING_DIST)
                                        {
    //                                        ROS_INFO("Merge cluster %d", clusters.at(j).id);
                                            merge_clusters = true;
                                            break;
                                        }
                                    }
                                    if(merge_clusters == true)
                                    {
                                        /*
                                         * Now merge cluster i with cluster j.
                                         * afterwards delete cluster j.
                                         */
                                        for(unsigned int n = 0; n < clusters.at(j).cluster_element.size(); n++)
                                        {
                                            frontier_t frontier_to_merge;
                                            frontier_to_merge = clusters.at(j).cluster_element.at(n);
                                            clusters.at(i).cluster_element.push_back(frontier_to_merge);
                                            
                                            //F
                                            frontier_to_merge.cluster_id = clusters.at(i).id;
                                            
                                        }
    //                                    ROS_INFO("Erasing cluster %d", clusters.at(j).id);
                                        clusters.erase(clusters.begin() + j);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    if(merge_clusters == true)
                        break;
                }
                if(merge_clusters == true)
                    break;
            }
            if(merge_clusters == false)
            {
                run_without_merging = true;
            }
    //        ROS_INFO("RUN WITHOUT MERGING: %d", run_without_merging);
        }
    }
    else if(strategy == 2)
    {
        int MAX_NEIGHBOURS = 4;
        double costmap_resolution = costmap_ros_->getCostmap()->getResolution();

        while(run_without_merging == false)
        {
            bool merge_clusters = false;
            for(unsigned int i = 0; i < clusters.size(); i++)
            {
                for(unsigned int m = 0; m < clusters.at(i).cluster_element.size(); m ++)
                {
                    if(clusters.at(i).cluster_element.size() > 1)
                    {
                        for(unsigned int j = 0; j < clusters.size(); j++)
                        {
                            if(clusters.at(i).id != clusters.at(j).id)
                            {
                                if(clusters.at(j).cluster_element.size() > 1)
                                {
                                    for(unsigned int n = 0; n < clusters.at(j).cluster_element.size(); n++)
                                    {
                                        unsigned char cost;
                                        unsigned int mx,my;
                                        int freespace_detected = 0;
                                        int obstacle_detected  = 0;
                                        int elements_detected = 0;

                                        ROS_INFO("Calculating ...");
                                        int x_length = abs(clusters.at(i).cluster_element.at(m).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate);
                                        int y_length = abs(clusters.at(i).cluster_element.at(m).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate);

                                        ROS_INFO("x_length: %d    y_length: %d   resolution: %f", x_length, y_length,costmap_resolution);
                                        int x_elements = abs(x_length / costmap_resolution);
                                        int y_elements = abs(y_length / costmap_resolution);

                                        ROS_INFO("alpha");
                                        double alpha;
                                        if(x_length == 0)
                                        {
                                            alpha = 0;
                                        }
                                        else
                                        {
                                            alpha = atan(y_length/x_length);
                                        }

                                        ROS_INFO("atan: %f   x_length: %d    y_length: %d", alpha, x_length, y_length);

                                        for(int x = 0; x <= x_elements; x++)
                                        {
                                            /* increase neighbor size*/
                                            for(int neighbour = 0; neighbour < MAX_NEIGHBOURS; neighbour++)
                                            {
                                                int y = tan(alpha) * x * costmap_ros_->getCostmap()->getResolution();
                                                ROS_INFO("x: %d    y: %d", x, y);
                                                if(!costmap_global_ros_->getCostmap()->worldToMap(clusters.at(i).cluster_element.at(m).x_coordinate + x, clusters.at(i).cluster_element.at(m).y_coordinate + y + neighbour,mx,my))
                                                {
                                                    ROS_ERROR("Cannot convert coordinates successfully.");
                                                    continue;
                                                }
                                                
                                                //F
                                                //cost = costmap_global_ros_->getCostmap()->getCost(mx, my);
                                                cost = getCost(costmap_global_ros_, mx, my);

                                                if(cost == costmap_2d::FREE_SPACE)
                                                {
                                                    freespace_detected = true;
                                                }
                                                else if(cost == costmap_2d::LETHAL_OBSTACLE)
                                                {
                                                    obstacle_detected = true;
                                                }
                                                elements_detected++;
                                            }

                                            /* decrease neighbor size*/
                                            for(int neighbour = 0; neighbour < MAX_NEIGHBOURS; neighbour++)
                                            {
                                                int y = tan(alpha) * x * costmap_ros_->getCostmap()->getResolution();
                                                ROS_INFO("x: %d    y: %d", x, y);
                                                if(!costmap_global_ros_->getCostmap()->worldToMap(clusters.at(i).cluster_element.at(m).x_coordinate + x, clusters.at(i).cluster_element.at(m).y_coordinate + y - neighbour,mx,my))
                                                {
                                                    ROS_ERROR("Cannot convert coordinates successfully.");
                                                    continue;
                                                }
                                                
                                                //F
                                                //cost = costmap_global_ros_->getCostmap()->getCost(mx, my);
                                                cost = getCost(costmap_global_ros_, mx, my);

                                                if(cost == costmap_2d::FREE_SPACE)
                                                {
                                                    freespace_detected = true;
                                                }
                                                else if(cost == costmap_2d::LETHAL_OBSTACLE)
                                                {
                                                    obstacle_detected = true;
                                                }
                                                elements_detected++;
                                            }
                                        }


                                        ROS_INFO("*******************************");
                                        ROS_INFO("elements: %d    obstacle: %d", elements_detected, obstacle_detected);
                                        ROS_INFO("*******************************");
                                        if(elements_detected * 0.25 < obstacle_detected)
                                        {
                                            ROS_INFO("Merging clusters");
                                            merge_clusters = true;
                                            break;
                                        }
                                        break;
//                                        if(fabs(clusters.at(i).cluster_element.at(m).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate) < CLUSTER_MERGING_DIST && fabs(clusters.at(i).cluster_element.at(m).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate) < CLUSTER_MERGING_DIST)
//                                        {
//    //                                        ROS_INFO("Merge cluster %d", clusters.at(j).id);
//                                            merge_clusters = true;
//                                            break;
//                                        }
                                    }
                                    if(merge_clusters == true)
                                    {
                                        ROS_INFO("Merging");
                                        /*
                                         * Now merge cluster i with cluster j.
                                         * afterwards delete cluster j.
                                         */
                                        for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
                                        {
                                            frontier_t frontier_to_merge;
                                            frontier_to_merge = clusters.at(j).cluster_element.at(n);
                                            clusters.at(i).cluster_element.push_back(frontier_to_merge);
                                            
                                            //F
                                            frontier_to_merge.cluster_id = clusters.at(i).id;
                                            
                                        }
    //                                    ROS_INFO("Erasing cluster %d", clusters.at(j).id);
                                        clusters.erase(clusters.begin() + j);
                                        j--;
                                        ROS_INFO("Done merging");
                                        break;
                                    }
                                }
                            }
                            break;
                        }
                    }
                    if(merge_clusters == true)
                        break;
                }
                if(merge_clusters == true)
                    break;
            }
            if(merge_clusters == false)
            {
                run_without_merging = true;
            }
    //        ROS_INFO("RUN WITHOUT MERGING: %d", run_without_merging);
        }
    }
    
}

void ExplorationPlanner::visualizeClustersConsole()
{
    ROS_INFO("------------------------------------------------------------------");
    for(int j = 0; j < clusters.size(); j++)
    {
        for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
        {
            if(robot_prefix_empty_param == true)
            {
                ROS_INFO("ID: %6d  x: %5.2f  y: %5.2f  cluster: %5d   robot: %s", clusters.at(j).cluster_element.at(n).id, clusters.at(j).cluster_element.at(n).x_coordinate, clusters.at(j).cluster_element.at(n).y_coordinate, clusters.at(j).id, clusters.at(j).cluster_element.at(n).detected_by_robot_str.c_str());
            }else
            {
                ROS_INFO("ID: %6d  x: %5.2f  y: %5.2f  cluster: %5d   dist: %d", clusters.at(j).cluster_element.at(n).id, clusters.at(j).cluster_element.at(n).x_coordinate, clusters.at(j).cluster_element.at(n).y_coordinate, clusters.at(j).id, clusters.at(j).cluster_element.at(n).dist_to_robot);
            }
        }
    }
    ROS_INFO("------------------------------------------------------------------");
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
 * Compute the length of the trajectory from the robots current position to the first ten frontiers in the frontiers array
 */
void ExplorationPlanner::trajectory_plan_10_frontiers()
{
    for(int i = 0; i<10 && i<frontiers.size(); i++)
    {
        frontiers.at(i).distance_to_robot = trajectory_plan(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
    }
}

/**
 * Compute the length of the trajectory from the robots current position to a given target
 * and store it in the global variable exploration_travel_path_global
 */
void ExplorationPlanner::trajectory_plan_store(double target_x, double target_y)
{
    //F
    //int distance = trajectory_plan(target_x, target_y);
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

/**
 * Compute the length of the trajectory from the robots current position to a given target
 */
//int ExplorationPlanner::reserve_trajectory_plan(double target_x, double target_y)
//{
//    if (!costmap_global_ros_->getRobotPose(robotPose))
//    {
//        ROS_ERROR("Failed to get RobotPose");
//        return -1;
//    }
//    //ROS_ERROR("%f", costmap_global_ros_->getCostmap()->getResolution());
//    return trajectory_plan(robotPose.getOrigin().getX(), robotPose.getOrigin().getY(), target_x, target_y);
//}

/**
 * Compute the length of the trajectory from the robots current position to a given target
 */
int ExplorationPlanner::trajectory_plan(double target_x, double target_y)
{
    if (!costmap_global_ros_->getRobotPose(robotPose))
    {
        ROS_ERROR("Failed to get RobotPose");
        return -1;
    }
    //ROS_ERROR("%f", costmap_global_ros_->getCostmap()->getResolution());
    return trajectory_plan(robotPose.getOrigin().getX(), robotPose.getOrigin().getY(), target_x, target_y);
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
 * Compute the length of the trajectory from a given start to a given target in number of grid cells
 */
int ExplorationPlanner::trajectory_plan(double start_x, double start_y, double target_x, double target_y)
{
    geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;
    int distance;

    std::vector<double> backoffGoal;
    bool backoff_flag = smartGoalBackoff(target_x,target_y, costmap_global_ros_, &backoffGoal);

    startPointSimulated.header.seq = start_point_simulated_message++;	// increase the sequence number
    startPointSimulated.header.stamp = ros::Time::now();
    startPointSimulated.header.frame_id = move_base_frame;
    startPointSimulated.pose.position.x = start_x;
    startPointSimulated.pose.position.y = start_y;
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
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_global_ros_->getCostmap()->getMutex()));
    bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> unlock(*(costmap_global_ros_->getCostmap()->getMutex()));
    //release_mutex(&costmap_mutex, __FUNCTION__);    
    
    if(successful == true)
    {
        //ROS_ERROR("Path from (%f, %f) to (%f, %f)", startPointSimulated.pose.position.x, startPointSimulated.pose.position.y, goalPointSimulated.pose.position.x, goalPointSimulated.pose.position.y);
        distance =  global_plan.size();
        //for(int i=0; i < global_plan.size(); i++)
            //ROS_ERROR("(%f, %f)", global_plan[i].pose.position.x, global_plan[i].pose.position.y);
        
        /*
        distance = 0;
        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
        geometry_msgs::PoseStamped prev_point = (*it);
        it++;
        for(; it != global_plan.end(); it++) {
            distance = sqrt( (prev_point.pose.position.x - (*it).pose.position.x) * (prev_point.pose.position.x - (*it).pose.position.x) + (prev_point.pose.position.y - (*it).pose.position.y) * (prev_point.pose.position.y - (*it).pose.position.y) );
            prev_point = (*it);
        }
        */
        
        
        return distance;
    }
    else
    {
        //ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        return -1;
    }
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
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> unlock(*(costmap_global_ros_->getCostmap()->getMutex()));
//    ROS_INFO("path computed");
    //release_mutex(&costmap_mutex, __FUNCTION__);
    
    //ROS_ERROR("%d", successful);
    //ros::Duration(2).sleep();
    
    if(successful == true)
    {
        //ROS_ERROR("Path from (%f, %f) to (%f, %f)", startPointSimulated.pose.position.x, startPointSimulated.pose.position.y, goalPointSimulated.pose.position.x, goalPointSimulated.pose.position.y);
        //distance =  global_plan.size();
        //for(int i=0; i < global_plan.size(); i++)
            //ROS_ERROR("(%f, %f)", global_plan[i].pose.position.x, global_plan[i].pose.position.y);
        
        distance = 0;
        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
        geometry_msgs::PoseStamped prev_point = (*it);
        it++;
        for(; it != global_plan.end(); it++) {
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

//double ExplorationPlanner::trajectory_plan_print(double start_x, double start_y, double target_x, double target_y)
//{
//    geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;
//    double distance;

//    std::vector<double> backoffGoal;
//    bool backoff_flag = smartGoalBackoff(target_x,target_y, costmap_global_ros_, &backoffGoal);

//    startPointSimulated.header.seq = start_point_simulated_message++;	// increase the sequence number
//    startPointSimulated.header.stamp = ros::Time::now();
//    startPointSimulated.header.frame_id = move_base_frame;
//    startPointSimulated.pose.position.x = start_x;
//    startPointSimulated.pose.position.y = start_y;
//    startPointSimulated.pose.position.z = 0;
//    startPointSimulated.pose.orientation.x = 0;
//    startPointSimulated.pose.orientation.y = 0;
//    startPointSimulated.pose.orientation.z = 0;
//    startPointSimulated.pose.orientation.w = 1;

//    goalPointSimulated.header.seq = goal_point_simulated_message++;	// increase the sequence number
//    goalPointSimulated.header.stamp = ros::Time::now();
//    goalPointSimulated.header.frame_id = move_base_frame;
//    if(backoff_flag == true)
//    {
//        goalPointSimulated.pose.position.x = backoffGoal.at(0);
//        goalPointSimulated.pose.position.y = backoffGoal.at(1);
//    }
//    else
//    {
//        goalPointSimulated.pose.position.x = target_x;
//        goalPointSimulated.pose.position.y = target_y;
//    }
//    goalPointSimulated.pose.position.z = 0;
//    goalPointSimulated.pose.orientation.x = 0;
//    goalPointSimulated.pose.orientation.y = 0;
//    goalPointSimulated.pose.orientation.z = 0;
//    goalPointSimulated.pose.orientation.w = 1;

//    std::vector<geometry_msgs::PoseStamped> global_plan;

//    //acquire_mutex(&costmap_mutex, __FUNCTION__);
//    bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);
//    //release_mutex(&costmap_mutex, __FUNCTION__);
//    
//    //ROS_ERROR("%d", successful);
//    //ros::Duration(2).sleep();
//    
//    if(successful == true)
//    {
//        //ROS_ERROR("Path from (%f, %f) to (%f, %f)", startPointSimulated.pose.position.x, startPointSimulated.pose.position.y, goalPointSimulated.pose.position.x, goalPointSimulated.pose.position.y);
//        //distance =  global_plan.size();
//        for(int i=0; i < global_plan.size(); i++)
//            ROS_ERROR("(%f, %f)", global_plan[i].pose.position.x, global_plan[i].pose.position.y);
//        
//        distance = 0;
//        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
//        geometry_msgs::PoseStamped prev_point = (*it);
//        it++;
//        for(; it != global_plan.end(); it++) {
//            distance += sqrt( (prev_point.pose.position.x - (*it).pose.position.x) * (prev_point.pose.position.x - (*it).pose.position.x) + (prev_point.pose.position.y - (*it).pose.position.y) * (prev_point.pose.position.y - (*it).pose.position.y) ); //* costmap_global_ros_->getCostmap()->getResolution();
//            prev_point = (*it);
//        }
//        
//        //ROS_ERROR("%f", distance);
//        return distance;
//    }
//    else
//    {
//        //ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//        return -1;
//    }
//}

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
                frontiers.erase(frontiers.begin()+i);
//                if(i > 0)
//                {
//                    i --;
//                }
                release_mutex(&store_frontier_mutex, __FUNCTION__);
                //break; //FIXME ... only a test
            }
        }else
        {
            if(frontiers.at(i).id == id)
            {
            
                //store_frontier_mutex.lock();
                acquire_mutex(&store_frontier_mutex, __FUNCTION__);
                frontiers.erase(frontiers.begin()+i);
                if(i > 0)
                {
                    i --;
                }
                release_mutex(&store_frontier_mutex, __FUNCTION__);
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

//bool ExplorationPlanner::removeUnreachableFrontier(int id, std::string detected_by_robot_str)
//{
//    for(int i= 0; i< unreachable_frontiers.size(); i++)
//    {
//        if(robot_prefix_empty_param == true)
//        {
//            if(unreachable_frontiers.at(i).id == id && unreachable_frontiers.at(i).detected_by_robot_str.compare(detected_by_robot_str) == 0)
//            {
//                ROS_INFO("Removing Unreachable Frontier ID: %d  at position: %d  of Robot: %s", unreachable_frontiers.at(i).id, i, unreachable_frontiers.at(i).detected_by_robot_str.c_str());

//                unreachable_frontiers.erase(unreachable_frontiers.begin()+i);
////                if(i > 0)
////                {
////                    i --;
////                }
//                break;
//            }
//        }else
//        {
//            if(unreachable_frontiers.at(i).id == id)
//            {
//                unreachable_frontiers.erase(unreachable_frontiers.begin()+i);
//                if(i > 0)
//                {
//                    i --;
//                }
//                break;
//            }
//        }
//    }

//    return true;
//}

bool ExplorationPlanner::publish_negotiation_list(frontier_t negotiation_frontier, int cluster_number)
{
//    ROS_ERROR("Publish negotiation list!!!!!!!!!!!!!!!!");
    adhoc_communication::ExpFrontier negotiation_msg;

//    for(int i = 0; i<negotiation_list.size(); i++)
//    {
//        adhoc_communication::FrontierElement negotiation_element;
//        negotiation_element.detected_by_robot = negotiation_list.at(i).detected_by_robot;
//        negotiation_element.x_coordinate = negotiation_list.at(i).x_coordinate;
//        negotiation_element.y_coordinate = negotiation_list.at(i).y_coordinate;
//        negotiation_element.id = negotiation_list.at(i).id;
//
//        negotiation_msg.frontier_element.push_back(negotiation_element);
////        pub_negotion.publish(negotiation_msg);
//        sendToMulticast("mc_",negotiation_msg, "negotiation_list");
//    }

    if(cluster_number != -1)
    {
        int cluster_vector_position = 0;
        for (int i = 0; i < clusters.size(); i++)
        {
            if(clusters.at(i).id == cluster_number)
            {
                ROS_DEBUG("Cluster ID: %d   is at vector position: %d", cluster_number, i);
                cluster_vector_position = i;
                break;
            }
        }

        for(int i = 0; i < clusters.at(cluster_vector_position).cluster_element.size(); i++)
        {
            adhoc_communication::ExpFrontierElement negotiation_element;
            negotiation_element.x_coordinate = clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate;
            negotiation_element.y_coordinate = clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate;
            negotiation_element.detected_by_robot = clusters.at(cluster_vector_position).cluster_element.at(i).detected_by_robot;
            negotiation_element.id = clusters.at(cluster_vector_position).cluster_element.at(i).id;

            frontier_t new_frontier;
            new_frontier.x_coordinate = clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate;
            new_frontier.y_coordinate = clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate;
            new_frontier.detected_by_robot = clusters.at(cluster_vector_position).cluster_element.at(i).detected_by_robot;
            new_frontier.id = clusters.at(cluster_vector_position).cluster_element.at(i).id;

            negotiation_msg.frontier_element.push_back(negotiation_element);
            my_negotiation_list.push_back(new_frontier);
        }
//        return true;
    }else
    {
        adhoc_communication::ExpFrontierElement negotiation_element;
        negotiation_element.detected_by_robot = negotiation_frontier.detected_by_robot;
        negotiation_element.x_coordinate = negotiation_frontier.x_coordinate;
        negotiation_element.y_coordinate = negotiation_frontier.y_coordinate;
        negotiation_element.id = negotiation_frontier.id;

        negotiation_msg.frontier_element.push_back(negotiation_element); //FIXME
    }

    sendToMulticast("mc_",negotiation_msg, "negotiation_list");

    first_negotiation_run = false;
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

void ExplorationPlanner::negotiationCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{
//        ROS_ERROR("Negotiation List!!!");

        bool entry_found = false;

        adhoc_communication::ExpFrontierElement frontier_element;
        for(int j = 0; j < msg.get()->frontier_element.size(); j++)
        {
            frontier_element = msg.get()->frontier_element.at(j);
            for(int i = 0; i < negotiation_list.size(); i++)
            {
                if(robot_prefix_empty_param == true)
                {
                    if(negotiation_list.at(i).id == frontier_element.id && negotiation_list.at(i).detected_by_robot_str.compare(frontier_element.detected_by_robot_str) == 0)
                    {
                        entry_found = true;
                    }
                }else
                {
                    if(negotiation_list.at(i).id == frontier_element.id)
                    {
                        entry_found = true;
                    }
                }

            }
            if(entry_found == false)
            {
                ROS_DEBUG("Negotiation frontier with ID: %d", frontier_element.id);
                frontier_t negotiation_frontier;
                negotiation_frontier.detected_by_robot = frontier_element.detected_by_robot;
                negotiation_frontier.id = frontier_element.id;
                negotiation_frontier.x_coordinate = frontier_element.x_coordinate;
                negotiation_frontier.y_coordinate = frontier_element.y_coordinate;

                store_negotiation_mutex.lock();
                negotiation_list.push_back(negotiation_frontier);
                store_negotiation_mutex.unlock();
            }
        }
}

bool ExplorationPlanner::respondToAuction(std::vector<requested_cluster_t> requested_cluster_ids, int auction_id_number)
{
//    ros::Rate r(1);
//    while(ros::ok())
//    {
//        r.sleep();
////        while(start_thr_auction == false);
//        if(start_thr_auction == true)
//        {
            ROS_INFO("Respond to auction");
            adhoc_communication::ExpAuction auction_msgs;

            for(int i = 0; i < requested_cluster_ids.size(); i++)
            {
                ROS_INFO("Responding to cluster ids: %d", requested_cluster_ids.at(i).own_cluster_id);
            }

            for(int n = 0; n < requested_cluster_ids.size(); n++)
            {
                cluster_mutex.lock();
                for(int i = 0; i < clusters.size(); i++)
                {
                    if(clusters.at(i).id == requested_cluster_ids.at(n).own_cluster_id)
                    {
                        adhoc_communication::ExpCluster cluster_msg;
                        adhoc_communication::ExpClusterElement cluster_element_msg;
                        for(int j = 0; j < requested_cluster_ids.at(n).requested_ids.size(); j++)
                        {
                            if(robot_prefix_empty_param == true)
                            {
                                cluster_element_msg.id = requested_cluster_ids.at(n).requested_ids.at(j).id;
                                cluster_element_msg.detected_by_robot_str = requested_cluster_ids.at(n).requested_ids.at(j).robot_str;
                            }else
                            {
                                cluster_element_msg.id = requested_cluster_ids.at(n).requested_ids.at(j).id;
                            }
                            cluster_msg.ids_contained.push_back(cluster_element_msg);
                        }
//                        ROS_INFO("Calculate the auction BID");
                        cluster_msg.bid = calculateAuctionBID(clusters.at(i).id, trajectory_strategy);
                        auction_msgs.available_clusters.push_back(cluster_msg);
                        break;
                    }
                }
                cluster_mutex.unlock();
            }

            /*
             * Visualize the auction message to send
             */
//            for(int i = 0; i < auction_msgs.available_clusters.size(); i++)
//            {
//                adhoc_communication::Cluster cluster_msg_check;
//                cluster_msg_check = auction_msgs.available_clusters.at(i);
//                ROS_INFO("Robot %d sending BID: %f cluster elements: %u", robot_name, cluster_msg_check.bid, cluster_msg_check.ids_contained.size());
//            }

            ROS_INFO("Robot %d publishes auction bids for all clusters", robot_name);

//            /* FIXME */
//            if(auction_msgs.available_clusters.size() == 0)
//            {
//                adhoc_communication::Cluster cluster_msg;
//                cluster_msg.bid = -1;
//                auction_msgs.available_clusters.push_back(cluster_msg);
//            }
//

            auction_msgs.auction_status_message = false;
            auction_msgs.auction_id = auction_id_number;

            std::stringstream ss;
            ss << robot_name;
            std::string prefix = "";
            std::string robo_name = prefix.append(ss.str());

            auction_msgs.robot_name = robo_name;

//            if(first_run == true)
//            {
//                pub_auctioning_first.publish(auction_msgs);
//            }else
//            {
                sendToMulticastAuction("mc_", auction_msgs, "auction");
//            }

            start_thr_auction = false;
//        }
//    }
}

int ExplorationPlanner::calculateAuctionBID(int cluster_number, std::string strategy)
{
//    ROS_INFO("Robot %d  calculates bid for cluster_number %d", robot_name, cluster_number);
    int auction_bid = 0;
    int cluster_vector_position = -1;
    bool cluster_could_be_found = false;

    if(clusters.size() > 0)
    {
        for (int i = 0; i < clusters.size(); i++)
        {
            if(clusters.at(i).id == cluster_number)
            {
//                if(clusters.at(i).cluster_element.size() > 0)
//                {
                    cluster_vector_position = i;
                    cluster_could_be_found = true;
                    break;
//                }else
//                {
//                    ROS_ERROR("Cluster found but empty, therefore do not calculate LOCAL BID");
//                    return(-1);
//                }
            }
        }
    }
    if(cluster_could_be_found == false)
    {
        /*
         * Cluster could not be found, set it to a high value like 100
         */
        ROS_WARN("Cluster could not be found");
        return(-1);
    }

    if (!costmap_global_ros_->getRobotPose(robotPose))
    {
            ROS_ERROR("Failed to get RobotPose");
    }

    int distance = -1;
    for(int i = 0; i < clusters.at(cluster_vector_position).cluster_element.size(); i++)
    {

        if(strategy == "trajectory")
        {
            distance = trajectory_plan(clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate, clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate);

        }else if(strategy == "euclidean")
        {
            double x = clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate - robotPose.getOrigin().getX();
            double y = clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate - robotPose.getOrigin().getY();
            distance = x * x + y * y;
            return distance;
        }

        if(distance > 0)
        {
            double x = clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate - robotPose.getOrigin().getX();
            double y = clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate - robotPose.getOrigin().getY();
            double euclidean_distance = x * x + y * y;

            /*
             * Check if the distance calculation is plausible.
             * The euclidean distance to this point need to be smaller then
             * the simulated trajectory path. If this stattement is not valid
             * the trajectory calculation has failed.
             */
//            ROS_INFO("Euclidean distance: %f   trajectory_path: %f", sqrt(euclidean_distance), distance* costmap_ros_->getCostmap()->getResolution());
            if (distance * costmap_ros_->getCostmap()->getResolution() <= sqrt(euclidean_distance)*0.95)
            {
                ROS_WARN("Euclidean distance smaller then trajectory distance to LOCAL CLUSTER!!!");
//                return(-1);
            }else
            {
                return distance;
            }
        }
    }
    if(distance == -1)
    {
//        ROS_ERROR("Unable to calculate LOCAL BID at position %d  --> BID: %d", cluster_vector_position, distance);
    }
//    ROS_INFO("Cluster at position %d  --> BID: %d", cluster_vector_position, distance);
    return(-1);
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

void ExplorationPlanner::auctionCallback(const adhoc_communication::ExpAuction::ConstPtr& msg)
{
    auction_running = true;
    //ROS_ERROR("CALLING AUCTION CALLBACK!!!!!!!!!!!!");
    int robots_int_name = -1;

    bool same_robot = false;
    if(robot_prefix_empty_param == true)
    {
        if(robot_str.compare(msg.get()->robot_name.c_str()) == 0)
        {
            same_robot = true;
        }
    }else
    {
        robots_int_name = atoi(msg.get()->robot_name.c_str());
        ROS_INFO("Robot name: %d      int_name: %d",robot_name, robots_int_name);
        if(robots_int_name == robot_name)
        {
            same_robot = true;
        }
    }

    if(same_robot == false)
    {
        /*
        * Cluster all available frontiers to be able to compare clusters
        * with other robots
        */

        /*
         * TODO
         * Check if following code is necessary or if simple navigation needs to
         * periodically update the frontiers in the frontier thread.
         */
        transformToOwnCoordinates_frontiers();
        transformToOwnCoordinates_visited_frontiers();

        clearVisitedFrontiers();
        clearUnreachableFrontiers();
        clearSeenFrontiers(costmap_global_ros_);

        clearVisitedAndSeenFrontiersFromClusters();
        clusterFrontiers();


//        visualize_Cluster_Cells();

        if(msg.get()->auction_status_message == true)
        {
            ROS_INFO("Calling Auction Status");

            /*
             * Visualize requested ids
             */

            for(unsigned int i = 0; i < msg.get()->requested_clusters.size(); i++)
            {
                adhoc_communication::ExpCluster cluster_req;
                cluster_req = msg.get()->requested_clusters.at(i);
                std::string requested_clusters;
                for(unsigned int j = 0; j < cluster_req.ids_contained.size(); j++)
                {
                    if(j >= 6)
                    {
                        break;
                    }
                    requested_clusters.append(NumberToString((int)cluster_req.ids_contained.at(j).id));
                    requested_clusters.append(", ");
                }
                ROS_INFO("Requested ids: %s from robot: %s", requested_clusters.c_str(), msg.get()->robot_name.c_str());
            }



            auction_start = msg.get()->start_auction;
            auction_finished = msg.get()->auction_finished;
            adhoc_communication::ExpCluster occupied_ids;
            std::vector<requested_cluster_t> requested_cluster_ids;


            /*
             * Grep all occupied ids and try to convert them into clusters in own
             * coordinate system. This ensures that all robots in the system know
             * which clusters had been occupied by others and do not select them
             * twice.
             */
            if(msg.get()->occupied_ids.size() > 0 || msg.get()->requested_clusters.size() > 0)
            {
                for(int i = 0; i < msg.get()->occupied_ids.size(); i++)
                {
                    adhoc_communication::ExpClusterElement cluster_element;

                    cluster_element.id = msg.get()->occupied_ids.at(i).id;
                    cluster_element.detected_by_robot_str = msg.get()->occupied_ids.at(i).detected_by_robot_str;

                    occupied_ids.ids_contained.push_back(cluster_element);
                }
                int occupied_cluster_id = checkClustersID(occupied_ids);
                ROS_INFO("Check occupied cluster to be the same. %d", occupied_cluster_id);

                if(occupied_cluster_id >=0)
                {
                    ROS_INFO("Adding occupied cluster: %d", occupied_cluster_id);
                    already_used_ids.push_back(occupied_cluster_id);
                }else
                {
                    /* Store undetected Clusters in a struct for later processing */
                    ROS_INFO("Adding occupied cluster as unrecognized!");
                    unrecognized_occupied_clusters.push_back(occupied_ids);
                }
                /*
                 * Now read from unrecognized cluster vector and try to convert
                 * these ids to a valid cluster to know which had already been
                 * occupied.
                 */
                for(unsigned int i = 0; i < unrecognized_occupied_clusters.size(); i++)
                {
                    int unrecognized_occupied_cluster_id = checkClustersID(unrecognized_occupied_clusters.at(i));
                    if(unrecognized_occupied_cluster_id >=0)
                    {
                        ROS_INFO("Unrecognized cluster: %d converted", unrecognized_occupied_cluster_id);
                        already_used_ids.push_back(unrecognized_occupied_cluster_id);
                        unrecognized_occupied_clusters.erase(unrecognized_occupied_clusters.begin() + i);

                        if(i > 0)
                            i--;
                    }
                }


                /*
                 * Grep the requested clusters to know which ones to answer to.
                 */
                for(unsigned int i = 0; i < msg.get()->requested_clusters.size(); i++)
                {
                    adhoc_communication::ExpCluster requested_cluster;
                    requested_cluster = msg.get()->requested_clusters.at(i);

                    int check_cluster_id = checkClustersID(requested_cluster);
                    if(check_cluster_id >= 0)
                    {
                        requested_cluster_t new_cluster_request;
                        for(unsigned int j = 0; j < requested_cluster.ids_contained.size(); j++)
                        {
                            transform_point_t cluster_element_point;
                            if(robot_prefix_empty_param == true)
                            {
                                cluster_element_point.id = requested_cluster.ids_contained.at(j).id;
                                cluster_element_point.robot_str = requested_cluster.ids_contained.at(j).detected_by_robot_str;
                            }else
                            {
                                cluster_element_point.id = requested_cluster.ids_contained.at(j).id;
                            }

                            new_cluster_request.requested_ids.push_back(cluster_element_point);
                        }
                        new_cluster_request.own_cluster_id = check_cluster_id;

                        requested_cluster_ids.push_back(new_cluster_request);
                    }else
                    {
                        ROS_WARN("No Matching Cluster Detected");
                    }
                }
            }


            if(auction_start == true)
            {
                start_thr_auction = true;
                thr_auction_status = boost::thread(&ExplorationPlanner::respondToAuction, this, requested_cluster_ids, msg.get()->auction_id);
            }

        }else if(msg.get()->auction_status_message == false)
        {
            bool continue_auction = false;
            if(robot_prefix_empty_param == true)
            {
                if(msg.get()->auction_id == auction_id_number)
                {
                    continue_auction = true;
                }
            }else
            {
                if(msg.get()->auction_id == 10000*robot_name + auction_id_number)
                {
                    continue_auction = true;
                }
            }


            if(continue_auction == true)
            {
    //            ROS_INFO("auction_id: %d       local_id: %d", msg.get()->auction_id, 10000*robot_name + auction_id_number);
                bool robot_already_answered = false;

                for(unsigned int i = 0; i < robots_already_responded.size(); i++)
                {
                    if(robot_prefix_empty_param == true)
                    {
                        if(msg.get()->robot_name.compare(robots_already_responded.at(i).robot_str) == 0 && msg.get()->auction_id == robots_already_responded.at(i).auction_number)
                        {
                            ROS_WARN("Same msg already received!!!");
                            robot_already_answered = true;
                            break;
                        }
                    }else
                    {
                        ROS_INFO("Compare msg name: %d  responded: %d       msg auction: %d   responded: %d", robots_int_name, robots_already_responded.at(i).robot_number, msg.get()->auction_id, robots_already_responded.at(i).auction_number);
                        if(robots_int_name == robots_already_responded.at(i).robot_number && msg.get()->auction_id == robots_already_responded.at(i).auction_number)
                        {
                            ROS_WARN("Same msg already received!!!");
                            robot_already_answered = true;
                            break;
                        }
                    }
                }

                /*
                 * Only proceed if the robot has not answered before.
                 */
                if(robot_already_answered == false)
                {
                    if(robot_prefix_empty_param == true)
                    {
                        ROS_INFO("Auction from robot %s received", msg.get()->robot_name.c_str());
                    }else
                    {
                        ROS_INFO("Auction from robot %d received", robot_name);
                    }
                    auction_pair_t auction_pair;
                    auction_element_t auction_elements;

                    int has_cluster_id = -1;

                    /*
                     * Visualize the received message
                     */
                    for(unsigned int i = 0; i < msg.get()->available_clusters.size(); i++)
                    {
                        adhoc_communication::ExpCluster cluster_req;
                        cluster_req = msg.get()->available_clusters.at(i);
                        ROS_INFO("---------------------- %d ----------------------------", i);
                        std::string requested_clusters;
                        for(unsigned int j = 0; j < cluster_req.ids_contained.size(); j++)
                        {
                            if(j >= 6)
                            {
                                break;
                            }
                            requested_clusters.append(NumberToString((int)cluster_req.ids_contained.at(j).id));
                            requested_clusters.append(", ");
                        }
                        ROS_INFO("Received ids: %s", requested_clusters.c_str());
                    }



                    for(unsigned int i = 0; i < msg.get()->available_clusters.size(); i++)
                    {
                        adhoc_communication::ExpCluster current_cluster;
                        current_cluster = msg.get()->available_clusters.at(i);
                //        ROS_INFO("------------------------------------------------------------------");
                //        for(int k = 0; k < current_cluster.ids_contained.size(); k++)
                //        {
                //            ROS_INFO("FRONTIER ID: %d", current_cluster.ids_contained.at(k));
                //        }
                        int current_cluster_id = checkClustersID(current_cluster);

                        ROS_INFO("Received ID converted to cluster ID: %d", current_cluster_id);
                        if(current_cluster_id >= 0)
                        {
                            auction_pair.cluster_id = current_cluster_id;
                            auction_pair.bid_value = current_cluster.bid;
                            auction_elements.auction_element.push_back(auction_pair);
                            ROS_INFO("BID: %f",current_cluster.bid);
                        }
                    }
                //    ROS_INFO("Robot %d received all bids for all clusters", robot_name);
                    auction_elements.robot_id = robots_int_name;
                    auction_elements.detected_by_robot_str = msg.get()->robot_name;

                    auction_mutex.lock();
                    auction.push_back(auction_elements);
                    auction_mutex.unlock();

                    /*
                     * A BID is received from one robot. Remember the robot who send the
                     * bid for a special auction id!
                     */
    //                if(msg.get()->auction_id == 10000*robot_name + auction_id_number)
    //                {
                        number_of_auction_bids_received++;
                        responded_t auction_response;
                        auction_response.auction_number = msg.get()->auction_id;
                        auction_response.robot_number = robots_int_name;
                        if(robot_prefix_empty_param == true)
                        {
                            auction_response.robot_str = msg.get()->robot_name;
                        }
                        robots_already_responded.push_back(auction_response);
    //                }
                }else
                {
                    ROS_WARN("Robot already answered on this auction");
                }
            }
        }
    }
    auction_running = false;
}

int ExplorationPlanner::checkClustersID(adhoc_communication::ExpCluster cluster_to_check)
{
//    ROS_INFO("Check for cluster id");
      std::vector<compare_pair_t> compare_clusters;

//        ROS_INFO("------------------- Checking ----------------------");
        std::string requested_clusters;

//        for(int j = 0; j < cluster_to_check.ids_contained.size(); j++)
//        {
//            requested_clusters.append(NumberToString((int)cluster_to_check.ids_contained.at(j)));
//            requested_clusters.append(", ");
//        }
//        ROS_INFO("%s", requested_clusters.c_str());




    for(unsigned int j = 0; j < clusters.size(); j++)
    {
        double same_id_found = 0;
//        ROS_INFO("--------------------------------------------------------------");
        for(unsigned int n = 0; n < clusters.at(j).cluster_element.size(); n++)
        {
//            for(int m= 0; m < clusters.at(j).cluster_element.size(); m++)
//            {
//                ROS_INFO("Ids in cluster to check with: %d",clusters.at(j).cluster_element.at(m).id);
//            }
//            ROS_INFO("Ids in cluster to check with: %d",clusters.at(j).cluster_element.at(n).id);
            for(unsigned int i = 0; i < cluster_to_check.ids_contained.size(); i++)
            {
                if(robot_prefix_empty_param == true)
                {
//                    ROS_INFO("Cluster id: %d  robot: %s     msg id: %d  robot: %s",clusters.at(j).cluster_element.at(n).id, clusters.at(j).cluster_element.at(n).detected_by_robot_str, cluster_to_check.ids_contained.at(i).id, cluster_to_check.ids_contained.at(i).detected_by_robot_str);
                    if(clusters.at(j).cluster_element.at(n).id == cluster_to_check.ids_contained.at(i).id && clusters.at(j).cluster_element.at(n).detected_by_robot_str.compare(cluster_to_check.ids_contained.at(i).detected_by_robot_str) == 0)
                    {
                        same_id_found++;
                    }
                }else
                {
                    if(clusters.at(j).cluster_element.at(n).id == cluster_to_check.ids_contained.at(i).id)
                    {
                        same_id_found++;
                    }
                }
            }
        }
        if(same_id_found > clusters.at(j).cluster_element.size() * 0.4)
        {
//            ROS_INFO("CLUSTER ID FOUND: %d", clusters.at(j).id);
            return (clusters.at(j).id);
        }
        else
        {
//            ROS_ERROR("NO MATCHING CLUSTER FOUND!!!");
        }
//        ROS_INFO("---------------------------------------------------");

//        ROS_INFO("j: %d   cluster_id: %d", j, clusters.at(j).id);
//        compare_pair_t compare_pair;
//        compare_pair.identical_ids = same_id_found;
//        compare_pair.cluster_id = clusters.at(j).id;
//        compare_clusters.push_back(compare_pair);
//        ROS_INFO("Same IDs found: %f   elements*0.7: %f", same_id_found,clusters.at(j).cluster_element.size() * 0.7);


//            if(same_id_found > clusters.at(j).cluster_element.size() * 0.8)
//            {
//                bool previously_detected = false;
//                for(int m = 0; m < id_previously_detected.size(); m++)
//                {
//                    if(clusters.at(j).id == id_previously_detected.at(m))
//                    {
//                        previously_detected = true;
//                        break;
//                    }
//                }
//                if(previously_detected == false)
//                {
//                    id_previously_detected.push_back(clusters.at(j).id);
//                    return (clusters.at(j).id);
//                }
//            }
    }

    /*
     * Sort compare_pairs and select the one with the highest similarity
     */
//    if(compare_clusters.size() > 0)
//    {
//
//        std::sort(compare_clusters.begin(), compare_clusters.end(), sortCompareElements);
////        for(int i = 0; i < compare_clusters.size(); i++)
////        {
////            ROS_INFO("identical: %d    cluster id: %d", compare_clusters.at(i).identical_ids, compare_clusters.at(i).cluster_id);
////        }
////        ROS_INFO("Selected identical: %d    cluster id: %d", compare_clusters.front().identical_ids, compare_clusters.front().cluster_id);
//        return(compare_clusters.front().cluster_id);
//    }else
//    {
//        ROS_INFO("Compare Cluster empty");
//        return (-1);
//    }
    return (-1);
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

/**
 * Check if the next goal is efficient enough to steer the robot to it.
 * If it is a goal, which has previously be seen, it is not required
 * to visit this goal again.
 * make sure that there are no failures in calculation! Therefore
 * this plausibility check is done. Only values of less then 50m
 * are valid. (Calculation of coordinates in the costmap often
 * produce very big values which are miss-interpretations)
 */
bool ExplorationPlanner::check_efficiency_of_goal(double x, double y)
{
    double diff_home_x = visited_frontiers.at(0).x_coordinate - x;
    double diff_home_y = visited_frontiers.at(0).y_coordinate - y;

    if (fabs(diff_home_x) <= MAX_DISTANCE && fabs(diff_home_y) <= MAX_DISTANCE)
    {
        for (unsigned int i = 1; i < visited_frontiers.size(); i++)
        {
            /*
             * Calculate the distance between all previously seen goals and the new
             * found frontier!!
             */
            double diff_x = visited_frontiers.at(i).x_coordinate - x;
            double diff_y = visited_frontiers.at(i).y_coordinate - y;

            if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE) {
                ROS_DEBUG("x: %f  y: %f too close to visited at x: %f   y: %f   diff_x: %f   diff_y: %f", x, y, visited_frontiers.at(i).x_coordinate, visited_frontiers.at(i).y_coordinate, diff_x, diff_y);
                return false;
            }
        }
        for (unsigned int i = 0; i < unreachable_frontiers.size(); i++)
        {
            /*
             * Calculate the distance between all previously seen goals and the new
             * found frontier!!
             */
            double diff_x = unreachable_frontiers.at(i).x_coordinate - x;
            double diff_y = unreachable_frontiers.at(i).y_coordinate - y;

            if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE) {
                ROS_DEBUG("x: %f  y: %f too close to unreachable at x: %f   y: %f   diff_x: %f   diff_y: %f", x, y, unreachable_frontiers.at(i).x_coordinate, unreachable_frontiers.at(i).y_coordinate, diff_x, diff_y);
                return false;
            }
        }
        return true;
    }
    else
    {
        ROS_WARN("OUT OF HOME RANGE");
        return false;
    }
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
        total_distance_eu = euclidean_distance(x, y, robot_x, robot_y) + euclidean_distance(x, y, 0, 0);
    //ROS_INFO("Euclidean distance to frontier and then home: %.2f",total_distance);
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
    total_distance = trajectory_plan_meters(x, y);
    if(total_distance < 0) {
        // if the distance between robot and target is less than a certain small value, consider the target reachable... this is necessary because sometimes goals too close to the robot are considered unreachable, which is a problem when the robot is starting the exploration, since it very often (almost every time) selects as first goal its starting position
        if(euclidean_distance(x, y, robot_x, robot_y) < 2) 
            distance = euclidean_distance(x, y, robot_x, robot_y); 
        else {
            ROS_WARN("Failed to compute distance! frontier at (%.1f, %.1f)", x, y);
            total_distance = fallback_distance_computation(x, y);
            if(errors == 0)
                my_error_counter++;
            errors++;
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
            if(optimal_ds_set)
                distance = fallback_distance_computation(x, y, robot_home_x, robot_home_y);
            else
                distance = fallback_distance_computation(x, y, optimal_ds_x, optimal_ds_y);
            if(errors == 0)
                my_error_counter++;
            errors++;
        }
    }
    frontier->my_distance_to_optimal_ds = distance;
    
    total_distance += distance;

    ROS_INFO("Distance to frontier and then DS: %.2f",total_distance);
    return available_distance > total_distance;

}

void ExplorationPlanner::clearVisitedAndSeenFrontiersFromClusters()
{
    ROS_INFO("Clear VisitedAndSeenFrontiers from Cluster");
    std::vector<int> goals_to_clear;



    for(unsigned int i = 0; i < clusters.size(); i++)
    {
        for(unsigned int j = 0; j < clusters.at(i).cluster_element.size(); j++)
        {
            /* Now iterate over all frontiers and check if cluster elements are
             * still available in the frontier vector
             */
            bool cluster_still_valid = false;
            for(unsigned int m = 0; m < frontiers.size(); m++)
            {
                if(clusters.at(i).cluster_element.at(j).id == frontiers.at(m).id)
                {
                    /*Cluster is still valid, do not clear it*/
                    cluster_still_valid = true;
                    break;
                }
            }
            if(cluster_still_valid == false)
            {
                clusters.at(i).cluster_element.erase(clusters.at(i).cluster_element.begin() +j);
                if(j > 0)
                   j--;
            }
        }
    }

    for(unsigned int i = 0; i < clusters.size(); i++)
    {
        if(clusters.at(i).cluster_element.size() <= 0)
        {
            clusters.erase(clusters.begin() + i);
        }
    }


//    if(visited_frontiers.size() > 1)
//    {
//        for (int i = 1; i < visited_frontiers.size(); i++)
//        {
//            bool found_flag = false;
//            for (int j = 0; j < clusters.size(); j++)
//            {
//                for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
//                {
//                    double diff_x = visited_frontiers.at(i).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate;
//                    double diff_y = visited_frontiers.at(i).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate;
//
//                    if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE)
//                    {
//                        ROS_DEBUG("Erasing visited frontier %d from cluster", clusters.at(j).cluster_element.at(n).id);
//                        clusters.at(j).cluster_element.erase(clusters.at(j).cluster_element.begin()+n);
//                        if(n > 0)
//                        {
//                            n --;
//                        }
//                        found_flag = true;
//    //                    break;
//                    }
//                }
//    //            if(found_flag == true)
//    //            {
//    //                break;
//    //            }
//            }
//        }
//    }

//    if(seen_frontier_list.size() > 1)
//    {
//        for (int i = 1; i < seen_frontier_list.size(); i++)
//        {
//            bool found_flag = false;
//            for (int j = 0; j < clusters.size(); j++)
//            {
//                for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
//                {
//                    double diff_x = seen_frontier_list.at(i).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate;
//                    double diff_y = seen_frontier_list.at(i).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate;
//
//                    if (fabs(diff_x) <= MAX_GOAL_RANGE*2 && fabs(diff_y) <= MAX_GOAL_RANGE*2)
//                    {
//                        ROS_DEBUG("Erasing seen frontier %d from cluster", clusters.at(j).cluster_element.at(n).id);
//                        clusters.at(j).cluster_element.erase(clusters.at(j).cluster_element.begin()+n);
//                        if(n > 0)
//                        {
//                            n --;
//                        }
//                        found_flag = true;
//    //                    break;
//                    }
//                }
//    //            if(found_flag == true)
//    //            {
//    //                break;
//    //            }
//            }
//        }
//    }
    ROS_INFO("Done");
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

bool ExplorationPlanner::auctioning(std::vector<double> *final_goal, std::vector<int> *clusters_available_in_pool, std::vector<std::string> *robot_str_name)
{

    ROS_INFO("Start Auctioning");

   /*
    * Check if Auction is running and wait until it is finished
    */
    int timer_count = 0;
    number_of_auction_bids_received = 0;
    std::string robo_name;

    float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float wait_if_auction_runs = r * 2;
//    wait_if_auction_runs = wait_if_auction_runs - robot_name; // FIXME
    if(wait_if_auction_runs < 0)
        wait_if_auction_runs = wait_if_auction_runs * (-1);

    ROS_INFO("Waiting %f second if a auction is running",wait_if_auction_runs);
    ros::Duration(wait_if_auction_runs).sleep();

    while(auction_running)
    {
        ROS_INFO("Waiting %f second because an auction is currently running",wait_if_auction_runs);
        ros::Duration(wait_if_auction_runs).sleep();
        float wait_if_auction_runs = r * 2;
        wait_if_auction_runs = wait_if_auction_runs - robot_name;
        if(wait_if_auction_runs < 0)
            wait_if_auction_runs = wait_if_auction_runs * (-1);

    }

    //EDIT Peter : Removed this wait part to speed up the process!

    /*
    if(robot_name != 0)
        auction_finished = false;
    //why here sleep so long ?
    while(auction_finished == false && timer_count <= 20)
    {
        timer_count++;
        ros::Duration(1).sleep();
    }*/

    /*
     * If no auction is running ... clear the auction vector and
     * start a new auction.
     */
    //how do i know that no auction is running ??
    //EDIT Peter: New bool to check if auction is running!

    if(auction_running)
    {
        ROS_INFO("Auction is running, leaving method");
        return false;
    }
    else
    {
        ROS_INFO("No auction is running, clearing");
        auction.clear();
    }


//    adhoc_communication::AuctionStatus auction_status;
    adhoc_communication::ExpAuction reqest_clusters, auction_msg;
//    auction_status.start_auction = true;
//    auction_status.auction_finished = false;

    auction_msg.start_auction = true;
    auction_msg.auction_finished = false;
    auction_msg.auction_status_message = true;

    if(robot_prefix_empty_param == false)
    {
    std::stringstream ss;
    ss << robot_name;
    std::string prefix = "";
    robo_name = prefix.append(ss.str());

    auction_msg.robot_name = robo_name;
    }else
    {
        auction_msg.robot_name = robot_str;
        robo_name = robot_str;
    }
    /*
     * visualize all cluster elements
     */

//    for(int i = 0; i < clusters.size(); i++)
//    {
//        ROS_INFO("---- Cluster %d ----", clusters.at(i).id);
//        std::string requested_clusters;
//        for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//        {
//            requested_clusters.append(NumberToString((int)clusters.at(i).cluster_element.at(j).id));
//            requested_clusters.append(", ");
//        }
//        ROS_INFO("Cluster ids: %s", requested_clusters.c_str());
//    }



    for(int i = 0; i < clusters.size(); i++)
    {
        adhoc_communication::ExpCluster cluster_request;


        for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
        {
            adhoc_communication::ExpClusterElement cluster_request_element;
            if(robot_prefix_empty_param == true)
            {
                cluster_request_element.id = clusters.at(i).cluster_element.at(j).id;
                cluster_request_element.detected_by_robot_str = clusters.at(i).cluster_element.at(j).detected_by_robot_str;
            }else
            {
                cluster_request_element.id = clusters.at(i).cluster_element.at(j).id;
            }
            cluster_request.ids_contained.push_back(cluster_request_element);
        }
//        auction_status.requested_clusters.push_back(cluster_request);
        auction_msg.requested_clusters.push_back(cluster_request);
    }

    if(robot_prefix_empty_param == true)
    {
        auction_msg.auction_id = auction_id_number;
    }else
    {
        auction_msg.auction_id = 10000*robot_name + auction_id_number;
    }

    /*
     * Following code is to visualize the message being send to the other robots
     */
//    for(int i = 0; i< auction_msg.requested_clusters.size(); i++)
//    {
//        adhoc_communication::Cluster cluster_req;
//        cluster_req = auction_msg.requested_clusters.at(i);
//        ROS_INFO("----------------------- REQUEST %d ---------------------------", i);
//        std::string requested_clusters;
//        for(int j = 0; j < cluster_req.ids_contained.size(); j++)
//        {
//            requested_clusters.append(NumberToString((int)cluster_req.ids_contained.at(j)));
//            requested_clusters.append(", ");
//        }
//        ROS_INFO("Auction with auction number: %d", auction_msg.auction_id);
//        ROS_INFO("Requested ids: %s", requested_clusters.c_str());
//    }


    std::string numbers_of_operating_robots;

    ROS_INFO("Robot %d starting an auction", robot_name);

//    if(first_run == true)
//    {
//        pub_auctioning_first.publish(auction_msg);
//    }else
//    {
        sendToMulticastAuction("mc_", auction_msg, "auction");
//    }



    /*
     * Wait for results of others
     */


       //EDIT Peter: I do not know how many answers i get
        /*
    nh.param("/robots_in_simulation",number_of_robots, 1);
    number_of_robots = 2; // To test with two robots if parameter does not work
    ROS_INFO("++++++++++ number of robots: %d ++++++++++", number_of_robots);
    */

//    ros::NodeHandle nh_robots;
//    nh_robots.param<std::string>("/robots_in_simulation",numbers_of_operating_robots, "1");
//    number_of_robots = atoi(numbers_of_operating_robots.c_str());
//    ROS_INFO("++++++++++ number of robots: %d ++++++++++", number_of_robots);

    /*
     * Number of responding robots is one less since the robot itself does not
     * respond to its own request
     */
    //ROS_INFO("Waiting for results from %d robots", number_of_robots);
    //wait 4 seconds to receive bids
        ros::Duration(3).sleep();
        /*
    while(timer_count < 50 && number_of_auction_bids_received < number_of_robots -1)
    {
        ROS_INFO("Waiting for results from %d robots", number_of_robots);
        timer_count++;
        ros::Duration(0.2).sleep();
    }*/
    ROS_INFO("number of auction bids received: %d", number_of_auction_bids_received);


    if(number_of_auction_bids_received > 0)
        number_of_completed_auctions++;
    else
        number_of_uncompleted_auctions++;
    /*if(number_of_auction_bids_received < number_of_robots-1)
    {
        ROS_ERROR("Wait for other robots timer exceeded");
        number_of_uncompleted_auctions++;
    }else if(number_of_auction_bids_received >= number_of_robots - 1)
    {
        number_of_completed_auctions++;
    }*/


    /*
     * Select the best cluster for yourself
     */
    ROS_INFO("Selecting the most attractive cluster");
    bool cluster_selected_flag = selectClusterBasedOnAuction(final_goal, clusters_available_in_pool, robot_str_name);

    /*
     * Stop the auction
     */
    ROS_INFO("Stop the auction");
//    auction_status.start_auction = false;
//    auction_status.auction_finished = true;

    auction_msg.start_auction = false;
    auction_msg.auction_finished = true;
    auction_msg.auction_status_message = true;
    auction_msg.robot_name = robo_name;
    auction_msg.requested_clusters.clear();

    if(cluster_selected_flag == true)// && final_goal->size() >= 4)
    {
        /*
         * Tell the others which cluster was selected
         */
        std::vector<transform_point_t> occupied_ids;
        clusterIdToElementIds(final_goal->at(4), &occupied_ids);
        for(int i = 0; i < occupied_ids.size(); i++)
        {
//            auction_status.occupied_ids.push_back(occupied_ids.at(i));
            adhoc_communication::ExpAuctionElement auction_element;
            auction_element.id = occupied_ids.at(i).id;
            auction_element.detected_by_robot_str = occupied_ids.at(i).robot_str;
            auction_msg.occupied_ids.push_back(auction_element);
        }
    }
//    else
//    {
//        ROS_ERROR("No Goal Selected from selectClusterBasedOnAuction");
//        cluster_selected_flag = false;
//    }

//    if(first_run == true)
//    {
//    pub_auctioning_status.publish(auction_status);
//        pub_auctioning_first.publish(auction_msg);
//    }else
//    {
        sendToMulticastAuction("mc_", auction_msg, "auction");
//    }

    first_run = false;
    auction_id_number++;

    ROS_INFO("return %d", cluster_selected_flag);
    return (cluster_selected_flag);
}

bool ExplorationPlanner::clusterIdToElementIds(int cluster_id, std::vector<transform_point_t>* occupied_ids)
{
    for(int i = 0; i < clusters.size(); i++)
    {
        if(clusters.at(i).id == cluster_id)
        {
            for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
            {
                transform_point_t point;
                point.id = clusters.at(i).cluster_element.at(j).id;
                point.robot_str = clusters.at(i).cluster_element.at(j).detected_by_robot_str;

                occupied_ids->push_back(point);
            }
        }
    }
}

bool ExplorationPlanner::selectClusterBasedOnAuction(std::vector<double> *goal, std::vector<int> *cluster_in_use_already_count, std::vector<std::string> *robot_str_name_to_return)
{
    ROS_INFO("Select the best cluster based on auction bids");

    int own_row_to_select_cluster = 0;
    int own_column_to_select = 0;

    /*
     * Select the method to select clusters from the matrix.
     * 0 ... select nearest cluster from OWN calculations
     * 1 ... gather information of others, estimate trajectory length based on
     *       distance information and select the optimal cluster using the
     */
    int method_used = 2;

    int count = 0;
    int auction_cluster_element_id = -1;

    auction_element_t auction_elements;
    auction_pair_t auction_pair;

    /*
     * Calculate my own auction BIDs to all my clusters
     * and store them in the auction vector
     */
    ROS_INFO("Cluster Size: %u", clusters.size());
    for(int i = 0; i < clusters.size(); i++)
    {
        ROS_INFO("Calculate cluster with ID: %d", clusters.at(i).id);
        int my_auction_bid = calculateAuctionBID(clusters.at(i).id, trajectory_strategy);
        if(my_auction_bid == -1)
        {
            ROS_WARN("Own BID calculation failed");
        }
        ROS_INFO("Own bid calculated for cluster '%d' is '%d'", clusters.at(i).id, my_auction_bid);
        auction_pair.cluster_id = clusters.at(i).id;
        auction_pair.bid_value = my_auction_bid;
        auction_elements.auction_element.push_back(auction_pair);
    }
    auction_elements.robot_id = robot_name;
    auction.push_back(auction_elements);


    /*
     * Remove all clusters which have previously been selected by other robots
     */

    std::vector<int> clusters_to_erase;
    for(int j = 0; j < clusters.size(); j++)
    {
        for(int i = 0; i < already_used_ids.size(); i++)
        {
            if(already_used_ids.at(i) == clusters.at(j).id)
            {
                /*Already used cluster found, therefore erase it*/
                for(int m = 0; m < auction.size(); m++)
                {
                    for(int n = 0; n < auction.at(m).auction_element.size(); n++)
                    {
                        if(auction.at(m).auction_element.at(n).cluster_id == already_used_ids.at(i))
                        {
                            ROS_INFO("Already used Cluster %d found .... unconsider it",auction.at(m).auction_element.at(n).cluster_id);
//                            auction.at(m).auction_element.erase(auction.at(m).auction_element.begin()+n);
//                            n--;
                              auction.at(m).auction_element.at(n).bid_value = 10000;
                        }
                    }
                }
            }
        }
    }


    ROS_INFO("Matrix");

    /*
     * Calculate the matrix size
     */
    int col = 0, row = 0;

    int robots_operating;
    nh.param<int>("robots_in_simulation",robots_operating, 1);
    ROS_INFO("robots operating: %d", robots_operating);

    row = auction.size();


    /* FIXME */
//    if(auction.size() < robots_operating)
//    {
//        row = robots_operating;
//    }else
//    {
//        row = auction.size();
//    }


//    for(int i= 0; i < auction.size(); i++)
//    {
//        if(auction.at(i).auction_element.size() > col)
//        {
//            col = auction.at(i).auction_element.size();
//        }
//    }
    col = clusters.size();
    ROS_INFO("robots: %d     clusters: %d", row, col);
    /*
    * create a matrix with boost library
    */

    int max_size = std::max(col,row);
    boost::numeric::ublas::matrix<double> m(max_size,max_size);

    ROS_INFO("Empty the matrix");
    /*
     * initialize the matrix with all -1
     */
    for(int i = 0; i < m.size1(); i++)
    {
        for(int j = 0; j < m.size2(); j++)
        {
            m(i,j) = 0;
        }
    }


    ROS_INFO("Fill matrix");
    /*
     * Now fill the matrix with real auction values and use
     * a method for the traveling salesman problem
     *
     * i ... number of robots
     * j ... number of clusters
     */
//    ROS_INFO("robots: %d   clusters: %d", row, col);
    for(int i = 0; i < row; i++)
    {
        ROS_INFO("                 ");
        ROS_INFO("****** i: %d   auction size: %u ******", i, auction.size());
        for(int j = 0; j < col; j++)
        {
            ROS_INFO("j: %d   cluster size: %u", j, clusters.size());
            bool found_a_bid = false;
            bool cluster_valid_flag = false;

//            if(i < auction.size())
//            {
                for(int n = 0; n < auction.at(i).auction_element.size(); n++)
                {
                    cluster_valid_flag = true;
//                    if(j < clusters.size())
//                    {
//                        ROS_INFO("j: %d    smaller then clusters.size(): %u", j, clusters.size());
                        if(clusters.at(j).id == auction.at(i).auction_element.at(n).cluster_id && auction.at(i).auction_element.at(n).bid_value >= 0)
                        {
        //                    /*
        //                     * Check if duplicates exist, if so change them
        //                     */
        //                    bool duplicate_exist = false;
        //                    for(int k = 0; k < row; k++)
        //                    {
        //                        for(int l = 0; l < col; l++)
        //                        {
        //                            if(m(l,k) == auction.at(i).auction_element.at(n).bid_value)
        //                            {
        //                                ROS_INFO("Duplicate found");
        //                                duplicate_exist = true;
        //                            }
        //                        }
        //                    }
        //
        //                    if(duplicate_exist == false)
        //                    {
        //                        ROS_INFO("assign value");
                                m(j,i) = auction.at(i).auction_element.at(n).bid_value;
        //                        ROS_INFO("done");
        //                    }else
        //                    {
        //                        m(j,i) = auction.at(i).auction_element.at(n).bid_value + 5;
        //                    }
                            found_a_bid = true;
                            break;
                        }
//                    }else if(j >= clusters.size())
//                    {
//                        ROS_ERROR("j >= clusters.size()");
//                        cluster_valid_flag = false;
//                        row = clusters.size();
//                        break;
//                    }
                }
//            }else if(i >= auction.size())
//            {
//                ROS_ERROR("i >= auction.size()");
//                col = auction.size();
//                break;
//            }

            ROS_INFO("Cluster elements checked, found BID: %d", found_a_bid);

            /*
             * The auction does not contain a BID value for this cluster,
             * therefore try to estimate it, using the other robots position.
             */
            if(found_a_bid == false) // && cluster_valid_flag == true)
            {
                ROS_INFO("No BID received ...");
                int distance = -1;
                other_robots_position_x = -1;
                other_robots_position_y = -1;

                ROS_INFO("---- clusters: %u element size: %u  robots positions: %u ----", clusters.size(), clusters.at(j).cluster_element.size(), other_robots_positions.positions.size());
                for(int d = 0; d < clusters.at(j).cluster_element.size(); d++)
                {
                    ROS_INFO("Access clusters.at(%d).cluster_element.at(%d)", j, d);
                    position_mutex.lock();
                    /*Check position of the current robot (i)*/
                    for(int u = 0; u < other_robots_positions.positions.size(); u++)
                    {
                        adhoc_communication::MmPoint position_of_robot;
                        position_of_robot = other_robots_positions.positions.at(u);

                        if(robot_prefix_empty_param == true)
                        {
                            ROS_INFO("position of robot: %s   compare with auction robot: %s", position_of_robot.src_robot.c_str(), auction.at(i).detected_by_robot_str.c_str());
                            if(position_of_robot.src_robot.compare(auction.at(i).detected_by_robot_str) == 0)
                            {
                                other_robots_position_x = position_of_robot.x;
                                other_robots_position_y = position_of_robot.y;

                                break;
                            }
                            else
                            {
                                ROS_ERROR("Unable to look up robots position!");
                            }
                        }else
                        {
                            int robots_int_id = atoi(position_of_robot.src_robot.substr(6,1).c_str());
                            ROS_INFO("Robots int id: %d", robots_int_id);
                            if(auction.at(i).robot_id == robots_int_id)
                            {
                                other_robots_position_x = position_of_robot.x;
                                other_robots_position_y = position_of_robot.y;

                                ROS_INFO("Robots %d     position_x: %f     position_y: %f", robots_int_id, other_robots_position_x, other_robots_position_y);
                                break;
                            }else
                            {
//                                ROS_ERROR("robot id requested: %d      Current robots position id: %d", auction.at(i).robot_id, robots_int_id);
                                ROS_ERROR("Unable to look up robots position!");
                            }
                        }
                    }
                    position_mutex.unlock();

//                    ROS_INFO("Got robot position");
                    if(other_robots_position_x != -1 && other_robots_position_y != -1)
                    {

                        if(trajectory_strategy == "trajectory")
                        {
                            distance = trajectory_plan(other_robots_position_x, other_robots_position_y, clusters.at(j).cluster_element.at(d).x_coordinate, clusters.at(j).cluster_element.at(d).y_coordinate);
                            ROS_INFO("estimated TRAJECTORY distance is: %d", distance);

                        }else if(trajectory_strategy == "euclidean" && j < clusters.size() && d < clusters.at(j).cluster_element.size())
                        {
                            double x = clusters.at(j).cluster_element.at(d).x_coordinate - other_robots_position_x;
                            double y = clusters.at(j).cluster_element.at(d).y_coordinate - other_robots_position_y;
                            distance = x * x + y * y;
                            ROS_INFO("estimated EUCLIDEAN distance is: %d", distance);

                            if(distance != -1)
                                break;
                        }
                    }
                    /*
                     * Check if the distance calculation is plausible.
                     * The euclidean distance to this point need to be smaller then
                     * the simulated trajectory path. If this stattement is not valid
                     * the trajectory calculation has failed.
                     */
                    if(distance != -1)
                    {
                        double x = clusters.at(j).cluster_element.at(d).x_coordinate - other_robots_position_x;
                        double y = clusters.at(j).cluster_element.at(d).y_coordinate - other_robots_position_y;
                        double euclidean_distance = x * x + y * y;

//                        ROS_INFO("Euclidean distance: %f   trajectory_path: %f", sqrt(euclidean_distance), distance * costmap_ros_->getCostmap()->getResolution());
                        if (distance * costmap_ros_->getCostmap()->getResolution() <= sqrt(euclidean_distance)*0.95)
                        {
                            ROS_ERROR("Euclidean distance is smaller then the trajectory path at recalculation");
                            distance = -1;
                        }else
                        {
                            break;
                        }
                    }
                }
                if(distance == -1)
                {
                    /*
                     * Cluster is definitely unknown. Therefore assign a high value
                     * to ensure not to select this cluster
                     */

                    ROS_ERROR("Unable to calculate the others BID at all");
                    m(j,i) = 10000;
                }
                else
                {
                    ROS_INFO("Estimated trajectory length: %d", distance);

//                    /*
//                     * Check if duplicates exist, if so change them
//                     */
//                    bool duplicate_exist = false;
//                    for(int k = 0; k < row; k++)
//                    {
//                        for(int l = 0; l < col; l++)
//                        {
//                            if(m(l,k) == distance)
//                            {
//                                ROS_INFO("Duplicate found");
//                                duplicate_exist = true;
//                            }
//                        }
//                    }
//
//                    if(duplicate_exist == false)
//                    {
                        m(j,i) = distance;
//                    }else
//                    {
//                        m(j,i) = distance +5;
//                    }
                }
            }
            ROS_INFO("Column filled");
        }
        ROS_INFO("No columns left. Check next robot");
    }
//    std::cout << m << std::endl;

    ROS_INFO("Completed");

    /*
     * If the number of clusters is less the number of robots,
     * the assigning algorithmn would not come to a solution. Therefore
     * select the nearest cluster
     */
    if(col < row)
    {
        ROS_ERROR("Number of clusters is less the number of robots. Select the closest");
        if(col == 0)
        {
            ROS_ERROR("No cluster anymore available");
            /*No clusters are available at all*/
            return false;
        }
        method_used = 0;
    }

    /*
     * Select the strategy how to select clusters from the matrix
     *
     * 0 ... select nearest cluster
     * 1 ...
     */
    if(method_used == 0)
    {

        /*
         * Select the nearest cluster (lowest bid value)
         * THE ROBOTS OWN BIDS ARE IN THE LAST ROW
         * initialize the bid value as first column in the first row
         */
//        int smallest_bid_value = auction.back().auction_element.front().bid_value;

//        ROS_INFO("smallest_bid_value at initialization: %d", smallest_bid_value);
        ROS_INFO("Columns left: %d", col);
//        for(int j = 0; j < col; j++)
//        {
//            ROS_INFO("col: %d", j);
//            if(auction.back().auction_element.at(j).bid_value <= smallest_bid_value && auction.back().auction_element.at(j).bid_value != -1)
//            {
//                smallest_bid_value = auction.back().auction_element.at(j).bid_value;
//                auction_cluster_element_id = auction.back().auction_element.at(j).cluster_id;
//            }
//        }
        if(auction.size() > 0)
        {
            if(auction.back().auction_element.size() > 0)
            {
                auction_cluster_element_id = auction.back().auction_element.front().cluster_id;
            }else
            {
                 return false;
            }
        }
    }
    if(method_used == 1)
    {
       /*
        * Use the Ungarische method/ KuhnMunkresAlgorithm in order to figure out which
        * cluster the most promising one is for myself
        */

        /* an example cost matrix */
        int mat[col*row];
        int counter = 0;
        for(int j = 0; j < col; j++)
        {
            for(int i = 0; i < row; i++)
            {
                mat[counter] = m(j,i);
                counter++;
            }
        }

        std::vector< std::vector<int> > m2 = array_to_matrix(mat,col,row);

        /*
         * Last row in the matrix contains BIDs of the robot itself
         * Remember the max row count before filling up with zeros.
         */
        own_row_to_select_cluster = row-1;
        ROS_INFO("Own row: %d", own_row_to_select_cluster);

        /* an example cost matrix */
        int r[3*3] =  {14,15,15,30,1,95,22,14,12};
        std::vector< std::vector<int> > m3 = array_to_matrix(r,3,3);

       /* initialize the gungarian_problem using the cost matrix*/
       Hungarian hungarian(m2, col, row, HUNGARIAN_MODE_MINIMIZE_COST);

//        Hungarian hungarian(m3, 3, 3, HUNGARIAN_MODE_MINIMIZE_COST);

       fprintf(stderr, "cost-matrix:");
       hungarian.print_cost();

       /* solve the assignment problem */
       for(int i = 0; i < 5; i++)
       {
           if(hungarian.solve() == true)
               break;
       }

       const std::vector< std::vector<int> > assignment = hungarian.assignment();

       /* some output */
       fprintf(stderr, "assignment:");
       hungarian.print_assignment();

//       for(int i = 0; i < assignment.size(); i++)
//       {
//           ROS_ERROR("---- %d -----", i);
//           for(int j = 0; j < assignment.at(i).size(); j++)
//           {
//               ROS_INFO("%d", assignment.at(i).at(j));
//           }
//       }


       for(int i = 0; i < assignment.size(); i++)
       {
           if(assignment.at(i).at(own_row_to_select_cluster) == 1)
           {
               auction_cluster_element_id = clusters.at(i).id;
               ROS_INFO("Selected Cluster at position : %d   %d",i ,own_row_to_select_cluster);
               break;
           }
       }

    }
    if(method_used == 2)
    {

	Matrix<double> mat = convert_boost_matrix_to_munkres_matrix<double>(m);
        ROS_INFO("Matrix (%ux%u):",mat.rows(),mat.columns());

	// Display begin matrix state.
	for ( int new_row = 0 ; new_row < mat.rows(); new_row++ ) {
            if(new_row > 9)
            {
                int rows_left = mat.rows() - new_row + 1;
                std::cout << "... (" << rows_left << " more)";
                break;
            }
            for ( int new_col = 0 ; new_col < mat.columns(); new_col++ ) {
                if(new_col > 9)
                {
                    int columns_left = mat.columns() - new_col + 1;
                    std::cout << "... (" << columns_left << " more)";
                    break;
                }
                std::cout.width(2);
                std::cout << mat(new_row,new_col) << " ";
            }
            std::cout << std::endl;
	}
	std::cout << std::endl;



//        for(int i = 0; i < mat.columns(); i++)
//        {
//            for(int j = 0; j < mat.rows(); j++)
//            {
//                /*Inner Matrix*/
////                bool element_found = false;
//                for(int n = 0; n < mat.columns(); n++)
//                {
//                    for(int m = 0; m < mat.rows(); m++)
//                    {
//                        if(abs(mat(i,j) - mat(n,m)) <= 20 && abs(mat(i,j) - mat(n,m)) != 0 && (mat(i,j) != 0 && mat(n,m) != 0))
//                        {
//                            mat(i,j) = 0;
////                            element_found = true;
////                            break;
//                        }
//                    }
////                    if(element_found = true)
////                        break;
//                }
//            }
//        }
//
//        ROS_INFO("Matrix with threshold :");
//        // Display begin matrix state.
//	for ( int new_row = 0 ; new_row < mat.rows(); new_row++ ) {
//		for ( int new_col = 0 ; new_col < mat.columns(); new_col++ ) {
//			std::cout.width(2);
//			std::cout << mat(new_row,new_col) << " ";
//		}
//		std::cout << std::endl;
//	}
//	std::cout << std::endl;


	// Apply Munkres algorithm to matrix.
	Munkres munk;
	munk.solve(mat);

        ROS_INFO("Solved :");
	// Display solved matrix.
	for ( int new_row = 0 ; new_row < mat.rows(); new_row++ ) {
            if(new_row > 9)
            {
                int rows_left = mat.rows() - new_row + 1;
                std::cout << "... (" << rows_left << " more)";
                break;
            }
            for ( int new_col = 0 ; new_col < mat.columns(); new_col++ ) {
                if(new_col > 9)
                {
                    int columns_left = mat.columns() - new_col + 1;
                    std::cout << "... (" << columns_left << " more)";
                    break;
                }
                std::cout.width(2);
                std::cout << mat(new_row,new_col) << " ";
            }
            std::cout << std::endl;
	}

	std::cout << std::endl;


       own_row_to_select_cluster = row-1;
       for(int i = 0; i < mat.columns(); i++)
       {
           if(mat(i,own_row_to_select_cluster) == 1)
           {
               auction_cluster_element_id = clusters.at(i).id;
               own_column_to_select = i;
               ROS_INFO("Selected Cluster at position : %d   %d  with BID: %f",i ,own_row_to_select_cluster, mat(i, own_row_to_select_cluster));
               break;
           }
       }




    }





    /*
     * Try to navigate to the selected cluster. If it failes, take the next efficient
     * cluster and try again
     */


    if(auction_cluster_element_id != -1)
    {

       /*
        * Select the above calculated goal. If this cluster is still in use,
        * set this cluster in the matrix to zero and restart the auction
        */





        /*
         * Following should just be executed if the selection using auctioning
         * does fail
         */
        while(ros::ok())
        {
            ROS_INFO("Try to determining goal");
            std::vector<double> new_goal;
            std::vector<std::string> robot_str_name;
            bool goal_determined = determine_goal(6, &new_goal, count, auction_cluster_element_id, &robot_str_name);



            if(goal_determined == true)
            {
//                if(new_goal.front() == -1)
//                {
//                    ROS_INFO("Unable to access goal points in the cluster");
//                    count++;
//                }else
//                {
                    double determined_goal_id = new_goal.at(4);
                    bool used_cluster = false;
                    for(int m = 0; m < already_used_ids.size(); m++)
                    {
                        if(determined_goal_id == already_used_ids.at(m))
                        {
                            ROS_INFO("Cluster in use already");
                            cluster_in_use_already_count->push_back(1);
                            used_cluster = true;
                            count++;
                            break;
                        }
                    }
                    if(used_cluster == false)
                    {
                        ROS_INFO("Cluster was not used by any robot before");
                        goal->push_back(new_goal.at(0));
                        goal->push_back(new_goal.at(1));
                        goal->push_back(new_goal.at(2));
                        goal->push_back(new_goal.at(3));
                        goal->push_back(new_goal.at(4));

                        robot_str_name_to_return->push_back(robot_str_name.at(0));
                        return true;
                    }
//                }
            }else
            {
                if(clusters.size() <= count)
                {
                    ROS_INFO("Not possible to select any goal from the available clusters");
                    return false;
                }else
                {
                    ROS_INFO("Current cluster empty ... select next one");
                }
                count++;
            }
        }
        return false;
    }else
    {
        /*
         * No auction elements are available. Auctioning has failed
         */

        ROS_INFO("Auction has failed, no auction elements are existent. Choosing nearest cluster");
        std::vector<double> new_goal;
        std::vector<std::string> robot_str_name;
        bool goal_determined = determine_goal(4, &new_goal, count, -1, &robot_str_name);
        if(goal_determined == true)
        {
            goal->push_back(new_goal.at(0));
            goal->push_back(new_goal.at(1));
            goal->push_back(new_goal.at(2));
            goal->push_back(new_goal.at(3));
            goal->push_back(new_goal.at(4));
            return true;
        }else
        {
            return false;
        }
    }
    return false;
}

bool ExplorationPlanner::negotiate_Frontier(double x, double y, int detected_by, int id, int cluster_id_number)
{
    ROS_INFO("Negotiating Frontier with id: %d  at Cluster: %d", id, cluster_id_number);

    int cluster_vector_position = 0;
    for (int i = 0; i < clusters.size(); i++)
    {
        if(clusters.at(i).id == cluster_id_number)
        {
            cluster_vector_position = i;
            break;
        }
    }

    ROS_DEBUG("cluster vector position: %d", cluster_vector_position);

    bool entry_found = false;
    bool id_in_actual_cluster = false;

    for(int i = 0; i< negotiation_list.size(); i++)
    {
        for(int k = 0; k < clusters.at(cluster_vector_position).cluster_element.size(); k++)
        {
            id_in_actual_cluster = false;
            if(negotiation_list.at(i).id == clusters.at(cluster_vector_position).cluster_element.at(k).id)
            {
                ROS_INFO("         Same ID detected");
                for(int j = 0; j < clusters.at(cluster_vector_position).cluster_element.size(); j++)
                {
                    for(int m = 0; m < my_negotiation_list.size(); m++)
                    {
                        if(clusters.at(cluster_vector_position).cluster_element.at(j).id == my_negotiation_list.at(m).id)
                        {
                            ROS_INFO("         But is in the current working cluster! everything alright");
                            id_in_actual_cluster = true;
                            return true;
                        }
                    }
                }

                double used_cluster_ids = 0;
                if(id_in_actual_cluster == false)
                {
                    ROS_INFO("Checking how many goals in the cluster are already occupied");
                    for(int n = 0; n < negotiation_list.size(); n++)
                    {
                        for(int m = 0; m < clusters.at(cluster_vector_position).cluster_element.size(); m++)
                        {
                            if(negotiation_list.at(n).id == clusters.at(cluster_vector_position).cluster_element.at(m).id)
                            {
                                used_cluster_ids++;
                            }
                        }
                    }
//                    ROS_INFO("%f goals of %f in the cluster  -> %f",used_cluster_ids, (double)clusters.at(cluster_vector_position).cluster_element.size(), double(used_cluster_ids / (double)clusters.at(cluster_vector_position).cluster_element.size()));
                    if(double(used_cluster_ids / (double)clusters.at(cluster_vector_position).cluster_element.size()) >= 0.1)
                    {
                        ROS_INFO("Negotiation failed the other Robot got cluster %d already", cluster_id_number);
                        entry_found = true;
                        return false;
                    }
                }
            }
        }
    }
    if(entry_found == false)
    {
        frontier_t new_frontier;
        new_frontier.x_coordinate = x;
        new_frontier.y_coordinate = y;
        new_frontier.detected_by_robot = detected_by;
        new_frontier.id = id;

        publish_negotiation_list(new_frontier, cluster_id_number);

        return true;
    }
    return false;
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
    bool exit = false;
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    for(unsigned int i=0; i < frontiers.size() && !exit; i++)
        for(unsigned int j=0; j < ds_list.size() && !exit; j++) {
            double total_distance;
            double x_f = frontiers[i].x_coordinate;
            double y_f = frontiers[i].y_coordinate;
            double x_ds = ds_list[j].x;
            double y_ds = ds_list[j].y;
            
            //check euclidean distances
            total_distance = euclidean_distance(x_ds, y_ds, x_f, y_f);
            if(total_distance * 2 > max_available_distance) //todo reduce the value from the one used by existFrontiersReachableWithFullBattery
                continue;
            
            // distance DS-frontier
            total_distance = trajectory_plan_meters(x_ds, y_ds, x_f, y_f);
            if(total_distance < 0){
                ROS_WARN("Failed to compute distance! (%.2f, %.2f), (%.2f, %.2f)", x_f, y_f, x_ds, y_ds);
//                total_distance = fallback_distance_computation(x_f, y_f, x_ds, y_ds) * 2;
//                if(errors == 0)
//                    my_error_counter++;
//                errors++;
                *error = true;
            }
            if(total_distance * 2 < max_available_distance) {
                found_reachable_frontier = true;
                exit = true;
            }
        }
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
    
    while (min_ds == NULL && retry < 5) {
        for (unsigned int i = 0; i < ds_list.size(); i++)
        {
            for (unsigned int j = 0; j < frontiers.size(); j++)
            {
                double dist = distance(ds_list.at(i).x, ds_list.at(i).y, frontiers.at(j).x_coordinate, frontiers.at(j).y_coordinate);
                if (dist < 0) {
                    ROS_WARN("Distance computation failed");
                    continue;
                }

                if (dist * 2 < max_available_distance)
                {
                    double dist2 = distance_from_robot(ds_list.at(i).x, ds_list.at(i).y);
                    if (dist2 < 0) {
                        ROS_WARN("Distance computation failed");
                        continue;
                    }

                    if (dist2 < min_dist)
                    {
                        min_dist = dist2;
                        min_ds = &ds_list.at(i);
                    }
                    
                    break;
                }
            }
        }
        retry++;
        if(min_ds == NULL)
            ros::Duration(3).sleep();
    }
    
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
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    // distance to next frontier
    for (int i = 0; i < frontiers.size(); i++) {
        double distance;
        
        //check euclidean distances
        distance = euclidean_distance(optimal_ds_x, optimal_ds_y, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
        if(distance * 2 > max_available_distance)
            continue;
        
        distance = trajectory_plan_meters(optimal_ds_x, optimal_ds_y, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate); //TODO safety coefficient is missing
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
    optimal_ds_id = msg.get()->id;
    optimal_ds_x = msg.get()->x;
    optimal_ds_y = msg.get()->y;
    optimal_ds_set = true;
}

bool ExplorationPlanner::my_negotiate()
{
    winner_of_auction = true;
    
#ifndef QUICK_SELECTION
     
    frontiers_under_auction.push_back(*my_selected_frontier);

    //ROS_ERROR("Publish fronter");
    adhoc_communication::ExpFrontier negotiation_list;
    adhoc_communication::ExpFrontierElement negotiation_element;
    //negotiation_element.detected_by_robot_str = 
    //negotiation_element.detected_by_robot = robot_name;
    negotiation_element.detected_by_robot = my_selected_frontier->detected_by_robot;
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
     ROS_WARN("Failed to call service sendToMulticast [%s]",ssendFrontier_2.getService().c_str());
     return false;
    }
}

void ExplorationPlanner::my_negotiationCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{
    ROS_DEBUG("Received frontier (%.2f, %.2f)", msg.get()->frontier_element[0].x_coordinate,  msg.get()->frontier_element[0].y_coordinate);

//    double robot_x, robot_y;
//    
//    //ROS_INFO("Transform frontier coordinates");
//        
//    std::stringstream robot_number;
//    robot_number << msg.get()->frontier_element[0].detected_by_robot;
//    std::string robo_name, prefix = "robot_";
//    robo_name = prefix.append(robot_number.str());

//    //std::string service_topic = robo_name2.append("/map_merger/transformPoint"); // FIXME for real scenario!!! robot_might not be used here
//    std::string service_topic = "map_merger/transformPoint";

////              ROS_INFO("Robo name: %s   Service to subscribe to: %s", robo_name.c_str(), service_topic.c_str());
//    
//    client = nh_transform.serviceClient<map_merger::TransformPoint>(service_topic);
//    ros::service::waitForService(service_topic, ros::Duration(3).sleep());
//    if(!ros::service::exists(service_topic, true))
//        return;

//    service_message.request.point.x = msg.get()->frontier_element[0].x_coordinate;
//    service_message.request.point.y = msg.get()->frontier_element[0].y_coordinate;
//    service_message.request.point.src_robot = robo_name;   
//    //ROS_DEBUG("Robot name:  %s", service_message.request.point.src_robot.c_str());
//    
//    //ROS_ERROR("Old x: %f   y: %f", msg.get()->frontier_element[0].x_coordinate, msg.get()->frontier_element[0].y_coordinate);

//    //ROS_ERROR("calling client");
//    if(client.call(service_message))
//        ; //ROS_ERROR("New x: %.1f   y: %.1f", service_message.response.point.x, service_message.response.point.y);
//    else {
//        //ROS_ERROR("FAILED!");
//        return;   
//    }

//    //acquire_mutex(&store_frontier_mutex, __FUNCTION__); //TODO maybe we need a mutex, but it causes deadlocks...
//    int index = -1;
//    for(int i=0; i<frontiers.size(); i++) //TODO inefficient (and the robot could be unable to send the frontier in time...)
//        if( fabs(frontiers.at(i).x_coordinate - service_message.response.point.x) < 1.0 && fabs(frontiers.at(i).y_coordinate - service_message.response.point.y) < 1.0 ) { //TODO correct?
//            index = i;
//            break;
//        }
//    if(index < 0 || !my_check_efficiency_of_goal(this->available_distance, &frontiers.at(index)))
//        return;
//    double cost = frontier_cost(frontiers.at(index));
//    //release_mutex(&store_frontier_mutex, __FUNCTION__);
//   
//    //ROS_ERROR("%d + %d + %d + %f", d_g, d_gb, d_gbe, theta);
//    
//    adhoc_communication::ExpFrontier negotiation_list;
//    adhoc_communication::ExpFrontierElement negotiation_element;
//    //negotiation_element.detected_by_robot = my_selected_frontier->detected_by_robot;
//    //negotiation_element.x_coordinate = my_selected_frontier->x_coordinate;
//    //negotiation_element.y_coordinate = my_selected_frontier->y_coordinate;
//    //negotiation_element.id = my_selected_frontier->id;
//    negotiation_element.bid = cost;
//    
//    negotiation_list.frontier_element.push_back(negotiation_element);
//    
//    my_sendToMulticast("mc_", negotiation_list, "reply_to_frontier");
    
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
        //ROS_ERROR("FAILED!");
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
        last_robot_auctioned_frontier_list.push_back(new_goal);
    }
}

void ExplorationPlanner::my_replyToNegotiationCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg) {
    ROS_DEBUG("%f vs %f", my_bid, msg.get()->frontier_element[0].bid);
    if(my_bid > msg.get()->frontier_element[0].bid ) //they are cost, so the higher the worse
        winner_of_auction = false;    
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

//void ExplorationPlanner::robot_amcl_position_callback(geometry_msgs::PoseWithCovarianceStamped

/*
bool ExplorationPlanner::get_robot_position(double *x, double *y) { //F WRONG!!!!!!!!!
    
    if(costmap_ros_ == NULL) {
        ROS_ERROR("NULL!!!");
        return false;   
    }
    tf::Stamped<tf::Pose> robot_pose;
    if(costmap_global_ros_->getRobotPose(robot_pose)) {
        *x = robot_pose.getOrigin().getX() * costmap_ros_->getCostmap()->getResolution();
        *y = robot_pose.getOrigin().getY() * costmap_ros_->getCostmap()->getResolution();
        return true;
    }
    
    return false;
}
*/

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
    
    int tries = 0;
    acquire_mutex(&costmap_mutex, __FUNCTION__);
    while (!costmap_ros_->getRobotPose(robotPose))
    {
        ROS_ERROR("Failed to get RobotPose");
        if(tries < 5) {
            tries++;
            ros::Duration(2).sleep();
        }
        else
            return false;
    }
    release_mutex(&costmap_mutex, __FUNCTION__);
    
    robot_x = robotPose.getOrigin().getX();
    robot_y = robotPose.getOrigin().getY();   
    
//    ROS_ERROR("%.2f, %.2f", robot_x, robot_y);
//    ROS_ERROR("%u", costmap_ros_->getCostmap()->getSizeInCellsX());
//      ROS_ERROR("%u", costmap_ros_->getCostmap()->getSizeInCellsX());
//    tf::Stamped<tf::Pose> robotPose;
//    if(getRobotPose(robotPose)) {
//        ROS_ERROR("%.2f, %.2f", robotPose.getOrigin().getX(), robotPose.getOrigin().getY());
//    }
//    double c1, c2;
//    unsigned int m1, m2;
//    costmap_ros_->getCostmap()->mapToWorld(0, 0, c1, c2);
//    ROS_ERROR("%.2f, %.2f", c1, c2); //coordinates in the /map frame, which should be placed in the starting position of the robot 
//    if(costmap_ros_->getCostmap()->worldToMap(0, 0, m1, m2))
//        ROS_ERROR("%.2f, %.2f", m1, m2); //coordinates in the /map frame, which should be placed in the starting position of the robot 
//    else
//        ROS_ERROR("no!!!");
//    if(costmap_ros_->getCostmap()->worldToMap(-49, -49, m1, m2))
//        ROS_ERROR("%u, %u", m1, m2); //coordinates in the /map frame, which should be placed in the starting position of the robot 
//    else
//        ROS_ERROR("no!!!");
//    if(costmap_ros_->getCostmap()->worldToMap(49, 49, m1, m2))
//        ROS_ERROR("%u, %u", m1, m2); //coordinates in the /map frame, which should be placed in the starting position of the robot 
//    else
//        ROS_ERROR("no!!!");
//    costmap_ros_->getCostmap()->mapToWorld(500, 500, c1, c2);
//    ROS_ERROR("%.2f, %.2f", c1, c2); // "0.0, 0.0" on a 1000x1000 cells and 100x100 meters map for every robot and robot starting in (0,0)!
//    //costmap_ros_->getCostmap()->mapToWorld(1100, 1100, c1, c2);
//    //ROS_ERROR("%.1f, %.1f", c1, c2);
//    costmap_ros_->getCostmap()->mapToWorld(1000, 1000, c1, c2);
//    ROS_ERROR("%.2f, %.2f", c1, c2); // "5.0, 5.0" on a 1000x1000 cells and 100x100 meters map for every robot and robot starting in (0,0)!
//    ros::Duration(10).sleep();
//    double d = trajectory_plan_meters(0, 36);
//    ROS_ERROR("%.1f", d); //it seems to make more or less sense (it's a little bit lower then what should be), a




    sorted_frontiers.clear();
    
    //TODO move to a separate function that is called by explorer, since in case of error (when my_... is recalled by itself), this code otherwise is re-executed every time...
    ROS_DEBUG("frontiers size: %u", frontiers.size());
    if(APPROACH == 0)
        my_sort_cost_0(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 1)
//        sorted_frontiers = frontiers;
//    else if(APPROACH == 2)
//        my_sort_cost_2(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 3)
//        my_sort_cost_3(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 4)
//        my_select_4(available_distance, energy_above_th, w1, w2, w3, w4, final_goal, robot_str_name);
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

    if(frontier_selected || APPROACH == 0) {
        frontier_selected = false;
        for(unsigned int i=0; i < sorted_frontiers.size(); i++)
        {
            if(APPROACH == 0)
                if(!my_check_efficiency_of_goal(available_distance, &sorted_frontiers.at(i))) {
                    ROS_INFO("frontier currentl unreachable: skipping");
                    continue;
                }

            //start auction
            my_selected_frontier = &sorted_frontiers.at(i);
            my_bid = sorted_frontiers.at(i).cost;
            ROS_INFO("start frontier negotiation!");
            my_negotiate();
     
            for(int j = 0; j < auction_timeout/0.1; j++) {
                ros::Duration(0.1).sleep();
                ros::spinOnce();
            }
            
            if(!winner_of_auction) {
                ROS_INFO("frontier under auction: skip");
                continue;
            }

            ROS_INFO("selected goal: %.2f, %.2f", my_selected_frontier->x_coordinate, my_selected_frontier->y_coordinate);
            final_goal->push_back(my_selected_frontier->x_coordinate);
            final_goal->push_back(my_selected_frontier->y_coordinate);            
            final_goal->push_back(my_selected_frontier->detected_by_robot);
            final_goal->push_back(my_selected_frontier->id);
            
            robot_str_name->push_back(robot_name_str); 
            
            frontiers_under_auction.clear();
            my_error_counter = 0;
            
            adhoc_communication::ExpFrontier negotiation_list;
            adhoc_communication::ExpFrontierElement negotiation_element;
            negotiation_element.detected_by_robot = robot_name;
            negotiation_element.x_coordinate = my_selected_frontier->x_coordinate;
            negotiation_element.y_coordinate = my_selected_frontier->y_coordinate;
            negotiation_element.id = my_selected_frontier->id;
                
            negotiation_list.frontier_element.push_back(negotiation_element);

            my_sendToMulticast("mc_", negotiation_list, "send_next_robot_goal");
            
            frontier_selected = true;            
            break;
        }
    }

    selection_time = (ros::Time::now() - start_time).toSec();    
    
    my_error_counter = 0;
    return frontier_selected;

}

//bool ExplorationPlanner::my_determine_goal_staying_alive(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id, bool energy_above_th, int w1, int w2, int w3, int w4)
//{
//    
//    ros::Time start_time;
//    selection_time = 0;
//    number_of_frontiers = 0;
//    frontier_selected = false;
//    start_time = ros::Time::now();
//    winner_of_auction = true;
//    this->available_distance = available_distance;

//    if (frontiers.size() <= 0 && clusters.size() <= 0)
//    {
//        ROS_ERROR("No frontier/cluster available");
//    } 
//    
//    sorted_frontiers.clear();
//    
//    //TODO move to a separate function that is called by explorer, since in case of error (when my_... is recalled by itself), this code otherwise is re-executed every time...
//    ROS_DEBUG("frontiers size: %u", frontiers.size());
//    if(APPROACH == 0)
//        sorted_frontiers = frontiers;
//    else if(APPROACH == 1)
//        sorted_frontiers = frontiers;
//    else if(APPROACH == 2)
//        my_sort_cost_2(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 3)
//        my_sort_cost_3(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 4)
//        my_select_4(available_distance, energy_above_th, w1, w2, w3, w4, final_goal, robot_str_name);
//    else {
//        ROS_ERROR("Invalid approach!");
//        sorted_frontiers = frontiers;
//    }
//    
//    selection_time = (ros::Time::now() - start_time).toSec();
//    number_of_frontiers = frontiers.size();

//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
//    if(frontier_selected)
//        for(int i=0; i < sorted_frontiers.size(); i++)
//        {

//            //start auction
//            my_selected_frontier = &sorted_frontiers.at(i);
//            my_bid = sorted_frontiers.at(i).cost;
//            ROS_INFO("start frontier negotiation!");
//            my_negotiate();
//     
//            for(int i = 0; i < auction_timeout/0.1; i++) {
//                ros::Duration(0.1).sleep();
//                ros::spinOnce();
//            }
//            
//            if(!winner_of_auction) {
//                ROS_INFO("frontier under auction: skip");
//                continue;
//            }

//            ROS_INFO("frontier selected");
//            
//            final_goal->push_back(my_selected_frontier->x_coordinate);
//            final_goal->push_back(my_selected_frontier->y_coordinate);
//            ROS_INFO("selected goal: %.2f, %.2f", my_selected_frontier->x_coordinate, my_selected_frontier->y_coordinate);
//            final_goal->push_back(my_selected_frontier->detected_by_robot);
//            final_goal->push_back(my_selected_frontier->id);
//            
//            robot_str_name->push_back(robot_name_str); 
//            
//            frontiers_under_auction.clear();
//            my_error_counter = 0;
//            errors = 0;
//            
//            adhoc_communication::ExpFrontier negotiation_list;
//            adhoc_communication::ExpFrontierElement negotiation_element;
//            negotiation_element.detected_by_robot = robot_name;
//            negotiation_element.x_coordinate = my_selected_frontier->x_coordinate;
//            negotiation_element.y_coordinate = my_selected_frontier->y_coordinate;
//            negotiation_element.id = my_selected_frontier->id;
//                
//            negotiation_list.frontier_element.push_back(negotiation_element);

//            my_sendToMulticast("mc_", negotiation_list, "send_next_robot_goal");
//            
//            release_mutex(&store_frontier_mutex, __FUNCTION__);
//            return true;
//        }

//    frontier_selected = false;
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    return false;

//}

/**
 * Check a frontier (or a cluster) if it is within reach of the robot considering its available energy
 * mode: 1=frontier, 2=cluster
 * strategy: 1=euclidean distance, 2=actual travel path
 */
bool ExplorationPlanner::determine_goal_staying_alive(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id)
{
return false;
//    if (!costmap_ros_->getRobotPose(robotPose))
//    {
//            ROS_ERROR("Failed to get RobotPose");
//    }

//    // check if robot needs to go home right away
//    double dist_home;
//    double dist_front;
//    double closest = 9999;
//    if(strategy == 1){
//        double xh = robot_home_x - robotPose.getOrigin().getX();
//        double yh = robot_home_y - robotPose.getOrigin().getY();
//        dist_home = sqrt(xh * xh + yh * yh) * 1.2; // 1.2 is extra reserve, just in case
//    }
//    else if(strategy == 2){
//        //F
//        //dist_home = trajectory_plan(robot_home_x, robot_home_y) * costmap_ros_->getCostmap()->getResolution();
//        dist_home = trajectory_plan(robot_home_x, robot_home_y);
//    }
//    if(dist_home > 0 && dist_home >= available_distance)
//        return false;

//    // look for a FRONTIER as goal
//    int errors = 0;
//    if (mode == 1)
//    {
//        for (int i = 0 + count; i < frontiers.size(); i++)
//        {
//            //we only check the first 9 frontiers, because we only sorted the first 9 frontiers by efficiency.
//            if(i>8){
//                // if the distances could not be computed, try again using euclidean distances instead
//                if(errors == i && strategy == 2){
//                    ROS_ERROR("Fallback to euclidean distance.");
//                    return this->determine_goal_staying_alive(1, 1, available_distance, final_goal, count, robot_str_name, -1);
//                }
//                return false;
//            }

//            if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
//            {
//                double distance;
//                double total_distance;
//                if(strategy == 1){
//                    // distance to next frontier
//                    double x1 = frontiers.at(i).x_coordinate - robotPose.getOrigin().getX();
//                    double y1 = frontiers.at(i).y_coordinate -  robotPose.getOrigin().getY();
//                    // distance from frontier to home base
//                    double x2 = robot_home_x - frontiers.at(i).x_coordinate;
//                    double y2 = robot_home_y -  frontiers.at(i).y_coordinate;
//                    total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                }
//                else if(strategy == 2){
//                    // distance to next frontier
//                    total_distance = trajectory_plan(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
//                    if(total_distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        errors++;
//                        continue;
//                    }
//                    // distance from frontier to home base
//                    distance = trajectory_plan(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, robot_home_x, robot_home_y);
//                    if(distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        errors++;
//                        continue;
//                    }
//                    total_distance += distance;

//                    if(total_distance < closest){
//                        closest = total_distance;
//                        dist_front = total_distance - distance;
//                        dist_home = distance;
//                    }

//                    // convert from cells to meters
//                    total_distance *= costmap_ros_->getCostmap()->getResolution();
//                }
//                else{
//                    ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                    return false;
//                }

//                ROS_INFO("Distance to frontier and then home: %.2f",total_distance);
//                if(available_distance > total_distance)
//                {
//                    ROS_INFO("------------------------------------------------------------------");
//                    ROS_INFO("Determined frontier with ID: %d   at x: %.2f     y: %.2f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

//                    final_goal->push_back(frontiers.at(i).x_coordinate);
//                    final_goal->push_back(frontiers.at(i).y_coordinate);
//                    final_goal->push_back(frontiers.at(i).detected_by_robot);
//                    final_goal->push_back(frontiers.at(i).id);

//                    robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
//                    return true;
//                }else{
//                    ROS_INFO("No frontier in energetic range (%.2f < %.2f + %.2f)", available_distance, dist_front * costmap_ros_->getCostmap()->getResolution(), dist_home * costmap_ros_->getCostmap()->getResolution());
//                    return false;
//                }
//            }
//        }
//    }

//    // look for a CLUSTER as goal
//    else if (mode == 2)
//    {
//        int cluster_vector_position = 0;

//        if(clusters.size() > 0)
//        {
//            for (int i = 0; i < clusters.size(); i++)
//            {
//                if(clusters.at(i).id == actual_cluster_id)
//                {
//                    if(clusters.at(i).cluster_element.size() > 0)
//                    {
//                        cluster_vector_position = i;
//                    }
//                    break;
//                }
//            }
//        }

//        ROS_INFO("Calculated vector position of cluster %d", actual_cluster_id);
//        /*
//         * Iterate over all clusters .... if cluster_vector_position is set
//         * also the clusters with a lower have to be checked if the frontier
//         * determination fails at clusters.at(cluster_vector_position). therefore
//         * a ring-buffer is operated to iterate also over lower clusters, since
//         * they might have changed.
//         */
//        int nothing_found_in_actual_cluster = 0;
//        int visited_clusters = 0;
//        for (int i = 0 + count; i < clusters.size(); i++)
//        {
//            i = i+ cluster_vector_position;
//            i = i % (clusters.size());
//            for (int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//            {
//                if (check_efficiency_of_goal(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate) == true)
//                {
//                    double distance;
//                    double total_distance;
//                    if(strategy == 1){
//                        // distance to cluster
//                        double x1 = clusters.at(i).cluster_element.at(j).x_coordinate - robotPose.getOrigin().getX();
//                        double y1 = clusters.at(i).cluster_element.at(j).y_coordinate - robotPose.getOrigin().getY();
//                        // distance from cluster to home base
//                        double x2 = robot_home_x - clusters.at(i).cluster_element.at(j).x_coordinate;
//                        double y2 = robot_home_y -  clusters.at(i).cluster_element.at(j).y_coordinate;
//                        total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                    }
//                    else if(strategy == 2){
//                        // distance to cluster
//                        total_distance = trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate);
//                        if(total_distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        // distance from cluster to home base
//                        distance = trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate, robot_home_x, robot_home_y);
//                        if(distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        total_distance += distance;
//                        // convert from cells to meters
//                        total_distance *= costmap_ros_->getCostmap()->getResolution();
//                    }
//                    else{
//                        ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                        return false;
//                    }

//                    ROS_INFO("distance to cluster and then home: %f",total_distance);
//                    if(available_distance > total_distance)
//                    {
//                        ROS_INFO("------------------------------------------------------------------");
//                        ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(i).cluster_element.at(j).id, (int)clusters.at(i).id);

//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).x_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).y_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).id);

//                        // number of the cluster we operate in
//                        final_goal->push_back(clusters.at(i).id);
//                        robot_str_name->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot_str);
//                        return true;
//                    }
//                }
//            }

//            nothing_found_in_actual_cluster ++;
//            visited_clusters ++;

//            if(nothing_found_in_actual_cluster == 1)
//            {
//                //start again at the beginning(closest cluster))
//                i=0;
//                cluster_vector_position = 0;
//            }

//            if(visited_clusters == clusters.size())
//            {
//                ROS_ERROR("No frontier in energetic range %.2f, going home for recharging", available_distance);
//                return false;
//            }
//        }
//    }
}

//void ExplorationPlanner::sort_cost_with_approach(bool energy_above_th, int w1, int w2, int w3, int w4)
//{
//    if(recompute_ds_graph) {
//        recompute_ds_graph = false;
//        for(int i=0; i < ds_list.size(); i++)
//            for(int j=0; j < ds_list.size(); j++)
//                if(ds_graph[i][j] >= 0 || i == j)
//                    continue;
//                else {
//                    double dist = trajectory_plan_meters(ds_list[i].x, ds_list[i].y, ds_list[j].x, ds_list[j].y);
//                    //ROS_ERROR("distance between (%.1f, %.1f) and (%.1f, %.1f): %f", ds_list[i].x, ds_list[i].y, ds_list[j].x, ds_list[j].y, dist);
//                    if(dist < 0) {
//                        ROS_ERROR("Unable to compute distance at the moment: retrying later...");
//                        recompute_ds_graph = true;
//                    }
//                    else {
//                        ds_graph[ds_list[i].id][ds_list[j].id] = dist;
//                        ds_graph[ds_list[j].id][ds_list[i].id] = dist;
//                    }
//                }
//    }
//    if(APPROACH == 0)
//        /* Original cost function */
//        sort_cost(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 1)
//        /* Cost function: (real distance = Disjktra's distance)
//         *     - (real) distance between the given frontier and target DS of the robot;
//         *     - distance on the graph of the DSs between the DS that is closest, according to the euclidean distance (not the real one!), to the given frontier and the robot; this distance is computed efficiently by the energy_mgmt node;
//         *     - d_r
//         *     - theta_rel
//         */
//        sort_cost_1(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == -1)
//        return;
//    else
//        ROS_ERROR("INVALID APPROACH!!!");
//}

//bool ExplorationPlanner::determine_goal_staying_alive_2(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id)
//{
//    if (!costmap_ros_->getRobotPose(robotPose))
//    {
//            ROS_ERROR("Failed to get RobotPose");
//    }
//    
//    //store_frontier_mutex.lock();
//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);

//    // check if robot needs to go recharging right away
//    double dist_home;
//    double dist_front;
//    double closest = 9999;
//    if(strategy == 1){
//        double xh = optimal_ds_x - robotPose.getOrigin().getX();
//        double yh = optimal_ds_y - robotPose.getOrigin().getY();
//        dist_home = sqrt(xh * xh + yh * yh) * 1.2; // 1.2 is extra reserve, just in case
//    }
//    else if(strategy == 2){
//        dist_home = trajectory_plan(optimal_ds_x, optimal_ds_y) * costmap_ros_->getCostmap()->getResolution();
//    }
//    //ROS_ERROR("available_distance: %f", available_distance);
//    if(dist_home > 0 && dist_home >= available_distance) {
//        ROS_ERROR("Target DS is too far to reach a frontier...\noptimal_ds_x: %f, optimal_ds_y: %f, distance: %f, available distance: %f", optimal_ds_x, optimal_ds_y, dist_home, available_distance);
//        release_mutex(&store_frontier_mutex, __FUNCTION__);
//        return false;
//    }

//    // look for a FRONTIER as goal
//    int errors = 0;
//    if (mode == 1)
//    {
//        for (int i = 0 + count; i < frontiers.size(); i++)
//        {
//            //we only check the first 9 frontiers, because we only sorted the first 9 frontiers by efficiency.
//            if(i>8){
//                // if the distances could not be computed, try again using euclidean distances instead
//                if(errors == i && strategy == 2){
//                    ROS_ERROR("Fallback to euclidean distance.");
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    return this->determine_goal_staying_alive_2(1, 1, available_distance, final_goal, count, robot_str_name, -1);
//                }
//                release_mutex(&store_frontier_mutex, __FUNCTION__);
//                ROS_ERROR("None of the %d checked frontiers is reachable! This shouldn't happen...", 8);
//                return false;
//            }

//            bool under_auction = false;
//            for(int k=0; !under_auction && k<frontiers_under_auction.size(); k++)
//                if(frontiers[i].id == frontiers_under_auction[k].id) {
//                    under_auction = true;
//                }
//            if(under_auction)
//                continue;

//            if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
//            {
//                double distance;
//                double total_distance;
//                if(strategy == 1){
//                    // distance to next frontier
//                    double x1 = frontiers.at(i).x_coordinate - robotPose.getOrigin().getX();
//                    double y1 = frontiers.at(i).y_coordinate -  robotPose.getOrigin().getY();
//                    // distance from frontier to home base
//                    double x2 = robot_home_x - frontiers.at(i).x_coordinate;
//                    double y2 = robot_home_y -  frontiers.at(i).y_coordinate;
//                    total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                }
//                else if(strategy == 2){
//                    // distance to next frontier
//                    total_distance = trajectory_plan(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
//                    if(total_distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        if(errors == 0)
//                            my_error_counter++;
//                        errors++;
//                        continue;
//                    }
//                    // distance from frontier to optimal ds
//                    distance = trajectory_plan(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, optimal_ds_x, optimal_ds_y);
//                    if(distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        if(errors == 0)
//                            my_error_counter++;
//                        errors++;
//                        continue;
//                    }
//                    total_distance += distance;

//                    if(total_distance < closest){
//                        closest = total_distance;
//                        dist_front = total_distance - distance;
//                        dist_home = distance;
//                    }

//                    // convert from cells to meters
//                    total_distance *= costmap_ros_->getCostmap()->getResolution();
//                }
//                else{
//                    ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    return false;
//                }

//                ROS_INFO("Distance to frontier and then home: %.2f",total_distance);
//                if(available_distance > total_distance)
//                {
//                    ROS_INFO("------------------------------------------------------------------");
//                    ROS_INFO("Determined frontier with ID: %d   at x: %.2f     y: %.2f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

//                    final_goal->push_back(frontiers.at(i).x_coordinate);
//                    final_goal->push_back(frontiers.at(i).y_coordinate);
//                    final_goal->push_back(frontiers.at(i).detected_by_robot);
//                    final_goal->push_back(frontiers.at(i).id);
//                    
//                    robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
//                    
//                    my_selected_frontier = &frontiers.at(i);
//                    
//#ifndef QUICK_SELECTION
//            
//                    // robot position
//                    double robot_x = robotPose.getOrigin().getX();
//                    double robot_y = robotPose.getOrigin().getY();

//					// frontier position
//                    double frontier_x = frontiers.at(i).x_coordinate;
//                    double frontier_y = frontiers.at(i).y_coordinate;

//					// calculate d_g
//                    int d_g = trajectory_plan(frontier_x, frontier_y);

//                    // calculate d_gb
//                    int d_gb = trajectory_plan(frontier_x, frontier_y, robot_home_x, robot_home_y);

//                    // calculate d_gbe
//                    int d_gbe;
//                    if(my_energy_above_th)
//                    {
//                        d_gbe = -d_gb;
//                    }
//                    else
//                    {
//                        d_gbe = d_gb;
//                    }

//                    // calculate theta
//                    double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
//                    double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
//                    double theta = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g) - M_PI));

//                    // calculate cost function
//                     my_bid = w1 * d_g + w2 * d_gb + w3 * d_gbe + w4 * theta;
//             
//#endif

//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    my_error_counter = 0;
//                    return true;
//                    
//                } else{
//                    ROS_ERROR("No frontier in energetic range (%.2f < %.2f + %.2f)", available_distance, dist_front * costmap_ros_->getCostmap()->getResolution(), dist_home * costmap_ros_->getCostmap()->getResolution());
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    my_error_counter++;
//                    return false;
//                }
//            }
//        }
//    }

//    // look for a CLUSTER as goal
//    else if (mode == 2)
//    {
//        int cluster_vector_position = 0;

//        if(clusters.size() > 0)
//        {
//            for (int i = 0; i < clusters.size(); i++)
//            {
//                if(clusters.at(i).id == actual_cluster_id)
//                {
//                    if(clusters.at(i).cluster_element.size() > 0)
//                    {
//                        cluster_vector_position = i;
//                    }
//                    break;
//                }
//            }
//        }

//        ROS_INFO("Calculated vector position of cluster %d", actual_cluster_id);
//        /*
//         * Iterate over all clusters .... if cluster_vector_position is set
//         * also the clusters with a lower have to be checked if the frontier
//         * determination fails at clusters.at(cluster_vector_position). therefore
//         * a ring-buffer is operated to iterate also over lower clusters, since
//         * they might have changed.
//         */
//        int nothing_found_in_actual_cluster = 0;
//        int visited_clusters = 0;
//        for (int i = 0 + count; i < clusters.size(); i++)
//        {
//            i = i+ cluster_vector_position;
//            i = i % (clusters.size());
//            for (int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//            {
//                if (check_efficiency_of_goal(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate) == true)
//                {
//                    double distance;
//                    double total_distance;
//                    if(strategy == 1){
//                        // distance to cluster
//                        double x1 = clusters.at(i).cluster_element.at(j).x_coordinate - robotPose.getOrigin().getX();
//                        double y1 = clusters.at(i).cluster_element.at(j).y_coordinate - robotPose.getOrigin().getY();
//                        // distance from cluster to home base
//                        double x2 = robot_home_x - clusters.at(i).cluster_element.at(j).x_coordinate;
//                        double y2 = robot_home_y -  clusters.at(i).cluster_element.at(j).y_coordinate;
//                        total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                    }
//                    else if(strategy == 2){
//                        // distance to cluster
//                        total_distance = trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate);
//                        if(total_distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        // distance from cluster to home base
//                        distance = trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate, robot_home_x, robot_home_y);
//                        if(distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        total_distance += distance;
//                        // convert from cells to meters
//                        total_distance *= costmap_ros_->getCostmap()->getResolution();
//                    }
//                    else{
//                        ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                        release_mutex(&store_frontier_mutex, __FUNCTION__);
//                        return false;
//                    }

//                    ROS_INFO("distance to cluster and then home: %f",total_distance);
//                    if(available_distance > total_distance)
//                    {
//                        ROS_INFO("------------------------------------------------------------------");
//                        ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(i).cluster_element.at(j).id, (int)clusters.at(i).id);

//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).x_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).y_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).id);

//                        // number of the cluster we operate in
//                        final_goal->push_back(clusters.at(i).id);
//                        robot_str_name->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot_str);
//                        release_mutex(&store_frontier_mutex, __FUNCTION__);
//                        return true;
//                    }
//                }
//            }

//            nothing_found_in_actual_cluster ++;
//            visited_clusters ++;

//            if(nothing_found_in_actual_cluster == 1)
//            {
//                //start again at the beginning(closest cluster))
//                i=0;
//                cluster_vector_position = 0;
//            }

//            if(visited_clusters == clusters.size())
//            {
//                ROS_ERROR("No frontier in energetic range %.2f, going home for recharging", available_distance);
//                release_mutex(&store_frontier_mutex, __FUNCTION__);
//                return false;
//            }
//        }
//    }
//    
//    release_mutex(&store_frontier_mutex, __FUNCTION__); //probablt useless here...
//    
//}

//bool ExplorationPlanner::determine_goal_staying_alive_2_reserve(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id)
//{
//    if (!costmap_ros_->getRobotPose(robotPose))
//    {
//            ROS_ERROR("Failed to get RobotPose");
//    }

//    // check if robot needs to go home right away
//    double dist_home;
//    double dist_front;
//    double closest = 9999;
//    if(strategy == 1){
//        double xh = robot_home_x - robotPose.getOrigin().getX();
//        double yh = robot_home_y - robotPose.getOrigin().getY();
//        dist_home = sqrt(xh * xh + yh * yh) * 1.2; // 1.2 is extra reserve, just in case
//    }
//    else if(strategy == 2){
//        dist_home = trajectory_plan(robot_home_x, robot_home_y) * costmap_ros_->getCostmap()->getResolution();
//    }
//    if(dist_home > 0 && dist_home >= available_distance)
//        return false;

//    // look for a FRONTIER as goal
//    int errors = 0;
//    if (mode == 1)
//    {
//        for (int i = 0 + count; i < sorted_frontiers.size(); i++)
//        {
//            //we only check the first 9 frontiers, because we only sorted the first 9 frontiers by efficiency.
//            if(i>8){
//                // if the distances could not be computed, try again using euclidean distances instead
//                if(errors == i && strategy == 2){
//                    ROS_ERROR("Fallback to euclidean distance.");
//                    return this->determine_goal_staying_alive(1, 1, available_distance, final_goal, count, robot_str_name, -1);
//                }
//                return false;
//            }

//            if (check_efficiency_of_goal(sorted_frontiers.at(i).x_coordinate, sorted_frontiers.at(i).y_coordinate) == true)
//            {
//                double distance;
//                double total_distance;
//                if(strategy == 1){
//                    // distance to next frontier
//                    double x1 = sorted_frontiers.at(i).x_coordinate - robotPose.getOrigin().getX();
//                    double y1 = sorted_frontiers.at(i).y_coordinate -  robotPose.getOrigin().getY();
//                    // distance from frontier to home base
//                    double x2 = robot_home_x - sorted_frontiers.at(i).x_coordinate;
//                    double y2 = robot_home_y -  sorted_frontiers.at(i).y_coordinate;
//                    total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                }
//                else if(strategy == 2){
//                    // distance to next frontier
//                    total_distance = trajectory_plan(sorted_frontiers.at(i).x_coordinate, sorted_frontiers.at(i).y_coordinate);
//                    if(total_distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        errors++;
//                        continue;
//                    }
//                    // distance from frontier to home base
//                    distance = trajectory_plan(sorted_frontiers.at(i).x_coordinate, sorted_frontiers.at(i).y_coordinate, robot_home_x, robot_home_y);
//                    if(distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        errors++;
//                        continue;
//                    }
//                    total_distance += distance;

//                    if(total_distance < closest){
//                        closest = total_distance;
//                        dist_front = total_distance - distance;
//                        dist_home = distance;
//                    }

//                    // convert from cells to meters
//                    total_distance *= costmap_ros_->getCostmap()->getResolution();
//                }
//                else{
//                    ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                    return false;
//                }

//                ROS_INFO("Distance to frontier and then home: %.2f",total_distance);
//                if(available_distance > total_distance)
//                {
//                    ROS_INFO("------------------------------------------------------------------");
//                    ROS_INFO("Determined frontier with ID: %d   at x: %.2f     y: %.2f   detected by Robot %d", sorted_frontiers.at(i).id, sorted_frontiers.at(i).x_coordinate, sorted_frontiers.at(i).y_coordinate, sorted_frontiers.at(i).detected_by_robot);

//                    final_goal->push_back(sorted_frontiers.at(i).x_coordinate);
//                    final_goal->push_back(sorted_frontiers.at(i).y_coordinate);
//                    final_goal->push_back(sorted_frontiers.at(i).detected_by_robot);
//                    final_goal->push_back(sorted_frontiers.at(i).id);

//                    robot_str_name->push_back(sorted_frontiers.at(i).detected_by_robot_str);
//                    return true;
//                    
//                }else{
//                    ROS_INFO("No frontier in energetic range (%.2f < %.2f + %.2f)", available_distance, dist_front * costmap_ros_->getCostmap()->getResolution(), dist_home * costmap_ros_->getCostmap()->getResolution());
//                    return false;
//                }
//            }
//        }
//    }

//    // look for a CLUSTER as goal
//    else if (mode == 2)
//    {
//        int cluster_vector_position = 0;

//        if(clusters.size() > 0)
//        {
//            for (int i = 0; i < clusters.size(); i++)
//            {
//                if(clusters.at(i).id == actual_cluster_id)
//                {
//                    if(clusters.at(i).cluster_element.size() > 0)
//                    {
//                        cluster_vector_position = i;
//                    }
//                    break;
//                }
//            }
//        }

//        ROS_INFO("Calculated vector position of cluster %d", actual_cluster_id);
//        /*
//         * Iterate over all clusters .... if cluster_vector_position is set
//         * also the clusters with a lower have to be checked if the frontier
//         * determination fails at clusters.at(cluster_vector_position). therefore
//         * a ring-buffer is operated to iterate also over lower clusters, since
//         * they might have changed.
//         */
//        int nothing_found_in_actual_cluster = 0;
//        int visited_clusters = 0;
//        for (int i = 0 + count; i < clusters.size(); i++)
//        {
//            i = i+ cluster_vector_position;
//            i = i % (clusters.size());
//            for (int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//            {
//                if (check_efficiency_of_goal(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate) == true)
//                {
//                    double distance;
//                    double total_distance;
//                    if(strategy == 1){
//                        // distance to cluster
//                        double x1 = clusters.at(i).cluster_element.at(j).x_coordinate - robotPose.getOrigin().getX();
//                        double y1 = clusters.at(i).cluster_element.at(j).y_coordinate - robotPose.getOrigin().getY();
//                        // distance from cluster to home base
//                        double x2 = robot_home_x - clusters.at(i).cluster_element.at(j).x_coordinate;
//                        double y2 = robot_home_y -  clusters.at(i).cluster_element.at(j).y_coordinate;
//                        total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                    }
//                    else if(strategy == 2){
//                        // distance to cluster
//                        total_distance = trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate);
//                        if(total_distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        // distance from cluster to home base
//                        distance = trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate, robot_home_x, robot_home_y);
//                        if(distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        total_distance += distance;
//                        // convert from cells to meters
//                        total_distance *= costmap_ros_->getCostmap()->getResolution();
//                    }
//                    else{
//                        ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                        return false;
//                    }

//                    ROS_INFO("distance to cluster and then home: %f",total_distance);
//                    if(available_distance > total_distance)
//                    {
//                        ROS_INFO("------------------------------------------------------------------");
//                        ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(i).cluster_element.at(j).id, (int)clusters.at(i).id);

//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).x_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).y_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).id);

//                        // number of the cluster we operate in
//                        final_goal->push_back(clusters.at(i).id);
//                        robot_str_name->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot_str);
//                        return true;
//                    }
//                }
//            }

//            nothing_found_in_actual_cluster ++;
//            visited_clusters ++;

//            if(nothing_found_in_actual_cluster == 1)
//            {
//                //start again at the beginning(closest cluster))
//                i=0;
//                cluster_vector_position = 0;
//            }

//            if(visited_clusters == clusters.size())
//            {
//                ROS_ERROR("No frontier in energetic range %.2f, going home for recharging", available_distance);
//                return false;
//            }
//        }
//    }
//    
//}

bool ExplorationPlanner::determine_goal(int strategy, std::vector<double> *final_goal, int count, int actual_cluster_id, std::vector<std::string> *robot_str_name)
{
    if (!costmap_ros_->getRobotPose(robotPose))
    {
            ROS_ERROR("Failed to get RobotPose");
    }

    if(strategy == 1)
    {
            for (int i = frontiers.size() - 1 - count; i >= 0; i--)
            {
                    if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
                    {
                            ROS_INFO("------------------------------------------------------------------");
                            ROS_INFO("Determined frontier with ID: %d   at x: %f     y: %f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

                            /*
                             * Solve the Problem of planning a path to a frontier which is located
                             * directly "in" the obstacle. Therefore back-off 5% of the targets
                             * coordinate. Since the direction of x and y can be positive and
                             * negative either, multiply the coordinate with 0.95 to get 95% of its
                             * original value.
                             */
                            final_goal->push_back(frontiers.at(i).x_coordinate); //final_goal.push_back(frontiers.at(i).x_coordinate*0.95 - robotPose.getOrigin().getX());
                            final_goal->push_back(frontiers.at(i).y_coordinate);
                            final_goal->push_back(frontiers.at(i).detected_by_robot);
                            final_goal->push_back(frontiers.at(i).id);

                            robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
                            return true;
                    }
            }
            return false;
        }

        else if(strategy == 2)
        {
            for (unsigned int i = 0 + count; i < frontiers.size(); i++)
            {
                if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
                {
                        ROS_INFO("------------------------------------------------------------------");
                        ROS_INFO("Determined frontier with ID: %d   at x: %f     y: %f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

                        final_goal->push_back(frontiers.at(i).x_coordinate);
                        final_goal->push_back(frontiers.at(i).y_coordinate);
                        final_goal->push_back(frontiers.at(i).detected_by_robot);
                        final_goal->push_back(frontiers.at(i).id);

                        robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
                        return true;
                }
            }
        }

        else if(strategy == 3)
        {
            while(true)
            {
                if(frontiers.size() > 0)
                {

                    int i = int(frontiers.size()*rand()/(RAND_MAX));

                    ROS_INFO("Random frontier ID: %d", frontiers.at(i).id);

                    if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
                    {
                            ROS_INFO("------------------------------------------------------------------");
                            ROS_INFO("Determined frontier with ID: %d   at x: %f     y: %f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

                            final_goal->push_back(frontiers.at(i).x_coordinate);
                            final_goal->push_back(frontiers.at(i).y_coordinate);
                            final_goal->push_back(frontiers.at(i).detected_by_robot);
                            final_goal->push_back(frontiers.at(i).id);

                            robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
                            return true;
                    }
                }
                break;
            }
	}
        else if(strategy == 4)
        {
                int cluster_vector_position = 0;

                if(actual_cluster_id != -1)
                {
                    if(clusters.size() > 0)
                    {
                        for (unsigned int i = 0; i < clusters.size(); i++)
                        {
                            if(clusters.at(i).id == actual_cluster_id)
                            {
                                if(clusters.at(i).cluster_element.size() > 0)
                                {
                                    cluster_vector_position = i;
                                }
                                break;
                            }
                        }
                    }
                }

                ROS_INFO("Calculated vector position of cluster %d", actual_cluster_id);
                /*
                 * Iterate over all clusters .... if cluster_vector_position is set
                 * also the clusters with a lower have to be checked if the frontier
                 * determination fails at clusters.at(cluster_vector_position). therefore
                 * a ring-buffer is operated to iterate also over lower clusters, since
                 * they might have changed.
                 */
                int nothing_found_in_actual_cluster = 0;
                int visited_clusters = 0;
                for (unsigned int i = 0 + count; i < clusters.size(); i++)
                {
//                    ROS_INFO("Cluster vector: %d  i: %d ", cluster_vector_position, i);
                    i = i+ cluster_vector_position;
                    i = i % (clusters.size());
                    for (unsigned int j = 0; j < clusters.at(i).cluster_element.size(); j++)
                    {
                        if (check_efficiency_of_goal(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate) == true)
                        {
                                ROS_INFO("------------------------------------------------------------------");
                                ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(i).cluster_element.at(j).id, (int)clusters.at(i).id);

                                final_goal->push_back(clusters.at(i).cluster_element.at(j).x_coordinate);
                                final_goal->push_back(clusters.at(i).cluster_element.at(j).y_coordinate);
                                final_goal->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot);
                                final_goal->push_back(clusters.at(i).cluster_element.at(j).id);

                                // number of the cluster we operate in
                                final_goal->push_back(clusters.at(i).id);
                                robot_str_name->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot_str);
                                return true;
                        }

                    }

                    nothing_found_in_actual_cluster ++;
                    visited_clusters ++;

                    if(nothing_found_in_actual_cluster == 1)
                    {
                        //start again at the beginning(closest cluster))
                        i=0;
                        cluster_vector_position = 0;
                    }

                    if(visited_clusters == clusters.size())
                    {
                        break;
                    }
               }
            }
            else if(strategy == 5)
            {
                int cluster_vector_position = 0;
                bool cluster_could_be_found = false;

                if(actual_cluster_id != -1)
                {
                    if(clusters.size() > 0)
                    {
                        for (unsigned int i = 0; i < clusters.size(); i++)
                        {
                            if(clusters.at(i).id == actual_cluster_id)
                            {
                                if(clusters.at(i).cluster_element.size() > 0)
                                {
                                    cluster_vector_position = i;
                                    cluster_could_be_found = true;
                                    break;
                                }
                            }
                        }
                    }
                    if(cluster_could_be_found == false)
                    {
                        /*
                         * The cluster we operated in is now empty
                         */
                        return false;
                    }
                }
                else if(actual_cluster_id == -1)
                {
                    // No cluster was previously selected
                    final_goal->push_back(-1.0);
                    return false;
                }
                ROS_INFO("Calculated vector position: %d of cluster %d", cluster_vector_position, actual_cluster_id);
                /*
                 * Iterate over all clusters .... if cluster_vector_position is set
                 * also the clusters with a lower have to be checked if the frontier
                 * determination fails at clusters.at(cluster_vector_position). therefore
                 * a ring-buffer is operated to iterate also over lower clusters, since
                 * they might have changed.
                 */
                int nothing_found_in_actual_cluster = 0;
                int visited_clusters = 0;

//                if(clusters.size() > 0)
//                {

                    int position = (cluster_vector_position +count) % (clusters.size());
                    for (unsigned int j = 0; j < clusters.at(position).cluster_element.size(); j++)
                    {
                        if(count >= clusters.size())
                        {
                            break;
                        }

                        if(clusters.at(position).cluster_element.size() > 0)
                        {
                            if (check_efficiency_of_goal(clusters.at(position).cluster_element.at(j).x_coordinate, clusters.at(position).cluster_element.at(j).y_coordinate) == true)
                            {
                                    ROS_INFO("------------------------------------------------------------------");
                                    ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(position).cluster_element.at(j).id, (int)clusters.at(position).id);

                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).x_coordinate);
                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).y_coordinate);
                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).detected_by_robot);
                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).id);

                                    // number of the cluster we operate in
                                    final_goal->push_back(clusters.at(position).id);
                                    robot_str_name->push_back(clusters.at(position).cluster_element.at(j).detected_by_robot_str);
                                    return true;
                            }
                        }

                    }

//                }
//                ROS_INFO("No clusters are available anymore");

                return false;
            }
            else if(strategy == 6)
            {
                int cluster_vector_position = 0;
                bool cluster_could_be_found = false;

                if(actual_cluster_id != -1)
                {
                    if(clusters.size() > 0)
                    {
                        for (unsigned int i = 0; i < clusters.size(); i++)
                        {
                            if(clusters.at(i).id == actual_cluster_id)
                            {
                                if(clusters.at(i).cluster_element.size() > 0)
                                {
                                    cluster_vector_position = i;
                                    cluster_could_be_found = true;
                                    break;
                                }
                            }
                        }
                    }
                    if(cluster_could_be_found == false)
                    {
                        /*
                         * The cluster we operated in is now empty
                         */
                        return false;
                    }
                }
                else if(actual_cluster_id == -1)
                {
                    // No cluster was previously selected
                    final_goal->push_back(-1.0);
                    return false;
                }
                ROS_INFO("Calculated vector position: %d of cluster %d", cluster_vector_position, actual_cluster_id);
                /*
                 * Iterate over all clusters .... if cluster_vector_position is set
                 * also the clusters with a lower have to be checked if the frontier
                 * determination fails at clusters.at(cluster_vector_position). therefore
                 * a ring-buffer is operated to iterate also over lower clusters, since
                 * they might have changed.
                 */
                int nothing_found_in_actual_cluster = 0;
                int visited_clusters = 0;

//                    ROS_INFO("position: %u", (cluster_vector_position +count) % (clusters.size()));
                    int position = (cluster_vector_position +count) % (clusters.size());
                    for (unsigned int j = 0; j < clusters.at(position).cluster_element.size(); j++)
                    {
                        if(count >= clusters.size())
                        {
                            break;
                        }

                        if(clusters.at(position).cluster_element.size() > 0)
                        {
                            if (check_efficiency_of_goal(clusters.at(position).cluster_element.at(j).x_coordinate, clusters.at(position).cluster_element.at(j).y_coordinate) == true)
                            {
                                    ROS_INFO("------------------------------------------------------------------");
                                    ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(position).cluster_element.at(j).id, (int)clusters.at(position).id);

                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).x_coordinate);
                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).y_coordinate);
                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).detected_by_robot);
                                    final_goal->push_back(clusters.at(position).cluster_element.at(j).id);

                                    // number of the cluster we operate in
                                    final_goal->push_back(clusters.at(position).id);
                                    robot_str_name->push_back(clusters.at(position).cluster_element.at(j).detected_by_robot_str);
                                    return true;
                            }
//                            else
//                            {
//                                final_goal->push_back(-1);
//                                return true;
//                            }
                        }

                    }
                return false;
            }
     return false;
}

//void ExplorationPlanner::smart_sort_cost(bool energy_above_th, int w1, int w2, int w3, int w4)
//{

//    //ROS_INFO("waiting for lock");
//    //store_frontier_mutex.lock();
//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
//    //ROS_INFO("lock acquired");
//    
//    tf::Stamped < tf::Pose > robotPose;
//    if(!costmap_ros_->getRobotPose(robotPose))
//    {
//        ROS_ERROR("Failed to get RobotPose");
//        release_mutex(&store_frontier_mutex, __FUNCTION__);
//        return;
//    }
//    
//    my_energy_above_th = energy_above_th;
//    this-> w1 = w1;
//    this-> w2 = w2;
//    this-> w3 = w3;
//    this-> w4 = w4;

//    // process only eight frontiers
//    //int max_front = 8;
//    int max_front = frontiers.size();
//    sorted_frontiers.clear();
//    
//    if(frontiers.size() > 0)
//    {
//        //ROS_ERROR("%u", frontiers.size());
//        double robot_x, robot_y;

//        //continue_bool = false;
//        //return;
//        //ROS_ERROR("sort: %d", (int)frontiers.size() - 1 - i);
//        //if( ((int)frontiers.size() - 1 - i) < 0) {
//        //    ROS_FATAL("Somethign bad happened....");
//        //    release_mutex(&store_frontier_mutex, __FUNCTION__);
//        //    return;
//        //}
//        
//        //TEMP
//        //if((int)frontiers.size() - 1 - i >= 1)
//        //    return;
//            
//        for(int j = 0; j < frontiers.size()-1 && j < max_front; ++j)
//        {

//            //ROS_ERROR("sort2: %d", j);
//            /*
//             * cost function
//             * f = w1 · d_g   +   w2 · d_gb   +   w3 · d_gbe   +   w4 · theta
//             *
//             * parameters
//             * w1, ..., w4 .. weights
//             * d_g         .. distance from the robot's current position to the frontier
//             * d_gb        .. distance from the frontier to the charging station
//             * d_gbe       .. -d_gb if battery charge is above threshold (e.g. 50%), d_gb if battery charge is below threshold
//             * theta       .. measure of how much the robot has to turn to reach frontier, theta in [0,1]
//             */

//            // robot position
//            robot_x = robotPose.getOrigin().getX();
//            robot_y = robotPose.getOrigin().getY();

//            // frontier position
//            double frontier_x = frontiers.at(j).x_coordinate;
//            double frontier_y = frontiers.at(j).y_coordinate;
//            double next_frontier_x = frontiers.at(j+1).x_coordinate;
//            double next_frontier_y = frontiers.at(j+1).y_coordinate;

//            // calculate d_g
//            int d_g = trajectory_plan(frontier_x, frontier_y);
//            int d_g_next = trajectory_plan(next_frontier_x, next_frontier_y);

//            // calculate d_gb
//            int d_gb = trajectory_plan(frontier_x, frontier_y, robot_home_x, robot_home_y);
//            int d_gb_next = trajectory_plan(next_frontier_x, next_frontier_y, robot_home_x, robot_home_y);

//            // calculate d_gbe
//            int d_gbe, d_gbe_next;
//            if(energy_above_th)
//            {
//                d_gbe = -d_gb;
//                d_gbe_next = -d_gb_next;
//            }
//            else
//            {
//                d_gbe = d_gb;
//                d_gbe_next = d_gb_next;
//            }

//            // calculate theta
//            double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
//            double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
//            double theta_g_next = atan2(robot_y - next_frontier_y, robot_x - next_frontier_x);
//            double theta = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g) - M_PI));
//            double theta_next = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g_next) - M_PI));

//            // calculate cost function
//            double cost = w1 * d_g + w2 * d_gb + w3 * d_gbe + w4 * theta;
//            double cost_next = w1 * d_g_next + w2 * d_gb_next + w3 * d_gbe_next + w4 * theta_next;

//            // sort frontiers according to cost function
//            //F ascending order
//            if(cost > cost_next)
//            {
//                frontier_t temp = frontiers.at(j+1);
//                frontiers.at(j+1) = frontiers.at(j);
//                frontiers.at(j) = temp;
//            }
//            
//            //bool to_be_inserted
//            //if(sorted_frontiers.size() == 0)
//            //    sorted_frontier.push_back(frontiers.at(j));
//            //else
//            //    for(int k=0; k < sorted_frontiers.size(); k++)
//            //        if(   
//            
//        }
//        
//        robot_last_x = robot_x;
//        robot_last_y = robot_y;
//    }
//    else
//    {
//        ROS_INFO("Sorting not possible, no frontiers available!!!");
//    }
//    
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    ROS_INFO("finished sort cost");
//  
//}

void ExplorationPlanner::my_sort_cost_2(bool energy_above_th, int w1, int w2, int w3, int w4)
{
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    
    // robot position
    double robot_x = robotPose.getOrigin().getX();
    double robot_y = robotPose.getOrigin().getY();
    
    this->my_energy_above_th = energy_above_th;
    this->w1 = w1;
    this->w2 = w2;
    this->w3 = w3;
    this->w4 = w4;

    for (int j = 0; j < frontiers.size() - 1; j++) {
        double x = frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
        double y = frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
        double cost = x * x + y * y;
        frontiers.at(j).cost = cost;
        
        add_to_sorted_fontiers_list_if_convinient(frontiers.at(j));
        
    }
    
    frontier_selected = true;
    ROS_DEBUG("sorted_frontiers size: %u", sorted_frontiers.size());
        
    robot_last_x = robot_x;
    robot_last_y = robot_y;
    
    release_mutex(&store_frontier_mutex, __FUNCTION__);
    ROS_INFO("finished my_sort_cost_2");
  
}

void ExplorationPlanner::my_sort_cost_3(bool energy_above_th, int w1, int w2, int w3, int w4)
{
    
    if(ds_list.size() == 0) {
        ROS_INFO("No DS is currently known: falling back to closest st");
        my_sort_cost_2(energy_above_th, w1, w2, w3, w4);
        return;   
    }
    
    ROS_INFO("Using this strategy");
    
    // robot position
    double robot_x = robotPose.getOrigin().getX();
    double robot_y = robotPose.getOrigin().getY();

//    if(recompute_ds_graph) {
//        recompute_ds_graph = false;
//        for(int i=0; i < ds_list.size(); i++)
//            for(int j=0; j < ds_list.size(); j++)
//                if(ds_graph[i][j] >= 0 || i == j)
//                    continue;
//                else {
//                    double dist = trajectory_plan(ds_list[i].x, ds_list[i].y, ds_list[j].x, ds_list[j].y);
//                    if(dist < 0) {
//                        ROS_ERROR("Unable to compute distance at the moment: retrying later...");
//                        recompute_ds_graph = true;
//                    }
//                    else {
//                        ds_graph[ds_list[i].id][ds_list[j].id] = dist;
//                        ds_graph[ds_list[j].id][ds_list[i].id] = dist;
//                    }
//                }
//    }
    
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    
    int index_closest_ds_to_robot = -1, index_optimal_ds = -1;
    double min_dist = std::numeric_limits<double>::max();
    for(unsigned int i=0; i < ds_list.size(); i++) {
        double distance = euclidean_distance(robot_x, robot_y, ds_list.at(i).x, ds_list.at(i).y);
        if(distance < min_dist) {
            min_dist = distance;
            index_closest_ds_to_robot = i;
        }
        if(ds_list.at(i).id == optimal_ds_id)
            index_optimal_ds = i;
    }
    
    for(unsigned int i=0; i < frontiers.size(); i++) {
        double min_dist = std::numeric_limits<double>::max();
        int index_closest_ds_to_frontier = -1;
        for(unsigned int j=0; j < ds_list.size(); j++) {
            double distance = euclidean_distance(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, ds_list.at(j).x, ds_list.at(j).y);
            if(distance < min_dist) {
                min_dist = distance;
                index_closest_ds_to_frontier = j;
            }
        }
        
        if(index_optimal_ds >= ds_graph.size() || index_closest_ds_to_robot >= ds_graph[index_optimal_ds].size() || index_closest_ds_to_robot < 0 || index_optimal_ds < 0 || index_closest_ds_to_frontier < 0)
            ROS_ERROR("invalid index(-ces)"); //: %d, %d, %u", index_optimal_ds, index_closest_ds_to_robot, ds_graph.size());
            //ROS_ERROR("index_optimal_ds is too high; ds_graph size: %u; index_optimal_ds: %d", ds_graph.size(), index_optimal_ds);
        
        //double d_gbe = dijkstra(ds_graph, index_optimal_ds, index_closest_ds_to_robot);
        double d_gbe = ds_graph[index_optimal_ds][index_closest_ds_to_robot];
        if(d_gbe < 0)
            ROS_ERROR("Invalid value");
        //double d_g = dijkstra(ds_graph, index_closest_ds_to_frontier, index_closest_ds_to_robot);
        double d_g = ds_graph[index_closest_ds_to_frontier][index_closest_ds_to_robot];
        if(d_g < 0)
            ROS_ERROR("Invalid value");
        double cost = d_g + d_gbe;
        frontiers.at(i).cost = cost;
        
        add_to_sorted_fontiers_list_if_convinient(frontiers.at(i));
        
    }
    
    frontier_selected = true;
    my_energy_above_th = energy_above_th;
    this->w1 = w1;
    this->w2 = w2;
    this->w3 = w3;
    this->w4 = w4;
    
    robot_last_x = robot_x;
    robot_last_y = robot_y;
    
    ROS_INFO("finished my_sort_cost_2");
    release_mutex(&store_frontier_mutex, __FUNCTION__);
  
}

void ExplorationPlanner::my_select_4(double available_distance, bool energy_above_th, int w1, int w2, int w3, int w4, std::vector<double> *final_goal, std::vector<std::string> *robot_str_name)
{
    
    // robot position
    double robot_x = robotPose.getOrigin().getX();
    double robot_y = robotPose.getOrigin().getY();
    
    my_energy_above_th = energy_above_th;
    this->w1 = w1;
    this->w2 = w2;
    this->w3 = w3;
    this->w4 = w4;
    
    if(frontiers.size() > 0)
    {
        ROS_INFO("%u", frontiers.size());
        
        double min_cost = std::numeric_limits<double>::max();    
        
        acquire_mutex(&store_frontier_mutex, __FUNCTION__);
        for(int j = 0; j < frontiers.size(); j++)
        {
        
            if(!my_check_efficiency_of_goal(available_distance, &frontiers.at(j)))
                continue;
                
            bool under_auction = false;
            for(int i=0; i < frontiers_under_auction.size(); i++)
                if(frontiers.at(j).id == frontiers_under_auction.at(i).id)
                    under_auction = true;
            if(under_auction)
                continue;
            
            frontiers.at(j).cost = frontier_cost(frontiers.at(j));
            add_to_sorted_fontiers_list_if_convinient(frontiers.at(j));
            
            if(frontiers.at(j).cost < min_cost) {
                frontier_selected = true;
                min_cost = frontiers.at(j).cost;
            }
                  
        }
        release_mutex(&store_frontier_mutex, __FUNCTION__);
        
        robot_last_x = robot_x;
        robot_last_y = robot_y;
        
    }
    else
    {
        ROS_INFO("Sorting not possible, no frontiers available");
    }
    
    if(frontier_selected) {
        
        final_goal->push_back(my_selected_frontier->x_coordinate);
        final_goal->push_back(my_selected_frontier->y_coordinate);
        ROS_ERROR("%.2f, %.2f", my_selected_frontier->x_coordinate, my_selected_frontier->y_coordinate);
        final_goal->push_back(my_selected_frontier->detected_by_robot);
        final_goal->push_back(my_selected_frontier->id);
        
        robot_str_name->push_back(robot_name_str);  //TODO ???
    
    }
    
    release_mutex(&store_frontier_mutex, __FUNCTION__);
    ROS_INFO("finished my_select_4");
  
}

/**
 * Sort frontiers in an energy aware manner according to a cost function with the weights w1, ..., w4
 */
//void ExplorationPlanner::sort_cost(bool energy_above_th, int w1, int w2, int w3, int w4)
//{

//#ifndef QUICK_SELECTION

//    //ROS_INFO("waiting for lock");
//    store_frontier_mutex.lock();
//    //ROS_INFO("lock acquired");
//    
//    tf::Stamped < tf::Pose > robotPose;
//    if(!costmap_ros_->getRobotPose(robotPose))
//    {
//        ROS_ERROR("Failed to get RobotPose");
//        release_mutex(&store_frontier_mutex, __FUNCTION__);
//        return;
//    }
//    
//    my_energy_above_th = energy_above_th;
//    this-> w1 = w1;
//    this-> w2 = w2;
//    this-> w3 = w3;
//    this-> w4 = w4;

//    // process only eight frontiers
//    int max_front = 8;

//    if(frontiers.size() > 0)
//    {
//        //ROS_ERROR("%u", frontiers.size());
//        double robot_x, robot_y;
//        
//        for(int i = frontiers.size()-1; i >= 0 && i > frontiers.size() - max_front; --i)
//        //for(int i = frontiers.size(); i >= 0 && i > frontiers.size() - max_front - skipped_due_to_auction; --i)
//        {
//          
//            //continue_bool = false;
//            //return;
//            //ROS_ERROR("sort: %d", (int)frontiers.size() - 1 - i);
//            if( ((int)frontiers.size() - 1 - i) < 0) {
//                ROS_FATAL("Somethign bad happened....");
//                release_mutex(&store_frontier_mutex, __FUNCTION__);
//                return;
//            }
//            
//            //TEMP
//            //if((int)frontiers.size() - 1 - i >= 1)
//            //    return;
//                
//            for(int j = 0; j < frontiers.size()-1 && j < max_front; ++j)
//            {

//                //ROS_ERROR("sort2: %d", j);
//                /*
//                 * cost function
//                 * f = w1 · d_g   +   w2 · d_gb   +   w3 · d_gbe   +   w4 · theta
//                 *
//                 * parameters
//                 * w1, ..., w4 .. weights
//                 * d_g         .. distance from the robot's current position to the frontier
//                 * d_gb        .. distance from the frontier to the charging station
//                 * d_gbe       .. -d_gb if battery charge is above threshold (e.g. 50%), d_gb if battery charge is below threshold
//                 * theta       .. measure of how much the robot has to turn to reach frontier, theta in [0,1]
//                 */

//                // robot position
//                robot_x = robotPose.getOrigin().getX();
//                robot_y = robotPose.getOrigin().getY();

//                // frontier position
//                double frontier_x = frontiers.at(j).x_coordinate;
//                double frontier_y = frontiers.at(j).y_coordinate;
//                double next_frontier_x = frontiers.at(j+1).x_coordinate;
//                double next_frontier_y = frontiers.at(j+1).y_coordinate;

//                // calculate d_g
//                int d_g = trajectory_plan(frontier_x, frontier_y);
//                int d_g_next = trajectory_plan(next_frontier_x, next_frontier_y);

//                // calculate d_gb
//                int d_gb = trajectory_plan(frontier_x, frontier_y, robot_home_x, robot_home_y);
//                int d_gb_next = trajectory_plan(next_frontier_x, next_frontier_y, robot_home_x, robot_home_y);

//                // calculate d_gbe
//                int d_gbe, d_gbe_next;
//                if(energy_above_th)
//                {
//                    d_gbe = -d_gb;
//                    d_gbe_next = -d_gb_next;
//                }
//                else
//                {
//                    d_gbe = d_gb;
//                    d_gbe_next = d_gb_next;
//                }

//                // calculate theta
//                double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
//                double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
//                double theta_g_next = atan2(robot_y - next_frontier_y, robot_x - next_frontier_x);
//                double theta = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g) - M_PI));
//                double theta_next = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g_next) - M_PI));

//                // calculate cost function
//                double cost = w1 * d_g + w2 * d_gb + w3 * d_gbe + w4 * theta;
//                double cost_next = w1 * d_g_next + w2 * d_gb_next + w3 * d_gbe_next + w4 * theta_next;

//                // sort frontiers according to cost function
//                //F ascending order
//                if(cost > cost_next)
//                {
//                    frontier_t temp = frontiers.at(j+1);
//                    frontiers.at(j+1) = frontiers.at(j);
//                    frontiers.at(j) = temp;
//                }
//            }
//        }
//        robot_last_x = robot_x;
//        robot_last_y = robot_y;
//    }
//    else
//    {
//        ROS_INFO("Sorting not possible, no frontiers available!!!");
//    }
//    
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    ROS_INFO("finished sort cost");
//    
//#endif
// 
//}

void ExplorationPlanner::new_ds_on_graph_callback(const adhoc_communication::EmDockingStation msg) {
    //ROS_ERROR("RECEVIED!");
    
    if(num_ds <= 0) {       
        num_ds = msg.total_number_of_ds;
        //ds_list.resize(num_ds);
        for (int i = 0; i < num_ds; i++)
        {
            std::vector<float> temp_f;
            for (int j = 0; j < num_ds; j++) {
                temp_f.push_back(-1);
            }
            ds_graph.push_back(temp_f);
        }
    }
    
    ds_t ds;
    ds.x = msg.x;
    ds.y = msg.y;
    ds.id = msg.id;
    //ROS_ERROR("%d, %f, %f", ds.id, ds.x, ds.y);
    
//    double dist = trajectory_plan_meters(0, 0, ds.x, ds.y);
    //ROS_ERROR("distance between (%d, %d) and (%.1f, %.1f): %f", 0, 0, ds.x, ds.y, dist);
    
    for(int i=0; i < ds_list.size(); i++) {
        double dist = trajectory_plan_meters(ds_list[i].x, ds_list[i].y, ds.x, ds.y);
        //ROS_ERROR("distance between (%.1f, %.1f) and (%.1f, %.1f): %f", ds_list[i].x, ds_list[i].y, ds.x, ds.y, dist);
        if(dist < 0) {
            ROS_ERROR("Unable to compute distance at the moment: retrying later...");
            recompute_ds_graph = true;
        }
        else {
            if(ds_list[i].id >= ds_graph.size() || ds.id >= ds_graph.size() || ds_list[i].id >= ds_graph[ds_list[i].id].size() || ds.id >= ds_graph[ds.id].size()) {
                ROS_FATAL("!!!OUT OF RANGE!!! %d, %d, %u", ds_list[i].id, ds.id, ds_graph.size());
                return;   
            }
            ds_graph[ds_list[i].id][ds.id] = dist;
            ds_graph[ds.id][ds_list[i].id] = dist;
        }
    }
    ds_graph[ds.id][ds.id] = 0;
    for(int i=0; i < ds_list.size(); i++)
        for(int j=0; j < ds_list.size(); j++)
            ; //ROS_ERROR("%f", ds_graph[i][j]);
            
    ds_list.push_back(ds);
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
            ds_graph.push_back(temp_f);
        }
    }
}

//void ExplorationPlanner::sort_cost_1(bool energy_above_th, int w1, int w2, int w3, int w4)
//{
//    //ROS_ERROR("calling %s", sc_distance_frontier_robot.getService().c_str());
//    //explorer::Distance distance_srv_msg;
//    //distance_srv_msg.request.x1 = 10;
//    //ros::service::waitForService("energy_mgmt/distance_on_graph");
//    //publish_frontier_list();
//    
//    /*
//    if(sc_distance_frontier_robot.call(distance_srv_msg))
//        ; //ROS_ERROR("call ok!");
//    else
//        ROS_ERROR("call failed!!!");
//    */
//    
//    /*
//    while(num_ds <= 0) {
//        ROS_ERROR("waiting DS count");
//        ros::Duration(2).sleep();
//        ros::spinOnce();
//    }
//    */
//    
//    if(num_ds <= 0)
//        return;
//    

//#ifndef QUICK_SELECTION

//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
//    
//    my_energy_above_th = energy_above_th;
//    this-> w1 = w1;
//    this-> w2 = w2;
//    this-> w3 = w3;
//    this-> w4 = w4;

//    // process only eight frontiers
//    int max_front = 8;

//    if(frontiers.size() > 0)
//    {
//        //ROS_ERROR("%u", frontiers.size());
//        double robot_x, robot_y;
//        
//        for(int i = frontiers.size()-1; i >= 0 && i > frontiers.size() - max_front; --i)
//        //for(int i = frontiers.size(); i >= 0 && i > frontiers.size() - max_front - skipped_due_to_auction; --i)
//        {
//            
//            //continue_bool = false;
//            //return;
//            //ROS_ERROR("sort: %d", (int)frontiers.size() - 1 - i);
//            if( ((int)frontiers.size() - 1 - i) < 0) {
//                ROS_FATAL("Somethign bad happened....");
//                release_mutex(&store_frontier_mutex, __FUNCTION__);
//                return;
//            }
//            
//            //TEMP
//            //if((int)frontiers.size() - 1 - i >= 1)
//            //    return;
//                
//            for(int j = 0; j < frontiers.size()-1 && j < max_front; ++j)
//            {

//                //ROS_ERROR("sort2: %d", j);
//                /*
//                 * cost function
//                 * f = w1 · d_g   +   w2 · d_gb   +   w3 · d_gbe   +   w4 · theta
//                 *
//                 * parameters
//                 * w1, ..., w4 .. weights
//                 * d_g         .. distance from the robot's current position to the frontier
//                 * d_gb        .. distance from the frontier to the charging station
//                 * d_gbe       .. -d_gb if battery charge is above threshold (e.g. 50%), d_gb if battery charge is below threshold
//                 * theta       .. measure of how much the robot has to turn to reach frontier, theta in [0,1]
//                 */

//                // robot position
//                robot_x = robotPose.getOrigin().getX();
//                robot_y = robotPose.getOrigin().getY();

//                // frontier position
//                double frontier_x = frontiers.at(j).x_coordinate;
//                double frontier_y = frontiers.at(j).y_coordinate;
//                double next_frontier_x = frontiers.at(j+1).x_coordinate;
//                double next_frontier_y = frontiers.at(j+1).y_coordinate;

//                // calculate d_g
//                int d_g = trajectory_plan_meters(frontier_x, frontier_y);
//                int d_g_next = trajectory_plan_meters(next_frontier_x, next_frontier_y);

//                // calculate d_gb
//                int d_gb = trajectory_plan_meters(frontier_x, frontier_y, robot_home_x, robot_home_y);
//                int d_gb_next = trajectory_plan_meters(next_frontier_x, next_frontier_y, robot_home_x, robot_home_y);

//                // calculate d_gbe
//                int d_gbe, d_gbe_next;
//                if(energy_above_th)
//                {
//                    d_gbe = -d_gb;
//                    d_gbe_next = -d_gb_next;
//                }
//                else
//                {
//                    d_gbe = d_gb;
//                    d_gbe_next = d_gb_next;
//                }

//                // calculate theta
//                double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
//                double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
//                double theta_g_next = atan2(robot_y - next_frontier_y, robot_x - next_frontier_x);
//                double theta = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g) - M_PI));
//                double theta_next = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g_next) - M_PI));

//                // calculate cost function
//                double cost = w1 * d_g + w2 * d_gb + w3 * d_gbe + w4 * theta;
//                double cost_next = w1 * d_g_next + w2 * d_gb_next + w3 * d_gbe_next + w4 * theta_next;

//                // sort frontiers according to cost function
//                //F ascending order
//                if(cost > cost_next)
//                {
//                    frontier_t temp = frontiers.at(j+1);
//                    frontiers.at(j+1) = frontiers.at(j);
//                    frontiers.at(j) = temp;
//                }
//            }
//        }
//        robot_last_x = robot_x;
//        robot_last_y = robot_y;
//    }
//    else
//    {
//        ROS_INFO("Sorting not possible, no frontiers available!!!");
//    }
//    
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    ROS_INFO("finished sort cost");
//    
//#endif
// 
//}

void ExplorationPlanner::sort_cost_reserve(bool energy_above_th, int w1, int w2, int w3, int w4)
{
//    tf::Stamped < tf::Pose > robotPose;
//    if(!costmap_ros_->getRobotPose(robotPose))
//    {
//        ROS_ERROR("Failed to get RobotPose");
//        return;
//    }

//    // process only eight frontiers
//    int max_front = 8;

//    if(sorted_frontiers.size() > 0)
//    {
//        //ROS_ERROR("%u", sorted_frontiers.size());
//        double robot_x, robot_y;
//        
//        //for(int i = sorted_frontiers.size(); i >= 0 && i > sorted_frontiers.size() - max_front; --i)
//        bool continue_bool = true;
//        for(int i = sorted_frontiers.size() - 1; continue_bool && i >= 0 && i > sorted_frontiers.size() - max_front - 1; --i)
//        {
//            //continue_bool = false;
//            //return;
//            //ROS_ERROR("sort: %d", (int)sorted_frontiers.size() - 1 - i);
//            if( ((int)sorted_frontiers.size() - 1 - i) < 0) {
//                ROS_FATAL("Somethign bad happened....");
//                return;
//            }
//            
//            //TEMP
//            //if((int)sorted_frontiers.size() - 1 - i >= 1)
//            //    return;
//                
//            for(int j = 0; j < sorted_frontiers.size()-1 && j < max_front; ++j)
//            {
//                //ROS_ERROR("sort2: %d", j);
//                /*
//                 * cost function
//                 * f = w1 · d_g   +   w2 · d_gb   +   w3 · d_gbe   +   w4 · theta
//                 *
//                 * parameters
//                 * w1, ..., w4 .. weights
//                 * d_g         .. distance from the robot's current position to the frontier
//                 * d_gb        .. distance from the frontier to the charging station
//                 * d_gbe       .. -d_gb if battery charge is above threshold (e.g. 50%), d_gb if battery charge is below threshold
//                 * theta       .. measure of how much the robot has to turn to reach frontier, theta in [0,1]
//                 */

//                // robot position
//                robot_x = robotPose.getOrigin().getX();
//                robot_y = robotPose.getOrigin().getY();

//                // frontier position
//                double frontier_x = sorted_frontiers.at(j).x_coordinate;
//                double frontier_y = sorted_frontiers.at(j).y_coordinate;
//                double next_frontier_x = sorted_frontiers.at(j+1).x_coordinate;
//                double next_frontier_y = sorted_frontiers.at(j+1).y_coordinate;

//                // calculate d_g
//                int d_g = trajectory_plan(frontier_x, frontier_y);
//                int d_g_next = trajectory_plan(next_frontier_x, next_frontier_y);

//                // calculate d_gb
//                int d_gb = trajectory_plan(frontier_x, frontier_y, robot_home_x, robot_home_y);
//                int d_gb_next = trajectory_plan(next_frontier_x, next_frontier_y, robot_home_x, robot_home_y);

//                // calculate d_gbe
//                int d_gbe, d_gbe_next;
//                if(energy_above_th)
//                {
//                    d_gbe = -d_gb;
//                    d_gbe_next = -d_gb_next;
//                }
//                else
//                {
//                    d_gbe = d_gb;
//                    d_gbe_next = d_gb_next;
//                }

//                // calculate theta
//                double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
//                double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
//                double theta_g_next = atan2(robot_y - next_frontier_y, robot_x - next_frontier_x);
//                double theta = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g) - M_PI));
//                double theta_next = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g_next) - M_PI));

//                // calculate cost function
//                double cost = w1 * d_g + w2 * d_gb + w3 * d_gbe + w4 * theta;
//                double cost_next = w1 * d_g_next + w2 * d_gb_next + w3 * d_gbe_next + w4 * theta_next;

//                // sort sorted_frontiers according to cost function
//                if(cost > cost_next)
//                {
//                    frontier_t temp = sorted_frontiers.at(j+1);
//                    sorted_frontiers.at(j+1) = sorted_frontiers.at(j);
//                    sorted_frontiers.at(j) = temp;
//                }
//            }
//        }
//        robot_last_x = robot_x;
//        robot_last_y = robot_y;
//    }
//    else
//    {
//        ROS_INFO("Sorting not possible, no frontiers available!!!");
//    }
//    
}



void ExplorationPlanner::sort(int strategy)
{
    tf::Stamped < tf::Pose > robotPose;
    if (!costmap_ros_->getRobotPose(robotPose))
    {
        ROS_ERROR("Failed to get RobotPose");
        return;
    }
    if (frontiers.size() <= 0 && clusters.size() <= 0)
    {
        ROS_INFO("Sorting not possible, no frontiers available!!!");
        return;
    }

    double pose_x = robotPose.getOrigin().getX();
    double pose_y = robotPose.getOrigin().getY();
    
    //store_frontier_mutex.lock();
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    
    //for (int i = frontiers.size()-1; i >= 0; i--) {
    //    ROS_ERROR("frontier %d: (%f, %f)", i, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
    //}

    /*
     * Following Sort algorithm normalizes all distances to the
     * robots actual position. As result, the list is sorted
     * from smallest to biggest deviation between goal point and
     * robot!
     */
    if(strategy == 1)
    {
        for (int i = frontiers.size(); i >= 0; i--) {
            for (int j = 0; j < frontiers.size() - 1; j++) {
                double x = frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
                double y = frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
                double x_next = frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
                double y_next = frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();
                double euclidean_distance = x * x + y * y;
                double euclidean_distance_next = x_next * x_next + y_next * y_next;

                if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next)) {
                    frontier_t temp = frontiers.at(j+1);
                    frontiers.at(j + 1) = frontiers.at(j);
                    frontiers.at(j) = temp;
                }
            }
        }
    }
    else if(strategy == 2)
    {
        for (int i = frontiers.size(); i >= 0; i--)
        {
            //ROS_ERROR("%d", i);
            for (int j = 0; j < frontiers.size() - 1; j++) {
                double x = frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
                double y = frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
                double x_next = frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
                double y_next = frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();
                double euclidean_distance = x * x + y * y;
                double euclidean_distance_next = x_next * x_next + y_next * y_next;

                if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next)) {
                    frontier_t temp = frontiers.at(j+1);
                    frontiers.at(j + 1) = frontiers.at(j);
                    frontiers.at(j) = temp;
                }
            }
        }
    }
    else if(strategy == 3)
    {
        trajectory_plan_10_frontiers();

        for (int i = frontiers.size(); i >= frontiers.size()-10; i--)
        {
            //ROS_ERROR("%d", i);
            if(frontiers.size()-i >= frontiers.size())
            {
                break;
            }
            else
            {
                for (int j = 0; j < 10-1; j++)
                {
                    if(j >= frontiers.size())
                    {
                        break;
                    }else
                    {

                        int dist = frontiers.at(j).distance_to_robot;
                        int dist_next = frontiers.at(j+1).distance_to_robot;

                        if (dist > dist_next) {
                                frontier_t temp = frontiers.at(j+1);
                                frontiers.at(j + 1) = frontiers.at(j);
                                frontiers.at(j) = temp;
                        }
                    }
                }
            }
        }
    }
    else if(strategy == 4)
    {
        for(int cluster_number = 0; cluster_number < clusters.size(); cluster_number++)
        {
            if (clusters.at(cluster_number).cluster_element.size() > 0)
            {
                ROS_DEBUG("Cluster %d  size: %u",cluster_number, clusters.at(cluster_number).cluster_element.size());
                for (int i = clusters.at(cluster_number).cluster_element.size(); i > 0; i--)
                {
                    ROS_DEBUG("Cluster element size: %d", i);
                        for (int j = 0; j < clusters.at(cluster_number).cluster_element.size()-1; j++)
                        {
                            ROS_DEBUG("Cluster element number: %d", j);
                            double x = clusters.at(cluster_number).cluster_element.at(j).x_coordinate - pose_x;
                            double y = clusters.at(cluster_number).cluster_element.at(j).y_coordinate - pose_y;
                            double x_next = clusters.at(cluster_number).cluster_element.at(j+1).x_coordinate - pose_x;
                            double y_next = clusters.at(cluster_number).cluster_element.at(j+1).y_coordinate - pose_y;
                            double euclidean_distance = x * x + y * y;
                            double euclidean_distance_next = x_next * x_next + y_next * y_next;

                            if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next))
                            {
                                frontier_t temp = clusters.at(cluster_number).cluster_element.at(j+1);
                                clusters.at(cluster_number).cluster_element.at(j+1) = clusters.at(cluster_number).cluster_element.at(j);
                                clusters.at(cluster_number).cluster_element.at(j) = temp;
                            }
                        }
                    }
            }else
            {
                ROS_INFO("Sorting not possible, no elements available!!!");
            }
        }
    }
    else if(strategy == 5)
    {
        ROS_DEBUG("Iterating over all cluster elements");

        for(int i = 0; i < clusters.size(); i++)
        {
            for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
            {
                clusters.at(i).cluster_element.at(j).dist_to_robot = 0;
            }
        }

        for(int i = 0; i < clusters.size(); i++)
        {
            if(clusters.at(i).cluster_element.size() > 0)
            {
                for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
                {
                    /* ********** EUCLIDEAN DISTANCE ********** */
                    double x = clusters.at(i).cluster_element.at(j).x_coordinate - pose_x;
                    double y = clusters.at(i).cluster_element.at(j).y_coordinate - pose_y;
                    double euclidean_distance = x * x + y * y;
                    int distance = euclidean_distance;

                    clusters.at(i).cluster_element.at(j).dist_to_robot = sqrt(distance);
                }
            }else
            {
                ROS_DEBUG("Erasing Cluster: %d", clusters.at(i).id);
                cluster_mutex.lock();
                clusters.erase(clusters.begin() + i);
                cluster_mutex.unlock();
                if(i > 0)
                {
                    i --;
                }
            }
        }
        ROS_INFO("Starting to sort the clusters itself");
        std::sort(clusters.begin(), clusters.end(), sortCluster);

    }
    else if(strategy == 6)
    {
        ROS_DEBUG("Iterating over all cluster elements");

        for(int i = 0; i < clusters.size(); i++)
        {
            if(clusters.at(i).cluster_element.size() > 0)
            {
                for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
                {
                    random_value = int(rand() % 100);
                    clusters.at(i).cluster_element.at(j).dist_to_robot = random_value;
                }
            }else
            {
                ROS_DEBUG("Erasing Cluster: %d", clusters.at(i).id);
                clusters.erase(clusters.begin() + i);
                if(i > 0)
                {
                    i --;
                }
            }
        }
        ROS_DEBUG("Starting to sort the clusters itself");
        std::sort(clusters.begin(), clusters.end(), sortCluster);
    }
    else if(strategy == 7)
    {
        double x,y,x_next,y_next,angle_robot,angle_frontier,angle_next_frontier,angle,angle_next;
        int costmap_width,costmap_height;
        close_frontiers.clear();
        far_frontiers.clear();

        // get size of local costmap
        nh.param<int>("local_costmap/width",costmap_width,-1);
        nh.param<int>("local_costmap/height",costmap_height,-1);

        // differentiate between frontiers inside (close frontiers) and outside (far frontiers) of local costmap
        for (int i = 0; i < frontiers.size(); i++)
        {
            x = frontiers.at(i).x_coordinate - robotPose.getOrigin().getX();
            y = frontiers.at(i).y_coordinate - robotPose.getOrigin().getY();
            if (fabs(x) <= CLOSE_FRONTIER_RANGE && fabs(y) <= CLOSE_FRONTIER_RANGE){
                close_frontiers.push_back(frontiers.at(i));
            }
            else{
                far_frontiers.push_back(frontiers.at(i));
                ROS_INFO("distance: (%.2f, %.2f) (far)", fabs(x), fabs(y));
            }
        }

        // sort close frontiers clock wise
        if (close_frontiers.size() > 0)
        {
            //std::sort(close_frontiers.begin(), close_frontiers.end(), *this);
            for (int i = 0; i< close_frontiers.size(); i++)
            {
                for (int j = 0; j < close_frontiers.size() - 1; j++)
                {
                    angle_robot = robotPose.getRotation().getAngle();

                    angle_frontier = atan2(robotPose.getOrigin().getX()-close_frontiers.at(j).x_coordinate, robotPose.getOrigin().getY()-close_frontiers.at(j).y_coordinate);
                    angle_next_frontier = atan2(robotPose.getOrigin().getX()-close_frontiers.at(j+1).x_coordinate, robotPose.getOrigin().getY()-close_frontiers.at(j+1).y_coordinate);

                    if (angle_frontier > angle_next_frontier)
                    {
                        frontier_t temp = close_frontiers.at(j+1);
                        close_frontiers.at(j+1) = close_frontiers.at(j);
                        close_frontiers.at(j) = temp;
                    }
                }
            }
        }

        // sort far frontiers by distance
        if (far_frontiers.size() > 0)
        {
            for (int i = 0; i < far_frontiers.size(); i++)
            {
                for (int j = 0; j < far_frontiers.size() - 1; j++)
                {
                    x = far_frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
                    y = far_frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
                    x_next = far_frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
                    y_next = far_frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();

                    if (x*x + y*y > x_next*x_next + y_next*y_next) {
                        frontier_t temp = far_frontiers.at(j+1);
                        far_frontiers.at(j + 1) = far_frontiers.at(j);
                        far_frontiers.at(j) = temp;
                    }
                }
            }
        }

        // put together close and far frontiers
        frontiers.clear();
        frontiers.reserve(close_frontiers.size() + far_frontiers.size());
        frontiers.insert(frontiers.end(), close_frontiers.begin(), close_frontiers.end());
        frontiers.insert(frontiers.end(), far_frontiers.begin(), far_frontiers.end());
    }
    
    release_mutex(&store_frontier_mutex, __FUNCTION__);

    ROS_INFO("Done sorting");
}

//void ExplorationPlanner::sort_reserve(int strategy)
//{
//    tf::Stamped < tf::Pose > robotPose;
//    if (!costmap_ros_->getRobotPose(robotPose))
//    {
//        ROS_ERROR("Failed to get RobotPose");
//        return;
//    }
//    if (frontiers.size() <= 0 && clusters.size() <= 0)
//    {
//        ROS_INFO("Sorting not possible, no frontiers available!!!");
//        return;
//    }

//    double pose_x = robotPose.getOrigin().getX();
//    double pose_y = robotPose.getOrigin().getY();
//    
//    //store_frontier_mutex.lock();
//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
//    
//    //ROS_ERROR("COPYING...");
//    sorted_frontiers.clear();
//    for(int i=0; i<frontiers.size(); i++)
//        sorted_frontiers.push_back(frontiers.at(i));
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    //ROS_ERROR("SORTING...");

//    /*
//     * Following Sort algorithm normalizes all distances to the
//     * robots actual position. As result, the list is sorted
//     * from smallest to biggest deviation between goal point and
//     * robot!
//     */
//    if(strategy == 1)
//    {
//        for (int i = sorted_frontiers.size(); i >= 0; i--) {
//            for (int j = 0; j < sorted_frontiers.size() - 1; j++) {
//                double x = sorted_frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
//                double y = sorted_frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
//                double x_next = sorted_frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
//                double y_next = sorted_frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();
//                double euclidean_distance = x * x + y * y;
//                double euclidean_distance_next = x_next * x_next + y_next * y_next;

//                if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next)) {
//                    frontier_t temp = sorted_frontiers.at(j+1);
//                    sorted_frontiers.at(j + 1) = sorted_frontiers.at(j);
//                    sorted_frontiers.at(j) = temp;
//                }
//            }
//        }
//    }
//    else if(strategy == 2)
//    {
//        for (int i = sorted_frontiers.size(); i >= 0; i--)
//        {
//            for (int j = 0; j < sorted_frontiers.size() - 1; j++) {
//                double x = sorted_frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
//                double y = sorted_frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
//                double x_next = sorted_frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
//                double y_next = sorted_frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();
//                double euclidean_distance = x * x + y * y;
//                double euclidean_distance_next = x_next * x_next + y_next * y_next;

//                if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next)) {
//                    frontier_t temp = sorted_frontiers.at(j+1);
//                    sorted_frontiers.at(j + 1) = sorted_frontiers.at(j);
//                    sorted_frontiers.at(j) = temp;
//                }
//            }
//        }
//    }
//    else if(strategy == 3)
//    {
//        trajectory_plan_10_frontiers();

//        for (int i = sorted_frontiers.size(); i >= sorted_frontiers.size()-10; i--)
//        {
//            if(sorted_frontiers.size()-i >= sorted_frontiers.size())
//            {
//                break;
//            }
//            else
//            {
//                for (int j = 0; j < 10-1; j++)
//                {
//                    if(j >= sorted_frontiers.size())
//                    {
//                        break;
//                    }else
//                    {

//                        int dist = sorted_frontiers.at(j).distance_to_robot;
//                        int dist_next = sorted_frontiers.at(j+1).distance_to_robot;

//                        if (dist > dist_next) {
//                                frontier_t temp = sorted_frontiers.at(j+1);
//                                sorted_frontiers.at(j + 1) = sorted_frontiers.at(j);
//                                sorted_frontiers.at(j) = temp;
//                        }
//                    }
//                }
//            }
//        }
//    }
//    else if(strategy == 4)
//    {
//        for(int cluster_number = 0; cluster_number < clusters.size(); cluster_number++)
//        {
//            if (clusters.at(cluster_number).cluster_element.size() > 0)
//            {
//                ROS_DEBUG("Cluster %d  size: %u",cluster_number, clusters.at(cluster_number).cluster_element.size());
//                for (int i = clusters.at(cluster_number).cluster_element.size(); i > 0; i--)
//                {
//                    ROS_DEBUG("Cluster element size: %d", i);
//                        for (int j = 0; j < clusters.at(cluster_number).cluster_element.size()-1; j++)
//                        {
//                            ROS_DEBUG("Cluster element number: %d", j);
//                            double x = clusters.at(cluster_number).cluster_element.at(j).x_coordinate - pose_x;
//                            double y = clusters.at(cluster_number).cluster_element.at(j).y_coordinate - pose_y;
//                            double x_next = clusters.at(cluster_number).cluster_element.at(j+1).x_coordinate - pose_x;
//                            double y_next = clusters.at(cluster_number).cluster_element.at(j+1).y_coordinate - pose_y;
//                            double euclidean_distance = x * x + y * y;
//                            double euclidean_distance_next = x_next * x_next + y_next * y_next;

//                            if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next))
//                            {
//                                frontier_t temp = clusters.at(cluster_number).cluster_element.at(j+1);
//                                clusters.at(cluster_number).cluster_element.at(j+1) = clusters.at(cluster_number).cluster_element.at(j);
//                                clusters.at(cluster_number).cluster_element.at(j) = temp;
//                            }
//                        }
//                    }
//            }else
//            {
//                ROS_INFO("Sorting not possible, no elements available!!!");
//            }
//        }
//    }
//    else if(strategy == 5)
//    {
//        ROS_DEBUG("Iterating over all cluster elements");

//        for(int i = 0; i < clusters.size(); i++)
//        {
//            for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//            {
//                clusters.at(i).cluster_element.at(j).dist_to_robot = 0;
//            }
//        }

//        for(int i = 0; i < clusters.size(); i++)
//        {
//            if(clusters.at(i).cluster_element.size() > 0)
//            {
//                for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//                {
//                    /* ********** EUCLIDEAN DISTANCE ********** */
//                    double x = clusters.at(i).cluster_element.at(j).x_coordinate - pose_x;
//                    double y = clusters.at(i).cluster_element.at(j).y_coordinate - pose_y;
//                    double euclidean_distance = x * x + y * y;
//                    int distance = euclidean_distance;

//                    clusters.at(i).cluster_element.at(j).dist_to_robot = sqrt(distance);
//                }
//            }else
//            {
//                ROS_DEBUG("Erasing Cluster: %d", clusters.at(i).id);
//                cluster_mutex.lock();
//                clusters.erase(clusters.begin() + i);
//                cluster_mutex.unlock();
//                if(i > 0)
//                {
//                    i --;
//                }
//            }
//        }
//        ROS_INFO("Starting to sort the clusters itself");
//        std::sort(clusters.begin(), clusters.end(), sortCluster);

//    }
//    else if(strategy == 6)
//    {
//        ROS_DEBUG("Iterating over all cluster elements");

//        for(int i = 0; i < clusters.size(); i++)
//        {
//            if(clusters.at(i).cluster_element.size() > 0)
//            {
//                for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//                {
//                    random_value = int(rand() % 100);
//                    clusters.at(i).cluster_element.at(j).dist_to_robot = random_value;
//                }
//            }else
//            {
//                ROS_DEBUG("Erasing Cluster: %d", clusters.at(i).id);
//                clusters.erase(clusters.begin() + i);
//                if(i > 0)
//                {
//                    i --;
//                }
//            }
//        }
//        ROS_DEBUG("Starting to sort the clusters itself");
//        std::sort(clusters.begin(), clusters.end(), sortCluster);
//    }
//    else if(strategy == 7)
//    {
//        double x,y,x_next,y_next,angle_robot,angle_frontier,angle_next_frontier,angle,angle_next;
//        int costmap_width,costmap_height;
//        close_frontiers.clear();
//        far_frontiers.clear();

//        // get size of local costmap
//        nh.param<int>("local_costmap/width",costmap_width,-1);
//        nh.param<int>("local_costmap/height",costmap_height,-1);

//        // differentiate between frontiers inside (close frontiers) and outside (far frontiers) of local costmap
//        for (int i = 0; i < sorted_frontiers.size(); i++)
//        {
//            x = sorted_frontiers.at(i).x_coordinate - robotPose.getOrigin().getX();
//            y = sorted_frontiers.at(i).y_coordinate - robotPose.getOrigin().getY();
//            if (fabs(x) <= CLOSE_FRONTIER_RANGE && fabs(y) <= CLOSE_FRONTIER_RANGE){
//                close_frontiers.push_back(sorted_frontiers.at(i));
//            }
//            else{
//                far_frontiers.push_back(sorted_frontiers.at(i));
//                ROS_INFO("distance: (%.2f, %.2f) (far)", fabs(x), fabs(y));
//            }
//        }

//        // sort close frontiers clock wise
//        if (close_frontiers.size() > 0)
//        {
//            //std::sort(close_frontiers.begin(), close_frontiers.end(), *this);
//            for (int i = 0; i< close_frontiers.size(); i++)
//            {
//                for (int j = 0; j < close_frontiers.size() - 1; j++)
//                {
//                    angle_robot = robotPose.getRotation().getAngle();

//                    angle_frontier = atan2(robotPose.getOrigin().getX()-close_frontiers.at(j).x_coordinate, robotPose.getOrigin().getY()-close_frontiers.at(j).y_coordinate);
//                    angle_next_frontier = atan2(robotPose.getOrigin().getX()-close_frontiers.at(j+1).x_coordinate, robotPose.getOrigin().getY()-close_frontiers.at(j+1).y_coordinate);

//                    if (angle_frontier > angle_next_frontier)
//                    {
//                        frontier_t temp = close_frontiers.at(j+1);
//                        close_frontiers.at(j+1) = close_frontiers.at(j);
//                        close_frontiers.at(j) = temp;
//                    }
//                }
//            }
//        }

//        // sort far frontiers by distance
//        if (far_frontiers.size() > 0)
//        {
//            for (int i = 0; i < far_frontiers.size(); i++)
//            {
//                for (int j = 0; j < far_frontiers.size() - 1; j++)
//                {
//                    x = far_frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
//                    y = far_frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
//                    x_next = far_frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
//                    y_next = far_frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();

//                    if (x*x + y*y > x_next*x_next + y_next*y_next) {
//                        frontier_t temp = far_frontiers.at(j+1);
//                        far_frontiers.at(j + 1) = far_frontiers.at(j);
//                        far_frontiers.at(j) = temp;
//                    }
//                }
//            }
//        }

//        // put together close and far frontiers
//        sorted_frontiers.clear();
//        sorted_frontiers.reserve(close_frontiers.size() + far_frontiers.size());
//        sorted_frontiers.insert(frontiers.end(), close_frontiers.begin(), close_frontiers.end());
//        sorted_frontiers.insert(frontiers.end(), far_frontiers.begin(), far_frontiers.end());
//    }

//    ROS_INFO("Done sorting");
//}

void ExplorationPlanner::simulate() {

	geometry_msgs::PointStamped goalPoint;


	tf::Stamped < tf::Pose > robotPose;
	if (!costmap_ros_->getRobotPose(robotPose))
	{
		ROS_ERROR("Failed to get RobotPose");
	}
	// Visualize in RVIZ

	for (int i = frontiers.size() - 1; i >= 0; i--) {
            goalPoint.header.seq = i + 1;
            goalPoint.header.stamp = ros::Time::now();
            goalPoint.header.frame_id = "map";
            goalPoint.point.x = frontiers.at(i).x_coordinate;
            goalPoint.point.y = frontiers.at(i).y_coordinate;

            pub_Point.publish < geometry_msgs::PointStamped > (goalPoint);

            ros::Duration(1.0).sleep();
	}
}

void ExplorationPlanner::clear_Visualized_Cluster_Cells(std::vector<int> ids)
{
    nav_msgs::GridCells clearing_cell;
    geometry_msgs::Point point;

    clearing_cell.header.frame_id = move_base_frame;
    clearing_cell.header.stamp = ros::Time::now();
    clearing_cell.cell_height = 0.1;
    clearing_cell.cell_width = 0.1;
    clearing_cell.header.seq = cluster_cells_seq_number++;

    point.x = 1000;
    point.y = 1000;
    point.z = 1000;
    clearing_cell.cells.push_back(point);

    for(int i = 0; i < 20; i++)
    {
        bool used_id = false;
        for(int n = 0; n < ids.size(); n++)
        {
            if(ids.at(n) == i)
            {
               used_id = true;
               break;
            }
        }
        if(used_id == false)
        {
            if(i == 0)
            {
                pub_cluster_grid_0.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 1)
            {
                pub_cluster_grid_1.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 2)
            {
                pub_cluster_grid_2.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 3)
            {
                pub_cluster_grid_3.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 4)
            {
                pub_cluster_grid_4.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 5)
            {
                pub_cluster_grid_5.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 6)
            {
                pub_cluster_grid_6.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 7)
            {
                pub_cluster_grid_7.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 8)
            {
                pub_cluster_grid_8.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 9)
            {
                pub_cluster_grid_9.publish<nav_msgs::GridCells>(clearing_cell);
            }

            if(i == 10)
            {
                pub_cluster_grid_10.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 11)
            {
                pub_cluster_grid_11.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 12)
            {
                pub_cluster_grid_12.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 13)
            {
                pub_cluster_grid_13.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 14)
            {
                pub_cluster_grid_14.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 15)
            {
                pub_cluster_grid_15.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 16)
            {
                pub_cluster_grid_16.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 17)
            {
                pub_cluster_grid_17.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 18)
            {
                pub_cluster_grid_18.publish<nav_msgs::GridCells>(clearing_cell);
            }
            if(i == 19)
            {
                pub_cluster_grid_19.publish<nav_msgs::GridCells>(clearing_cell);
            }
        }
    }
}

void ExplorationPlanner::visualize_Cluster_Cells()
{
    ROS_INFO("Visualize clusters");

    std::vector<int> used_ids;
    for(int i= 0; i< clusters.size(); i++)
    {
        if(clusters.at(i).cluster_element.size() > 0)
        {
            nav_msgs::GridCells cluster_cells;
            geometry_msgs::Point point;
            for(int j= 0; j < clusters.at(i).cluster_element.size(); j++)
            {
                point.x = clusters.at(i).cluster_element.at(j).x_coordinate;
                point.y = clusters.at(i).cluster_element.at(j).y_coordinate;
                point.z = 0;

                cluster_cells.cells.push_back(point);
            }
            cluster_cells.header.frame_id = move_base_frame;
            cluster_cells.header.stamp = ros::Time::now();
            cluster_cells.cell_height = 0.5;
            cluster_cells.cell_width = 0.5;
            cluster_cells.header.seq = cluster_cells_seq_number++;

            used_ids.push_back(clusters.at(i).id % 20);

            if(clusters.at(i).id % 20 == 0)
            {
                pub_cluster_grid_0.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 1)
            {
                pub_cluster_grid_1.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 2)
            {
                pub_cluster_grid_2.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 3)
            {
                pub_cluster_grid_3.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 4)
            {
                pub_cluster_grid_4.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 5)
            {
                pub_cluster_grid_5.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 6)
            {
                pub_cluster_grid_6.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 7)
            {
                pub_cluster_grid_7.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 8)
            {
                pub_cluster_grid_8.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 9)
            {
                pub_cluster_grid_9.publish<nav_msgs::GridCells>(cluster_cells);
            }

            if(clusters.at(i).id % 20 == 10)
            {
                pub_cluster_grid_10.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 11)
            {
                pub_cluster_grid_11.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 12)
            {
                pub_cluster_grid_12.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 13)
            {
                pub_cluster_grid_13.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 14)
            {
                pub_cluster_grid_14.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 15)
            {
                pub_cluster_grid_15.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 16)
            {
                pub_cluster_grid_16.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 17)
            {
                pub_cluster_grid_17.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 18)
            {
                pub_cluster_grid_18.publish<nav_msgs::GridCells>(cluster_cells);
            }
            else if(clusters.at(i).id % 20 == 19)
            {
                pub_cluster_grid_19.publish<nav_msgs::GridCells>(cluster_cells);
            }
        }
    }
    clear_Visualized_Cluster_Cells(used_ids);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
void ExplorationPlanner::visualize_Clusters()
{
    ROS_INFO("Visualize clusters");

    geometry_msgs::PolygonStamped cluster_polygon;

    for(int i= 0; i< clusters.size(); i++)
    {
        double upper_left_x, upper_left_y;
        double upper_right_x, upper_right_y;
        double down_left_x, down_left_y;
        double down_right_x, down_right_y;
        double most_left, most_right, upper1, lower1, upper2, lower2;

        for(int j= 0; j < clusters.at(i).cluster_element.size(); j++)
        {
            if(clusters.at(i).cluster_element.at(j).x_coordinate < most_left)
            {
                most_left = clusters.at(i).cluster_element.at(j).x_coordinate;
                if(clusters.at(i).cluster_element.at(j).y_coordinate < lower1)
                {
                    lower1 = clusters.at(i).cluster_element.at(j).y_coordinate;
                    down_left_x = clusters.at(i).cluster_element.at(j).x_coordinate;
                    down_left_y = clusters.at(i).cluster_element.at(j).y_coordinate;
                }
                if(clusters.at(i).cluster_element.at(j).y_coordinate > upper1)
                {
                    upper1 = clusters.at(i).cluster_element.at(j).y_coordinate;
                    upper_left_x = clusters.at(i).cluster_element.at(j).x_coordinate;
                    upper_left_y = clusters.at(i).cluster_element.at(j).y_coordinate;
                }
            }
            if(clusters.at(i).cluster_element.at(j).x_coordinate > most_right)
            {
                most_right = clusters.at(i).cluster_element.at(j).x_coordinate;
                if(clusters.at(i).cluster_element.at(j).y_coordinate < lower2)
                {
                    lower2 = clusters.at(i).cluster_element.at(j).y_coordinate;
                    down_right_x = clusters.at(i).cluster_element.at(j).x_coordinate;
                    down_right_y = clusters.at(i).cluster_element.at(j).y_coordinate;
                }
                if(clusters.at(i).cluster_element.at(j).y_coordinate > upper2)
                {
                    upper2 = clusters.at(i).cluster_element.at(j).y_coordinate;
                    upper_right_x = clusters.at(i).cluster_element.at(j).x_coordinate;
                    upper_right_y = clusters.at(i).cluster_element.at(j).y_coordinate;
                }
            }

        }

        geometry_msgs::Point32 polygon_point;
        polygon_point.x = upper_left_x;
        polygon_point.y = upper_left_y;
        polygon_point.z = 0;
        cluster_polygon.polygon.points.push_back(polygon_point);

        polygon_point.x = upper_right_x;
        polygon_point.y = upper_right_y;
        polygon_point.z = 0;
        cluster_polygon.polygon.points.push_back(polygon_point);

        polygon_point.x = down_left_x;
        polygon_point.y = down_left_y;
        polygon_point.z = 0;
        cluster_polygon.polygon.points.push_back(polygon_point);

        polygon_point.x = down_right_x;
        polygon_point.y = down_right_y;
        polygon_point.z = 0;
        cluster_polygon.polygon.points.push_back(polygon_point);

        cluster_polygon.header.frame_id = move_base_frame;
        cluster_polygon.header.stamp = ros::Time::now();
        cluster_polygon.header.seq = i+1;

        pub_clusters.publish<geometry_msgs::PolygonStamped>(cluster_polygon);
        //break;// FIXME
    }
}
#pragma GCC diagnostic pop


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

void ExplorationPlanner::deleteMapData() {
	if (exploration_trans_array_) {
		delete[] exploration_trans_array_;
		exploration_trans_array_ = 0;
	}
	if (obstacle_trans_array_) {
		delete[] obstacle_trans_array_;
		obstacle_trans_array_ = 0;
	}
	if (is_goal_array_) {
		delete[] is_goal_array_;
		is_goal_array_ = 0;
	}
	if (frontier_map_array_) {
		delete[] frontier_map_array_;
		frontier_map_array_ = 0;
	}
}

void ExplorationPlanner::resetMaps() {
	std::fill_n(exploration_trans_array_, num_map_cells_, INT_MAX);
	std::fill_n(obstacle_trans_array_, num_map_cells_, INT_MAX);
	std::fill_n(is_goal_array_, num_map_cells_, false);
}

bool ExplorationPlanner::isSameFrontier(int frontier_point1, int frontier_point2) {
	unsigned int fx1, fy1;
	unsigned int fx2, fy2;
	double wfx1, wfy1;
	double wfx2, wfy2;
	costmap_.indexToCells(frontier_point1, fx1, fy1);
	costmap_.indexToCells(frontier_point2, fx2, fy2);
	costmap_.mapToWorld(fx1, fy1, wfx1, wfy1);
	costmap_.mapToWorld(fx2, fy2, wfx2, wfy2);

	double dx = wfx1 - wfx2;
	double dy = wfy1 - wfy2;

	if ((dx * dx) + (dy * dy)
			< (p_same_frontier_dist_ * p_same_frontier_dist_)) {
		return true;
	}
	return false;
}

// Used to generate direction for frontiers
double ExplorationPlanner::getYawToUnknown(int point) {
	int adjacentPoints[8];
	getAdjacentPoints(point, adjacentPoints);

	int max_obs_idx = 0;

	for (int i = 0; i < 8; ++i) {
		if (isValid(adjacentPoints[i])) {
			if (occupancy_grid_array_[adjacentPoints[i]]
					== costmap_2d::NO_INFORMATION) {
				if (obstacle_trans_array_[adjacentPoints[i]]
						> obstacle_trans_array_[adjacentPoints[max_obs_idx]]) {
					max_obs_idx = i;
				}
			}
		}
	}

	int orientationPoint = adjacentPoints[max_obs_idx];
	unsigned int sx, sy, gx, gy;
	costmap_.indexToCells((unsigned int) point, sx, sy);
	costmap_.indexToCells((unsigned int) orientationPoint, gx, gy);
	int x = gx - sx;
	int y = gy - sy;
	double yaw = std::atan2(y, x);

	return yaw;

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

int ExplorationPlanner::backoff (int point)
{
    				if(occupancy_grid_array_[up(point)] == costmap_2d::NO_INFORMATION && occupancy_grid_array_[left(point)] == costmap_2d::NO_INFORMATION)
				{
					return(downright(downright(downright(point))));
				}
				if(occupancy_grid_array_[up(point)] == costmap_2d::NO_INFORMATION && occupancy_grid_array_[right(point)] == costmap_2d::NO_INFORMATION)
				{
					return(downleft(downleft(downleft(point))));
				}
				if(occupancy_grid_array_[down(point)] == costmap_2d::NO_INFORMATION && occupancy_grid_array_[left(point)] == costmap_2d::NO_INFORMATION)
				{
					return(upright(upright(upright(point))));
				}
				if(occupancy_grid_array_[down(point)] == costmap_2d::NO_INFORMATION && occupancy_grid_array_[right(point)] == costmap_2d::NO_INFORMATION)
				{
					return(upleft(upleft(upleft(point))));
				}


				if(occupancy_grid_array_[right(point)] == costmap_2d::NO_INFORMATION)
				{
					return(left(left(left(point))));
				}
				if(occupancy_grid_array_[down(point)] == costmap_2d::NO_INFORMATION)
				{
					return(up(up(up(point))));
				}
				if(occupancy_grid_array_[left(point)] == costmap_2d::NO_INFORMATION)
				{
					return(right(right(right(point))));
				}
				if(occupancy_grid_array_[up(point)] == costmap_2d::NO_INFORMATION)
				{
					return(down(down(down(point))));
				}

//				if(occupancy_grid_array_[upleft(point)] == costmap_2d::NO_INFORMATION)
//				{
//					return(downright(downright(point)));
//				}
//				if(occupancy_grid_array_[upright(point)] == costmap_2d::NO_INFORMATION)
//				{
//					return(downleft(downleft(point)));
//				}
//				if(occupancy_grid_array_[downright(point)] == costmap_2d::NO_INFORMATION)
//				{
//					return(upleft(upleft(point)));
//				}
//				if(occupancy_grid_array_[downleft(point)] == costmap_2d::NO_INFORMATION)
//				{
//					return(upright(upright(point)));
//				}
				else
				{
					return(point);
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

//float ExplorationPlanner::new_optimal_ds(int id, float new_optimal_ds_x, float new_optimal_ds_y) {
//    optimal_ds_id = id;
//    optimal_ds_x = new_optimal_ds_x; 
//    optimal_ds_y = new_optimal_ds_y;
//    optimal_ds_set = true;
//}

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

//bool ExplorationPlanner::my2_determine_goal_staying_alive(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id, bool energy_above_th, int w1, int w2, int w3, int w4)
//{
//    
//    ros::Time start_time;
//    sort_time = 0;
//    selection_time = 0;
//    number_of_frontiers = 0;
//    frontier_selected = false;
//    this->available_distance = available_distance;

//    if (frontiers.size() <= 0 && clusters.size() <= 0)
//    {
//        ROS_ERROR("No frontier/cluster available");
//    } 
//    
//    //if(APPROACH == 0)
//        /* Original cost function */
//    //    smart_sort_cost(energy_above_th, w1, w2, w3, w4);
//    //else if(APPROACH == 1)
//        /* Cost function: (real distance = Disjktra's distance)
//         *     - (real) distance between the given frontier and target DS of the robot;
//         *     - distance on the graph of the DSs between the DS that is closest, according to the euclidean distance (not the real one!), to the given frontier and the robot; this distance is computed efficiently by the energy_mgmt node;
//         *     - d_r
//         *     - theta_rel
//         */
//    //    sort_cost_1(energy_above_th, w1, w2, w3, w4);
//    //else if(APPROACH == -1)
//    //    return false;
//    //else
//    //    ROS_ERROR("INVALID APPROACH!!!");
//    
//    sorted_frontiers.clear();
//    
//    start_time = ros::Time::now();
//    number_of_frontiers = frontiers.size();
//    
//    //TODO move to a separate function that is called by explorer, since in case of error (when my_... is recalled by itself), this code otherwise is re-executed every time...
//    ROS_DEBUG("frontiers size: %u", frontiers.size());
//    if(APPROACH == 0)
//        sorted_frontiers = frontiers;
//    else if(APPROACH == 1)
//        sorted_frontiers = frontiers;
//    else if(APPROACH == 2)
//        my_sort_cost_2(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 3)
//        my_sort_cost_3(energy_above_th, w1, w2, w3, w4);
//    else if(APPROACH == 4)
//        my_sort_cost_4(energy_above_th, w1, w2, w3, w4);
//    else {
//        ROS_ERROR("Invalid approach!");
//        sorted_frontiers = frontiers;
//    }
//    
//    //return determine_goal_staying_alive( mode,  strategy,  available_distance, final_goal,  count, robot_str_name,  actual_cluster_id);
//    //sorted_frontiers = frontiers;

//    if (!costmap_ros_->getRobotPose(robotPose)) 
//    {
//            ROS_ERROR("Failed to get RobotPose"); //TODO handle "exception"
//    }
//    
//    if(sorted_frontiers.size() == 0) {
//        my_error_counter = 0;
//        //force to conciser also frontiers under auction (if there are)
//        for(int i=0; i < frontiers_under_auction.size(); i++)
//            sorted_frontiers.push_back(frontiers_under_auction.at(i));
//    }
//    
//    sort_time = (ros::Time::now() - start_time).toSec();
//    start_time = ros::Time::now();
//    
//    //store_frontier_mutex.lock();
//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);

//    // check if robot needs to go recharging right away
//    double dist_home;
//    double dist_front;
//    double closest = 9999;
//    if(strategy == 1){
//        double xh = optimal_ds_x - robotPose.getOrigin().getX();
//        double yh = optimal_ds_y - robotPose.getOrigin().getY();
//        dist_home = sqrt(xh * xh + yh * yh) * 1.2; // 1.2 is extra reserve, just in case
//    }
//    else if(strategy == 2){
//        dist_home = trajectory_plan_meters(optimal_ds_x, optimal_ds_y); // * costmap_ros_->getCostmap()->getResolution();
//    }
//    //ROS_ERROR("available_distance: %f", available_distance);
//    if(dist_home > 0 && dist_home >= available_distance) {
//        ROS_ERROR("Target DS is too far to reach a frontier...\noptimal_ds_x: %f, optimal_ds_y: %f, distance: %f, available distance: %f", optimal_ds_x, optimal_ds_y, dist_home, available_distance);
//        release_mutex(&store_frontier_mutex, __FUNCTION__);
//        return false;
//    }

//    ROS_INFO("look for a FRONTIER as goal");
//    // look for a FRONTIER as goal
//    int errors = 0;
//    if (mode == 1)
//    {
//        //for (int i = 0 + count; i < sorted_frontiers.size(); i++)
//        ROS_DEBUG("sorted_frontiers size: %u", sorted_frontiers.size());
//        for (int i = 0; i < sorted_frontiers.size(); i++)
//        {
//            //we only check the first 9 frontiers, because we only sorted the first 9 frontiers by efficiency.
//            if(i>limit_search){
//                // if the distances could not be computed, try again using euclidean distances instead
//                //if(errors == i && strategy == 2){
//                if(errors >= 10 && strategy == 2){
//                    ROS_ERROR("Fallback to euclidean distance.");
//                    ROS_INFO("Fallback to euclidean distance.");
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    selection_time = (ros::Time::now() - start_time).toSec();
//                    frontier_selected=my2_determine_goal_staying_alive(1, 1, available_distance, final_goal, count, robot_str_name, -1, energy_above_th, w1, w2, w3, w4);
//                    return frontier_selected;
//                }
//                ROS_ERROR("None of the %d checked frontiers is reachable! This shouldn't happen...", limit_search);
//                release_mutex(&store_frontier_mutex, __FUNCTION__);
//                frontier_selected=false;
//                selection_time = (ros::Time::now() - start_time).toSec();
//                return false;
//            }
//         
////            bool under_auction = false;
////            for(int k=0; !under_auction && k<frontiers_under_auction.size(); k++)
////                if(frontiers[i].id == frontiers_under_auction[k].id) {
////                    under_auction = true;
////                }
////            if(under_auction)
////                continue;

//            if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
//            {
//                double distance;
//                double total_distance;
//                if(strategy == 1){
//                    // distance to next frontier
//                    double x1 = frontiers.at(i).x_coordinate - robotPose.getOrigin().getX();
//                    double y1 = frontiers.at(i).y_coordinate -  robotPose.getOrigin().getY();
//                    // distance from frontier to home base
//                    double x2 = optimal_ds_x - frontiers.at(i).x_coordinate;
//                    double y2 = optimal_ds_y -  frontiers.at(i).y_coordinate;
//                    total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                }
//                else if(strategy == 2){
//                    // distance to next frontier
//                    total_distance = trajectory_plan_meters(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
//                    if(total_distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        ROS_INFO("Failed to compute distance!");
//                        if(errors == 0)
//                            my_error_counter++;
//                        errors++;
//                        continue;
//                    }
//                    // distance from frontier to optimal ds
//                    distance = trajectory_plan_meters(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, optimal_ds_x, optimal_ds_y);
//                    if(distance < 0){
//                        ROS_ERROR("Failed to compute distance!");
//                        ROS_INFO("Failed to compute distance!");
//                        if(errors == 0)
//                            my_error_counter++;
//                        errors++;
//                        continue;
//                    }
//                    total_distance += distance;

//                    if(total_distance < closest){
//                        closest = total_distance;
//                        dist_front = total_distance - distance;
//                        dist_home = distance;
//                    }

//                    // convert from cells to meters
//                    //F
//                    //total_distance *= costmap_ros_->getCostmap()->getResolution();
//                }
//                else{
//                    ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    frontier_selected=false;
//                    selection_time = (ros::Time::now() - start_time).toSec();
//                    return false;
//                }

//                ROS_INFO("Distance to frontier and then home: %.2f",total_distance);
//                if(available_distance > total_distance)
//                {
//                    ROS_INFO("------------------------------------------------------------------");
//                    ROS_INFO("Determined frontier with ID: %d   at x: %.2f     y: %.2f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

//                    final_goal->push_back(frontiers.at(i).x_coordinate);
//                    final_goal->push_back(frontiers.at(i).y_coordinate);
//                    final_goal->push_back(frontiers.at(i).detected_by_robot);
//                    final_goal->push_back(frontiers.at(i).id);
//                    
//                    robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
//                    
//#ifndef QUICK_SELECTION
//            
//                    // robot position
//                    double robot_x = robotPose.getOrigin().getX();
//                    double robot_y = robotPose.getOrigin().getY();

//					// frontier position
//                    double frontier_x = frontiers.at(i).x_coordinate;
//                    double frontier_y = frontiers.at(i).y_coordinate;

//					// calculate d_g
//                    int d_g = trajectory_plan_meters(frontier_x, frontier_y);

//                    // calculate d_gb
//                    int d_gb = trajectory_plan_meters(frontier_x, frontier_y, robot_home_x, robot_home_y);

//                    // calculate d_gbe
//                    int d_gbe;
//                    if(my_energy_above_th)
//                    {
//                        d_gbe = -d_gb;
//                    }
//                    else
//                    {
//                        d_gbe = d_gb;
//                    }

//                    // calculate theta
//                    double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
//                    double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
//                    double theta = 1/M_PI * (M_PI - abs(abs(theta_s - theta_g) - M_PI));

//                    // calculate cost function
//                     my_bid = w1 * d_g + w2 * d_gb + w3 * d_gbe + w4 * theta;
//             
//#endif

//                    //start auction
//                    ROS_INFO("start frontier negotiation!");
//                    my_selected_frontier = &frontiers.at(i);
//                    //my_negotiate();
//             
//                    for(int i = 0; i < auction_timeout/0.1; i++) {
//                        ros::Duration(0.1).sleep();
//                        ros::spinOnce();
//                    }
//                    
//                    winner_of_auction = true;
//                    if(!winner_of_auction) {
//                        ROS_INFO("frontier under auction: skip");
//                        continue;
//                    }

//                    ROS_INFO("frontier selected");
//                    frontiers_under_auction.clear();
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    my_error_counter = 0;
//                    //ROS_INFO("final_goal size before return: %u", final_goal->size());
//                    frontier_selected=true;
//                    selection_time = (ros::Time::now() - start_time).toSec();
//                    return true;
//                    
//                } else{
//                    //ROS_ERROR("No frontier in energetic range (%.2f < %.2f + %.2f)", available_distance, dist_front * costmap_ros_->getCostmap()->getResolution(), dist_home * costmap_ros_->getCostmap()->getResolution());
//                    ROS_ERROR("No frontier in energetic range (%.2f < %.2f + %.2f)", available_distance, dist_front // * costmap_ros_->getCostmap()->getResolution(),
//                    , dist_home);
//                    release_mutex(&store_frontier_mutex, __FUNCTION__);
//                    my_error_counter++;
//                    frontier_selected=false;
//                    selection_time = (ros::Time::now() - start_time).toSec();
//                    return false;
//                }
//            }
//        }
//    }

//    // look for a CLUSTER as goal
//    else if (mode == 2)
//    {
//        int cluster_vector_position = 0;

//        if(clusters.size() > 0)
//        {
//            for (int i = 0; i < clusters.size(); i++)
//            {
//                if(clusters.at(i).id == actual_cluster_id)
//                {
//                    if(clusters.at(i).cluster_element.size() > 0)
//                    {
//                        cluster_vector_position = i;
//                    }
//                    break;
//                }
//            }
//        }

//        ROS_INFO("Calculated vector position of cluster %d", actual_cluster_id);
//        /*
//         * Iterate over all clusters .... if cluster_vector_position is set
//         * also the clusters with a lower have to be checked if the frontier
//         * determination fails at clusters.at(cluster_vector_position). therefore
//         * a ring-buffer is operated to iterate also over lower clusters, since
//         * they might have changed.
//         */
//        int nothing_found_in_actual_cluster = 0;
//        int visited_clusters = 0;
//        for (int i = 0 + count; i < clusters.size(); i++)
//        {
//            i = i+ cluster_vector_position;
//            i = i % (clusters.size());
//            for (int j = 0; j < clusters.at(i).cluster_element.size(); j++)
//            {
//                if (check_efficiency_of_goal(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate) == true)
//                {
//                    double distance;
//                    double total_distance;
//                    if(strategy == 1){
//                        // distance to cluster
//                        double x1 = clusters.at(i).cluster_element.at(j).x_coordinate - robotPose.getOrigin().getX();
//                        double y1 = clusters.at(i).cluster_element.at(j).y_coordinate - robotPose.getOrigin().getY();
//                        // distance from cluster to home base
//                        double x2 = robot_home_x - clusters.at(i).cluster_element.at(j).x_coordinate;
//                        double y2 = robot_home_y -  clusters.at(i).cluster_element.at(j).y_coordinate;
//                        total_distance = sqrt(x1 * x1 + y1 * y1) + sqrt(x2 * x2 + y2 * y2);
//                    }
//                    else if(strategy == 2){
//                        // distance to cluster
//                        total_distance = trajectory_plan_meters(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate);
//                        if(total_distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        // distance from cluster to home base
//                        distance = trajectory_plan_meters(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate, robot_home_x, robot_home_y);
//                        if(distance < 0){
//                            ROS_ERROR("Failed to compute distance!");
//                            errors++;
//                            continue;
//                        }
//                        total_distance += distance;
//                        // convert from cells to meters
//                        //total_distance *= costmap_ros_->getCostmap()->getResolution();
//                    }
//                    else{
//                        ROS_ERROR("Wrong strategy, cannot compute distance to goal!");
//                        release_mutex(&store_frontier_mutex, __FUNCTION__);
//                        frontier_selected=false;
//                        selection_time = (ros::Time::now() - start_time).toSec();
//                        return false;
//                    }

//                    ROS_INFO("distance to cluster and then home: %f",total_distance);
//                    if(available_distance > total_distance)
//                    {
//                        ROS_INFO("------------------------------------------------------------------");
//                        ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(i).cluster_element.at(j).id, (int)clusters.at(i).id);

//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).x_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).y_coordinate);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot);
//                        final_goal->push_back(clusters.at(i).cluster_element.at(j).id);

//                        // number of the cluster we operate in
//                        final_goal->push_back(clusters.at(i).id);
//                        robot_str_name->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot_str);
//                        release_mutex(&store_frontier_mutex, __FUNCTION__);
//                        frontier_selected=true;
//                        selection_time = (ros::Time::now() - start_time).toSec();
//                        return true;
//                    }
//                }
//            }

//            nothing_found_in_actual_cluster ++;
//            visited_clusters ++;

//            if(nothing_found_in_actual_cluster == 1)
//            {
//                //start again at the beginning(closest cluster))
//                i=0;
//                cluster_vector_position = 0;
//            }

//            if(visited_clusters == clusters.size())
//            {
//                ROS_ERROR("No frontier in energetic range %.2f, going home for recharging", available_distance);
//                release_mutex(&store_frontier_mutex, __FUNCTION__);
//                frontier_selected=false;
//                selection_time = (ros::Time::now() - start_time).toSec();
//                return false;
//            }
//        }
//    }
//    
//    //F //probably useless here...
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    frontier_selected=false;
//    selection_time = (ros::Time::now() - start_time).toSec();
//    return false;

//}

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
            frontiers.at(j).cost = frontier_cost_0(frontiers.at(j));
            add_to_sorted_fontiers_list_if_convinient(frontiers.at(j));        
        }
        robot_last_x = robot_x;
        robot_last_y = robot_y;
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
    return available_distance > dist; //TODO safety coefficient
}

double ExplorationPlanner::frontier_cost(frontier_t frontier) {
    return frontier_cost_0(frontier); //TODO
}

double ExplorationPlanner::frontier_cost_0(frontier_t frontier) {
    /*
     * cost function
     * f = w1 · d_g   +   w2 · d_gb   +   w3 · d_gbe   +   w4 · theta
     *
     * parameters
     * w1, ..., w4 .. weights
     * d_g         .. euclidean distance from the robot's current position to the frontier
     * d_gbe       .. -d_gb if battery charge is above threshold (e.g. 50%), d_gb if battery charge is below threshold,
     *                where d_gb is the euclidean distance from the frontier to the charging station
     * d_r         .. (MISSING DESCRIPTION) //TODO                 
     * theta       .. measure of how much the robot has to turn to reach frontier, theta in [0,1]
     */

    // frontier position
    double frontier_x = frontier.x_coordinate;
    double frontier_y = frontier.y_coordinate;

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
    d_r = -d_r;

    // calculate theta
    double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
    double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
    double theta = 1/M_PI * (M_PI - fabs(fabs(theta_s - theta_g) - M_PI)); //it seems complex, but it's just to keep into account the sign of the angle (which would be lost with just the fabs calls)

    // calculate cost function
    return w1 * d_g + w2 * d_gbe + w3 * d_r + w4 * theta;
 
}

double ExplorationPlanner::frontier_cost_1(frontier_t frontier) {
    /*
     * cost function
     * f = w1 · d_g   +   w2 · d_gb   +   w3 · d_gbe   +   w4 · theta
     *
     * parameters
     * w1, ..., w4 .. weights
     * d_g         .. euclidean distance from the robot's current position to the frontier
     * d_gbe       .. -d_gb if battery charge is above threshold (e.g. 50%), d_gb if battery charge is below threshold,
     *                where d_gb is the euclidean distance from the frontier to the charging station
     * d_r         .. (MISSING DESCRIPTION) //TODO                 
     * theta       .. measure of how much the robot has to turn to reach frontier, theta in [0,1]
     */

    // frontier position
    double frontier_x = frontier.x_coordinate;
    double frontier_y = frontier.y_coordinate;

    // calculate d_g
    double d_g = frontier.my_distance_to_robot;

    // calculate d_gbe
    double d_gb;
    d_gb = frontier.my_distance_to_optimal_ds;
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
    for(unsigned int i=0; i<last_robot_auctioned_frontier_list.size(); i++) {
        double distance = trajectory_plan_meters(frontier_x, frontier_y, last_robot_auctioned_frontier_list.at(i).x_coordinate, last_robot_auctioned_frontier_list.at(i).y_coordinate);
        if(distance < 0) {
            ROS_ERROR("failed distance");
            continue;
        }
        if(distance < d_r || d_r == 0) 
            d_r = distance;
    }
    d_r = -d_r;

    // calculate theta
    double theta_s = atan2(robot_last_y - robot_y, robot_last_x - robot_x);
    double theta_g = atan2(robot_y - frontier_y, robot_x - frontier_x);
    double theta = 1/M_PI * (M_PI - fabs(fabs(theta_s - theta_g) - M_PI));

    // calculate cost function
    return w1 * d_g + w2 * d_gbe + w3 * d_r + w4 * theta;
 
}

double ExplorationPlanner::fallback_distance_computation(double end_x, double end_y) {
    //return euclidean_distance(robot_x, robot_y, end_x, end_y);
    return 10000000000;
}

double ExplorationPlanner::fallback_distance_computation(double start_x, double start_y, double end_x, double end_y) {
    //return euclidean_distance(start_x, start_y, end_x, end_y);
    return 10000000000;
}

void ExplorationPlanner::pushFrontier(frontier_t frontier) {
    frontiers.push_back(frontier);
}

std::vector<frontier_t> ExplorationPlanner::getFrontierList() {
    return frontiers;
}
