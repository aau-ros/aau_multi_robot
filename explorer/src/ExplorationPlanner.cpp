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

#define MAX_DISTANCE 2000			// max distance to starting point
#define MAX_GOAL_RANGE 0.2			// min distance between frontiers (search)
#define MINIMAL_FRONTIER_RANGE 0.2	// distance between frontiers (selection)
#define INNER_DISTANCE 5			// radius (in cells) around goal point without obstacles (backoff goal point)
#define MAX_NEIGHBOR_DIST 1			// radius (in cells) around selected goal without obstacles
#define CLUSTER_MERGING_DIST 0.8	// max (euclidean) distance between clusters that are merged

using namespace explorationPlanner;

template <typename T>
  std::string NumberToString ( T Number )
  {
     std::ostringstream ss;
     ss << Number;
     return ss.str();
  }

ExplorationPlanner::ExplorationPlanner(int robot_id, bool robot_prefix_empty, std::string robot_name_parameter):
		costmap_ros_(0), occupancy_grid_array_(0), exploration_trans_array_(0), obstacle_trans_array_(
				0), frontier_map_array_(0), is_goal_array_(0), map_width_(0), map_height_(
				0), num_map_cells_(0), initialized_(false), last_mode_(
				FRONTIER_EXPLORE), p_alpha_(0), p_dist_for_goal_reached_(1), p_goal_angle_penalty_(
				0), p_min_frontier_size_(0), p_min_obstacle_dist_(0), p_plan_in_unknown_(
				true), p_same_frontier_dist_(0), p_use_inflated_obs_(false), previous_goal_(
				0), inflated(0), lethal(0), free(0), threshold_free(127) 
				, threshold_inflated(252), threshold_lethal(253),frontier_id_count(0), 
                                exploration_travel_path_global(0), cluster_id(0), initialized_planner(false), 
                                auction_is_running(false), auction_start(false), auction_finished(true), 
                                start_thr_auction(false), auction_id_number(1), next_auction_position_x(0), 
                                next_auction_position_y(0), other_robots_position_x(0), other_robots_position_y(0),
                                number_of_completed_auctions(0), number_of_uncompleted_auctions(0), first_run(true),
                                first_negotiation_run(true), robot_prefix_empty_param(false){
    
    
    trajectory_strategy = "euclidean";
    robot_prefix_empty_param = robot_prefix_empty;

            
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
    ssendAuction = nh_service->serviceClient<adhoc_communication::SendExpAuction>(sendAuction_msgs);

    
//    pub_negotion_first = nh_negotiation_first.advertise <adhoc_communication::Frontier> ("negotiation_list_first", 10000);
    
//    pub_frontiers = nh_frontier.advertise <adhoc_communication::Frontier> ("frontiers", 10000);
//    pub_visited_frontiers = nh_visited_frontier.advertise <adhoc_communication::Frontier> ("visited_frontiers", 10000);
    
    pub_visited_frontiers_points = nh_visited_Point.advertise <visualization_msgs::MarkerArray> ("visitedfrontierPoints", 2000, true);
    
    pub_Point = nh_Point.advertise < geometry_msgs::PointStamped> ("goalPoint", 100, true);
    pub_frontiers_points = nh_frontiers_points.advertise <visualization_msgs::MarkerArray> ("frontierPoints", 2000, true);

//    pub_auctioning_status = nh_auction_status.advertise<adhoc_communication::AuctionStatus> ("auctionStatus", 1000);
//    pub_auctioning_first = nh_auction_first.advertise<adhoc_communication::Auction> ("auction_first", 1000);
    
    //pub_clusters = nh_cluster.advertise<geometry_msgs::PolygonStamped>("clusters", 2000, true);
    
    
    
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
            
    srand((unsigned)time(0));
}

void ExplorationPlanner::Callbacks()
{
    ros::Rate r(10);
    while(ros::ok())
    {
//        publish_subscribe_mutex.lock();
      
        if(robot_name == 1)
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

//void ExplorationPlanner::new_robot_callback(const std_msgs::StringConstPtr &msg)
//{
//    std::string newRobotName = msg.get()->data;
//    for(int i = 0; i < new_robots->size(); i++)
//    {
//        if(new_robots->at(i) == newRobotName)
//        {          
//            return;
//        }
//    }
//    new_robots.push_back(newRobotName);
//    ROS_ERROR("New robot:%s",newRobotName.c_str());
//}

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
	//Occupancy_grid_array is updated here
	this->setupMapData();

	last_mode_ = FRONTIER_EXPLORE;
	this->initialized_ = true;

	/*
	 * reset all counter variables, used to count the number of according blocks
	 * within the occupancy grid.
	 */
	unknown = 0, free = 0, lethal = 0, inflated = 0;
       
}


bool ExplorationPlanner::clusterFrontiers()
{
//    ROS_INFO("Clustering frontiers");
    
    int strategy = 1;
    /*
     * Strategy:
     * 1 ... merge clusters close together
     * 2 ... merge clusters based on model
     */
    
    bool cluster_found_flag = false, same_id = false;
    
        for(int i = 0; i < frontiers.size(); i++)
        {
            ROS_DEBUG("Frontier at: %d   and cluster size: %lu",i, clusters.size());
            cluster_found_flag = false;
            bool frontier_used = false;
            same_id = false;
            
            for(int j = 0; j < clusters.size(); j++)
            {
                ROS_DEBUG("cluster %d contains %lu elements", j, clusters.at(j).cluster_element.size());
                for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
                {
                   ROS_DEBUG("accessing cluster %d  and element: %d", j, n);
                   
                   if(fabs(frontiers.at(i).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate) < MAX_NEIGHBOR_DIST && fabs(frontiers.at(i).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate) < MAX_NEIGHBOR_DIST) 
                   {    
                       for(int m = 0; m < clusters.at(j).cluster_element.size(); m++)
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
            for(int i = 0; i < clusters.size(); i++)
            {
                for(int m = 0; m < clusters.at(i).cluster_element.size(); m ++)
                {   
                    if(clusters.at(i).cluster_element.size() > 1)
                    {
                        for(int j = 0; j < clusters.size(); j++)
                        {
                            if(clusters.at(i).id != clusters.at(j).id)
                            {
                                if(clusters.at(j).cluster_element.size() > 1)
                                {                            
                                    for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
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
                                        for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
                                        {
                                            frontier_t frontier_to_merge;
                                            frontier_to_merge = clusters.at(j).cluster_element.at(n);
                                            clusters.at(i).cluster_element.push_back(frontier_to_merge);
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
            for(int i = 0; i < clusters.size(); i++)
            {
                for(int m = 0; m < clusters.at(i).cluster_element.size(); m ++)
                {   
                    if(clusters.at(i).cluster_element.size() > 1)
                    {
                        for(int j = 0; j < clusters.size(); j++)
                        {
                            if(clusters.at(i).id != clusters.at(j).id)
                            {
                                if(clusters.at(j).cluster_element.size() > 1)
                                {                            
                                    for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
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
                                                cost = costmap_global_ros_->getCostmap()->getCost(mx, my);  
                                               
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
                                                cost = costmap_global_ros_->getCostmap()->getCost(mx, my);  
                                               
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

//std::string ExplorationPlanner::lookupRobotName(int robot_name_int)
//{
//    if(robot_name_int == 0)
//        return ("turtlebot");
//    if(robot_name_int == 1)
//        return ("joy");
//    if(robot_name_int == 2)
//        return ("marley");
//    if(robot_name_int == 3)
//        return ("bob"); 
//    if(robot_name_int == 4)
//        return ("hans"); 
//}

bool ExplorationPlanner::transformToOwnCoordinates_frontiers()
{
    ROS_INFO("Transform frontier coordinates");
    
    store_frontier_mutex.lock();
    
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
                
                service_message.request.point.x = frontiers.at(i).x_coordinate;
                service_message.request.point.y = frontiers.at(i).y_coordinate;
                service_message.request.point.src_robot = robo_name;

                ROS_DEBUG("Robot name:  %s", service_message.request.point.src_robot.c_str());
                ROS_DEBUG("Old x: %f   y: %f", frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
               
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
            }
        }
    }
    store_frontier_mutex.unlock();
    ROS_INFO(" Transform frontier coordinates DONE");
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
                
                service_message.request.point.x = visited_frontiers.at(i).x_coordinate;
                service_message.request.point.y = visited_frontiers.at(i).y_coordinate;
                service_message.request.point.src_robot = robo_name;

                ROS_DEBUG("Old visited x: %f   y: %f", visited_frontiers.at(i).x_coordinate, visited_frontiers.at(i).y_coordinate);
               
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
            }
        }
    }
    ROS_INFO(" Transform visited frontier coordinates DONE");
}



bool ExplorationPlanner::check_trajectory_plan()
{       
    geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;
    int distance; 
    
    ROS_INFO("Check Trajectory Length");
    if (!costmap_global_ros_->getRobotPose(robotPose))
    {
            ROS_ERROR("Failed to get RobotPose");
    }

    startPointSimulated.header.seq = start_point_simulated_message++;	// increase the sequence number
    startPointSimulated.header.stamp = ros::Time::now();
    startPointSimulated.header.frame_id = move_base_frame;
    startPointSimulated.pose.position.x = robotPose.getOrigin().getX();
    startPointSimulated.pose.position.y = robotPose.getOrigin().getY();
    startPointSimulated.pose.position.z = 0;
    startPointSimulated.pose.orientation.x = robotPose.getRotation().getX();
    startPointSimulated.pose.orientation.y = robotPose.getRotation().getY();
    startPointSimulated.pose.orientation.z = robotPose.getRotation().getZ();
    startPointSimulated.pose.orientation.w = 1;

    for(int i = 0; i<10; i++)
    {
        if(frontiers.size() > i)
        {   
            std::vector<double> backoffGoal;
            bool backoff_flag = smartGoalBackoff(frontiers.at(i).x_coordinate,frontiers.at(i).y_coordinate, costmap_global_ros_, &backoffGoal);
            
            
            goalPointSimulated.header.seq = goal_point_simulated_message++;	// increase the sequence number
            goalPointSimulated.header.stamp = ros::Time::now();
            goalPointSimulated.header.frame_id = move_base_frame;
            
//            backoff_flag = false; // TODO
            if(backoff_flag == true)
            {
                goalPointSimulated.pose.position.x = backoffGoal.at(0);
                goalPointSimulated.pose.position.y = backoffGoal.at(1);
            }else
            {
                goalPointSimulated.pose.position.x = frontiers.at(i).x_coordinate;
                goalPointSimulated.pose.position.y = frontiers.at(i).y_coordinate;
            }
            goalPointSimulated.pose.position.z = 0;
            goalPointSimulated.pose.orientation.x = 0;
            goalPointSimulated.pose.orientation.y = 0;
            goalPointSimulated.pose.orientation.z = 0;
            goalPointSimulated.pose.orientation.w = 1;

            std::vector<geometry_msgs::PoseStamped> global_plan;

            nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);
            distance =  global_plan.size();
            frontiers.at(i).distance_to_robot = distance;

            ROS_DEBUG("Distance: %d Frontier: %d",distance, frontiers.at(i).id);
        }else
        {
            break;
        }
    }
    ROS_DEBUG("trajectory finished");
    return(true);
}


int ExplorationPlanner::calculate_travel_path(double x, double y)
{
        geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;
    double distance; 
    
    ROS_DEBUG("Check Trajectory");
    if (!costmap_global_ros_->getRobotPose(robotPose))
    {       
            ROS_ERROR("Failed to get RobotPose");
    }

    std::vector<double> backoffGoal;
    bool backoff_flag = smartGoalBackoff(x,y, costmap_global_ros_, &backoffGoal);
            
    startPointSimulated.header.seq = start_point_simulated_message++;	// increase the sequence number
    startPointSimulated.header.stamp = ros::Time::now();
    startPointSimulated.header.frame_id = move_base_frame;
    startPointSimulated.pose.position.x = robotPose.getOrigin().getX();
    startPointSimulated.pose.position.y = robotPose.getOrigin().getY();
    startPointSimulated.pose.position.z = 0;
    startPointSimulated.pose.orientation.x = 0;
    startPointSimulated.pose.orientation.y = 0;
    startPointSimulated.pose.orientation.z = 0;
    startPointSimulated.pose.orientation.w = 1;

   
    goalPointSimulated.header.seq = goal_point_simulated_message++;	// increase the sequence number
    goalPointSimulated.header.stamp = ros::Time::now();
    goalPointSimulated.header.frame_id = move_base_frame;
    
//    backoff_flag = false; // TODO
    if(backoff_flag == true)
    {
        goalPointSimulated.pose.position.x = backoffGoal.at(0);
        goalPointSimulated.pose.position.y = backoffGoal.at(1);
    }else
    {
        goalPointSimulated.pose.position.x = x;
        goalPointSimulated.pose.position.y = y;
    }
    goalPointSimulated.pose.position.z = 0;
    goalPointSimulated.pose.orientation.x = 0;
    goalPointSimulated.pose.orientation.y = 0;
    goalPointSimulated.pose.orientation.z = 0;
    goalPointSimulated.pose.orientation.w = 1;

    std::vector<geometry_msgs::PoseStamped> global_plan;

    bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);
    
    ROS_INFO("Plan elements: %f", (double)global_plan.size()*0.02);

    if(successful == true)
    {
        distance =  global_plan.size();   
        exploration_travel_path_global = exploration_travel_path_global + distance;  
        return distance;
    }else
    {
        return 0;
    }
}

int ExplorationPlanner::check_trajectory_plan(double x, double y)
{       
    geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;
    int distance; 
    
    ROS_DEBUG("Check Trajectory");
    if (!costmap_global_ros_->getRobotPose(robotPose))
    {       
            ROS_ERROR("Failed to get RobotPose");
    }
   
    std::vector<double> backoffGoal;
    bool backoff_flag = smartGoalBackoff(x,y, costmap_global_ros_, &backoffGoal);
    
    startPointSimulated.header.seq = start_point_simulated_message++;	// increase the sequence number
    startPointSimulated.header.stamp = ros::Time::now();
    startPointSimulated.header.frame_id = move_base_frame;
    startPointSimulated.pose.position.x = robotPose.getOrigin().getX();
    startPointSimulated.pose.position.y = robotPose.getOrigin().getY();
    startPointSimulated.pose.position.z = 0;
    startPointSimulated.pose.orientation.x = 0;
    startPointSimulated.pose.orientation.y = 0;
    startPointSimulated.pose.orientation.z = 0;
    startPointSimulated.pose.orientation.w = 1;

   
    goalPointSimulated.header.seq = goal_point_simulated_message++;	// increase the sequence number
    goalPointSimulated.header.stamp = ros::Time::now();
    goalPointSimulated.header.frame_id = move_base_frame;
    
//    backoff_flag = false; // TODO
    if(backoff_flag == true)
    {
        goalPointSimulated.pose.position.x = backoffGoal.at(0);
        goalPointSimulated.pose.position.y = backoffGoal.at(1);
    }else
    {
        goalPointSimulated.pose.position.x = x;
        goalPointSimulated.pose.position.y = y;
    }
    goalPointSimulated.pose.position.z = 0;
    goalPointSimulated.pose.orientation.x = 0;
    goalPointSimulated.pose.orientation.y = 0;
    goalPointSimulated.pose.orientation.z = 0;
    goalPointSimulated.pose.orientation.w = 1;

    std::vector<geometry_msgs::PoseStamped> global_plan;

//    tf::TransformListener tf_planner(ros::Duration(10));
//    base_local_planner::TrajectoryPlannerROS tp;
//    tp.initialize("my_trajectory_planner", &tf_planner, &costmap_global_ros_);

    bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);
    
    if(successful == true)
    {       
        distance =  global_plan.size();          
        return distance;
    }else
    {
//        ROS_ERROR("Distance calculated wrongly");
        return -1;
    }
}

int ExplorationPlanner::estimate_trajectory_plan(double start_x, double start_y, double target_x, double target_y)
{       
    geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;
    int distance; 
    
    ROS_DEBUG("Check Trajectory");
    if (!costmap_ros_->getRobotPose(robotPose))
    {       
            ROS_ERROR("Failed to get RobotPose");
    }

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
    }else
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

    bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);

    if(successful == true)
    {
        distance =  global_plan.size();      
        return distance;
    }else
    {
        return -1;
    }
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

        store_frontier_mutex.lock(); 
        frontiers.push_back(new_frontier);
        store_frontier_mutex.unlock();
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

        store_frontier_mutex.lock(); 
        frontiers.push_back(new_frontier);
        store_frontier_mutex.unlock();
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
                store_frontier_mutex.lock();
                frontiers.erase(frontiers.begin()+i);
//                if(i > 0)
//                {
//                    i --;
//                }
                store_frontier_mutex.unlock();
                //break; //FIXME ... only a test
            }
        }else
        {
            if(frontiers.at(i).id == id)
            {
                store_frontier_mutex.lock();
                frontiers.erase(frontiers.begin()+i);
                if(i > 0)
                {
                    i --;
                }
                store_frontier_mutex.unlock();
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

        frontiers.push_back(unreachable_frontier);
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

bool ExplorationPlanner::removeUnreachableFrontier(int id, std::string detected_by_robot_str)
{
    for(int i= 0; i< unreachable_frontiers.size(); i++)
    {
        if(robot_prefix_empty_param == true)
        {
            if(unreachable_frontiers.at(i).id == id && unreachable_frontiers.at(i).detected_by_robot_str.compare(detected_by_robot_str) == 0)
            {
                ROS_INFO("Removing Unreachable Frontier ID: %d  at position: %d  of Robot: %s", unreachable_frontiers.at(i).id, i, unreachable_frontiers.at(i).detected_by_robot_str.c_str());
               
                unreachable_frontiers.erase(unreachable_frontiers.begin()+i);
//                if(i > 0)
//                {
//                    i --;
//                }
                break;
            }
        }else
        {
            if(unreachable_frontiers.at(i).id == id)
            {
                unreachable_frontiers.erase(unreachable_frontiers.begin()+i);
                if(i > 0)
                {
                    i --;
                }
                break;
            }
        }
    }

    return true;
}

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
                ROS_DEBUG("Negotiation frontier with ID: %ld", frontier_element.id);
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


//void ExplorationPlanner::auctionStatusCallback(const adhoc_communication::AuctionStatus::ConstPtr& msg)
//{
//    ROS_INFO("Calling AuctionStatusCallback");
//    
//    auction_start = msg.get()->start_auction;
//    auction_finished = msg.get()->auction_finished;
//    adhoc_communication::Cluster occupied_ids;
//    std::vector<int> requested_cluster_ids;
//    
//    /*
//     * Visualizing the received message
//     */
////    for(int i = 0; i < msg.get()->requested_clusters.size(); i++)
////    {
////        adhoc_communication::Cluster cluster_req; 
////        cluster_req = msg.get()->requested_clusters.at(i);
////        ROS_INFO("------------------- RECEIVED REQUEST %d ----------------------", i);
////        std::string requested_clusters;
////        for(int j = 0; j < cluster_req.ids_contained.size(); j++)
////        {
////            requested_clusters.append(NumberToString((int)cluster_req.ids_contained.at(j)));
////            requested_clusters.append(", ");
////        }
////        ROS_INFO("Received auction with auction number: %d", msg.get()->auction_id);
////        ROS_INFO("Received requested ids: %s", requested_clusters.c_str());
////    }
//    
//    /*
//     * Cluster all available frontiers to be able to compare clusters
//     * with other robots
//     */
//    clusterFrontiers();
//    
//    if(msg.get()->occupied_ids.size() > 0 || msg.get()->requested_clusters.size() > 0)
//    {
//        for(int i = 0; i < msg.get()->occupied_ids.size(); i++)
//        {
//            occupied_ids.ids_contained.push_back(msg.get()->occupied_ids.at(i));
//        }
//        int occupied_cluster_id = checkClustersID(occupied_ids);     
//        if(occupied_cluster_id >=0)
//        {
//            already_used_ids.push_back(occupied_cluster_id);
//        }      
////        ROS_INFO("Requested cluster size: %lu", msg.get()->requested_clusters.size());
//        for(int i = 0; i < msg.get()->requested_clusters.size(); i++)
//        {
//            adhoc_communication::Cluster requested_cluster,requested_ids; 
//            
//            requested_cluster = msg.get()->requested_clusters.at(i);
////            ROS_INFO("IDS_contained: %lu",requested_cluster.ids_contained.size());
//            
////            for(int j = 0; j < requested_cluster.ids_contained.size(); j++)
////            {
//////                ROS_INFO("received ids: %d", requested_cluster.ids_contained.at(j));
////                requested_ids.ids_contained.push_back(requested_cluster.ids_contained.at(j));
////            }
////            ROS_INFO("----------------------------------------------------------");
//            requested_cluster_ids.push_back(checkClustersID(requested_cluster));
//        }   
//    }
//    
//    
//    if(auction_start == true)
//    {
//        start_thr_auction = true;
//        thr_auction_status = boost::thread(&ExplorationPlanner::respondToAuction, this, requested_cluster_ids);
//    }
//}

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
//                ROS_INFO("Robot %d sending BID: %f cluster elements: %lu", robot_name, cluster_msg_check.bid, cluster_msg_check.ids_contained.size());
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
               distance = check_trajectory_plan(clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate, clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate);
//               distance = estimate_trajectory_plan(robotPose.getOrigin().getX(), robotPose.getOrigin().getY(), clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate, clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate);
      
//               ROS_INFO("Distance for cluster %d: %d",clusters.at(cluster_vector_position).id, distance);
       
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
    ROS_DEBUG("Position Callback !!!!!");
    position_mutex.lock();
    
    other_robots_positions.positions.clear();
    ROS_INFO("positions size: %lu", msg.get()->positions.size());
    for(int i = 0; i < msg.get()->positions.size(); i++)
    {    
        other_robots_positions.positions.push_back(msg.get()->positions.at(i));
    }
    position_mutex.unlock();
}


void ExplorationPlanner::auctionCallback(const adhoc_communication::ExpAuction::ConstPtr& msg)
{
    auction_running = true;
    //ROS_ERROR("CALLING AUCTION CALLBACK!!!!!!!!!!!!");
    int robots_int_name;
    
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
           
            for(int i = 0; i < msg.get()->requested_clusters.size(); i++)
            {  
                adhoc_communication::ExpCluster cluster_req; 
                cluster_req = msg.get()->requested_clusters.at(i);
                std::string requested_clusters; 
                for(int j = 0; j < cluster_req.ids_contained.size(); j++)
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
                for(int i = 0; i < unrecognized_occupied_clusters.size(); i++)
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
                for(int i = 0; i < msg.get()->requested_clusters.size(); i++)
                {    
                    adhoc_communication::ExpCluster requested_cluster;
                    requested_cluster = msg.get()->requested_clusters.at(i);
                    
                    int check_cluster_id = checkClustersID(requested_cluster);
                    if(check_cluster_id >= 0)
                    {
                        requested_cluster_t new_cluster_request; 
                        for(int j = 0; j < requested_cluster.ids_contained.size(); j++)
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

                for(int i = 0; i < robots_already_responded.size(); i++)
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
                    for(int i = 0; i < msg.get()->available_clusters.size(); i++)
                    {
                        adhoc_communication::ExpCluster cluster_req; 
                        cluster_req = msg.get()->available_clusters.at(i);
                        ROS_INFO("---------------------- %d ----------------------------", i);
                        std::string requested_clusters;
                        for(int j = 0; j < cluster_req.ids_contained.size(); j++)
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



                    for(int i = 0; i < msg.get()->available_clusters.size(); i++)
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


bool sortCompareElements(const ExplorationPlanner::compare_pair_t &lhs, const ExplorationPlanner::compare_pair_t &rhs)
{       
        return lhs.identical_ids > rhs.identical_ids;    
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
    
    
    
    
    for(int j = 0; j < clusters.size(); j++)
    {
        double same_id_found = 0;
//        ROS_INFO("--------------------------------------------------------------");
        for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
        {
//            for(int m= 0; m < clusters.at(j).cluster_element.size(); m++)
//            {
//                ROS_INFO("Ids in cluster to check with: %d",clusters.at(j).cluster_element.at(m).id);
//            }
//            ROS_INFO("Ids in cluster to check with: %d",clusters.at(j).cluster_element.at(n).id);
            for(int i = 0; i < cluster_to_check.ids_contained.size(); i++)
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


//void ExplorationPlanner::controlCallback( const bla& msg)
//{
//    /* Check if manual control or automatic. If automatic, set simulation to false, otherwise set simulation to true in simple_navigation */
//    
//    SimpleNavigation.Simulation = true; 
//}



void ExplorationPlanner::frontierCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{  
    
//    ROS_ERROR("FRONTIER RECEIVED!!!!!!!!!!!!!!!!!!!!!!!!");
    adhoc_communication::ExpFrontierElement frontier_element; 
    for(int i = 0; i < msg.get()->frontier_element.size(); i++)
    {
        frontier_element = msg.get()->frontier_element.at(i);
        bool result = true;
        for (unsigned int j = 0; j < frontiers.size(); j++)
        {
            if(robot_prefix_empty_param == true)
            {
//                ROS_ERROR("FrontierCallback ... ");
                if(frontiers.at(j).detected_by_robot_str.compare(frontier_element.detected_by_robot_str) == 0 && frontier_element.id == frontiers.at(j).id)
                {
//                    ROS_ERROR("Same Detected ...");
                    result = false;
                    break;
                }
            }
            else
            {
                if(frontier_element.detected_by_robot == robot_name)
                {          
                    result = false;
                    break;      
                }
                else
                {
                    if (frontier_element.id == frontiers.at(j).id)
                    {  
                        result = false;
                        break;
                    }
                }
            }
        }
        if (result == true)
        {
            
            if(robot_prefix_empty_param == true)
            {
                ROS_DEBUG("Received New Frontier with ID: %ld  Robot: %s", frontier_element.id, frontier_element.detected_by_robot_str.c_str());
                storeFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, frontier_element.detected_by_robot_str, frontier_element.id);
            }else
            {
                ROS_DEBUG("Received New Frontier of Robot %ld with ID %ld", frontier_element.detected_by_robot, frontier_element.id);
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
    for(int i = 0; i < msg.get()->frontier_element.size(); i++)
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
            ROS_DEBUG("Received New Visited Frontier of Robot %ld with ID %ld", frontier_element.detected_by_robot, frontier_element.id);
            if(robot_prefix_empty_param == true)
            {
                ROS_DEBUG("Storing Visited Frontier ID: %ld  Robot: %s", frontier_element.id, frontier_element.detected_by_robot_str.c_str());
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
    for(int i = 0; i<frontiers.size(); i++)
    {
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
    
//    pub_frontiers.publish(frontier_msg);
    sendToMulticast("mc_",frontier_msg, "frontiers");
   
    publish_subscribe_mutex.unlock();
}

bool ExplorationPlanner::publish_visited_frontier_list()
{
//    ROS_INFO("PUBLISHING VISITED FRONTIER LIST");
    
    publish_subscribe_mutex.lock();
    
    adhoc_communication::ExpFrontier visited_frontier_msg;
    
    for(int i = 0; i<visited_frontiers.size(); i++)
    {
        adhoc_communication::ExpFrontierElement visited_frontier_element;
//        visited_frontier_element.id = frontiers.at(i).id;
        visited_frontier_element.detected_by_robot = visited_frontiers.at(i).detected_by_robot;
        visited_frontier_element.detected_by_robot_str = visited_frontiers.at(i).detected_by_robot_str;
        visited_frontier_element.robot_home_position_x = visited_frontiers.at(i).robot_home_x; 
        visited_frontier_element.robot_home_position_y = visited_frontiers.at(i).robot_home_y;
        visited_frontier_element.x_coordinate = visited_frontiers.at(i).x_coordinate;
        visited_frontier_element.y_coordinate = visited_frontiers.at(i).y_coordinate;
       
        visited_frontier_msg.frontier_element.push_back(visited_frontier_element);
    }
   
//    pub_visited_frontiers.publish(visited_frontier_msg);
    sendToMulticast("mc_",visited_frontier_msg, "visited_frontiers");
    
    publish_subscribe_mutex.unlock();
}

/*
 * Check if the next goal is efficient enough to steer the robot to it.
 * If it is a goal, which has previously be seen, it is not required
 * to visit this goal again.
 * make sure that there are no failures in calculation! Therefore
 * this plausibility check is done. Only values of less then 50m
 * are valid. (Calculation of coordinates in the costmap often
 * produce very big values which are miss-interpretations)
 */
bool ExplorationPlanner::check_efficiency_of_goal(double x, double y) {

//    ROS_INFO("Check efficiency");
	double diff_home_x = visited_frontiers.at(0).x_coordinate - x;
	double diff_home_y = visited_frontiers.at(0).y_coordinate - y;
        
	if (fabs(diff_home_x) <= MAX_DISTANCE && fabs(diff_home_y) <= MAX_DISTANCE) 
        {
		for (int i = 0; i < visited_frontiers.size(); i++)
		{
                    /*
                     * Calculate the distance between all previously seen goals and the new
                     * found frontier!!
                     */
                    double diff_x = visited_frontiers.at(i).x_coordinate - x;
                    double diff_y = visited_frontiers.at(i).y_coordinate - y;

                    if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE) {
                        ROS_DEBUG("x: %f  y: %f too close to visited at x: %f   y: %f   dif_x: %f   dif_y: %f", x, y, visited_frontiers.at(i).x_coordinate, visited_frontiers.at(i).y_coordinate, diff_x, diff_y);
                        return false;
                    }
		}
                for (int i = 0; i < unreachable_frontiers.size(); i++)
		{
                    /*
                     * Calculate the distance between all previously seen goals and the new
                     * found frontier!!
                     */
                    double diff_x = unreachable_frontiers.at(i).x_coordinate - x;
                    double diff_y = unreachable_frontiers.at(i).y_coordinate - y;

                    if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE) {
                        ROS_DEBUG("x: %f  y: %f too close to unreachable at x: %f   y: %f   dif_x: %f   dif_y: %f", x, y, unreachable_frontiers.at(i).x_coordinate, unreachable_frontiers.at(i).y_coordinate, diff_x, diff_y);
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

void ExplorationPlanner::clearVisitedAndSeenFrontiersFromClusters()
{
    ROS_INFO("Clear VisitedAndSeenFrontiers from Cluster");
    std::vector<int> goals_to_clear;
    
    
    
    for(int i = 0; i < clusters.size(); i++)
    {
        for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
        {
            /* Now iterate over all frontiers and check if cluster elements are 
             * still available in the frontier vector
             */
            bool cluster_still_valid = false; 
            for(int m = 0; m < frontiers.size(); m++)
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
    
    for(int i = 0; i < clusters.size(); i++)
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
    
    for (int i = 1; i < visited_frontiers.size(); i++)
    {
        for (int j = 0; j < frontiers.size(); j++)
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
    
    for (int i = 1; i < unreachable_frontiers.size(); i++)
    {
        for (int j = 0; j < frontiers.size(); j++)
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
        for(int i = 0; i < frontiers.size(); i++)
        {
            unsigned int new_mx, new_my;
            bool unknown_found = false; 
            bool obstacle_found = false;
            bool freespace_found = false;

            mx = 0; 
            my = 0;
            
           // ROS_INFO("frontier x: %f   y: %f", frontiers.at(i).x_coordinate,frontiers.at(i).y_coordinate);
            if(!global_costmap->getCostmap()->worldToMap(frontiers.at(i).x_coordinate,frontiers.at(i).y_coordinate,mx,my))
            {
                ROS_ERROR("Cannot convert coordinates successfully.");
                continue;
            }
//            ROS_INFO("Map coordinates mx: %d  my: %d",mx,my);

            neighbours = getMapNeighbours(mx, my, 6);
            
//            ROS_INFO("Neighbours: %lu", neighbours.size());
            for (int j = 0; j < neighbours.size()/2; j++)
            {

               
//                ROS_INFO("Get Neighbour %d and %d",j*2, j*2+1);
                new_mx = neighbours.at(j*2);
                new_my = neighbours.at(j*2+1);
//                ROS_INFO("got access");
                
                    
//                ROS_INFO("Calculating at position x: %d    y: %d", new_mx, new_my);     
                unsigned char cost = global_costmap->getCostmap()->getCost(new_mx, new_my);
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
        
//        ROS_INFO("Clearing %lu frontiers", goals_to_clear.size());
//        for(int i= 0; i< goals_to_clear.size(); i++)
//        {
//    //        ROS_DEBUG("Frontier with ID: %d already seen and therefore deleted", goals_to_clear.at(i));
//            removeStoredFrontier(goals_to_clear.at(i)); 
//        }
    }
}

bool ExplorationPlanner::smartGoalBackoff(double x, double y, costmap_2d::Costmap2DROS *global_costmap, std::vector<double> *new_goal)
{
    unsigned int mx, my, new_mx, new_my, inner_mx, inner_my;
    double wx, wy;
    std::vector<int> neighbours, inner_neighbours;
   
//    this->costmap_global_ros_ = global_costmap;
//    costmap_global_ros_->getCostmapCopy(costmap_global_);
//    costmap_global_ = costmap_global_ros_;
    
    if(!global_costmap->getCostmap()->worldToMap(x,y,mx,my))
    {
        ROS_ERROR("Cannot convert coordinates successfully.");
    }
//    ROS_DEBUG("Map coordinates mx: %d  my: %d",mx,my);
  
    neighbours = getMapNeighbours(mx, my, 40);
//    ROS_DEBUG("Got neighbours");
    for (int j = 0; j< neighbours.size()/2; j++)
    {
//        ROS_DEBUG("Get neighbours %d and %d",j*2,j*2+1);
        new_mx = neighbours.at(j*2);
        new_my = neighbours.at(j*2+1);

        unsigned char cost = global_costmap->getCostmap()->getCost(new_mx, new_my);
        
        if( cost == costmap_2d::FREE_SPACE)
        {
            bool back_off_goal_found = true;
            
            inner_neighbours = getMapNeighbours(new_mx, new_my, INNER_DISTANCE);
            for (int i = 0; i< inner_neighbours.size()/2; i++)
            {
//                ROS_DEBUG("Get inner neighbours %d and %d",i*2,i*2+1);
                inner_mx = inner_neighbours.at(i*2);
                inner_my = inner_neighbours.at(i*2+1);
                
                unsigned char inner_cost = global_costmap->getCostmap()->getCost(inner_mx, inner_my);
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
void ExplorationPlanner::findFrontiers() {

        ROS_INFO("Find Frontiers");
        allFrontiers.clear();
	int select_frontier = 1;
	std::vector<double> final_goal,start_points;
	// list of all frontiers in the occupancy grid

//	// get latest costmap
//	clearFrontiers();

	/*
	 * check for all cells in the occupancy grid whether
	 * or not they are frontier cells. If a possible frontier is found, true is
	 * returned
	 */
	int new_frontier_point = 0;
	for (unsigned int i = 0; i < num_map_cells_; i++) {

		int new_frontier_point = isFrontier(i);
		if (new_frontier_point != 0) {
			/*
			 * If isFrontier() returns true, the point which is checked to be a frontier
			 * is indeed a frontier.
			 *
			 * Push back adds data x to a vector.
			 * If a frontier was found, the position of the frontier is stored
			 * in the allFrontiers vector.
			 */
			allFrontiers.push_back(new_frontier_point);
		}
	}
        ROS_INFO("Found %lu frontier cells which are transformed into frontiers points. Starting transformation...", allFrontiers.size());

	/*
	 * Iterate over all frontiers. The frontiers stored in allFrontiers are
	 * already REAL frontiers and can be approached by the robot to get new
	 * informations of the environment.
	 * To limit the amount of frontiers and only pick those which are valuable to
	 * drive there, check neighboring frontiers and only remember them if they are
	 * further then a distance of 40cm away from each other, discard otherwise.
	 * The rest of the frontiers are stored in a goal buffer which contain all
	 * frontiers within the map. Additionally check whether or not a newly found
	 * frontier has already been added to the list. If it is already in the list, do
	 * not make a new entry with the coordinates of the frontier!
	 */
	for (unsigned int i = 0; i < allFrontiers.size(); ++i) {

		geometry_msgs::PoseStamped finalFrontier;
		double wx, wy, wx2, wy2, wx3, wy3;
		unsigned int mx, my, mx2, my2, mx3, my3;
		bool result;

                
		costmap_ros_->getCostmap()->indexToCells(allFrontiers.at(i), mx, my);
		costmap_ros_->getCostmap()->mapToWorld(mx, my, wx, wy);
              
//                ROS_INFO("index: %d   map_x: %d   map_y: %d   world_x: %f   world_y: %f", allFrontiers.at(i), mx, my, wx, wy);
                
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
			if (result == true)
			{
				storeFrontier(wx,wy,robot_name,robot_str,-1);
			}
		}
		else if(select_frontier == 2)
		{
			std::vector<int> neighbour_index;

			for (unsigned int j = 0; j < allFrontiers.size(); ++j)
			{
				costmap_ros_->getCostmap()->indexToCells(allFrontiers[j], mx2, my2);
				costmap_ros_->getCostmap()->mapToWorld(mx2, my2, wx2, wy2);

				if (fabs(wx - wx2) <= MINIMAL_FRONTIER_RANGE && fabs(wy - wy2) <= MINIMAL_FRONTIER_RANGE && fabs(wx - wx2) != 0 && fabs(wy - wy2) != 0)
				{
					neighbour_index.push_back(allFrontiers[j]);
				}
			}


			for (unsigned int n = 0; n < neighbour_index.size(); ++n)
			{
				costmap_ros_->getCostmap()->indexToCells(neighbour_index[n], mx2, my2);
				costmap_ros_->getCostmap()->mapToWorld(mx2, my2, wx2, wy2);

				while(true)
				{
					bool end_point_found = true;
					for (unsigned int k = 0; k < allFrontiers.size(); ++k)
					{
						costmap_ros_->getCostmap()->indexToCells(allFrontiers[k], mx3, my3);
						costmap_ros_->getCostmap()->mapToWorld(mx3, my3, wx3, wy3);

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

	ROS_INFO("Size of all frontiers in the list: %lu", frontiers.size());
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
     *       KuhnMunkres algorithm.
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
    ROS_INFO("Cluster Size: %lu", clusters.size());
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
        ROS_INFO("****** i: %d   auction size: %lu ******", i, auction.size());
        for(int j = 0; j < col; j++)
        {     
            ROS_INFO("j: %d   cluster size: %lu", j, clusters.size());
            bool found_a_bid = false;
            bool cluster_valid_flag = false; 
            
//            if(i < auction.size())
//            {    
                for(int n = 0; n < auction.at(i).auction_element.size(); n++)
                {            
                    cluster_valid_flag = true; 
//                    if(j < clusters.size())
//                    {
//                        ROS_INFO("j: %d    smaller then clusters.size(): %lu", j, clusters.size());
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
                
                ROS_INFO("---- clusters: %lu element size: %lu  robots positions: %lu ----", clusters.size(), clusters.at(j).cluster_element.size(), other_robots_positions.positions.size());
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
                            distance = estimate_trajectory_plan(other_robots_position_x, other_robots_position_y, clusters.at(j).cluster_element.at(d).x_coordinate, clusters.at(j).cluster_element.at(d).y_coordinate);                    
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
            for (int i = 0 + count; i < frontiers.size(); i++)
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
                        for (int i = 0; i < clusters.size(); i++)
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
                for (int i = 0 + count; i < clusters.size(); i++)
                {
//                    ROS_INFO("Cluster vector: %d  i: %d ", cluster_vector_position, i);
                    i = i+ cluster_vector_position;
                    i = i % (clusters.size());
                    for (int j = 0; j < clusters.at(i).cluster_element.size(); j++)
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
                        for (int i = 0; i < clusters.size(); i++)
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
                    for (int j = 0; j < clusters.at(position).cluster_element.size(); j++)
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
                        for (int i = 0; i < clusters.size(); i++)
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
                              
//                    ROS_INFO("position: %lu", (cluster_vector_position +count) % (clusters.size()));
                    int position = (cluster_vector_position +count) % (clusters.size());
                    for (int j = 0; j < clusters.at(position).cluster_element.size(); j++)
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

bool sortCluster(const ExplorationPlanner::cluster_t &lhs, const ExplorationPlanner::cluster_t &rhs)
{   
    if(lhs.cluster_element.size() > 1 && rhs.cluster_element.size() > 1)
    {
        return lhs.cluster_element.front().dist_to_robot < rhs.cluster_element.front().dist_to_robot; 
    }
}

void ExplorationPlanner::sort(int strategy)
{
//    ROS_INFO("Sorting");
        /*
         * Following Sort algorithm normalizes all distances to the 
         * robots actual position. As result, the list is sorted 
         * from smallest to biggest deviation between goal point and
         * robot! 
         */
        if(strategy == 1)
        {
            tf::Stamped < tf::Pose > robotPose;
            if (!costmap_ros_->getRobotPose(robotPose))
            {
                    ROS_ERROR("Failed to get RobotPose");
            }
            
            if (frontiers.size() > 0) 
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
            } else 
            {
                    ROS_INFO("Sorting not possible, no frontiers available!!!");
            }
        }
        else if(strategy == 2)
        {
            tf::Stamped < tf::Pose > robotPose;
            if (!costmap_ros_->getRobotPose(robotPose))
            {
                    ROS_ERROR("Failed to get RobotPose");
            }
            
            if (frontiers.size() > 0) 
            {
                    for (int i = frontiers.size(); i >= 0; i--) 
                    {
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
            }else 
            {
                    ROS_INFO("Sorting not possible, no frontiers available!!!");
            }
        }
        else if(strategy == 3)
        {
            tf::Stamped < tf::Pose > robotPose;
            if (!costmap_ros_->getRobotPose(robotPose))
            {
                    ROS_ERROR("Failed to get RobotPose");
            }
            
            if (frontiers.size() > 0) 
            {  
                check_trajectory_plan();

                for (int i = frontiers.size(); i >= frontiers.size()-10; i--) 
                {


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
            } else 
            {
                    ROS_INFO("Sorting not possible, no frontiers available!!!");
            }
        }
        else if(strategy == 4)
        {
            ROS_INFO("sort(4)");
            tf::Stamped < tf::Pose > robotPose;
            if (!costmap_ros_->getRobotPose(robotPose))
            {
                    ROS_ERROR("Failed to get RobotPose");
            }
            double pose_x = robotPose.getOrigin().getX();
            double pose_y = robotPose.getOrigin().getY();
            if(clusters.size() > 0)
            {
                for(int cluster_number = 0; cluster_number < clusters.size(); cluster_number++)
                {
                    if (clusters.at(cluster_number).cluster_element.size() > 0) 
                    {
                        ROS_DEBUG("Cluster %d  size: %lu",cluster_number, clusters.at(cluster_number).cluster_element.size());
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

//                                        int distance = check_trajectory_plan(clusters.at(cluster_number).cluster_element.at(j).x_coordinate, clusters.at(cluster_number).cluster_element.at(j).y_coordinate);
//                                        int distance_next = check_trajectory_plan(clusters.at(cluster_number).cluster_element.at(j+1).x_coordinate,clusters.at(cluster_number).cluster_element.at(j+1).y_coordinate);
                                    
//                                        if(distance > distance_next) 
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
//                         ROS_INFO("Sorting not possible, no elements available!!!");
                    }
                }
            }else
            {
                ROS_INFO("Sorting not possible, no clusters available!!!");
            }
        }        
        else if(strategy == 5)
        {
            ROS_INFO("sort(5)");
            tf::Stamped < tf::Pose > robotPose;
            if (!costmap_ros_->getRobotPose(robotPose))
            {
                    ROS_ERROR("Failed to get RobotPose");
            }
            double pose_x = robotPose.getOrigin().getX();
            double pose_y = robotPose.getOrigin().getY();
            
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
                        
                        /* ********** TRAVEL PATH ********** */
//                        int distance = check_trajectory_plan(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate);
                                                
//                        clusters.at(i).cluster_element.at(j).dist_to_robot = distance;
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
            
//            if (clusters.size() > 0) 
//            {
//                    for (int i = clusters.size(); i > 0; i--) 
//                    {                   
//                            for (int j = 0; j < clusters.size() - 1; j++) 
//                            {
////                                ROS_ERROR("Compare cluster: %d id: %d    ||    cluster: %d id: %d", j, clusters.at(j).cluster_element.front().id, j+1, clusters.at(j+1).cluster_element.front().id);
//                                if(clusters.at(j).cluster_element.size() > 0 && clusters.at(j+1).cluster_element.size() > 0)
//                                {
//                                    
//                                    double x = clusters.at(j).cluster_element.front().x_coordinate - pose_x;
//                                    double y = clusters.at(j).cluster_element.front().y_coordinate - pose_y;
//                                    double x_next = clusters.at(j+1).cluster_element.front().x_coordinate - pose_x;
//                                    double y_next = clusters.at(j+1).cluster_element.front().y_coordinate - pose_y;
//                                                                       
//                                    double euclidean_distance = x * x + y * y;
//                                    double euclidean_distance_next = x_next * x_next + y_next * y_next;
//                                    
////                                  ROS_INFO("Cluster %d dist: %d     Cluster %d dist: %d",j,(int)euclidean_distance, j+1, (int)euclidean_distance_next);
//                                    clusters.at(j).cluster_element.front().dist_to_robot = euclidean_distance;
//                                    clusters.at(j+1).cluster_element.front().dist_to_robot = euclidean_distance_next;
//                                    
////                                    int distance = check_trajectory_plan(clusters.at(j).cluster_element.front().x_coordinate, clusters.at(j).cluster_element.front().y_coordinate);
////                                    int distance_next = check_trajectory_plan(clusters.at(j+1).cluster_element.front().x_coordinate,clusters.at(j+1).cluster_element.front().y_coordinate);
//                                    
////                                    if(distance > distance_next)
//                                    if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next)) 
//                                    {
//                                            cluster_t temp = clusters.at(j+1);                                            
//                                            clusters.at(j+1) = clusters.at(j);                                          
//                                            clusters.at(j) = temp;                  
//                                    }
//                                }
//                            }
//                    }
//            }else 
//            {
//                    ROS_INFO("Sorting not possible, no frontiers available!!!");
//            }
        }
        else if(strategy == 6)
        {
            ROS_INFO("sort(6)");
            tf::Stamped < tf::Pose > robotPose;
            if (!costmap_ros_->getRobotPose(robotPose))
            {
                    ROS_ERROR("Failed to get RobotPose");
            }
            double pose_x = robotPose.getOrigin().getX();
            double pose_y = robotPose.getOrigin().getY();
            
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
        
        ROS_INFO("Done sorting");
    
//	for (int i = frontiers.size() -1 ; i >= 0; i--) {
//		ROS_ERROR("Frontier with ID: %d   at x: %f     y: %f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);
//	}
}

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
            if(frontiers.at(i).detected_by_robot == 1)
            {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }  
            if(frontiers.at(i).detected_by_robot == 2)
            {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }  
            
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

bool ExplorationPlanner::isSameFrontier(int frontier_point1,
		int frontier_point2) {
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
		 * The point is a either a obstacle or a point with not enough
		 * information about
		 * Therefore, check if the point is surrounded by other NO_INFORMATION
		 * points. Then further space to explore is found ---> frontier
		 */
		//int Neighbours = 0;
		//int points[((int)pow(8,Neighbours+1))+8]; // NEIGHBOURS points, each containing 8 adjacent points
		int no_inf_count = 0;
		int inscribed_count = 0;
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
;                           return(point);
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

