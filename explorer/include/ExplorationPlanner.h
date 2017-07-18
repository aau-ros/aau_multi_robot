#ifndef PLANNER_H___
#define PLANNER_H___

//#include <navfn/navfn_ros.h>
#include <global_planner/planner_core.h>
#include "ros/ros.h"
#include "hungarian.h"
#include <ros/console.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <adhoc_communication/ExpFrontier.h>
#include <adhoc_communication/ExpCluster.h>
#include <adhoc_communication/ExpAuction.h>
#include <adhoc_communication/MmPoint.h>
#include <adhoc_communication/MmListOfPoints.h>
#include <adhoc_communication/EmDockingStation.h>
#include <adhoc_communication/EmRobot.h>
#include <map_merger/TransformPoint.h>
#include <math.h>
#include <std_msgs/Int32.h>
#include <stdlib.h>
#include <limits>
#include <iostream>
#include <boost/filesystem.hpp>
#include <ros/console.h>
#include <boost/lexical_cast.hpp>
#include <move_base/MoveBaseConfig.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/observation.h>
#include <costmap_2d/observation_buffer.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <navfn/navfn_ros.h>
#include <boost/filesystem.hpp>
#include <map_merger/LogMaps.h>
//#include "energy_mgmt/battery_state.h"
#include "explorer/battery_state.h"
#include "explorer/Speed.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include "nav_msgs/GetMap.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <adhoc_communication/EmDockingStation.h>
#include <fake_network/RobotPositionSrv.h>
#include <explorer/Distance.h>
#include <explorer/DistanceFromRobot.h>
#include <adhoc_communication/EmRobot.h>
#include <adhoc_communication/MmListOfPoints.h>
#include <adhoc_communication/MmPoint.h>
//#include <robot_state/GetRobotState.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

namespace explorationPlanner
{
    struct frontier_t
    {
        int id;
        int detected_by_robot;
        std::string detected_by_robot_str;
        double robot_home_x;
        double robot_home_y;
        double x_coordinate; //world coordinate
        double y_coordinate; //world coordinate
        double smart_x_coordinate; //world coordinate
        double smart_y_coordinate; //world coordinate
        bool smart_goal_set;
        int distance_to_robot;
        int dist_to_robot;
        double my_distance_to_robot;
        double my_distance_to_optimal_ds;
        float cost;
        int cluster_id; //the id of the cluster in which the frontiers has been inserted; -1 if it is in no cluster
        ros::Time timestamp;
        std::vector<double> list_distance_from_ds;
    } frontier, unreachable_frontier;


    class ExplorationPlanner
    {
        public:

            struct responded_t
            {
                int robot_number;
                std::string robot_str;
                int auction_number;
            };

            struct cluster_t
            {
                std::vector<frontier_t> cluster_element;
                int id;
                int unreachable_frontier_count;
            } cluster;

            struct transform_point_t
            {
                int id;
                std::string robot_str;
            };

            struct requested_cluster_t
            {
                std::vector<transform_point_t> requested_ids;
                int own_cluster_id;
            };

            struct auction_pair_t
            {
                int bid_value;
                int cluster_id;
            };

            struct auction_element_t
            {
                int robot_id;
                std::string detected_by_robot_str;
                std::vector<auction_pair_t> auction_element;
            };

            struct compare_pair_t
            {
                int cluster_id;
                int identical_ids;
            };
            


            boost::mutex store_frontier_mutex, store_visited_mutex, store_negotiation_mutex, mutex_optimal_ds, mutex_erase_frontier;
            boost::mutex publish_subscribe_mutex, callback_mutex, negotiation_mutex, negotiation_callback_mutex;
            boost::mutex position_mutex, auction_mutex;
            boost::mutex cluster_mutex;
            boost::thread thr_auction_status, thr_auction_pid;

            ros::ServiceClient client;

            ros::Publisher pub_frontiers, pub_visited_frontiers, pub_negotion, pub_negotion_first, pub_Point;
            ros::Publisher pub_frontiers_points, pub_visited_frontiers_points;
            ros::Publisher pub_auctioning, pub_auctioning_status;
            ros::Subscriber sub_frontiers, sub_visited_frontiers, sub_negotioation, sub_negotioation_first, sub_negotiation_2, sub_reply_negotiation;
            ros::Subscriber sub_control;
            ros::Subscriber sub_auctioning, sub_auctioning_status;
            ros::Subscriber sub_position, sub_robot;

            ros::Publisher pub_auctioning_first;
            ros::Subscriber sub_auctioning_first;

            ros::Publisher pub_clusters, pub_cluster_grid_0, pub_cluster_grid_1, pub_cluster_grid_2, pub_cluster_grid_3, pub_cluster_grid_4, pub_cluster_grid_5, pub_cluster_grid_6, pub_cluster_grid_7, pub_cluster_grid_8, pub_cluster_grid_9,
            pub_cluster_grid_10,pub_cluster_grid_11,pub_cluster_grid_12,pub_cluster_grid_13,pub_cluster_grid_14,pub_cluster_grid_15,pub_cluster_grid_16,pub_cluster_grid_17,pub_cluster_grid_18,pub_cluster_grid_19;

            ros::NodeHandle nh_frontier, nh_visited_frontier;
            ros::NodeHandle nh_Point, nh_visited_Point, nh_frontiers_points;
            ros::NodeHandle nh_transform, nh_negotiation, nh_negotiation_first;
            ros::NodeHandle nh_auction, nh_auction_status;
            ros::NodeHandle nh_cluster, nh_cluster_grid;
            ros::NodeHandle nh_position, nh_robot;
            ros::NodeHandle nh_control;
            ros::NodeHandle *nh_service;
            ros::NodeHandle nh;
            ros::NodeHandle nh_auction_first;

            ros::MultiThreadedSpinner spinner;

            std::vector<int> allFrontiers;

            std::vector<auction_element_t> auction;
            std::vector<frontier_t> frontiers;
            std::vector<frontier_t> visited_frontiers;
            std::vector<frontier_t> unreachable_frontiers;
            std::vector<frontier_t> close_frontiers;
            std::vector<frontier_t> far_frontiers;

            std::vector<frontier_t> seen_frontier_list;
            std::vector<frontier_t> negotiation_list, my_negotiation_list;

            std::vector <responded_t> robots_already_responded;

            std::vector <double> goal_buffer_x;
            std::vector <double> goal_buffer_y;

            std::vector<double> last_goal_position_x;
            std::vector<double> last_goal_position_y;

            std::vector<cluster_t> clusters;
            std::vector<adhoc_communication::ExpCluster> unrecognized_occupied_clusters;

            std::vector<transform_point_t> transformedPointsFromOtherRobot_frontiers, transformedPointsFromOtherRobot_visited_frontiers;

            std::vector<int> already_used_ids, id_previously_detected;

            adhoc_communication::MmListOfPoints other_robots_positions;

            tf::TransformListener listener;
            tf::StampedTransform transform;
            tf::Stamped < tf::Pose > robotPose;

            map_merger::TransformPoint service_message;

            //F
            //navfn::NavfnROS nav;
            global_planner::GlobalPlanner nav;

            ros::ServiceClient ssendFrontier, ssendFrontier_2, ssendAuction;

            std::string trajectory_strategy;
            bool first_run, first_negotiation_run;
            bool start_thr_auction;

            int number_of_auction_runs;
            int cluster_id, cluster_cells_seq_number;
            int number_of_completed_auctions, number_of_uncompleted_auctions;
            bool initialized_planner;
            bool auction_is_running, auction_start, auction_finished;
            bool robot_prefix_empty_param;
            int number_of_auction_bids_received, number_of_robots;
            double other_robots_position_x, other_robots_position_y;
            int auction_id_number;
            int auction_pid;
            int frontier_seq_number;
            double exploration_travel_path_global, exploration_travel_path_global_meters;
            double x_value,y_value,w_value,x_distance,y_distance,robot_pose_x,robot_pose_y;
            int goal_buffer_counter,goal_buffer_length,random_number, random_value;
            int inflated,free,lethal,unknown;
            double free_space;
            int frontier_point_message;
            int frontier_id_count, visited_frontier_id_count,unreachable_frontier_id_count;
            int goal_point_simulated_message, start_point_simulated_message;
            bool simulation;
            int threshold_free, threshold_inflated, threshold_lethal;
            int robot_name;
            double robot_home_x, robot_home_y;
            std::string move_base_frame, robots_in_simulation;
            std::string robot_str;
            std::vector<std::string> new_robots;
            int next_auction_position_x, next_auction_position_y;

            ExplorationPlanner(int robot_id, bool robot_prefix_empty, std::string robot_name_parameter);
            void printFrontiers();
            bool respondToAuction(std::vector<requested_cluster_t> requested_cluster_ids, int auction_id_number);
            bool clusterIdToElementIds(int cluster_id, std::vector<transform_point_t>* occupied_ids);
            bool initialize_auctioning(std::vector<double> *final_goal);
            bool auctioning(std::vector<double> *final_goal, std::vector<int> *clusters_available_in_pool, std::vector<std::string> *robot_str_name);
            bool selectClusterBasedOnAuction(std::vector<double> *goal, std::vector<int> *cluster_in_use_already_count, std::vector<std::string> *robot_str_name_to_return);
            bool InitSelectClusterBasedOnAuction(std::vector<double> *goal);
            int calculateAuctionBID(int cluster_number, std::string strategy);
            std::string lookupRobotName(int robot_name_int);
            void trajectory_plan_10_frontiers();
            void trajectory_plan_store(double x, double y);
            int reserve_trajectory_plan(double x, double y);
            int trajectory_plan(double x, double y);
            double trajectory_plan_meters(double x, double y);
            int reserve_trajectory_plan(double start_x, double start_y, double target_x, double target_y);
            int trajectory_plan(double start_x, double start_y, double target_x, double target_y);
            double trajectory_plan_print(double start_x, double start_y, double target_x, double target_y);
            double trajectory_plan_meters(double start_x, double start_y, double target_x, double target_y);
            void Callbacks();
            int checkClustersID(adhoc_communication::ExpCluster cluster_to_check);
            bool determine_goal(int strategy, std::vector<double> *final_goal, int count, int actual_cluster_id, std::vector<std::string> *robot_str_name);
            void sort(int strategy);
            void sort_reserve(int strategy);
            void sort_cost_with_approach(bool energy_above_th, int w1, int w2, int w3, int w4);
            void sort_cost(bool energy_above_th, int w1, int w2, int w3, int w4);
            //void smart_sort_cost(bool energy_above_th, int w1, int w2, int w3, int w4);
            void my_sort_cost_2(bool energy_above_th, int w1, int w2, int w3, int w4);
            void my_sort_cost_3(bool energy_above_th, int w1, int w2, int w3, int w4);
            void my_sort_cost_0(bool energy_above_th, int w1, int w2, int w3, int w4);
            void sort_cost_1(bool energy_above_th, int w1, int w2, int w3, int w4);
            void sort_cost_reserve(bool energy_above_th, int w1, int w2, int w3, int w4);
            void simulate();
            void visualize_Frontiers();
            void visualize_visited_Frontiers();
            void visualizeClustersConsole();
            void clear_Visualized_Cluster_Cells(std::vector<int> ids);
            void initialize_planner(std::string name, costmap_2d::Costmap2DROS *costmap, costmap_2d::Costmap2DROS *costmap_global);
            void findFrontiers();
            bool my_determine_goal_staying_alive(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id, bool energy_above_th, int w1, int w2, int w3, int w4);
            bool determine_goal_staying_alive(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id);
            //bool determine_goal_staying_alive_2(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id);
            //bool determine_goal_staying_alive_2_reserve(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id);
            bool check_efficiency_of_goal(double x, double y);
            void clearVisitedAndSeenFrontiersFromClusters();
            void clearSeenFrontiers(costmap_2d::Costmap2DROS *global_costmap);
            void clearVisitedFrontiers();
            void clearUnreachableFrontiers();
            bool storeFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id);
            bool storeVisitedFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id);
            bool storeUnreachableFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id);
            bool removeStoredFrontier(int id, std::string detected_by_robot_str);
            bool removeVisitedFrontier(int id, std::string detected_by_robot_str);
            bool removeUnreachableFrontier(int id, std::string detected_by_robot_str);
            bool publish_frontier_list();
            bool publish_visited_frontier_list();
            bool publish_negotiation_list(frontier_t negotiation_frontier, int cluster);
            bool subscribe_negotiation_list();
            bool subscribe_frontier_list();
            bool subscribe_visited_frontier_list();
            void negotiationCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg);
            void visited_frontierCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg);
            void frontierCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg);
            void auctionCallback(const adhoc_communication::ExpAuction::ConstPtr& msg);
            void positionCallback(const adhoc_communication::MmListOfPoints::ConstPtr& msg);
            bool smartGoalBackoff(double x, double y, costmap_2d::Costmap2DROS *global_costmap, std::vector<double> *new_goal);
            void setRobotConfig(int name, double robot_home_position_x, double robot_home_position_y, std::string frame);
            bool negotiate_Frontier(double x, double y, int detected_by, int id, int cluster);
            void clusterFrontiers();
            void visualize_Clusters();
            void visualize_Cluster_Cells();

            bool transformToOwnCoordinates_frontiers();
            bool transformToOwnCoordinates_visited_frontiers();

            bool sendToMulticast(std::string multi_cast_group, adhoc_communication::ExpFrontier frontier_to_send, std::string topic);
            bool sendToMulticastAuction(std::string multi_cast_group, adhoc_communication::ExpAuction auction_to_send, std::string topic);


            //F
            bool existFrontiers();
            bool existFrontiersReachableWithFullBattery(float max_available_distance, bool *error);
            double distance_from_robot(double x, double y);
            double distance(double x1, double y1, double x2, double y2);
            bool getRobotPose(tf::Stamped < tf::Pose > &robotPose);
            //bool get_robot_position(double *x, double *y);
            bool reachable_target(double x, double y);
			unsigned char getCost(costmap_2d::Costmap2DROS *costmap, unsigned int cell_x, unsigned int cell_y);
            bool my_negotiate();
            void my_replyToNegotiationCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg);
            void my_negotiationCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg);
            bool my_sendToMulticast(std::string multi_cast_group, adhoc_communication::ExpFrontier frontier_list, std::string topic);
            std::string robot_name_str;
            double w1,w2,w3,w4;
            std::vector<frontier_t> frontiers_under_auction;
            frontier_t *my_selected_frontier;
            bool winner_of_auction;
            float my_bid;
            bool my_energy_above_th;
            void clean_frontiers_under_auction();
            int my_error_counter;
            bool recomputeGoal();
            int optimal_ds_id, next_optimal_ds_id;
            float optimal_ds_x, optimal_ds_y;
            float next_optimal_ds_x, next_optimal_ds_y;
            ros::Subscriber sub_new_optimal_ds;
            void new_optimal_ds_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg);
            ros::ServiceClient sc_distance_frontier_robot;
            void new_ds_on_graph_callback(const adhoc_communication::EmDockingStation msg);
            ros::Subscriber sub_new_ds_on_graph;
            float new_optimal_ds(int id, float new_optimal_ds_x, float new_optimal_ds_y);
            void set_auction_timeout(int timeout);
            float sort_time, selection_time;
            unsigned int number_of_frontiers;
            bool frontier_selected;
            bool my2_determine_goal_staying_alive(int mode, int strategy, double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id, bool energy_above_th, int w1, int w2, int w3, int w4);
            bool home_is_reachable(double available_distance);
            bool existReachableFrontiersWithDsGraphNavigation(double available_distance, bool *error);
            bool compute_and_publish_ds_path(double maximum_available_distance, int *result);
            void logRemainingFrontiers(std::string csv_file);
            bool discovered_new_frontier;
            bool updateRobotPose();
            void updateOptimalDs();
            void updateDistances(double max_available_distance);
//            std::vector<frontier_t>::const_iterator update_distances_index;

            // Debugging
            void setTestMode(bool test_mode);
            void pushFrontier(frontier_t frontier);
            std::vector<frontier_t> getFrontierList();
            std::vector<std::vector<double> > distance_list;
            void addDistance(double x1, double y1, double x2, double y2, double distance);
            void setRobotPosition(double x, double y);
            void setOptimalDs(unsigned int id, double x, double y);
            double _f1, _f2, _f3, _f4, _f5, _f6, _f7;
            bool _b1;
            int _i1, _i2, _i3;
            double getOptimalDsX();

        private:
            bool auction_running;

            void home_position_(const geometry_msgs::PointStamped::ConstPtr& msg);
            void clearFrontiers();
            int isFrontier(int point);
            int isFrontier_2(int point);
            bool isFree(int point);
            inline bool isValid(int point);
            inline void getAdjacentPoints(int point, int points[]);
            std::vector<int> getMapNeighbours(unsigned int point_x, unsigned int point_y, int distance);
            bool isFrontierReached(int point);
            int backoff (int point);
            double getYawToUnknown(int point);
            bool isSameFrontier(int frontier_point1, int frontier_point2);
            void setupMapData();
            void deleteMapData();
            void resetMaps();
            bool countCostMapBlocks(int point);

            int left(int point);
            int upleft(int point);
            int up(int point);
            int upright(int point);
            int right(int point);
            int downright(int point);
            int down(int point);
            int downleft(int point);

            enum LastMode
            {
                FRONTIER_EXPLORE,
                INNER_EXPLORE
            } last_mode_;

            costmap_2d::Costmap2DROS *costmap_ros_;
            costmap_2d::Costmap2DROS *costmap_global_ros_;
            costmap_2d::Costmap2D costmap_;
            costmap_2d::Costmap2D costmap_global_;

            ros::Publisher visualization_pub_;

            const unsigned char* occupancy_grid_array_;
            unsigned int* exploration_trans_array_;
            unsigned int* obstacle_trans_array_;
            int* frontier_map_array_;
            int home_position_x_,home_position_y_;
            double robot_last_x, robot_last_y;

            bool* is_goal_array_;
            bool initialized_;
            int previous_goal_;

            std::string name;
            unsigned int map_width_;
            unsigned int map_height_;
            unsigned int num_map_cells_;

            // Parameters
            bool p_plan_in_unknown_;
            bool p_use_inflated_obs_;
            bool p_security_constant_;	//FIXME
            int p_goal_angle_penalty_;
            int p_min_obstacle_dist_;
            int p_min_frontier_size_;
            double p_alpha_;
            double p_dist_for_goal_reached_;
            double p_same_frontier_dist_;
            
            std::vector<frontier_t> sorted_frontiers;
            int num_ds;
            void ds_count_callback(const std_msgs::Int32 msg);
            ros::Subscriber sub_ds_count;
            struct ds_t
            {
                int id;
                double x;
                double y;
                bool vacant;
            };
            std::vector<ds_t> ds_list;
            std::vector<std::vector<float> > ds_graph;
            bool recompute_ds_graph;
            double euclidean_distance(float x1, float y1, float x2, float y2);
            float target_ds_x, target_ds_y;
            bool target_ds_set;
            
            void acquire_mutex(boost::mutex *mutex, std::string function_name);
            void release_mutex(boost::mutex *mutex, std::string function_name);
            bool storeFrontier_without_locking(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id);
            int retrying_searching_frontiers;
            bool received_scan;
            void add_to_sorted_fontiers_list_if_convinient(frontier_t frontier);
            int auction_timeout;
            //std::vector<frontier_t> my_unreachable_frontiers;
            bool my_check_efficiency_of_goal(double available_distance, frontier_t * frontier);
            bool selected_frontier;
            void my_select_4(double available_distance, bool energy_above_th, int w1, int w2, int w3, int w4, std::vector<double> *final_goal, std::vector<std::string> *robot_str_name);
            int errors;
            double robot_x, robot_y;
            double frontier_cost(frontier_t frontier);
            double frontier_cost_0(frontier_t frontier);
            double frontier_cost_1(frontier_t frontier);
            double available_distance;
            std::vector<frontier_t> last_robot_auctioned_frontier_list;
            void robot_next_goal_callback(const adhoc_communication::ExpFrontier::ConstPtr& msg);
            ros::Subscriber sub_next_goal;
            bool my_quick_check_efficiency_of_goal(double available_distance, frontier_t * frontier);
            boost::mutex costmap_mutex;
            double fallback_distance_computation(double end_x, double end_y);
            double fallback_distance_computation(double start_x, double start_y, double end_x, double end_y);
            bool optimal_ds_set;
            ros::Subscriber sub_this_robot;
            bool received_robot_info;
            double robot_home_world_x, robot_home_world_y;
            void this_robot_callback(const adhoc_communication::EmRobot::ConstPtr &msg);
            ros::Time time_start;
            ros::Publisher publish_goal_ds_for_path_navigation;
            std::fstream fs_csv;
            double simplifiedDistanceFromDs(unsigned int ds_index, unsigned int frontier_index);
            bool erased;
            ds_t *min_ds_for_path_traversal;
            
            // Debugging
            bool test_mode;
            
    };
}

#endif

