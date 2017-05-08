#include "ros/ros.h"
#include "ExplorationPlanner.h"
#include <ros/console.h>
#include <ExplorationPlanner.h>
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
#include "energy_mgmt/battery_state.h"
#include "explorer/Speed.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include "nav_msgs/GetMap.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <adhoc_communication/EmDockingStation.h>
#include <explorer/RobotPosition.h>
#include <explorer/Distance.h>
#include <explorer/DistanceFromRobot.h>
#include <adhoc_communication/EmRobot.h>
#include <adhoc_communication/MmListOfPoints.h>
#include <adhoc_communication/MmPoint.h>
//#include <robot_state/GetRobotState.h>

//#define PROFILE

#ifdef PROFILE
#include "google/profiler.h"
#include "google/heap-profiler.h"
#endif

#define OPERATE_ON_GLOBAL_MAP true
#define OPERATE_WITH_GOAL_BACKOFF true
//#define EXIT_COUNTDOWN 5
#define EXIT_COUNTDOWN 50  // F
//#define STUCK_COUNTDOWN 10
#define STUCK_COUNTDOWN 1000  // F

#define SAFETY_COEFF 0.003
//#define SAFETY_COEFF 10.005
#define INCR 1.7
#define OPP_ONLY_TWO_DS false
#define SAFETY_COEFF_2 0.8
#define IMM_CHARGE 0
#define DEBUG false

boost::mutex costmap_mutex;

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        ros::Duration(1.0).sleep();
}

class Explorer
{
  public:
    // TODO(minor) move in better place
    bool ready, moving_along_path, explorer_ready;
    int my_counter, ds_path_counter, ds_path_size;
    ros::Publisher pub_robot, pub_wait;
    ros::Subscriber sub_wait;
    int path[2][2];
    std::vector<adhoc_communication::MmPoint> complex_path;
    ros::ServiceServer ss_robot_pose, ss_distance_from_robot, ss_distance, ss_reachable_target;
    ros::ServiceClient sc_get_robot_state;
    bool created;
    float queue_distance, max_av_distance;
    float safety_coeff, min_distance_queue_ds;
    float stored_robot_x, stored_robot_y;

    /*******************
     * CLASS FUNCTIONS *
     *******************/
    Explorer(tf::TransformListener &tf)  // TODO(minor) put comments (until CREATE LOG PATH)
        : counter(0),
          rotation_counter(0),
          nh("~"),
          number_of_robots(1),
          accessing_cluster(0),
          cluster_element_size(0),
          cluster_flag(false),
          cluster_element(-1),
          cluster_initialize_flag(false),
          global_iterations(0),
          global_iterations_counter(0),
          counter_waiting_for_clusters(0),
          global_costmap_iteration(0),
          robot_prefix_empty(false),
          robot_id(0),
          battery_charge(100),
          recharge_cycles(0),
          battery_charge_temp(100),
          energy_consumption(0),
          available_distance(0),
          // robot_state(fully_charged),
          charge_time(0),
          pose_x(0),
          pose_y(0),
          pose_angle(0),
          prev_pose_x(0),
          prev_pose_y(0),
          prev_pose_angle(0)
    {
        // available_distance = 100; //F
        // F
        test = true;
        vacant_ds = true;
        ready = false;
        moving_along_path = false;
        created = false;
        exploration = NULL;
        explorer_ready = false;
        max_av_distance = 0;
        available_distance = -1;

        /* Initial robot state */
        robot_state = fully_charged;  // TODO(minor) what if instead it is not fully charged?
        robot_state_next = current_state;

        /* Robot state publishers */
        pub_check_vacancy = nh.advertise<std_msgs::Empty>("check_vacancy", 1);  // to publish vacancy check requests

        /* Robot state subscribers */

        sub_check_vacancy =
            nh.subscribe("adhoc_communication/reply_for_vacancy", 1, &Explorer::reply_for_vacancy_callback,
                         this);  // to receive replies for vacancy checks

        // TODO(minor) improve this
        sub_lost_own_auction = nh.subscribe("lost_own_auction", 1, &Explorer::lost_own_auction_callback,
                                            this);  // to know when a robot lost its own auction
        sub_won_auction =
            nh.subscribe("won_auction", 1, &Explorer::won_callback, this);  // to know when a robot won an auction
        sub_lost_other_robot_auction = nh.subscribe("lost_other_robot_auction", 1, &Explorer::lost_other_robot_callback,
                                                    this);  // to know when a robot lost another robot auction

        pub_robot = nh.advertise<adhoc_communication::EmRobot>("robot", 1);  // to publish robot state updates
        
        sub_wait = nh.subscribe("are_you_ready", 10, &Explorer::wait_for_explorer_callback, this);
        pub_wait = nh.advertise<std_msgs::Empty>("im_ready", 10);

        ros::NodeHandle n;
        //sc_get_robot_state = n.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");

        /* Load parameters */
        nh.param("frontier_selection", frontier_selection, 1);
        nh.param("local_costmap/width", costmap_width, 0);
        nh.param<double>("local_costmap/resolution", costmap_resolution, 0);
        nh.param("number_unreachable_for_cluster", number_unreachable_frontiers_for_cluster, 3);
        nh.param("recharging", recharging, false);
        nh.param("w1", w1, 1);
        nh.param("w2", w2, 0);
        nh.param("w3", w3, 0);
        nh.param("w4", w4, 0);
        nh.param<float>("safety_coeff", safety_coeff, 0.9);
        nh.param<float>("queue_distance", queue_distance, 7.0);
        nh.param<float>("min_distance_queue_ds", min_distance_queue_ds, 3.0);
        //ROS_ERROR("%f", queue_distance);
        nh.param<std::string>("move_base_frame", move_base_frame, "map");
        nh.param<int>("wait_for_planner_result", waitForResult, 3);

        ROS_INFO("Costmap width: %d", costmap_width);
        ROS_INFO("Frontier selection is set to: %d", frontier_selection);

        srand((unsigned)time(0));  // TODO(minor) ???

        // determine host name
        nh.param<std::string>("robot_prefix", robot_prefix, "");
        ROS_INFO("robot prefix: \"%s\"", robot_prefix.c_str());

        /* Create map_merger service */
        std::string service = robot_prefix + std::string("/map_merger/logOutput");
        mm_log_client = nh.serviceClient<map_merger::LogMaps>(service.c_str());

        // TODO(minor) hmm
        if (robot_prefix.empty())
        {
            /* The robot prefix is empty, i.e., we are in a real experiment */
            robot_prefix_empty = true;
            ROS_INFO("NO SIMULATION!");

            char hostname_c[1024];
            hostname_c[1023] = '\0';
            gethostname(hostname_c, 1023);
            robot_name = std::string(hostname_c);

            /*
             * THIS IS REQUIRED TO PERFORM COORDINATED EXPLORATION
             *
             * Assign numbers to robot host names in order to make auctioning and frontier selection UNIQUE !!!!!
             * To use explorer node on a real robot system, add your robot names here and at
             *ExplorationPlanner::lookupRobotName function ...
             */
            std::string bob = "bob";
            std::string marley = "marley";
            std::string turtlebot = "turtlebot";
            std::string joy = "joy";
            std::string hans = "hans";

            if (robot_name.compare(turtlebot) == 0)
                robot_id = 0;
            if (robot_name.compare(joy) == 0)
                robot_id = 1;
            if (robot_name.compare(marley) == 0)
                robot_id = 2;
            if (robot_name.compare(bob) == 0)
                robot_id = 3;
            if (robot_name.compare(hans) == 0)
                robot_id = 4;

            ROS_INFO("Robot name: %s; robot_id: %d", robot_name.c_str(), robot_id);
        }
        else
        {
            // be caureful: robot_name is used by some ExplorerPlanner functions, so do not touch it...
            robot_name = robot_prefix;  // TODO(minor) handle better

            ROS_INFO("Move_base_frame: %s", move_base_frame.c_str());
            robot_id = atoi(move_base_frame.substr(7, 1).c_str());

            ROS_INFO("Robot name: %s    robot_id: %d", robot_name.c_str(), robot_id);
        }

        /*
         * CREATE LOG PATH
         *
         * Following code enables to write the output to a file
         * which is localized at the log_path
         */
        initLogPath();
        csv_file = log_path + std::string("periodical.log");
        csv_state_file = log_path + std::string("robot_state.log");
        log_file = log_path + std::string("exploration.log");
        
        fs_csv_state.open(csv_state_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        fs_csv_state << "#time,robot_state" << std::endl;
        fs_csv_state.close();


        ROS_INFO("*********************************************");
        ROS_INFO("******* Initializing Simple Navigation ******");
        ROS_INFO("                                             ");

        ROS_DEBUG("Creating global costmap ...");
        costmap2d_global = new costmap_2d::Costmap2DROS("global_costmap", tf);
        ROS_DEBUG("Global costmap created ... now performing costmap -> pause");
        costmap2d_global->pause();
        ROS_DEBUG("Pausing performed");

        ROS_DEBUG("                                             ");

        if (OPERATE_ON_GLOBAL_MAP == true)
        {
            costmap2d_local_size = new costmap_2d::Costmap2DROS("local_costmap", tf);
            costmap2d_local_size->pause();
            ROS_DEBUG("Starting Global costmap ...");
            costmap2d_global->start();
            costmap2d_local_size->start();

            costmap2d_local = costmap2d_global;
        }
        else
        {
            ROS_INFO("Creating local costmap ...");
            costmap2d_local = new costmap_2d::Costmap2DROS("local_costmap", tf);
            ROS_INFO("Local costmap created ... now performing costmap -> pause");
            costmap2d_local->pause();
            ROS_INFO("Pausing performed");
            ROS_INFO("Cost maps created");

            ROS_INFO("                                             ");

            ROS_INFO("Starting Global costmap ...");
            costmap2d_global->start();
            ROS_INFO("Starting Local costmap ... ");
            costmap2d_local->start();
            ROS_INFO("BOTH COSTMAPS STARTED AND RUNNING ...");
        }

        ROS_INFO("---------------- COSTMAP DONE ---------------");

        /*
         * Set the first goal as PointStamped message to visualize in RVIZ.
         * RVIZ requires a minimal history length of 1, which means that at least
         * one entry has to be buffered, before the first Goal is able to be
         * visualized. Therefore set the "first" goal to the point of origin
         * (home position).
         */

        if (!costmap2d_local->getRobotPose(robotPose))
        {
            ROS_ERROR("Failed to get RobotPose");
        }
        visualize_goal_point(robotPose.getOrigin().getX(), robotPose.getOrigin().getY());
        
        //ROS_ERROR("origin: (%f, %f)", robotPose.getOrigin().getX(), robotPose.getOrigin().getY());

        // transmit three times, since rviz need at least 1 to buffer before
        // visualizing the point
        for (int i = 0; i <= 2; i++)
        {
            visualize_home_point();
        }

        ROS_INFO("---------- SET HOME/GOAL POINT DONE ---------");

        // instantiate the planner
        exploration = new explorationPlanner::ExplorationPlanner(robot_id, robot_prefix_empty, robot_name);

        /*
         * Define the first goal. This is required to have at least one entry
         * within the vector. Therefore set it to the home position.
         */

        robot_home_position_x = robotPose.getOrigin().getX();
        robot_home_position_y = robotPose.getOrigin().getY();

        exploration->next_auction_position_x = robotPose.getOrigin().getX();
        exploration->next_auction_position_y = robotPose.getOrigin().getY();

        exploration->storeVisitedFrontier(robot_home_position_x, robot_home_position_y, robot_id, robot_name, -1);
        exploration->storeFrontier(robot_home_position_x, robot_home_position_y, robot_id, robot_name, -1);

        exploration->setRobotConfig(robot_id, robot_home_position_x, robot_home_position_y, move_base_frame);

        ROS_INFO("                                             ");
        ROS_INFO("************* INITIALIZING DONE *************");

        /* Load strings in enum_string vector */
        // TODO(minor) currently, the strings must be inserted in teh same order of the enum, which is not very nice...
        enum_string.push_back("exploring");
        enum_string.push_back("going_charging");
        enum_string.push_back("charging");
        enum_string.push_back("finished");
        enum_string.push_back("fully_charged");
        enum_string.push_back("stuck");
        enum_string.push_back("in_queue");
        enum_string.push_back("auctioning");
        enum_string.push_back("auctioning_2");
        enum_string.push_back("going_in_queue");
        enum_string.push_back("going_checking_vacancy");
        enum_string.push_back("checking_vacancy");
        enum_string.push_back("moving_to_frontier_before_going_charging");
        enum_string.push_back("moving_to_frontier");
        enum_string.push_back("leaving_ds");
    }

    void explore()  // TODO(minor) comments
    {
        //ROS_ERROR("STARTING explore()");
        // TODO(minor) put to sleep also the other nodes
        /*
         * Sleep is required to get the actual a
         * costmap updated with obstacle and inflated
         * obstacle information. This is rquired for the
         * first time explore() is called.
         */
        ROS_DEBUG("Sleeping 5s for costmaps to be updated.");
        geometry_msgs::Twist twi;
        // should be a parameter!! only for testing on Wed/17/9/14 //TODO(minor)
        ros::Publisher twi_publisher = nh.advertise<geometry_msgs::Twist>("/Rosaria/cmd_vel", 1);
        twi.angular.z = 0.75;
        twi_publisher.publish(twi);
        ros::Duration(5.0).sleep();
        twi_publisher.publish(twi);

        /* Set countdowns */
        int exit_countdown = EXIT_COUNTDOWN;
        int charge_countdown = EXIT_COUNTDOWN; // TODO(minor) hmm...

        /* Start taking the time during exploration */
        time_start = ros::Time::now();

        ros::NodeHandle nh;

        ros::Subscriber sub, sub2, sub3, pose_sub, sub_finish;

        ros::Subscriber my_sub =
            nh.subscribe("charging_completed", 1, &Explorer::battery_charging_completed_callback, this);

        ros::Subscriber sub_new_target_ds = nh.subscribe("new_target_docking_station_selected", 1,
                                                         &Explorer::new_target_docking_station_selected_callback, this);

        ros::Subscriber sub_moving_along_path =
            nh.subscribe("moving_along_path", 1, &Explorer::moving_along_path_callback, this);
            
        ros::Publisher pub_next_ds = nh.advertise<std_msgs::Empty>("next_ds", 1);

        ros::Publisher pub_occupied_ds = nh.advertise<std_msgs::Empty>("occupied_ds", 1);

        /* Subscribe to battery management topic */
        sub = nh.subscribe("battery_state", 1, &Explorer::bat_callback, this);

        /* Subscribe to robot pose to check if robot is stuck */
        pose_sub = nh.subscribe("amcl_pose", 1, &Explorer::poseCallback, this);
        
        sub_finish = nh.subscribe("explorer/finish", 1, &Explorer::finish_callback, this);

        ROS_INFO("STARTING EXPLORATION");

        /* Start main loop (it loops till the end of the exploration) */
        while (robot_state != finished)
        {
            /* Update robot state */
            update_robot_state();

            //ROS_ERROR("EXPLORING");  // TODO(minor) here???

            // TODO(minor) better while loops
            // do nothing while recharging
            if (robot_state == charging)
            {
                ROS_INFO("Waiting for battery to charge...");

                // NO! i cannot put to sleep, since I have to detect if the charging process is interrupted!!!!
                // if (charge_time > 0)
                //    ros::Duration(charge_time).sleep();
                // else
                //    ros::Duration(1).sleep();
                // ros::spinOnce();  // update charge_time

                ros::spinOnce();
                ros::Duration(1).sleep();
                update_robot_state();

                continue;
            }

            /**************************
             * FRONTIER DETERMINATION *
             **************************/
            std::vector<double> final_goal;
            std::vector<double> backoffGoal;
            std::vector<std::string> robot_str;

            bool navigate_to_goal = false;
            bool negotiation;
            int count = 0;

            ROS_ERROR("****************** EXPLORE ******************");

            /*
            * Use mutex to lock the critical section (access to the costmap)
            * since rosspin tries to update the costmap continuously
            */
            //ROS_ERROR("COSTMAP STUFF");
            costmap_mutex.lock();
            ROS_ERROR("COSTMAP STUFF, lock aquired");

            exploration->transformToOwnCoordinates_frontiers();
            exploration->transformToOwnCoordinates_visited_frontiers();

            ROS_ERROR("initialize planner");
            exploration->initialize_planner("exploration planner", costmap2d_local, costmap2d_global);
            ROS_ERROR("planner initialized");
            exploration->findFrontiers();

            exploration->clearVisitedFrontiers();
            exploration->clearUnreachableFrontiers(); //should remove frontiers that are marked as unreachable from 'frontiers' vector
            exploration->clearSeenFrontiers(costmap2d_global);

            costmap_mutex.unlock();
            //ROS_ERROR("PUBLISHING FRONTIERS");
            
            tf::Stamped<tf::Pose> robotPose;
            
            if(!created) {
                while(!exploration->getRobotPose(robotPose)) {
                    ROS_ERROR("HERE");   
                    ros::Duration(3).sleep();
                } 
            
                ss_robot_pose = nh.advertiseService("explorer/robot_pose", &Explorer::robot_pose_callback, this);
                ss_distance_from_robot =
                    nh.advertiseService("explorer/distance_from_robot", &Explorer::distance_from_robot_callback, this);
                ss_reachable_target =
                    nh.advertiseService("explorer/reachable_target", &Explorer::reachable_target_callback, this);
                    
                ss_distance =
                    nh.advertiseService("explorer/distance", &Explorer::distance, this);
                    
                created = true;
            }
            
            

            /*
             * Sleep to ensure that frontiers are exchanged
             */
            ros::Duration(2).sleep();
            
            /*
            while(available_distance <= 0) {
                ROS_ERROR("Waiting battery state...");
                ros::spinOnce();
                ros::Duration(3).sleep();
            }
            */
            
            explorer_ready = true;
            
            
            
            store_current_position();
            
            // if (robot_state == exploring || robot_state == fully_charged)
            if (robot_state == exploring || robot_state == fully_charged || robot_state == leaving_ds)
            {
                if(available_distance <= 0) {
                    ROS_ERROR("waiting battery info");
                    continue;
                   }  
                    
                  
                    
                ROS_INFO("START FRONTIER SELECTION");
                    
                /**********************
                 * FRONTIER SELECTION *
                 **********************/

                /*********** EXPLORATION STRATEGIES ************
                * 0 ... Navigate to nearest frontier TRAVEL PATH
                * 1 ... Navigate using auctioning with cluster selection using NEAREST
                *       selection (Kuhn-Munkres)
                * 2 ... Navigate to furthest frontier
                * 3 ... Navigate to nearest frontier EUCLIDEAN DISTANCE
                * 4 ... Navigate to random Frontier
                * 5 ... Cluster frontiers, then navigate to nearest cluster using
                *       EUCLIDEAN DISTANCE (with and without negotiation)
                * 6 ... Cluster frontiers, then navigate to random cluster (with and
                *       without negotiation)
                *
                * ENERGY AWARE STRATEGIES:
                * 7 ... Navigate to frontier that satisfies energy efficient cost
                *       function with staying-alive path planning
                * 8 ... Navigate to leftmost frontier (Mei et al. 2006) with
                *       staying-alive path planning
                * 9 ... Just like strategy 0 but with staying-alive path planning
                */

                /*************** SORTING METHODS ***************
                * Choose which strategy to take.
                * 1 ... Sort the buffer from furthest to nearest frontier
                * 2 ... Sort the buffer from nearest to furthest frontier, normalized to
                *       the robots actual position (EUCLIDEAN DISTANCE)
                * 3 ... Sort the last 10 entries to shortest TRAVEL PATH
                * 4 ... Sort all cluster elements from nearest to furthest (EUCLIDEAN
                *       DISTANCE)
                * 5 ... (missing)
                * 6 ... (missing)
                * 7 ... Sort frontiers in sensor range clock wise (starting from left of
                *       robot) and sort the remaining frontiers from nearest to furthest
                *       (EUCLIDEAN DISTANCE)
                */

                /* Compute next frontier to be explored according to the selected
                 * exploration strategy */
                if (frontier_selection == 0)
                {
                    exploration->sort(2);
                    exploration->sort(3);

                    while (true)
                    {
                        goal_determined = exploration->determine_goal(2, &final_goal, count, 0, &robot_str);
                        ROS_DEBUG("Goal_determined: %d   counter: %d", goal_determined, count);
                        if (goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            // negotiation =
                            // exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
                            negotiation = true;
                            if (negotiation == true)
                            {
                                break;
                            }
                            count++;
                        }
                    }
                }
                else if (frontier_selection == 1)
                {
                    costmap_mutex.lock();
                    if (cluster_initialize_flag == true)
                    {
                        exploration->clearVisitedAndSeenFrontiersFromClusters();
                    }
                    else
                    {
                        /*
                        * This is only necessary for the first run
                        */

                        exploration->sort(2);
                        cluster_initialize_flag = true;
                    }

                    exploration->clusterFrontiers();
                    exploration->sort(4);
                    exploration->sort(5);

                    costmap_mutex.unlock();

                    exploration->visualizeClustersConsole();

                    while (true)
                    {
                        final_goal.clear();
                        robot_str.clear();
                        goal_determined = exploration->determine_goal(5, &final_goal, 0, cluster_element, &robot_str);

                        ROS_ERROR("Cluster element: %d", cluster_element);

                        int cluster_vector_position = -1;

                        if (cluster_element != -1)
                        {
                            if (exploration->clusters.size() > 0)
                            {
                                for (int i = 0; i < exploration->clusters.size(); i++)
                                {
                                    if (exploration->clusters.at(i).id == cluster_element)
                                    {
                                        if (exploration->clusters.at(i).cluster_element.size() > 0)
                                        {
                                            cluster_vector_position = i;
                                        }
                                        break;
                                    }
                                }
                            }
                        }
                        ROS_ERROR("Cluster vector position: %d", cluster_vector_position);

                        if (cluster_vector_position >= 0)
                        {
                            if (exploration->clusters.at(cluster_vector_position).unreachable_frontier_count >=
                                number_unreachable_frontiers_for_cluster)
                            {
                                goal_determined = false;
                                ROS_ERROR("Cluster inoperateable");
                            }
                            else
                            {
                                ROS_ERROR("Cluster operateable");
                            }
                        }

                        if (goal_determined == false)
                        {
                            ROS_INFO("No goal was determined, cluster is empty. Bid for "
                                     "another one");

                            final_goal.clear();
                            robot_str.clear();
                            clusters_available_in_pool.clear();

                            bool auctioning =
                                exploration->auctioning(&final_goal, &clusters_available_in_pool, &robot_str);
                            if (auctioning == true)
                            {
                                goal_determined = true;
                                cluster_element = final_goal.at(4);
                                counter_waiting_for_clusters = 0;
                                break;
                            }
                            else
                            {
                                if (exploration->clusters.size() > 0)  // clusters_available_in_pool.size() > 0)
                                {
                                    ROS_INFO("No cluster was selected but other robots are "
                                             "operating ... waiting for new clusters");
                                    counter_waiting_for_clusters++;
                                    break;
                                }
                                else
                                {
                                    /*
                                    * If NO goals are selected at all, iterate over the global
                                    * map to find some goals.
                                    */
                                    ROS_ERROR("No goals are available at all");
                                    cluster_element = -1;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            ROS_INFO("Still some goals in current cluster, finish this job first");
                            break;
                        }
                    }
                }
                else if (frontier_selection == 2)
                {
                    exploration->sort(1);

                    /*
                    * Choose which strategy to take.
                    * 1 ... least to first goal in list
                    * 2 ... first to last goal in list
                    *
                    * Determine_goal() takes world coordinates stored in
                    * the frontiers list(which contain every found frontier with
                    *sufficient
                    * spacing in between) and returns the most attractive one, based on
                    *the
                    * above defined strategies.
                    */
                    while (true)
                    {
                        goal_determined = exploration->determine_goal(1, &final_goal, count, 0, &robot_str);
                        if (goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            // negotiation =
                            // exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
                            negotiation = true;
                            if (negotiation == true)
                            {
                                break;
                            }
                            count++;
                        }
                    }
                }
                else if (frontier_selection == 3)
                {
                    exploration->sort(2);

                    while (true)
                    {
                        goal_determined = exploration->determine_goal(2, &final_goal, count, 0, &robot_str);
                        if (goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            // negotiation =
                            // exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
                            negotiation = true;
                            if (negotiation == true)
                            {
                                break;
                            }
                            count++;
                        }
                    }
                }
                else if (frontier_selection == 4)
                {
                    while (true)
                    {
                        goal_determined = exploration->determine_goal(3, &final_goal, count, 0, &robot_str);
                        ROS_DEBUG("Goal_determined: %d   counter: %d", goal_determined, count);
                        if (goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            // negotiation =
                            // exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
                            negotiation = true;
                            if (negotiation == true)
                            {
                                break;
                            }
                            count++;
                        }
                    }
                }
                else if (frontier_selection == 5)
                {
                    if (cluster_initialize_flag == true)
                    {
                        exploration->clearVisitedAndSeenFrontiersFromClusters();
                    }
                    else
                    {
                        /*
                        * This is necessary for the first run
                        */
                        if (robot_id == 0)
                            ros::Duration(10).sleep();

                        exploration->sort(2);
                    }

                    exploration->clusterFrontiers();
                    exploration->sort(4);
                    exploration->sort(5);

                    clusters_available_in_pool.clear();
                    while (true)
                    {
                        final_goal.clear();
                        goal_determined =
                            exploration->determine_goal(4, &final_goal, count, cluster_element, &robot_str);

                        if (goal_determined == false)
                        {
                            ROS_INFO("Another cluster is not available, no cluster determined");
                            break;
                        }
                        else
                        {
                            if (cluster_initialize_flag == false)
                            {
                                /*
                                *  todo ... just to reduce the selection of the same
                                * clusters since no auctioning is implemented jet.
                                */
                                if (robot_id == 1)
                                {
                                    ros::Duration(5).sleep();
                                }

                                cluster_initialize_flag = true;
                            }

                            /*
                            * If negotiation is not needed, simply uncomment
                            * and set the negotiation to TRUE.
                            */
                            negotiation =
                                exploration->negotiate_Frontier(final_goal.at(0), final_goal.at(1), final_goal.at(2),
                                                                final_goal.at(3), final_goal.at(4));
                            if (negotiation == true)
                            {
                                ROS_DEBUG("Negotiation was successful");
                                cluster_element = final_goal.at(4);
                                counter_waiting_for_clusters = 0;
                                break;
                            }
                            else
                            {
                                cluster_element = final_goal.at(4);
                                counter_waiting_for_clusters++;
                                ROS_ERROR("Negotiation was not successful, try next cluster");
                            }
                            count++;
                            /*
                            * In order to make one robot wait until the other finds
                            * another cluster to operate in, one have to fill the
                            * clusters_available_in_pool
                            * vector if count is increased at least once which determines
                            * that at least one cluster
                            */
                            clusters_available_in_pool.push_back(1);
                        }
                    }
                }
                else if (frontier_selection == 6)
                {
                    if (cluster_initialize_flag == true)
                    {
                        exploration->clearVisitedAndSeenFrontiersFromClusters();
                    }

                    exploration->clusterFrontiers();
                    exploration->sort(6);
                    exploration->visualizeClustersConsole();

                    if (cluster_initialize_flag == false)
                    {
                        ros::Duration(2).sleep();
                    }
                    cluster_initialize_flag = true;

                    while (true)
                    {
                        goal_determined =
                            exploration->determine_goal(4, &final_goal, count, cluster_element, &robot_str);

                        if (goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            /*
                            * If negotiation is not needed, simply uncomment
                            * and set the negotiation to TRUE.
                            */
                            negotiation =
                                exploration->negotiate_Frontier(final_goal.at(0), final_goal.at(1), final_goal.at(2),
                                                                final_goal.at(3), final_goal.at(4));

                            if (negotiation == true)
                            {
                                cluster_element = final_goal.at(4);
                                break;
                            }
                            else
                            {
                                cluster_element = int(exploration->clusters.size() * rand() / (RAND_MAX));
                                ROS_INFO("Random cluster_element: %d  from available %lu clusters", cluster_element,
                                         exploration->clusters.size());
                            }
                            count++;
                        }
                    }
                }

                /* 7 ... Navigate to frontier that satisfies energy efficient cost
                   function with staying-alive path planning */
                else if (frontier_selection == 17)
                    ;
                
                else if (frontier_selection == 7 || frontier_selection == 10)
                {

                    
                    if(moving_along_path) { //if not used, this var is always false...
                        if(OPP_ONLY_TWO_DS)
                            if (ds_path_counter < 2)
                            {
                                target_ds_x = path[ds_path_counter][0];
                                target_ds_y = path[ds_path_counter][1];
                                ds_path_counter++;
                                update_robot_state_2(going_checking_vacancy);
                            }
                            else
                            {
                                moving_along_path = false;
                                // update_robot_state_2(exploring);
                                update_robot_state_2(leaving_ds);
                            }
                        else
                            if(ds_path_counter < ds_path_size - 1)
                            {
                                double next_ds_x = path[ds_path_counter+1][0];
                                double next_ds_y = path[ds_path_counter+1][0];
                                double dist = -1;
                                for(int i=0; i<5 || dist >= 0; i++) {
                                    dist = exploration->distance_from_robot(next_ds_x, next_ds_y); //TODO(minor) very bad way to check... -> parameter...
                                    ros::Duration(1).sleep();   
                                }
                                
                                if(dist > available_distance * safety_coeff)
                                    if(robot_state == fully_charged) {
                                        ROS_FATAL("ERROR WITH OPPORTUNE STRATEGY");
                                        update_robot_state_2(finished);
                                    }
                                    else {
                                        ROS_ERROR("Cannot reach next DS on the path: reauction for current one");
                                        update_robot_state_2(auctioning);
                                    }
                                else {
                                    ROS_ERROR("Going to next DS in path");
                                    ds_path_counter++;
                                    target_ds_x = path[ds_path_counter][0];
                                    target_ds_y = path[ds_path_counter][1];
                                    std_msgs::Empty msg;
                                    pub_next_ds.publish(msg);
                                    update_robot_state_2(going_checking_vacancy); //TODO(minor) maybe it should start an auction before, but in that case we must check that it is not too close to the last target_ds (in fact target_ds is the next one)
                                }
                            }
                            else {
                                moving_along_path = false;
                                std_msgs::Empty msg;
                                pub_next_ds.publish(msg);
                                // update_robot_state_2(exploring);
                            } 
                    
                        continue;
                    
                    }
                    else
                    {
                        // TODO(minor) do those sorting works correclty?
                        /* Sort frontiers, firstly from nearest to farthest and then by
                         * efficiency */
                        ROS_INFO("SORTING FRONTIERS...");
                        exploration->sort(2);
                        exploration->sort(3);
                        exploration->sort_cost(battery_charge > 50, w1, w2, w3, w4);

                        /* Look for a frontier as goal */
                        ROS_INFO("DETERMINE GOAL...");
                        
                        // goal_determined = exploration->determine_goal_staying_alive(1, 2,
                        // available_distance, &final_goal, count, &robot_str, -1);
                        if(DEBUG)
                            goal_determined = exploration->determine_goal_staying_alive_2(
                                1, 2, available_distance * SAFETY_COEFF +
                                          available_distance * SAFETY_COEFF * INCR * number_of_recharges,
                                &final_goal, count, &robot_str, -1);
                        else
                            goal_determined = exploration->determine_goal_staying_alive_2(1, 2, available_distance, &final_goal, count, &robot_str, -1);
                        ROS_INFO("GOAL DETERMINED: %s; counter: %d", (goal_determined ? "yes" : "no"), count);
                        
                        if(DEBUG && IMM_CHARGE && number_of_recharges == 0 ) {
                            goal_determined  = false;
                            update_robot_state_2(exploring);
                            ros::Duration(5).sleep();
                        }
                            
                        

                        /* Check if the robot has found a reachable frontier */
                        if (goal_determined == true)
                        {
                            /* The robot has found a reachable frontier: it can move toward it */
                            update_robot_state_2(moving_to_frontier);

                            // TODO(minor) ...
                            if (exit_countdown != EXIT_COUNTDOWN)
                            {
                                exit_countdown = EXIT_COUNTDOWN;
                                ROS_ERROR("To be safe, resetting exit countdown at starting "
                                          "value; something must have gone wrong however, "
                                          "because this shoulw never happen...");
                            }

                            // TODO(minor) useless
                            // charge_countdown = EXIT_COUNTDOWN;
                        }

                        else
                        {
                            if (robot_state == fully_charged)
                            {
                                /* The robot wasn't able to find a reachable frontier even if it is fully charged: this means
                                 * that it will never be able to reach a frontier: exploration is over */
                                // TODO(minor) we shoudl better check if there are still unvisited
                                // frontier and the robot is not able to reach any of them even if
                                // fully charged, because clearly this is a problem... which could be solved by navigating the
                                // ds graph to reach one of this frontiers
                                if(exploration->existFrontiers()) //TODO(minor) are we sure taht it works correctly? theoretically yes, but the "error" message below is printed too ofter...
                                {
                                 ROS_INFO("There are still unvisited frontiers, but the robot cannot reach them even with full battery: try searching for a DS with EOs (if the DS selection strategy allows it...)"); //notice that it is true that the existing frontiers are unreachable with full battery at the moment, or the execution flow wouldn't be here...//TODO(minor) bad text... 
                                    update_robot_state_2(auctioning_2);
                                }
                                else
                                {
                                    ROS_INFO("No more frontiers to be visited: exploration is completed");
                                    /* Start countdown to finalize exploration */
                                    // TODO(minor)
                                    // exit_countdown--;
                                    // ROS_ERROR("Shutdown in: %d", exit_countdown);
                                    // if (exit_countdown <= 0)
                                    finalize_exploration();
                                    // continue;                                    
                                }                                
                            }

                            else if(robot_state == leaving_ds) {
                                if (robot_state_next != going_charging_next && robot_state_next != going_queue_next) {
                                    // check the existence of reachable frontiers with full battery life
                                    if(exploration->existFrontiers()) {
                                        if(exploration->existFrontiersReachableWithFullBattery(max_av_distance)) { //TODO(minor) really necessary?
                                            ROS_INFO("There are still unvisited frontiers, but the robot cannot reach them even with full battery: try searching for a DS with EOs (if the DS selection strategy allows it...)"); //TODO(minor) bad text...
                                            update_robot_state_2(auctioning_2);
                                         }
                                         else {
                                            ROS_ERROR("Charging was interrupted and there are still unvisited frontiers, but they cannot reach even when the robot has full battery: finalize exploration...");
                                            finalize_exploration();
                                         }
                                    } else {
                                        ROS_INFO("Charging was interrupted, but there are no more frontiers that can be reached byt the robot");
                                        finalize_exploration();
                                    }
                                }
                            }
                            else
                            {
                                /* Robot cannot reach any frontier but it is not fully charged, so a recharging is in order */
                                // TODO(minor) check if the robot has not just won an auction meanwhile... but should be solved by docking now...
                                // TODO(minor) call udpate_robot_state() instead!!!
                                // TODO(minor) but also calling update_robot_state does not solve the problem, because maybe the
                                // docking node knows that all the auctions have been concluded, but it has not notified the
                                // explorer node about it yet, which means that update_robot_state hase nothing to update at the
                                // moment... it could be solved using a service or using robot_state node
                                if (robot_state_next != going_charging_next && robot_state_next != going_queue_next)
                                {
                                    ROS_ERROR("Robot cannot reach any frontier: starting auction to "
                                              "acquire access to a DS to recharge");  // TODO(minor) this message could be misleading
                                                                                      // if the robot does not really start a
                                                                                      // new auction...
                                    update_robot_state_2(auctioning);
                                }
                            }
                        }
                    }
                }

                else if (frontier_selection == 8)
                {
                    // sort frontiers clock wise starting from left
                    exploration->sort(7);

                    // look for a frontier as goal
                    ROS_INFO("DETERMINE GOAL...");
                    goal_determined = exploration->determine_goal_staying_alive(1, 2, available_distance, &final_goal,
                                                                                count, &robot_str, -1);
                    ROS_INFO("Goal_determined: %d   counter: %d", goal_determined, count);

                    // found a frontier, go there
                    if (goal_determined == true)
                    {
                        robot_state = exploring;
                        exit_countdown = EXIT_COUNTDOWN;
                        charge_countdown = EXIT_COUNTDOWN;
                    }

                    // robot cannot reach any frontier, even if fully charged
                    // simulation is over
                    else if (recharging == false || robot_state == fully_charged)
                    {
                        exit_countdown--;
                        ROS_ERROR("Shutdown in: %d", exit_countdown);
                        if (exit_countdown <= 0)
                            finalize_exploration();
                        continue;
                    }

                    // robot cannot reach any frontier
                    // go charging
                    else
                    {
                        charge_countdown--;
                        if (charge_countdown <= 0)
                        {
                            ROS_INFO("Could not determine goal, need to recharge!");
                            robot_state = going_charging;
                        }
                        else
                            continue;
                    }
                }
                else if (frontier_selection == 9)
                {
                    // sort the frontiers from near to far
                    exploration->sort(2);
                    exploration->sort(3);

                    // look for a frontier as goal
                    goal_determined = exploration->determine_goal_staying_alive(1, 2, available_distance, &final_goal,
                                                                                count, &robot_str, -1);
                    ROS_DEBUG("Goal_determined: %d   counter: %d", goal_determined, count);

                    // found a frontier
                    // go there
                    if (goal_determined == true)
                    {
                        robot_state = exploring;
                        exit_countdown = EXIT_COUNTDOWN;
                        charge_countdown = EXIT_COUNTDOWN;
                    }

                    // robot cannot reach any frontier, even if fully charged
                    // simulation is over
                    else if (recharging == false || robot_state == fully_charged)
                    {
                        exit_countdown--;
                        ROS_ERROR("Shutdown in: %d", exit_countdown);
                        if (exit_countdown <= 0)
                            finalize_exploration();
                        continue;
                    }

                    // robot cannot reach any frontier
                    // go charging
                    else
                    {
                        charge_countdown--;
                        if (charge_countdown <= 0)
                        {
                            ROS_INFO("Could not determine goal, need to recharge!");
                            robot_state = going_charging;
                        }
                        else
                            continue;
                    }
                }
            }

            /*************************
             * FRONTIER COORDINATION *
             *************************/
             
            ROS_ERROR("FRONTIER COORDINATION");

            /* Produce frontier/cluster points for rviz */
            exploration->visualize_Cluster_Cells();
            exploration->visualize_Frontiers();

            if (robot_state == moving_to_frontier ||
                robot_state == moving_to_frontier_before_going_charging)  // TODO(minor) is
                                                                          // moving_to_frontier_before_going_charging
                                                                          // still necessary? no if the state update is
                                                                          // done in the correct point... yes otherwise
            {
                ROS_INFO("Navigating to Goal"); 
                // TODO(minor) what is this part???
                if (OPERATE_WITH_GOAL_BACKOFF == true)
                {
                    ROS_INFO("Doing smartGoalBackoff");
                    if (exploration->smartGoalBackoff(final_goal.at(0), final_goal.at(1), costmap2d_global,
                                                      &backoffGoal))
                    {
                        ROS_INFO("Navigate to backoff goal (%.2f,%.2f) --> (%.2f,%.2f)", final_goal.at(0),
                                 final_goal.at(1), backoffGoal.at(0), backoffGoal.at(1));
                        navigate_to_goal = navigate(backoffGoal);
                    }
                    else
                    {
                        /* Failed to Failed to find backoff goal */
                        ROS_ERROR("Failed to find backoff goal, mark original goal "
                                  "(%.2f,%.2f) as unreachable",
                                  final_goal.at(0), final_goal.at(1));
                        navigate_to_goal = false;  // navigate(final_goal);
                    }
                }
                else
                {
                    ROS_INFO("Navigate to goal");
                    navigate_to_goal = navigate(final_goal);
                }
            }

            // TODO(minor) hmm... here??
            if (robot_state == auctioning || robot_state == auctioning_2 )
            {
                
                if (exploration->distance_from_robot(target_ds_x, target_ds_y) <
                        min_distance_queue_ds)  // TODO could the DS change meanwhile???
                        {
                            ROS_INFO("ROBOT TOO CLOSE TO DS to start an auction: moving a little bit farther..."); 
                            move_robot_away();  // TODO(minor) move robot away also if in queue and too close...
                            ROS_INFO("NOW it is ok...");
                        }
                else
                    ROS_INFO("Starting owm auction...");
                
                while (robot_state == auctioning || robot_state == auctioning_2 )  // TODO(minor) better management of the while loop
                {
                    
                    // ROS_ERROR("\n\t\e[1;34mAuctioning...\e[0m");
                    ros::Duration(0.1).sleep();
                    ros::spinOnce();  // TODO(minor) is spin necessary? isn't it called by update_robot_State or in main() already?

                    update_robot_state();
                }
                ROS_INFO("Auction completed");
            }

            // TODO(minor) hmm... here??
            if (robot_state == in_queue)
            {
                while (robot_state == in_queue)  // TODO(minor) better management of the while loop
                {
                    ROS_DEBUG("Waiting in queue...");
                    ros::Duration(0.1).sleep();  // TODO(minor) are all these sleeps necessary? and do they have the
                                                 // corrent sleep time???
                    ros::spinOnce();  // TODO(minor) is spin necessary? isn't it called by update_robot_State already?

                    if (exploration->distance_from_robot(target_ds_x, target_ds_y) <
                        min_distance_queue_ds)
                        ; //ROS_ERROR("ROBOT TOO CLOSE TO DS!!!!!");  // TODO(minor) move robot away in this case...

                    update_robot_state();
                }
                ROS_DEBUG("No more in queue");
            }

            if (robot_state == checking_vacancy)
            {
                /* Robot reached frontier */
                ROS_ERROR("\n\t\e[1;34mchecking_for_vacancy...\e[0m");

                occupied_ds = false;

                // TODO(minor) use a bterr way!!!
                int i = 5;
                while (i > 0)
                {
                    ros::Duration(1).sleep();
                    ros::spinOnce();
                    i--;
                }
                if (occupied_ds)
                {
                    // TODO(minor) use opportune? no because if I ahve this DS in the path, it's because I need to recharge here

                    /* The DS is already (or is going to be) occupied by another robot: put robot in queue */
                    ROS_INFO("\n\t\e[1;34moccupied ds...\e[0m");
                    update_robot_state_2(in_queue);
                }
                else
                {
                    //ROS_ERROR("\n\t\e[1;34m FREE!!!\e[0m");
                    update_robot_state_2(going_charging);
                }
            }

            // navigate robot home for recharging
            // IMPORTANT: do not put an else-if here, because when I exit from the
            // above else if(robot_state == auctioning), I may need to go home / to ds
            // since now I may be in state going_charging because I won an auction!!!!
            // TODO(minor) am I sure???
            if (robot_state == going_in_queue || robot_state == going_checking_vacancy || robot_state == going_charging)
            {
                if (robot_state == going_checking_vacancy)
                    ROS_INFO("Approaching ds%d (%f, %f) to check if it is free", -1, target_ds_x, target_ds_y);
                else if (robot_state == going_in_queue)
                    ROS_INFO("Travelling to DS to go in queue");
                else
                    ROS_INFO("Robot can finally prepare itself to recharge");

                counter++;  // TODO(minor) what is this counter?
                navigate_to_goal = move_robot(counter, target_ds_x, target_ds_y);
            }

            /* NAVIGATION COMPLETED */ //F
            /* Check if the robot was able to reach the selected goal */
            if (navigate_to_goal == true)  // IMPORTANT do not put an else-if //TODO(minor) sure??
            {
                /* Result of navigation successful */

                /* If the robot was going in a queue, if it has reached the goal it means that it reached the queue */
                if (robot_state == going_in_queue)
                {
                    update_robot_state_2(in_queue);
                }
                
                /* ... */
                else if(robot_state == going_checking_vacancy)
                {
                    update_robot_state_2(checking_vacancy);
                }    

                /* If the robot was going in a queue, if it has reached the goal it means that it reached the target DS,
                 * so it can start recharging */
                else if (robot_state == going_charging)
                {
                    ROS_INFO("Reached DS for recharging");

                    std_msgs::Empty msg;
                    pub_occupied_ds.publish(msg);  // TODO(minor) it seems not to be used by any other node... remove it

                    number_of_recharges++;  // TODO(minor) remove
                    update_robot_state_2(charging);

                    /* Compute path length */  //TODO(minor) maybe we shoiuld also call this method when a robot reaches a queue, when it is moving from a queue to the DS, when it leaves the DS to queue again, ...
                    //exploration->trajectory_plan_store(target_ds_x, target_ds_y);
                }

                else if (robot_state == moving_to_frontier || robot_state == moving_to_frontier_before_going_charging)
                {
                    /* Robot reached frontier */
                    if (robot_state == moving_to_frontier)
                        update_robot_state_2(exploring);

                    ROS_INFO("STORING PATH");
                    //exploration->trajectory_plan_store(
                    //    exploration->visited_frontiers.at(exploration->visited_frontiers.size() - 1).x_coordinate,
                    //    exploration->visited_frontiers.at(exploration->visited_frontiers.size() - 1).y_coordinate);

                    ROS_DEBUG("Storing visited...");
                    exploration->storeVisitedFrontier(final_goal.at(0), final_goal.at(1), final_goal.at(2),
                                                      robot_str.at(0), final_goal.at(3));
                    ROS_DEBUG("Stored Visited frontier");
                }

            }

            else
            {
                /* Robot could not reach goal */

                if (robot_state == going_charging)
                {
                    ROS_ERROR("Robot cannot reach home for recharging!");
                    exit_countdown--;
                    ROS_ERROR("Shutdown in: %d", exit_countdown);
                    if (exit_countdown <= 0)
                        finalize_exploration();
                }

                else if (robot_state == moving_to_frontier || robot_state == moving_to_frontier_before_going_charging)
                {
                    if (robot_state == moving_to_frontier)
                    {
                        update_robot_state_2(exploring);
                        ROS_ERROR("Robot could not reach goal - explore again");
                    }
                    else
                        update_robot_state_2(going_charging);  // TODO(minor) if
                                                               // moving_to_frontier_before_going_charging is not
                                                               // required anymore, this else branch is never executed

                    ROS_DEBUG("Storing unreachable...");
                    exploration->storeUnreachableFrontier(final_goal.at(0), final_goal.at(1), final_goal.at(2),
                                                          robot_str.at(0), final_goal.at(3));
                    ROS_DEBUG("Stored unreachable frontier");
                }
            }
            
            exploration->trajectory_plan_store(stored_robot_x, stored_robot_y); //TODO(minor) here? yes it should be ok because even if the robot failes reaching the goal, in the worst case the travelled distance is 0, so it's ok if I add it to the global_travelled distance, and if instead it moved a little, I have to take note of this travelled distance...

            ROS_DEBUG("                                             ");
            ROS_DEBUG("                                             ");

            ROS_INFO("DONE EXPLORING");
        }
    }
    
    void store_current_position() {
        tf::Stamped<tf::Pose> robotPose;
        while(!exploration->getRobotPose(robotPose)) {
            ROS_ERROR("Failed getting current position... retrying in a moment");
            ros::Duration(2).sleep();
        }
        stored_robot_x = robotPose.getOrigin().getX();
        stored_robot_y = robotPose.getOrigin().getY();
    }
    
    void update_robot_state_2(int new_state)
    {  // TODO(minor) comments in the update_blabla functions, and lso in the other callbacks

        ROS_ERROR("State transition: %s -> %s", get_text_for_enum(robot_state).c_str(),
                  get_text_for_enum(new_state).c_str());
        adhoc_communication::EmRobot msg;
        msg.state = new_state;
        robot_state = static_cast<state_t>(new_state);
        pub_robot.publish(msg);

        if (robot_state == auctioning)
            need_to_recharge = true;
        // else if(robot_state == exploring || robot_state == fully_charged)
        else if (robot_state == leaving_ds || robot_state == fully_charged)
            need_to_recharge = false;

        //robot_state::GetRobotState srv;
        // sc_get_robot_state.call(srv);
        
        ros::Duration time = ros::Time::now() - time_start;

        fs_csv_state.open(csv_state_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        fs_csv_state << time << "," << get_text_for_enum(robot_state).c_str() << std::endl;
        fs_csv_state.close();
        
    }

    void update_robot_state()
    {

        ros::spinOnce();
        if(robot_state_next == finish_next) {
            ROS_INFO("Have to finish...");
            finalize_exploration();
        }
        

        /* If the next state is equal to the current one, do nothing */
        else if (robot_state_next == current_state)
            ROS_INFO("Next state is equal to the current state: do nothing");

        // TODO(minor) does this works???
        /* If a robot has already begun charging, and then discover that it has lost
         * its own auction (which means that meanwhile the auction of another player
         * was going on, it started its own auction, but before taht its own auction
         * was concluded, he discovered itself winner of the other auction and went
         * to occupy the DS; of course this could happen only if the auction timeout
         * is quite high: with a low timeout this scenario is not possible */
        // TODO(minor) the robot after having discovered that the DS is free should check
        // its state to see if instead it shouldn't put itself in in_queue state
        // maybe...
        else if (robot_state_next == going_queue_next && robot_state == charging)
        {
            ROS_ERROR("Strange case: ideally, it should not happen...");
          
            /*
            if (DS_SELECTION_POLICY == 2 && moving_along_path)  
                if(OPP_ONLY_TWO_DS)
                    if (ds_path_counter < 2)
                    {
                        target_ds_x = path[ds_path_counter][0];
                        target_ds_y = path[ds_path_counter][1];
                        ds_path_counter++;
                        update_robot_state_2(going_checking_vacancy);
                    }
                    else
                    {
                        moving_along_path = false;
                        // update_robot_state_2(exploring);
                        update_robot_state_2(leaving_ds);
                    }
                else
                    if(ds_path_counter < ds_path_size)
                    {
                        target_ds_x = path[ds_path_counter][0];
                        target_ds_y = path[ds_path_counter][1];
                        ds_path_counter++;
                        update_robot_state_2(going_checking_vacancy);
                    }
                    else {
                         moving_along_path = false;
                        // update_robot_state_2(exploring);
                        update_robot_state_2(leaving_ds);
                    } 
                    
                    
            else
            */
            {
                // update_robot_state_2(exploring);
                update_robot_state_2(leaving_ds);
            }
        }

        /* If the robot has completed the recharging process, set it to
           fully_charged */
        else if (robot_state_next == fully_charged_next)
        {
            /*
             if (DS_SELECTION_POLICY == 2 && moving_along_path) 
                if(OPP_ONLY_TWO_DS)
                    if (ds_path_counter < 2)
                    {
                        target_ds_x = path[ds_path_counter][0];
                        target_ds_y = path[ds_path_counter][1];
                        ds_path_counter++;
                        update_robot_state_2(going_checking_vacancy);
                    }
                    else
                    {
                        moving_along_path = false;
                        // update_robot_state_2(exploring);
                        update_robot_state_2(leaving_ds);
                    }
                else
                    if(ds_path_counter < ds_path_size) {
                        target_ds_x = path[ds_path_counter][0];
                        target_ds_y = path[ds_path_counter][1];
                        ds_path_counter++;
                        update_robot_state_2(going_checking_vacancy);
                    }
                    else {
                         moving_along_path = false;
                        update_robot_state_2(fully_charged);
                    }               

            else
            */
            {
                update_robot_state_2(fully_charged);
            }
        }

        /* */
        else if (robot_state_next == going_charging_next)
        {
            if (robot_state != charging && robot_state != going_charging && robot_state != going_checking_vacancy &&
                robot_state != checking_vacancy)
            {
                // robot_state = going_charging;
                update_robot_state_2(going_checking_vacancy);
            }
            else
                ;  // ROS_ERROR("\n\t\e[1;34m already charging (or approaching charging, etc.) \e[0m");
        }

        /* Check if the robot should go in a queue */
        else if (robot_state_next == going_queue_next)
        {
            /* If it is already in a queue, just signal that it is in a queue, to make
             * the rescheduling timer in docking restart */
            // TODO(minor) hmm... if the timer instead is not blocked in docking, it is not
            // necessary...
            if (robot_state == in_queue)
            {
                ROS_INFO("\n\t\e[1;34m already in_queue... \e[0m");
                update_robot_state_2(in_queue); //VERY IMPORTANT: necessary to force restarting of periodic auction timer!!!!!!!!
            }

            /* If the robot is already preparing to enter in a queue, do nothing */
            else if (robot_state == going_in_queue)
            {
                ROS_INFO("\n\t\e[1;34m aready going_in_queue... \e[0m");
            }

            /* If the robot is going to charge, let it charge at least a little: the other robot will start later a new
             * auction */
            // TODO(minor) something probably went wrong in this case...
            else if (robot_state == going_charging)
            {
                ROS_INFO("\n\t\e[1;34m i want to charge a little, first... \e[0m");
            }

            /* Otherwise, really prepare the robot to go in queue */
            else
            {
                update_robot_state_2(going_in_queue);
            }
        }

        /* */
        else if (robot_state_next == exploring_next)
        {
            /* If the robot is recharing, it must stop */
            if (robot_state == charging)
            {
                /*
                if (DS_SELECTION_POLICY == 2 && moving_along_path)
                    if (ds_path_counter < 2)
                    {
                        if (exploration->distance_from_robot(path[ds_path_counter + 1][0],
                                                             path[ds_path_counter + 1][1]) <
                            available_distance) 
                        {
                            ds_path_counter++;
                            target_ds_x = path[ds_path_counter][0];
                            target_ds_y = path[ds_path_counter][1];
                            update_robot_state_2(going_checking_vacancy);
                        }
                        else
                            update_robot_state_2(going_in_queue);  
                    }
                    else
                    {
                        moving_along_path = false;
                        std_msgs::Empty path_msg;
                        // update_robot_state_2(exploring);
                        update_robot_state_2(leaving_ds);
                    }
                else
                */
                {
                    // update_robot_state_2(exploring);
                    update_robot_state_2(leaving_ds);
                }
            }
        }

        /* Reset next state */
        robot_state_next = current_state;
    }

    void frontiers()
    {
        while (ros::ok())
        {
            costmap_mutex.lock();
            //ROS_ERROR("frontiers(): lock acquired");
            ROS_INFO("frontiers(): lock acquired");

            /* Clean frontiers */
            exploration->clearSeenFrontiers(costmap2d_global);
            exploration->clearVisitedFrontiers();
            exploration->clearUnreachableFrontiers();

            /* Publish frontiers */
            exploration->publish_frontier_list();
            exploration->publish_visited_frontier_list();  // TODO(minor) this doesn0t work really were since it publish
                                                           // only the frontier visited by this robot...

            /* Publish frontier points for rviz */
            exploration->visualize_Frontiers();

            costmap_mutex.unlock();
            //ROS_ERROR("frontiers(): lock released");
            ROS_INFO("frontiers(): lock released");

            ros::Rate(1).sleep();
        }
    }

    void map_info()
    {
        /*
        * Publish average speed of robot
        */
        ros::NodeHandle nh_pub_speed;
        ros::Publisher publisher_speed = nh_pub_speed.advertise<explorer::Speed>("avg_speed", 1);

        fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        fs_csv << "#time,exploration_travel_path_global_meters,available_distance," //TODO(minor) maybe there is a better way to obtain exploration_travel_path_global_meters without modifying ExplorationPlanner...
                  "global_map_progress,local_map_progress,battery_state,"
                  "recharge_cycles,energy_consumption,frontier_selection_strategy"
               << std::endl;
        fs_csv.close();

        while (ros::ok() && robot_state != finished)
        {
            // double angle_robot = robotPose.getRotation().getAngle();
            // ROS_ERROR("angle of robot: %.2f\n", angle_robot);

            costmap_mutex.lock();
            //ROS_ERROR("map_info(): lock acquired");
            ROS_INFO("map_info(): lock acquired");

            ros::Duration time = ros::Time::now() - time_start;

            map_progress.global_freespace = global_costmap_size();
            map_progress.local_freespace = local_costmap_size();
            map_progress.time = time.toSec();
            map_progress_during_exploration.push_back(map_progress);

            double exploration_travel_path_global =
                (double)exploration->exploration_travel_path_global * costmap_resolution;

            if (battery_charge_temp >= battery_charge)
                energy_consumption += battery_charge_temp - battery_charge;
            battery_charge_temp = battery_charge;

            fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
            fs_csv << map_progress.time << "," << exploration_travel_path_global << "," << available_distance << ","
                   << map_progress.global_freespace << "," << map_progress.local_freespace << "," << battery_charge
                   << "," << recharge_cycles << "," << energy_consumption << "," << frontier_selection << std::endl;
            fs_csv.close();

            costmap_mutex.unlock();
            //ROS_ERROR("map_info(): lock released");
            ROS_INFO("map_info(): lock released");

            // call map_merger to log data
            map_merger::LogMaps log;
            log.request.log = 12;  /// request local and global map progress
            //ROS_ERROR("Calling map_merger service logOutput");
            if (!mm_log_client.call(log))
                ROS_ERROR("Could not call map_merger service to store log.");
            ROS_DEBUG("Finished service call.");

            save_progress();

            // publish average speed
            explorer::Speed speed_msg;
            speed_msg.avg_speed = exploration_travel_path_global / map_progress.time;
            publisher_speed.publish(speed_msg);

            ros::Duration(10.0).sleep();
        }
    }

    int global_costmap_size()
    {
        occupancy_grid_global = costmap2d_global->getCostmap()->getCharMap();
        int num_map_cells_ =
            costmap2d_global->getCostmap()->getSizeInCellsX() * costmap2d_global->getCostmap()->getSizeInCellsY();
        int free = 0;

        for (unsigned int i = 0; i < num_map_cells_; i++)
        {
            if ((int)occupancy_grid_global[i] == costmap_2d::FREE_SPACE)
            {
                free++;
            }
        }
        return free;
    }

    int local_costmap_size()
    {
        if (OPERATE_ON_GLOBAL_MAP == false)
        {
            occupancy_grid_local = costmap2d_local->getCostmap()->getCharMap();
            int num_map_cells_ =
                costmap2d_local->getCostmap()->getSizeInCellsX() * costmap2d_local->getCostmap()->getSizeInCellsY();
            int free = 0;

            for (unsigned int i = 0; i < num_map_cells_; i++)
            {
                if ((int)occupancy_grid_local[i] == costmap_2d::FREE_SPACE)
                {
                    free++;
                }
            }
            return free;
        }
        else
        {
            occupancy_grid_local = costmap2d_local_size->getCostmap()->getCharMap();
            int num_map_cells_ = costmap2d_local_size->getCostmap()->getSizeInCellsX() *
                                 costmap2d_local_size->getCostmap()->getSizeInCellsY();
            int free = 0;

            for (unsigned int i = 0; i < num_map_cells_; i++)
            {
                if ((int)occupancy_grid_local[i] == costmap_2d::FREE_SPACE)
                {
                    free++;
                }
            }
            return free;
        }
    }

    void initLogPath()
    {
        /*
         * CREATE LOG PATH
         * Following code enables to write the output to a file
         * which is localized at the log_path
         */

        nh.param<std::string>("log_path", log_path, "");

        std::stringstream robot_number;
        robot_number << robot_id;
        std::string prefix = "/robot_";
        std::string robo_name = prefix.append(robot_number.str());

        log_path = log_path.append("/explorer");
        log_path = log_path.append(robo_name);
        ROS_INFO("Logging files to %s", log_path.c_str());

        boost::filesystem::path boost_log_path(log_path.c_str());
        if (!boost::filesystem::exists(boost_log_path))
            try
            {
                if (!boost::filesystem::create_directories(boost_log_path))
                    ROS_ERROR("Cannot create directory %s.", log_path.c_str());
            }
            catch (const boost::filesystem::filesystem_error &e)
            {
                ROS_ERROR("Cannot create path %s.", log_path.c_str());
            }

        log_path = log_path.append("/");
    }

    void save_progress(bool final = false)
    {
        ros::Duration ros_time = ros::Time::now() - time_start;

        double exploration_time = ros_time.toSec();
        int navigation_goals_required = counter;
        double exploration_travel_path = (double)exploration->exploration_travel_path_global_meters;
        double size_global_map =
            map_progress_during_exploration.at(map_progress_during_exploration.size() - 1).global_freespace;

        double efficiency_value = (exploration_time) / (number_of_robots * navigation_goals_required);

        std::string tmp_log;
        if (!final)
        {
            tmp_log = log_file + std::string(".tmp");
            fs.open(tmp_log.c_str(), std::fstream::in | std::fstream::trunc | std::fstream::out);
        }
        else
        {
            tmp_log = log_file;
            fs.open(tmp_log.c_str(), std::fstream::in | std::fstream::trunc | std::fstream::out);
        }

        /*
         * WRITE LOG FILE
         * Write all the output to the log file
         */
        time_t raw_time;
        struct tm *timeinfo;
        time(&raw_time);
        timeinfo = localtime(&raw_time);

        fs << "[Exploration]" << std::endl;
        fs << "time_file_written    			= " << asctime(timeinfo);  // << std::endl;
        fs << "start_time           			= " << time_start << std::endl;
        fs << "end_time             			= " << ros::Time::now() << std::endl;
        fs << "exploration_time    	            = " << exploration_time << std::endl;
        fs << "required_goals                   = " << navigation_goals_required << std::endl;
        fs << "unreachable_goals                = " << exploration->unreachable_frontiers.size() << std::endl;
        fs << "travel_path_overall  	        = " << exploration_travel_path << std::endl;
        fs << "number_of_completed_auctions     = " << exploration->number_of_completed_auctions << std::endl;
        fs << "number_of_uncompleted_auctions   = " << exploration->number_of_uncompleted_auctions << std::endl;
        fs << "frontier_selection_strategy      = " << frontier_selection << std::endl;
        fs << "costmap_size                     = " << costmap_width << std::endl;
        fs << "global costmap iterations        = " << global_costmap_iteration << std::endl;
        fs << "number of recharges              = " << recharge_cycles << std::endl;
        fs << "energy_consumption               = " << energy_consumption << std::endl;
        fs << "available_distance               = " << available_distance << std::endl;

        double param_double;
        int param_int;
        std::string param;
        /*param = robot_prefix + "/explorer/local_costmap/height";
            ros::param::get(param,param_double);*/
        nh.param<int>("local_costmap/height", param_int, -1);
        fs << "explorer_local_costmap_height 		= " << param_int << std::endl;

        /*param = robot_prefix + "/explorer/local_costmap/width";
            ros::param::get(param,param_double);*/
        nh.param<int>("local_costmap/width", param_int, -1);
        fs << "explorer_local_costmap_width 		= " << param_int << std::endl;

        param = robot_prefix + "/move_base/local_costmap/height";
        ros::param::get(param, param_double);
        fs << "move_base_local_costmap_height 		= " << param_double << std::endl;

        param = robot_prefix + "/move_base/local_costmap/width";
        ros::param::get(param, param_double);
        fs << "move_base_local_costmap_width 		= " << param_double << std::endl;

        param = robot_prefix + "/move_base/global_costmap/obstacle_layer/raytrace_range";
        ros::param::get(param, param_double);
        fs << "move_base_raytrace_range 		= " << param_double << std::endl;

        param = robot_prefix + "/move_base/global_costmap/obstacle_layer/obstacle_range";
        ros::param::get(param, param_double);
        fs << "move_base_obstacle_range 		= " << param_double << std::endl;

        //	    param = robot_prefix +
        //"/navigation/global_costmap/obstacle_layer/raytrace_range";
        nh.getParam("/global_costmap/obstacle_layer/raytrace_range", param_double);
        ros::param::get(param, param_double);
        fs << "explorer_raytrace_range 		= " << param_double << std::endl;

        // param = robot_prefix +
        // "/navigation/global_costmap/obstacle_layer/obstacle_range";
        //    ros::param::get(param,param_double);
        nh.getParam("/global_costmap/obstacle_layer/obstacle_range", param_double);
        fs << "explorer_obstacle_range 		= " << param_double << std::endl;

        if (final)
            fs << "complete             			= "
               << "1" << std::endl;
        else
            fs << "complete             			= "
               << "0" << std::endl;

        fs.close();
        //            ROS_INFO("Wrote file %s\n", log_file.c_str());

        /*
         * Inform map_merger to save maps
         */

        if (final)
        {
            map_merger::LogMaps log;
            log.request.log = 3;  /// request local and global map
            ROS_INFO("Logging");
            if (!mm_log_client.call(log))
                ROS_ERROR("Could not call map_merger service to store log.");
        }
    }

    void finalize_exploration()
    {
        // finished exploration
        update_robot_state_2(finished);

        // finish log files
        exploration_has_finished();

        // TODO(minor) hmm...

        visualize_goal_point(home_point_x, home_point_y);

        bool completed_navigation = false;
        for (int i = 0; i < 5; i++)
        {
            if (completed_navigation == false)
            {
                counter++;
                completed_navigation = move_robot(counter, home_point_x, home_point_y);
            }
            else
            {
                break;
            }
        }

        // Indicte end of simulation for this robot
        // When the multi_robot_simulation/multiple_exploration_runs.sh script is
        // run, this kills all processes and starts a new run
        this->indicateSimulationEnd();

        ROS_INFO("Shutting down...");
        //ros::shutdown();
    }

    void exploration_has_finished()
    {
        ros::Duration ros_time = ros::Time::now() - time_start;

        double exploration_time = ros_time.toSec();
        int navigation_goals_required = counter;
        double exploration_travel_path = (double)exploration->exploration_travel_path_global * 0.02;
        double size_global_map =
            map_progress_during_exploration.at(map_progress_during_exploration.size() - 1).global_freespace;
        ROS_INFO("overall freespace in the global map: %f", size_global_map);

        for (int i = 0; i < map_progress_during_exploration.size(); i++)
        {
            ROS_INFO("map progress: %f",
                     (map_progress_during_exploration.at(i).global_freespace / size_global_map) * 100);
        }

        ROS_DEBUG("******************************************");
        ROS_DEBUG("******************************************");
        ROS_DEBUG("TIME: %f sec  GOALS: %d  PATH: %f  COMPLETED AUCTIONS: %d  "
                  "UNCOMPLETED AUCTIONS: %d",
                  exploration_time, navigation_goals_required, exploration_travel_path,
                  exploration->number_of_completed_auctions, exploration->number_of_uncompleted_auctions);
        ROS_DEBUG("******************************************");
        ROS_DEBUG("******************************************");

        double efficiency_value = (exploration_time) / (number_of_robots * navigation_goals_required);

        /*
         * WRITE LOG FILE
         * Write all the output to the log file
         */
        save_progress(true);
        ROS_INFO("Wrote file %s\n", log_file.c_str());

        /*
         * Inform map_merger to save maps
         */

        map_merger::LogMaps log;
        log.request.log = 3;  /// request local and global map
        ROS_INFO("Logging");
        if (!mm_log_client.call(log))
            ROS_ERROR("Could not call map_merger service to store log.");

        //#ifdef PROFILE
        // HeapProfilerStop();
        // ProfilerStop();
        //#endif
    }

    void indicateSimulationEnd()
    {
        /// FIXME: remove this stuff once ported to multicast

        
        std::stringstream robot_number;
        robot_number << robot_id;

        std::string prefix = "/robot_";
        std::string status_directory = "/simulation_status";
        std::string robo_name = prefix.append(robot_number.str());
        std::string file_suffix(".finished");

        std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
        std::string status_path = ros_package_path + status_directory;
        std::string status_file = status_path + robo_name + file_suffix;

        // TODO(minor): check whether directory exists
        boost::filesystem::path boost_status_path(status_path.c_str());
        if(!boost::filesystem::exists(boost_status_path))
            if(!boost::filesystem::create_directories(boost_status_path))
                ROS_ERROR("Cannot create directory %s.", status_path.c_str());
        std::ofstream outfile(status_file.c_str());
        outfile.close();
        ROS_INFO("Creating file %s to indicate end of exploration.",
        status_file.c_str());
        
    }

    // TODO(minor) interesting function
    /**
     * Iterate over the whole map to see if there are any remaining frontiers
     */
    bool iterate_global_costmap(std::vector<double> *global_goal, std::vector<std::string> *robot_str)
    {
        global_costmap_iteration++;
        int counter = 0;
        bool exploration_flag;

        costmap_mutex.lock();
        //ROS_ERROR("iterate_global_costmap(): lock acquired");
        ROS_INFO("iterate_global_costmap(): lock acquired");

        exploration->transformToOwnCoordinates_frontiers();
        exploration->transformToOwnCoordinates_visited_frontiers();

        exploration->initialize_planner("exploration planner", costmap2d_global, costmap2d_global);
        exploration->findFrontiers();

        exploration->clearVisitedFrontiers();
        exploration->clearUnreachableFrontiers();
        exploration->clearSeenFrontiers(costmap2d_global);

        costmap_mutex.unlock();
        //ROS_ERROR("iterate_global_costmap(): lock released");
        ROS_INFO("iterate_global_costmap(): lock released");

        // exploration->visualize_Frontiers();

        if (frontier_selection < 5 && frontier_selection != 1)
        {
            exploration->sort(2);

            while (true)
            {
                exploration_flag = exploration->determine_goal(2, global_goal, counter, -1, robot_str);

                if (exploration_flag == false)
                {
                    break;
                }
                else
                {
                    bool negotiation = true;
                    negotiation = exploration->negotiate_Frontier(global_goal->at(0), global_goal->at(1),
                                                                  global_goal->at(2), global_goal->at(3), -1);

                    if (negotiation == true)
                    {
                        return true;
                    }
                    counter++;
                }
            }
        }

        else if (frontier_selection == 1 || frontier_selection == 6 || frontier_selection == 5)
        {
            costmap_mutex.lock();
            exploration->clearVisitedAndSeenFrontiersFromClusters();

            exploration->clusterFrontiers();

            exploration->sort(4);
            exploration->sort(5);

            costmap_mutex.unlock();

            cluster_element = -1;

            while (true)
            {
                std::vector<double> goal_vector;
                std::vector<std::string> robot_str_name;
                std::vector<int> clusters_used_by_others;

                goal_determined = exploration->determine_goal(5, &goal_vector, 0, cluster_element, robot_str);

                if (goal_determined == false)
                {
                    ROS_INFO("No goal was determined, cluster is empty. Bid for another one");

                    goal_vector.clear();
                    bool auctioning = exploration->auctioning(&goal_vector, &clusters_used_by_others, &robot_str_name);
                    if (auctioning == true)
                    {
                        goal_determined = true;
                        cluster_element = goal_vector.at(4);

                        global_goal->push_back(goal_vector.at(0));
                        global_goal->push_back(goal_vector.at(1));
                        global_goal->push_back(goal_vector.at(2));
                        global_goal->push_back(goal_vector.at(3));
                        global_goal->push_back(goal_vector.at(4));

                        robot_str->push_back(robot_str_name.at(0));
                        return true;
                    }
                    else
                    {
                        /*
                        * If NO goals are selected at all, iterate over the global
                        * map to find some goals.
                        */
                        ROS_ERROR("No goals are available at all");
                        cluster_element = -1;
                        break;
                    }
                }
                else
                {
                    ROS_INFO("Still some goals in current cluster, finish this job first");
                    break;
                }
            }

            exploration->visualize_Cluster_Cells();
        }

        else
        {
            ROS_ERROR("Could not iterate over map, wrong strategy!");
        }

        global_iterations++;
        return false;
    }

    /*
     * If received goal is not empty (x=0 y=0), drive the robot to this point
     * and mark that goal as seen in the last_goal_position vector!!!
     * Otherwise turn the robot by 90° to the right and search again for a better
     * frontier.
     */
    bool navigate(std::vector<double> goal)
    {
        bool completed_navigation = false;

        // valid goal, drive robot there
        if (goal_determined == true)
        {
            visualize_goal_point(goal.at(0), goal.at(1));

            counter++;
            ROS_INFO("GOAL %d:  x: %f      y: %f", counter, goal.at(0), goal.at(1));
            completed_navigation = move_robot(counter, goal.at(0), goal.at(1));
            rotation_counter = 0;

            // F
            // publish goal position
        }

        // no valid goal found
        else
        {
            rotation_counter++;
            ROS_INFO("In navigation .... cluster_available: %lu     counter: %d", exploration->clusters.size(),
                     counter_waiting_for_clusters);

            // ???
            if (exploration->clusters.size() == 0 || counter_waiting_for_clusters > 10)
            {
                // check the whole map for any remaining frontiers
                ROS_INFO("Iterating over GLOBAL COSTMAP to find a goal!!!!");
                std::vector<double> global_goal;
                std::vector<std::string> robot_str;
                bool global_costmap_goal = iterate_global_costmap(&global_goal, &robot_str);

                // no frontiers available anymore, exploration finished
                if (global_costmap_goal == false)
                {
                    counter++;
                    ROS_INFO("GOAL %d: BACK TO HOME   x: %f    y: %f", counter, home_point_x, home_point_y);

                    finalize_exploration();
                }

                // found frontiers, try to navigate there
                else
                {
                    counter++;
                    ROS_INFO("GOAL %d:  x: %f      y: %f", counter, global_goal.at(0), global_goal.at(1));
                    std::vector<double> backoffGoal;
                    bool backoff_sucessfull = exploration->smartGoalBackoff(global_goal.at(0), global_goal.at(1),
                                                                            costmap2d_global, &backoffGoal);

                    if (backoff_sucessfull == true)
                    {
                        ROS_DEBUG("doing navigation to back-off goal");
                        visualize_goal_point(backoffGoal.at(0), backoffGoal.at(1));
                        completed_navigation = move_robot(counter, backoffGoal.at(0), backoffGoal.at(1));
                        rotation_counter = 0;
                        if (completed_navigation == true)
                        {
                            // compute path length
                            exploration->trajectory_plan_store(
                                exploration->visited_frontiers.at(exploration->visited_frontiers.size() - 1)
                                    .x_coordinate,
                                exploration->visited_frontiers.at(exploration->visited_frontiers.size() - 1)
                                    .y_coordinate);

                            ROS_INFO("Storing visited...");
                            ROS_ERROR("%f, %f, %f", global_goal.at(0), global_goal.at(1), global_goal.at(2));
                            exploration->storeVisitedFrontier(global_goal.at(0), global_goal.at(1), global_goal.at(2),
                                                              robot_str.at(0), global_goal.at(3));
                            ROS_INFO("Stored Visited frontier");
                        }
                        else
                        {
                            ROS_INFO("Storing unreachable...");
                            exploration->storeUnreachableFrontier(global_goal.at(0), global_goal.at(1),
                                                                  global_goal.at(2), robot_str.at(0),
                                                                  global_goal.at(3));
                            ROS_INFO("Stored unreachable frontier");
                        }
                    }
                    else if (backoff_sucessfull == false)
                    {
                        ROS_ERROR("Navigation to global costmap back-off goal not possible");
                        ROS_INFO("Storing as unreachable...");
                        exploration->storeUnreachableFrontier(global_goal.at(0), global_goal.at(1), global_goal.at(2),
                                                              robot_str.at(0), global_goal.at(3));
                        ROS_INFO("Stored unreachable frontier");
                    }
                }
            }

            // just turn the robot and continue exploration
            else
            {
                counter++;
                ROS_INFO("GOAL %d:  rotation", counter);
                completed_navigation = turn_robot(counter);
            }
        }

        // return outcome of navigation
        return (completed_navigation);
    }
    
    void wait_for_explorer_callback(const std_msgs::Empty &msg) {
        if(explorer_ready) {
            std_msgs::Empty msg;
            pub_wait.publish(msg);
        }
    }

    void visualize_goal_point(double x, double y)
    {
        goalPoint.header.seq = goal_point_message++;
        goalPoint.header.stamp = ros::Time::now();
        goalPoint.header.frame_id = move_base_frame;  //"map"
        goalPoint.point.x = x;                        // - robotPose.getOrigin().getX();
        goalPoint.point.y = y;                        // - robotPose.getOrigin().getY();

        ros::NodeHandle nh_Point("goalPoint");
        pub_Point = nh_Point.advertise<geometry_msgs::PointStamped>("goalPoint", 100, true);
        pub_Point.publish<geometry_msgs::PointStamped>(goalPoint);
    }

    void visualize_home_point()
    {
        homePoint.header.seq = home_point_message++;
        homePoint.header.stamp = ros::Time::now();
        homePoint.header.frame_id = move_base_frame;  //"map";
        home_point_x = robotPose.getOrigin().getX();
        home_point_y = robotPose.getOrigin().getY();
        homePoint.point.x = home_point_x;
        homePoint.point.y = home_point_y;

        ros::NodeHandle nh("homePoint");
        pub_home_Point = nh.advertise<geometry_msgs::PointStamped>("homePoint", 100, true);
        pub_home_Point.publish<geometry_msgs::PointStamped>(homePoint);
    }

    bool move_robot(int seq, double position_x, double position_y)
    {
        ROS_INFO("Preparing to move toward goal...");

        exploration->next_auction_position_x = position_x;
        exploration->next_auction_position_y = position_y;
        int stuck_countdown = EXIT_COUNTDOWN;

        /* Move the robot with the help of an action client. Goal positions are transmitted to the robot and feedback is
         * given about the actual driving state of the robot. */
        if (!costmap2d_local->getRobotPose(robotPose))
        {
            ROS_ERROR("Failed to get RobotPose");  // TODO(minor) so what???
        }

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

        while (!ac.waitForServer(ros::Duration(10.0)))
            ;

        move_base_msgs::MoveBaseGoal goal_msgs;

        goal_msgs.target_pose.header.seq = seq;                   // increase the sequence number
        goal_msgs.target_pose.header.frame_id = move_base_frame;  //"map";
        goal_msgs.target_pose.pose.position.x = position_x;
        goal_msgs.target_pose.pose.position.y = position_y;
        goal_msgs.target_pose.pose.position.z = 0;
        goal_msgs.target_pose.pose.orientation.x = 0;
        goal_msgs.target_pose.pose.orientation.y = 0;
        goal_msgs.target_pose.pose.orientation.z = 0;
        goal_msgs.target_pose.pose.orientation.w = 1;

        // TODO(minor) needed???
        // ROS_ERROR("\n\t\e[1;34mFinally moving!\e[0m");
        // if(robot_state == leaving_ds)
        // if(leaving_ds) {
        //    std_msgs::Empty empty_msg;
        //    pub_vacant_ds.publish(empty_msg);
        //    leaving_ds = false;
        //}

        /* Get distance from goal */
        double remaining_distance = exploration->distance_from_robot(position_x, position_y);

        /* If the robot is moving toward a DS, check if it is already close to the DS: if it is, do not move it */
        if (remaining_distance < queue_distance && (robot_state == going_in_queue || robot_state == going_checking_vacancy) )
        {
            //ROS_ERROR("\n\t\e[1;34mSTOP!! let's wait...\e[0m");
            //exploration->next_auction_position_x = robotPose.getOrigin().getX();
            //exploration->next_auction_position_y = robotPose.getOrigin().getY();
            return true;
        }

        /* Start moving */
        ROS_DEBUG("Setting goal...");
        ac.sendGoal(goal_msgs);

        /* Wait until the goal is set */
        ac.waitForResult(ros::Duration(waitForResult));  // TODO(minor) necessary?
        while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
        {
            ros::Duration(0.5).sleep();
        }
        ROS_DEBUG("Goal correctly set");
        ROS_DEBUG("Moving toward goal...");

        while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            // robot seems to be stuck
            if (prev_pose_x == pose_x && prev_pose_y == pose_y && prev_pose_angle == pose_angle)  // TODO(minor) ...
            {
                //stuck_countdown--; //TODO(minor)
                // if(stuck_countdown <= 5){

                // TODO(minor) if STUCK_COUNTDOWN is too low, even when the robot is
                // computing the frontier, it is believed to be stucked...
                if (stuck_countdown <= 10)
                {
                    ROS_ERROR("Robot is not moving anymore, shutdown in: %d", stuck_countdown);
                }

                if (stuck_countdown <= 0)
                {
                    //F
                    //exit(0);
                    exit(-1);
                }

                // F
                ros::spinOnce();
                // ros::Duration(0.5).sleep();
                ros::Duration(1).sleep();
            }
            else
            {
                ROS_ERROR("(%f, %f; %f) : (%f, %f; %f)", prev_pose_x, prev_pose_y, prev_pose_angle, pose_x, pose_y,
                          pose_angle);
                stuck_countdown = STUCK_COUNTDOWN;  // robot is moving again
                prev_pose_x = pose_x;
                prev_pose_y = pose_y;
                prev_pose_angle = pose_angle;
            }

            remaining_distance = exploration->distance_from_robot(position_x, position_y);

            /* Print remaining distance to be travelled to reach goal if the goal is a DS */
            if (robot_state == going_checking_vacancy || robot_state == going_in_queue)
                ROS_DEBUG("Remaining distance: %.3f\e[0m", remaining_distance);

            /* If the robot is approaching a DS to queue or to check if it is free, stop it when it is close enough to
             * the DS */
            if (remaining_distance < queue_distance && (robot_state == going_checking_vacancy || robot_state == going_in_queue) )
            {
                ac.cancelGoal();
                exploration->next_auction_position_x = robotPose.getOrigin().getX();
                exploration->next_auction_position_y = robotPose.getOrigin().getY();
                return true;
            }

            // ros::Duration(0.5).sleep(); //TODO(minor)
        }

        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
            {
                ROS_INFO("ABORTED");

                exploration->next_auction_position_x = robotPose.getOrigin().getX();
                exploration->next_auction_position_y = robotPose.getOrigin().getY();
                return false;
            }
        }

        ROS_INFO("Goal reached");

        exploration->next_auction_position_x = robotPose.getOrigin().getX();
        exploration->next_auction_position_y = robotPose.getOrigin().getY();

        return true;
    }
    
    bool move_robot_away()
    {
        ROS_INFO("Preparing to move toward goal...");
        int stuck_countdown = EXIT_COUNTDOWN;

        /* Move the robot with the help of an action client. Goal positions are transmitted to the robot and feedback is
         * given about the actual driving state of the robot. */
        if (!costmap2d_local->getRobotPose(robotPose))
        {
            ROS_ERROR("Failed to get RobotPose");  // TODO(minor) so what???
        }

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

        while (!ac.waitForServer(ros::Duration(10.0)))
            ;

        move_base_msgs::MoveBaseGoal goal_msgs;

        goal_msgs.target_pose.header.seq = seq;                   // increase the sequence number
        goal_msgs.target_pose.header.frame_id = move_base_frame;  //"map";
        goal_msgs.target_pose.pose.position.x = home_point_x;
        goal_msgs.target_pose.pose.position.y = home_point_y;
        goal_msgs.target_pose.pose.position.z = 0;
        goal_msgs.target_pose.pose.orientation.x = 0;
        goal_msgs.target_pose.pose.orientation.y = 0;
        goal_msgs.target_pose.pose.orientation.z = 0;
        goal_msgs.target_pose.pose.orientation.w = 1;

        /* Start moving */
        ROS_DEBUG("Setting goal...");
        ac.sendGoal(goal_msgs);

        /* Wait until the goal is set */
        ac.waitForResult(ros::Duration(waitForResult));  // TODO(minor) necessary?
        while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
        {
            ros::Duration(0.5).sleep();
        }
        ROS_DEBUG("Goal correctly set");
        ROS_DEBUG("Moving toward goal...");

        while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            // robot seems to be stuck
            if (prev_pose_x == pose_x && prev_pose_y == pose_y && prev_pose_angle == pose_angle)  // TODO(minor) ...
            {
                //stuck_countdown--;
                // if(stuck_countdown <= 5){

                // TODO(minor) if STUCK_COUNTDOWN is too low, even when the robot is
                // computing the frontier, it is believed to be stucked...
                if (stuck_countdown <= 10)
                {
                    ROS_ERROR("Robot is not moving anymore, shutdown in: %d", stuck_countdown);
                }

                if (stuck_countdown <= 0)
                {
                    //F
                    //exit(0);
                    exit(-1);
                }

                // F
                ros::spinOnce();
                // ros::Duration(0.5).sleep();
                ros::Duration(1).sleep();
            }
            else
            {
                ROS_ERROR("(%f, %f; %f) : (%f, %f; %f)", prev_pose_x, prev_pose_y, prev_pose_angle, pose_x, pose_y,
                          pose_angle);
                stuck_countdown = STUCK_COUNTDOWN;  // robot is moving again
                prev_pose_x = pose_x;
                prev_pose_y = pose_y;
                prev_pose_angle = pose_angle;
            }

            double remaining_distance = exploration->distance_from_robot(target_ds_x, target_ds_y);

            /* Print remaining distance to be travelled to reach goal if the goal is a DS */
            if (remaining_distance > queue_distance / 2.0) {
                ROS_DEBUG("Remaining distance: %.3f\e[0m", remaining_distance);

            /* If the robot is approaching a DS to queue or to check if it is free, stop it when it is close enough to
             * the DS */
                ac.cancelGoal();

                //ROS_ERROR("\n\t\e[1;34mOK!!!\e[0m");

                break;
            }

            // ros::Duration(0.5).sleep(); //TODO(minor)
        }

        ROS_INFO("Goal reached");

        exploration->next_auction_position_x = robotPose.getOrigin().getX();
        exploration->next_auction_position_y = robotPose.getOrigin().getY();

        return true;
    }

    bool turn_robot(int seq)
    {
        double angle = 45;

        if (!costmap2d_local->getRobotPose(robotPose))
        {
            ROS_ERROR("Failed to get RobotPose");
        }

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        while (!ac.waitForServer(ros::Duration(10.0)))
            ;

        move_base_msgs::MoveBaseGoal goal_msgs;

        goal_msgs.target_pose.header.seq = seq;  // increase the sequence number
        goal_msgs.target_pose.header.stamp = ros::Time::now();

        goal_msgs.target_pose.header.frame_id = move_base_frame;  //"map";
        goal_msgs.target_pose.pose.position.x = robotPose.getOrigin().getX();
        goal_msgs.target_pose.pose.position.y = robotPose.getOrigin().getY();
        goal_msgs.target_pose.pose.position.z = 0;
        goal_msgs.target_pose.pose.orientation.x = 0;  // sin(angle/2); // goals[0].pose.orientation.x;
        goal_msgs.target_pose.pose.orientation.y = 0;
        goal_msgs.target_pose.pose.orientation.z = sin(angle / 2);
        goal_msgs.target_pose.pose.orientation.w = cos(angle / 2);

        ac.sendGoal(goal_msgs);
        ac.waitForResult(ros::Duration(waitForResult));

        while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
        {
            ros::Duration(0.5).sleep();
        }
        ROS_INFO("Not longer PENDING");

        while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            ros::Duration(0.5).sleep();
        }
        ROS_INFO("Not longer ACTIVE");

        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
            {
                ROS_INFO("ABORTED");
                return false;
            }
        }
        ROS_INFO("ROTATION ACCOMBLISHED");
        return true;
    }

    void lost_own_auction_callback(const std_msgs::Empty::ConstPtr &msg)
    {
        ROS_ERROR("\n\t\e[1;34m lost_own_auction_callback \e[0m");
        robot_state_next = going_queue_next;
    }

    void won_callback(const std_msgs::Empty::ConstPtr &msg)
    {
        ROS_ERROR("\n\t\e[1;34m won_callback \e[0m");
        robot_state_next = going_charging_next;
    }

    void lost_other_robot_callback(const std_msgs::Empty::ConstPtr &msg)
    {
        ROS_ERROR("\n\t\e[1;34m lost_other_robot_callback \e[0m");
        if(robot_state == in_queue) //to force the resetting of the timer to restart an auction
            robot_state_next = going_queue_next;
        
        else if (robot_state_next != fully_charged_next)
            robot_state_next = exploring_next;
    }

    // TODO(minor) use this instead than all the other auction callbacks
    void lost_callback()
    {
        if (need_to_recharge)
            robot_state_next = going_queue_next;
        else
            robot_state_next = exploring_next;
    }

    void reply_for_vacancy_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg)
    {
        ROS_INFO("Target DS is occupied");
        occupied_ds = true;
    }

    void battery_charging_completed_callback(const std_msgs::Empty::ConstPtr &msg)
    {
        ROS_INFO("Recharging completed");
        if (robot_state != moving_to_frontier)
            robot_state_next = fully_charged_next;
    }

    void new_target_docking_station_selected_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        ROS_INFO("Storing new target DS position");
        target_ds_x = msg.get()->point.x;
        target_ds_y = msg.get()->point.y;
        ROS_DEBUG("New target DS is placed at (%f, %f)", target_ds_x, target_ds_y);

        // TODO(minor)
        /*
        map_merger::TransformPoint point;
        if (robot_prefix == "/robot_1")
            point.request.point.src_robot = "robot_0";
        else if (robot_prefix == "/robot_0")
            point.request.point.src_robot = "robot_1";
        else
            ;  // ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        point.request.point.x = target_ds_x;
        point.request.point.y = target_ds_y;

        ros::ServiceClient sc_trasform = nh.serviceClient<map_merger::TransformPoint>("map_merger/transformPoint");
        if (!sc_trasform.call(point))
            ;  // ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
               // ROS_ERROR("\e[1;34mCalling: %s\e[0m",
               // sc_trasform.getService().c_str());sc_trasform.call(point);
               // ROS_ERROR("\e[1;34mOriginal point:   (%f, %f)\e[0m",
               // point.request.point.x, point.request.point.y);
               // ROS_ERROR("\e[1;34mTansformed point: (%f, %f)\e[0m",
               // point.response.point.x, point.response.point.y);
               */
    }

    void moving_along_path_callback(const adhoc_communication::MmListOfPoints::ConstPtr &msg)
    {
        ROS_INFO("MOVING!!!!!!!!!!!!");
        moving_along_path = true;
        ds_path_counter = 0;
        
        if(OPP_ONLY_TWO_DS) {
            path[0][0] = msg.get()->positions[0].x;
            path[0][1] = msg.get()->positions[0].y;
            path[1][0] = msg.get()->positions[1].x;
            path[1][1] = msg.get()->positions[1].y;
        }
        else {
            complex_path.clear();
            ds_path_size = msg.get()->positions.size();
            for(int i=0; i < ds_path_size; i++)
                complex_path.push_back(msg.get()->positions[i]);
        }
    }

    void bat_callback(const energy_mgmt::battery_state::ConstPtr &msg)
    {
        ROS_DEBUG("Received battery state");
        battery_charge = (int)msg->soc;
        charge_time = msg->remaining_time_charge;
        available_distance = msg->remaining_distance;
        //ROS_ERROR("SOC: %d%%; available distance: %f", battery_charge * 100, available_distance);

        if (msg->charging == false && battery_charge == 100 && charge_time == 0)
            recharge_cycles++;  // TODO(minor) hmm... soc, charge, ...

        /* If the robot has run out of energy, it cannot move anymore: terminate exploration... */
        if (available_distance <= 0 && robot_state != charging)
        {
            ROS_FATAL("Robot has run out of energy!");
            abort();
        }
        
        if(max_av_distance < available_distance)
            max_av_distance = available_distance;
    }
    
    void finish_callback(const std_msgs::Empty &msg) {
        robot_state_next = finish_next;
    }
    
    void abort() {
        ROS_FATAL("Exploration is going to be brutally aborted...");
        ros::Duration(3).sleep();
        exit(-1);
    }

    bool robot_pose_callback(explorer::RobotPosition::Request &req, explorer::RobotPosition::Response &res)
    {
        tf::Stamped<tf::Pose> robotPose;
        if(!exploration->getRobotPose(robotPose))
            return false;
        res.x = robotPose.getOrigin().getX();
        res.y = robotPose.getOrigin().getY();
        return true;
        
        /*
        double x, y;
        if(!exploration->get_robot_position(&x, &y))
            return false;
        res.x = x;
        res.y = y;
        return true;
        */
    }
       
    bool reachable_target_callback(explorer::DistanceFromRobot::Request &req,
                                      explorer::DistanceFromRobot::Response &res)
    {
        res.reachable = exploration->reachable_target(req.x, req.y);
        return true;
    }
    
    

    bool distance_from_robot_callback(explorer::DistanceFromRobot::Request &req,
                                      explorer::DistanceFromRobot::Response &res)
    {
        res.distance = exploration->distance_from_robot(req.x, req.y);
        if(res.distance >= 0)
            return true;
        return false;
    }
    
    bool distance(explorer::Distance::Request &req, explorer::Distance::Response &res)
    {
        res.distance = exploration->distance(req.x1, req.y1, req.x2, req.y2);
        if(res.distance >= 0)
            return true;
        return false;
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
    {
        /* Get rotation of robot relative to starting position */
        tf::Quaternion orientation = tf::Quaternion(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y,
                                                    pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);
        pose_angle = orientation.getAngle();

        /* Get robot position */
        pose_x = pose->pose.pose.position.x;
        pose_y = pose->pose.pose.position.y;
    }

    void feedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr &msg)
    {
        feedback_value = msg.get()->status.status;
        feedback_succeed_value = msg.get()->feedback.base_position.pose.orientation.w;
    }

    bool target_reached(void)
    {
        ros::NodeHandle nh_sub_move_base;
        sub_move_base = nh_sub_move_base.subscribe("feedback", 1000, &Explorer::feedbackCallback, this);

        while (feedback_value != feedback_succeed_value)
            ;

        return true;
    }
    
    void safety_checks() {
        int duration = 30; //minutes;
        
        int sleeping_time = 5.0;
        int starting_value = 60/sleeping_time * duration;
        int countdown = starting_value;
        float prev_robot_x = 0, prev_robot_y = 0, robot_x = 0, robot_y = 0;
        tf::Stamped<tf::Pose> robotPose;
        while(exploration == NULL)
            ros::Duration(sleeping_time).sleep();
        while(ros::ok()) {
            ros::Duration(sleeping_time).sleep();
            //ROS_DEBUG("Checking...");
            if(exploration->getRobotPose(robotPose)) {
                robot_x = robotPose.getOrigin().getX();
                robot_y = robotPose.getOrigin().getY();
            }
            if(robot_x == prev_robot_x && robot_y == prev_robot_y) {
                if(countdown != starting_value && countdown*sleeping_time % 100 == 0)
                    ROS_ERROR("Countdown to shutdown at %dmin...", countdown*sleeping_time/60);
                else
                    ROS_DEBUG("Countdown to shutdown at %dmin...", countdown*sleeping_time/60);
                countdown--;
                if(countdown < 0) {
                    ROS_FATAL("Robot is not moving anymore");
                    abort();
                }
            } else {
                ROS_DEBUG("Robot is moving");
                countdown = starting_value;
                prev_robot_x = robot_x;
                prev_robot_y = robot_y;
            }

        }
    
    }

    /********************
     * CLASS ATTRIBUTES *
     ********************/
    struct map_progress_t
    {
        double local_freespace;
        double global_freespace;
        double time;
    } map_progress;

    ros::Subscriber sub_move_base, sub_obstacle;

    // create a costmap
    costmap_2d::Costmap2DROS *costmap2d_local;
    costmap_2d::Costmap2DROS *costmap2d_local_size;
    costmap_2d::Costmap2DROS *costmap2d_global;
    costmap_2d::Costmap2D costmap;

    std::vector<map_progress_t> map_progress_during_exploration;

    std::vector<int> clusters_available_in_pool;

    int home_position_x, home_position_y;
    int robot_id, number_of_robots;
    int frontier_selection, costmap_width, global_costmap_iteration, number_unreachable_frontiers_for_cluster;
    int counter_waiting_for_clusters;

    double robot_home_position_x, robot_home_position_y, costmap_resolution;
    bool goal_determined;
    bool robot_prefix_empty;
    int battery_charge, battery_charge_temp, energy_consumption, recharge_cycles;
    double old_simulation_time, new_simulation_time;
    double available_distance, charge_time;

    int accessing_cluster, cluster_element_size, cluster_element;
    int global_iterations;
    bool cluster_flag, cluster_initialize_flag;
    int global_iterations_counter;
    int waitForResult;
    std::string move_base_frame;
    std::string robot_prefix;  /// The prefix for the robot when used in simulation
    std::string robot_name;
    const unsigned char *occupancy_grid_global;
    const unsigned char *occupancy_grid_local;

    std::string csv_file, csv_state_file, log_file;
    std::string log_path;
    std::fstream fs_csv, fs_csv_state, fs;

    int number_of_recharges = 0;

    bool test, occupied_ds;
    ros::Publisher pub_robot_pos, pub_check_vacancy;
    ros::Subscriber sub_vacant_ds, sub_occupied_ds, sub_check_vacancy;

    ros::Subscriber sub_going_charging, sub_going_queue, sub_exploring;
    ros::Subscriber sub_lost_own_auction, sub_won_auction, sub_lost_other_robot_auction;

    enum state_t
    {
        exploring,       // the robot is computing which is the next frontier to be
                         // explored
        going_charging,  // the robot has the right to occupy a DS to recharge
        charging,        // the robot is charging at a DS
        finished,        // the robot has finished the exploration
        fully_charged,   // the robot has recently finished a charging process; notice
                         // that the robot is in this state even if it is not really
                         // fully charged (since just after a couple of seconds after
                         // the end of the recharging process the robot has already
                         // lost some battery energy, since it consumes power even
                         // when it stays still
        stuck,
        in_queue,                                  // the robot is in a queue, waiting for a DS to be vacant
        auctioning,                                // auctioning: the robot has started an auction; notice that if
                                                   // the robot is aprticipating to an auction that it was not
                                                   // started by it, its state is not equal to auctioning!!!
        auctioning_2,
        going_in_queue,                            // the robot is moving near a DS to later put itself in
                                                   // in_queue state
        going_checking_vacancy,                    // the robot is moving near a DS to check if it
                                                   // vacant, so that it can occupy it and start
                                                   // recharging
        checking_vacancy,                          // the robot is currently checking if the DS is vacant,
                                                   // i.e., it is waiting information from the other robots
                                                   // about the state of the DS
        moving_to_frontier_before_going_charging,  // TODO hmm...
        moving_to_frontier,                        // the robot has selected the next frontier to be
                                                   // reached, and it is moving toward it
        leaving_ds                                 // the robot was recharging, but another robot stopped
    };
    state_t robot_state;

    // TODO

    std::vector<std::string> enum_string;

    std::string get_text_for_enum(int enumVal)
    {
        return enum_string[enumVal];
    }

    bool vacant_ds;
    bool need_to_recharge;

    enum state_next_t
    {
        exploring_next,
        going_charging_next,
        going_queue_next,
        fully_charged_next,
        current_state,
        finish_next
    };
    state_next_t robot_state_next;

  private:
    // enum state_t {exploring, going_charging, charging, finished, fully_charged,
    // stuck, in_queue};
    // state_t robot_state;

    ros::Publisher pub_move_base;
    ros::Publisher pub_Point;
    ros::Publisher pub_home_Point;
    ros::Publisher pub_frontiers;

    ros::ServiceClient mm_log_client;

    ros::NodeHandle nh;
    ros::Time time_start;

    // Create a move_base_msgs to define a goal to steer the robot to
    move_base_msgs::MoveBaseActionGoal action_goal_msg;
    move_base_msgs::MoveBaseActionFeedback feedback_msgs;

    geometry_msgs::PointStamped goalPoint;
    geometry_msgs::PointStamped homePoint;

    std::vector<geometry_msgs::PoseStamped> goals;
    tf::Stamped<tf::Pose> robotPose;

    explorationPlanner::ExplorationPlanner *exploration;

    double pose_x, pose_y, pose_angle, prev_pose_x, prev_pose_y, prev_pose_angle;

    double x_val, y_val, home_point_x, home_point_y, target_ds_x, target_ds_y;
    int seq, feedback_value, feedback_succeed_value, rotation_counter, home_point_message, goal_point_message;
    int counter;
    bool recharging;
    bool pioneer;
    int w1, w2, w3, w4;
};

/********
 * MAIN *
 ********/
int main(int argc, char **argv)
{
#ifdef PROFILE
    const char fname[3] = "TS";
    ProfilerStart(fname);
    HeapProfilerStart(fname);
#endif

    /*
     * ROS::init() function needs argc and argv to perform
     * any argument and remapping that is provided by the
     * command line. The third argument is the name of the node
     */
    ros::init(argc, argv, "simple_navigation");

    /*
     * Create instance of Explorer
     */
    tf::TransformListener tf(ros::Duration(10));
    Explorer explorer(tf);

    /*
     * The ros::spin command is needed to wait for any call-back. This could for
     * example be a subscription on another topic. Do this to be able to receive a
     * message.
     */
    boost::thread thr_explore(boost::bind(&Explorer::explore, &explorer));

    /*
     * The following thread is only necessary to log simulation results.
     * Otherwise it produces unused output.
     */
    boost::thread thr_map(boost::bind(&Explorer::map_info, &explorer));

    /* Create thread to periodically publish unexplored frontiers */
    boost::thread thr_frontiers(boost::bind(&Explorer::frontiers, &explorer));
    
    boost::thread thr_safety_checks(boost::bind(&Explorer::safety_checks, &explorer));

    /*
     * FIXME
     * Which rate is required in order not to oversee
     * any callback data (frontiers, negotiation ...)
     */
    while (ros::ok())
    {
        costmap_mutex.lock();
        //ROS_ERROR("main(): lock acquired");
        ROS_INFO("main(): lock acquired");
        ros::spinOnce();
        costmap_mutex.unlock();
        //ROS_ERROR("main(): lock released");
        ROS_INFO("main(): lock released");

        ros::Duration(0.1).sleep();
    }

    /* Terminate threads */
    thr_explore.interrupt();
    thr_map.interrupt();
    thr_explore.join();
    thr_map.join();
// TODO thr_frontiers...

#ifdef PROFILE
    HeapProfilerStop();
    ProfilerStop();
#endif

    return 0;
}

