#include "ros/ros.h"
#include "ExplorationPlanner.h"
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
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include "distance_computer.h"
#include "explorer/ChargingCompleted.h"
#include "explorer/AuctionResult.h"
#include <robot_state/robot_state_management.h>
#include "robot_state/SetRobotState.h"
#include "robot_state/GetRobotState.h"

#ifdef PROFILE
#include "google/profiler.h"
#include "google/heap-profiler.h"
#endif

#define OPERATE_ON_GLOBAL_MAP true
#define OPERATE_WITH_GOAL_BACKOFF true
#define STUCK_COUNTDOWN 1000

#define INCR 1.7
#define OPP_ONLY_TWO_DS false
#define IMM_CHARGE 0
#define DEBUG false
#define TIMEOUT_CHECK_1 3
#define DS_GRAPG_NAVIGATION_ALLOWED true

#define LOG_STATE_TRANSITION true

#define ANTICIPATE_TERMINATION true

#pragma GCC diagnostic ignored "-Wenum-compare"

using namespace robot_state;

bool exploration_finished;

boost::mutex costmap_mutex;
boost::mutex log_mutex;
boost::mutex state_mutex;

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        ros::Duration(1.0).sleep();
}

class Explorer
{
  public:
    // TODO(minor) move in better place
    bool moving_along_path, explorer_ready;
    int ds_path_counter, ds_path_size;
    ros::Publisher pub_robot, pub_wait, pub_finished_exploration, pub_finished_exploration_id;
    ros::Subscriber sub_wait, sub_free_cells_count, sub_discovered_free_cells_count, sub_force_in_queue;
    std::vector<adhoc_communication::MmPoint> complex_path;
    ros::ServiceServer ss_robot_pose, ss_distance_from_robot, ss_distance, ss_reachable_target;
    ros::ServiceClient sc_get_robot_state;
    bool created;
    float queue_distance, min_distance_queue_ds, max_av_distance;
    float stored_robot_x, stored_robot_y;
    float auction_timeout, checking_vacancy_timeout;
    bool already_navigated_DS_graph;
    int free_cells_count, discovered_free_cells_count;
    float percentage;
    int failures_going_to_DS;
    int approximate_success;
    bool going_home, checked_percentage;
    int major_errors, minor_errors;
    bool skip_findFrontiers;
    int retry_recharging_current_ds;
    ros::Timer checking_vacancy_timer;
    bool ds_graph_navigation_allowed;
    double conservative_maximum_available_distance;
    bool moving_to_ds, home_point_set;
    unsigned int retries, retries2, retries3, retries4, retries5, retries6, retries_moving;
    double next_available_distance;
    double moving_time;
    bool received_battery_info;
    ros::Publisher pub_next_ds;
    bool full_battery, frontiers_found;
    double traveled_distance, last_x, last_y;
    bool optima_ds_set;
    bool explorer_count;
    
    ros::ServiceClient set_robot_state_sc, get_robot_state_sc;
    ros::ServiceServer charging_complete_ss;

    /*******************
     * CLASS FUNCTIONS *
     *******************/
    Explorer(tf::TransformListener &tf)  // TODO(minor) put comments (until CREATE LOG PATH)
        : nh("~")
    {
        if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
           ros::console::notifyLoggerLevelsChanged();
        }
        
        rotation_counter = 0;
        accessing_cluster = 0;
        cluster_element_size = 0;
        cluster_flag=false;
        cluster_element=-1;
        cluster_initialize_flag=false;
        global_iterations = 0;
        global_iterations_counter = 0;
        counter_waiting_for_clusters = 0;
        global_costmap_iteration = 0;
        robot_prefix_empty = false;
        robot_id = 0;
        battery_charge=100;
        counter = 0;
        charge_time = 0;
        pose_x = 0;
        pose_y = 0;
        pose_angle = 0;
        prev_pose_x = 0;
        prev_pose_y = 0;
        prev_pose_angle = 0;
        recharge_cycles= 0;
        battery_charge_temp = 100;
        energy_consumption= 0;
        available_distance= 0;
        starting_x = 0, starting_y = 0;
        retries = 0, retries2 = 0, retries3 = 0, retries4 = 0, retries5 = 0, retries6 = 0, retries_moving = 0;
        next_available_distance = -1;
        moving_time = 0;
        traveled_distance = 0;
        last_x = 0, last_y = 0;
    
        moving_along_path = false;
        created = false;
        exploration = NULL;
        explorer_ready = false;
        available_distance = -1;
        already_navigated_DS_graph = false;
        discovered_free_cells_count = 0;
        free_cells_count = 0;
        failures_going_to_DS = 0;
        approximate_success = 0;
        going_home = false, checked_percentage = false;
        major_errors = 0;
        minor_errors = 0;
        last_printed_pose_x = 0, last_printed_pose_y = 0;
        skip_findFrontiers = false;
        retry_recharging_current_ds = 0;
        ds_graph_navigation_allowed = DS_GRAPG_NAVIGATION_ALLOWED;
        conservative_maximum_available_distance = -1;
        moving_to_ds = false;
        home_point_set = false;
        received_battery_info = false;
        full_battery = false;
        frontiers_found = false;
        optima_ds_set = false;
        explorer_count = 0;

        /* Initial robot state */
        robot_state = robot_state::INITIALIZING;

        /* Robot state publishers */
        
        ros::NodeHandle nh2;
        sub_free_cells_count = nh2.subscribe("free_cells_count", 10, &Explorer::free_cells_count_callback, this);
        sub_discovered_free_cells_count = nh2.subscribe("discovered_free_cells_count", 10, &Explorer::discovered_free_cells_count_callback, this);
        
        set_robot_state_sc = nh2.serviceClient<robot_state::SetRobotState>("robot_state/set_robot_state");
        get_robot_state_sc = nh2.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");
        
        /* Robot state subscribers */
        sub_check_vacancy =
            nh.subscribe("adhoc_communication/reply_for_vacancy", 10, &Explorer::reply_for_vacancy_callback,
                         this);  // to receive replies for vacancy checks
        
        sub_wait = nh.subscribe("are_you_ready", 10, &Explorer::wait_for_explorer_callback, this);
        pub_wait = nh.advertise<std_msgs::Empty>("im_ready", 10);
        
        pub_finished_exploration = nh2.advertise<std_msgs::Empty>("finished_exploration", 10);
        pub_finished_exploration_id = nh2.advertise<adhoc_communication::EmRobot>("finished_exploration_id", 10);
        pub_next_ds = nh2.advertise<std_msgs::Empty>("next_ds", 1);

        ros::NodeHandle n;

        /* Load parameters */
        nh.param("frontier_selection", frontier_selection, 1);
        nh.param<double>("local_costmap/resolution", costmap_resolution, 0);
        nh.param("number_unreachable_for_cluster", number_unreachable_frontiers_for_cluster, 3);
        nh.param("recharging", recharging, false);
        nh.param("w1", w1, 1);
        nh.param("w2", w2, 0);
        nh.param("w3", w3, 0);
        nh.param("w4", w4, 0);
        nh.param<float>("queue_distance", queue_distance, 7.0);
        nh.param<float>("min_distance_queue_ds", min_distance_queue_ds, 3.0);
        nh.param<std::string>("move_base_frame", move_base_frame, "map");
        nh.param<int>("wait_for_planner_result", waitForResult, 3);
        nh.param<float>("auction_timeout", auction_timeout, 3);
        nh.param<float>("checking_vacancy_timeout", checking_vacancy_timeout, 3);

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

            ROS_INFO("move_base_frame: %s", move_base_frame.c_str());
            robot_id = atoi(move_base_frame.substr(7, 1).c_str());

            ROS_INFO("Robot name: %s    robot_id: %d", robot_name.c_str(), robot_id);
        }

        createLogFiles();

        ROS_INFO("*********************************************");
        ROS_INFO("******* Initializing Simple Navigation ******");
        ROS_INFO("                                             ");

        costmap2d_global = new costmap_2d::Costmap2DROS("global_costmap", tf);
        costmap2d_global->pause();

        costmap2d_local_size = new costmap_2d::Costmap2DROS("local_costmap", tf);
        costmap2d_local_size->pause();
        costmap2d_global->start();
        costmap2d_local_size->start();

        costmap2d_local = costmap2d_global;

        ROS_INFO("---------------- COSTMAP DONE ---------------");

        visualizeFirstGoal(); 

        ROS_INFO("---------- SET HOME/GOAL POINT DONE ---------");

        initialize();

        ROS_INFO("************* INITIALIZING DONE *************");

        /* Load strings in enum_string vector */
        // TODO currently, the strings must be inserted in teh same order of the enum, which is not very nice...
        fillEnumString();
        
        checking_vacancy_timer = nh.createTimer(ros::Duration(checking_vacancy_timeout), &Explorer::vacancy_callback, this, true, false);

    }

    void explore()
    {   
        
        ROS_INFO("STARTING EXPLORATION");
        // TODO put to sleep also the other nodes
        /*
         * Sleep is required to get the actual a
         * costmap updated with obstacle and inflated
         * obstacle information. This is rquired for the
         * first time explore() is called.
         */
        ROS_DEBUG("Sleeping 5s for costmaps to be updated.");
        ros::Duration(5).sleep();

        rosariaInitialization();

        /* Start taking the time during exploration */
        time_start = ros::Time::now();
        wall_time_start = ros::WallTime::now();

        ros::NodeHandle nh;

        ros::Subscriber sub, sub2, sub3, pose_sub, sub_finish;

        ros::Subscriber sub_new_optimal_ds = nh.subscribe("explorer/new_optimal_ds", 10,
                                                         &Explorer::new_optimal_docking_station_selected_callback, this);

        ros::Subscriber sub_moving_along_path =
            nh.subscribe("moving_along_path", 10, &Explorer::moving_along_path_callback, this);
        
        ros::Publisher pub_path = nh.advertise<std_msgs::String>("error_path", 1);

        /* Subscribe to battery management topic */
        sub = nh.subscribe("battery_state", 10, &Explorer::bat_callback, this);

        /* Subscribe to robot pose to check if robot is stuck */
        pose_sub = nh.subscribe("amcl_pose", 10, &Explorer::poseCallback, this);
        
        sub_finish = nh.subscribe("explorer/finish", 10, &Explorer::finish_callback, this);
        
        exploration->set_auction_timeout(auction_timeout);

        ROS_INFO("********** STARTING EXPLORATION *************");

        /* Start main loop (it loops till the end of the exploration) */
        while (!exploration_finished)
        {       
            std::vector<double> final_goal;
            std::vector<double> backoffGoal;
            std::vector<std::string> robot_str;

            bool navigate_to_goal = false;
            bool negotiation;
            int count = 0;
        
            update_robot_state();
            
            std_msgs::String msg;
            msg.data = ros::package::getPath("multi_robot_analyzer");
            pub_path.publish(msg); //TODO put in better place

            ros::Time time = ros::Time::now();

            if(!frontiers_found) {
                ROS_INFO("****************** FRONTIER DETERMINATION ******************");

                frontier_finder->findFrontiers();
            
            }
            
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
            ros::Duration(10).sleep(); //TODO reduce time?
            
            explorer_ready = true;

            if(!home_point_set) {
                if(exploration->getRobotPose(robotPose)) {
                    home_point_x = robotPose.getOrigin().getX();
                    home_point_y = robotPose.getOrigin().getY();
                }
                else {
                    log_major_error("cannot get robot home position");
                    ROS_INFO("it will be assumed that the robot is starting at (0,0) in is local reference systen");
                    home_point_x = 0;
                    home_point_y = 0;
                }
                home_point_set = true;
            }


            ROS_INFO("****************** ACTING ACCORDING TO STATE ******************");
            
            ROS_INFO("robot_state is %s", get_text_for_enum(robot_state).c_str());

            // TODO(minor) better while loops
            if (robot_state == robot_state::CHARGING)
            {
                ROS_INFO("Waiting for battery to charge...");
                ros::spinOnce();
                ros::Duration(1).sleep();
                update_robot_state();
                continue;
            }
            
            if (robot_state == robot_state::COMPUTING_NEXT_GOAL || robot_state == robot_state::CHARGING_COMPLETED || robot_state == robot_state::LEAVING_DS)
            {
                if(!received_battery_info && next_available_distance <= 0 && conservative_maximum_available_distance <= 0) {
                    ROS_DEBUG("waiting battery info");
                    ros::Duration(3).sleep();
                    ros::spinOnce();
                    continue;
                }
                   
                received_battery_info = true;
                   
                if(robot_state == robot_state::CHARGING_COMPLETED) {
                    ROS_INFO("using max distance");
                    available_distance = conservative_maximum_available_distance;
                } else
                    available_distance = next_available_distance;
                   
                if(robot_state == robot_state::LEAVING_DS || robot_state == robot_state::CHARGING_COMPLETED )
                {
                    //TODO
//                    double distance = -1;
//                    int i = 0;
//                    while(i < 10) {
//                        exploration->distance_from_robot(optimal_ds_x, optimal_ds_y);
//                        i++;
//                        if(distance > 0)
//                            break;
//                        else
//                            ros::Duration(2).sleep();
//                    }
//                    if(distance < 0) {
//                        ROS_INFO("cannot comptue distance between robot and DS: leaving robot where it is");
//                    }
//                    else
//                        if (distance <
//                                min_distance_queue_ds)  // TODO could the DS change meanwhile???
//                                {
//                                    ROS_INFO("Robot is too close to the DS: moving a little bit farther...");
//                                    ROS_DEBUG("distance: %.2f; min_distance_queue_ds: %.2f", distance, min_distance_queue_ds);
                    counter++;
                    move_robot_away(counter);  // TODO(minor) move robot away also if in queue and too close...
//                                    ROS_INFO("Now it is ok...");
//                                }
                        update_robot_state_2(robot_state::CHOOSING_ACTION);
                        continue;
//                    }
                }              
            }

             
            ROS_INFO("****************** START FRONTIER SELECTION ********************");

            /* 7 ... Navigate to frontier that satisfies energy efficient cost
               function with staying-alive path planning */
            if (frontier_selection == 17) //TODO
                ;
            
            else if (frontier_selection == 7 || frontier_selection == 10) {
                bool continue_bool = selectFrontier();
                if(continue_bool)
                    continue;
            }


            ROS_INFO("******************** FRONTIER COORDINATION  ********************");
            
            if(robot_state == robot_state::INITIALIZING) {
                initializingState();
                continue;
            }

            if(robot_state == robot_state::CHARGING_ABORTED || robot_state == robot_state::CHARGING_COMPLETED) {
                update_robot_state_2(robot_state::LEAVING_DS);
            }
            
            if(robot_state == robot_state::COMPUTING_NEXT_GOAL)
            {
                ROS_INFO("continue in state 'robot_state::COMPUTING_NEXT_GOAL'");
                continue;
            }
            
            if (robot_state == robot_state::MOVING_TO_FRONTIER)
                navigate_to_goal = movingToFrontierState();

            // TODO(minor) hmm... here??
            if (robot_state == robot_state::AUCTIONING || robot_state == auctioning_2 || robot_state == auctioning_3 )
                auctioningState();

            // TODO(minor) hmm... here??
            if (robot_state == robot_state::IN_QUEUE)
                inQueueState();

            if (robot_state == robot_state::CHECKING_VACANCY)
                checkingVacancyState();

            if (robot_state == robot_state::GOING_IN_QUEUE || robot_state == robot_state::GOING_CHECKING_VACANCY || robot_state == robot_state::GOING_CHARGING)
                navigate_to_goal = goingState();

            if (navigate_to_goal == true)
            {
                ROS_INFO("navigation to goal succeeded");
                failures_going_to_DS = 0;
                
                if (robot_state == robot_state::GOING_IN_QUEUE)
                    update_robot_state_2(robot_state::IN_QUEUE);
                
                else if(robot_state == robot_state::GOING_CHECKING_VACANCY)
                    update_robot_state_2(robot_state::CHECKING_VACANCY);

                else if (robot_state == robot_state::GOING_CHARGING)
                {
                    ROS_INFO("Reached DS for recharging");
                    update_robot_state_2(robot_state::CHARGING);
                }

                else if (robot_state == robot_state::MOVING_TO_FRONTIER)
                {
                    if (robot_state == robot_state::MOVING_TO_FRONTIER)
                        update_robot_state_2(robot_state::CHOOSING_ACTION);

                    // do not store travelled distance here... the move_robot called at the previous iteration has done it...
//                    ROS_INFO("STORING PATH");
                    //exploration->trajectory_plan_store(
                    //    exploration->visited_frontiers.at(exploration->visited_frontiers.size() - 1).x_coordinate,
                    //    exploration->visited_frontiers.at(exploration->visited_frontiers.size() - 1).y_coordinate);

                    ROS_DEBUG("Storing visited...");
                    exploration->storeVisitedFrontier(final_goal.at(0), final_goal.at(1), final_goal.at(2),
                                                      robot_str.at(0), final_goal.at(3));
                    ROS_DEBUG("Stored Visited frontier");
                }
            }

            /* Robot could not reach goal */
            else
            {                       
                if (robot_state == robot_state::GOING_CHARGING)
                {
                    ROS_ERROR("Robot cannot reach DS at (%.1f, %.1f) for recharging!", optimal_ds_x, optimal_ds_y);
                    ROS_INFO("Robot cannot reach DS at (%.1f, %.1f) for recharging!", optimal_ds_x, optimal_ds_y);
                    failures_going_to_DS++;
                    if(failures_going_to_DS > 5) {
                        log_major_error("tried too many times to reach DS... terminating exploration...");
                        log_stopped();
                    }
                    else {
                        ROS_ERROR("retrying to reach DS...");
                        ROS_INFO("retrying to reach DS...");
                    }
                }

                else if (robot_state == robot_state::MOVING_TO_FRONTIER)
                {
                    if (robot_state == robot_state::MOVING_TO_FRONTIER)
                    {
                        ROS_ERROR("Robot could not reach goal: mark goal as unreachable and explore again");
                        ROS_INFO("Robot could not reach goal: mark goal as unreachable and explore again");
                        update_robot_state_2(robot_state::CHOOSING_ACTION);
                    }
                    else
                        update_robot_state_2(robot_state::GOING_CHARGING);  // TODO(minor) if
                                                               // moving_to_frontier_before_going_charging is not
                                                               // required anymore, this else branch is never executed

                    ROS_DEBUG("Storing unreachable...");
                    exploration->storeUnreachableFrontier(final_goal.at(0), final_goal.at(1), final_goal.at(2),
                                                          robot_str.at(0), final_goal.at(3));
                    ROS_DEBUG("Stored unreachable frontier");
                    ros::Duration(3).sleep();
                }
            }
            
//            exploration->trajectory_plan_store(stored_robot_x, stored_robot_y); //TODO(minor) here? yes it should be ok because even if the robot failes reaching the goal, in the worst case the travelled distance is 0, so it's ok if I add it to the global_travelled distance, and if instead it moved a little, I have to take note of this travelled distance...

            ROS_DEBUG("                                             ");
            ROS_DEBUG("                                             ");
            
            ROS_INFO("DONE EXPLORING");
            
        }
        
        ROS_INFO("out of while loop of explore()");
    }
    
    void update_robot_state_2(int new_state)
    {  // TODO(minor) comments in the update_blabla functions, and lso in the other callbacks
        ROS_INFO_COND(LOG_STATE_TRANSITION, "State transition: %s -> %s", get_text_for_enum(robot_state).c_str(),
                  get_text_for_enum(new_state).c_str());

        previous_state = robot_state;
        robot_state = static_cast<robot_state::robot_state_t>(new_state);

        if(percentage >= 100 && !checked_percentage && robot_state != finished) {
            if(percentage > 100.1) {
                log_major_error("Exploration percentage is higher that 100.0");
                ROS_ERROR("percentage: %.1f", percentage);
            }
            //ROS_INFO("100%% of the environment explored: the robot can conclude its exploration");
            checked_percentage = true;
        }
        
        if(robot_state != finished && (percentage >= 95.0 || (percentage >= 90.0 && ros::Time::now().toSec() > 7200)) && ANTICIPATE_TERMINATION) {
            finalize_exploration();
        }
        
//        if(robot_state_next == finished_next) {
//            ROS_INFO("Have to finish...");
//            //robot_state == auctioning_3;
//            move_home_if_possible();
//            finalize_exploration();
//        }
        
        if(robot_state == robot_state::CHARGING || robot_state == robot_state::IN_QUEUE)
            if(exploration != NULL) {
                ROS_DEBUG("Setting 'use_theta' to false");
                exploration->use_theta = false;
            }
        
        if(robot_state == robot_state::CHARGING_COMPLETED)
            full_battery = true;
            
        if(robot_state == robot_state::MOVING_TO_FRONTIER || robot_state == robot_state::AUCTIONING || robot_state == auctioning_2 || robot_state == auctioning_3 || robot_state == robot_state::IN_QUEUE)
            full_battery = false;

        if(robot_state == robot_state::MOVING_TO_FRONTIER) 
            already_navigated_DS_graph = false;
        
        ros::Duration time = ros::Time::now() - time_start;

        fs_csv_state.open(csv_state_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        fs_csv_state << time << "," << get_text_for_enum(robot_state).c_str() << "," << moving_along_path << "," << ros::Time::now() << "," << ros::WallTime::now() << std::endl;
        fs_csv_state.close();
        
        if(robot_state == stuck && (previous_state == robot_state::AUCTIONING || previous_state == auctioning_2 || robot_state == auctioning_3) )
            log_major_error("stucked after auction!!!");
        if(robot_state == stuck &&  previous_state == robot_state::GOING_CHARGING)
            log_major_error("stuck when going_charging!!!");

        robot_state::SetRobotState set_msg;
        set_msg.request.robot_state = robot_state;
        while(!set_robot_state_sc.call(set_msg))
            ROS_ERROR("call to set robot state failed, retrying...");
            
       if(robot_state == robot_state::MOVING_TO_FRONTIER) {
            ROS_INFO("increasing counter");
            explorer_count++;
       }
        
    }

    void update_robot_state()
    {
        ROS_DEBUG("Updating robot state...");
        
        robot_state::GetRobotState srv;
        while(!get_robot_state_sc.call(srv))
            ROS_INFO("call to get_robot_state failed");

        if(robot_state != srv.response.robot_state) {
            ROS_INFO("robot state changed by another robot");
            update_robot_state_2(srv.response.robot_state);
            return;
        } 
        
        ros::spinOnce();
    }

    void finalize_exploration()
    {
    
        // finished exploration
        update_robot_state_2(finished);
    
        // Indicte end of simulation for this robot
        // When the multi_robot_simulation/multiple_exploration_runs.sh script is
        // run, this kills all processes and starts a new run
        this->indicateSimulationEnd();
    
        exploration->logRemainingFrontiers(log_path + std::string("remaining_frontiers.log"));

        // finish log files
        exploration_has_finished();

        ROS_INFO("Shutting down...");
        shutdown();
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

        //ROS_INFO("iterate_global_costmap(): acquiring");
        print_mutex_info("iterate_global_costmap()", "acquiring");
        costmap_mutex.lock();
        //ROS_INFO("iterate_global_costmap(): lock acquired");
        print_mutex_info("iterate_global_costmap()", "lock");

        exploration->transformToOwnCoordinates_frontiers();
        exploration->transformToOwnCoordinates_visited_frontiers();
        
//        DistanceComputer *dc = new DistanceComputer(costmap2d_global);

        exploration->initialize_planner("exploration planner", costmap2d_global, costmap2d_global, NULL);
        
        exploration->findFrontiers();
        exploration->clearVisitedFrontiers();
        exploration->clearUnreachableFrontiers();
        exploration->clearSeenFrontiers(costmap2d_global);
        
        ROS_INFO("publish_frontier_list");
        exploration->publish_frontier_list();

        costmap_mutex.unlock();
        //ROS_INFO("iterate_global_costmap(): lock released");
        print_mutex_info("iterate_global_costmap()", "unlock");

        // exploration->visualize_Frontiers();

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

        }

        // no valid goal found
        else
        {
            rotation_counter++;
            ROS_INFO("In navigation .... cluster_available: %lu     counter: %d", (long unsigned int)exploration->clusters.size(),
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
        homePoint.point.x = home_point_x;
        homePoint.point.y = home_point_y;

        ros::NodeHandle nh("homePoint");
        pub_home_Point = nh.advertise<geometry_msgs::PointStamped>("homePoint", 100, true);
        pub_home_Point.publish<geometry_msgs::PointStamped>(homePoint);
        
        /*
        visualization_msgs::Marker marker;

        marker.header.frame_id = move_base_frame;
        marker.header.stamp = ros::Time::now();
        marker.header.seq = frontier_seq_number++;
        marker.ns = "my_namespace";
        marker.id = frontier_seq_number;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(10); //TODO //F
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
        */
         
    }

    void reply_for_vacancy_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg)
    {
        state_mutex.lock();

        if(robot_state == robot_state::CHECKING_VACANCY) { //TODO is it possible to received a message about vacancy when not performing vacancy checks? probably yes due to broadcasting
            ROS_INFO("Target DS is (going to be) occupied by robot %d", msg.get()->used_by_robot_id);
            //checking_vacancy_timer.stop(); //TODO it doesn't work here... why???
             //TODO this check should be already in update_robot_state() probably...
            update_robot_state_2(robot_state::GOING_IN_QUEUE);            
        }

        state_mutex.unlock();
    }
    
    void vacancy_callback(const ros::TimerEvent &event) {
        state_mutex.lock();

        ROS_INFO("Timeout for vacancy check");
        if(robot_state == robot_state::CHECKING_VACANCY)
            update_robot_state_2(robot_state::GOING_CHARGING);
        else
            update_robot_state_2(robot_state::GOING_IN_QUEUE);

        state_mutex.unlock();
    }

    void new_optimal_docking_station_selected_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg)
    {
            optima_ds_set = true;
            optimal_ds_x = msg.get()->x;
            optimal_ds_y = msg.get()->y;
            ROS_DEBUG("Storing new optimal DS position, which is placed at (%f, %f)", optimal_ds_x, optimal_ds_y);
    }

    void moving_along_path_callback(const adhoc_communication::MmListOfPoints::ConstPtr &msg)
    {
        ROS_INFO("Moving along path");
        moving_along_path = true;
        ds_path_counter = 0;

        complex_path.clear();
        ds_path_size = msg.get()->positions.size();
        for(int i=0; i < ds_path_size; i++)
            complex_path.push_back(msg.get()->positions[i]);

    }

    void bat_callback(const explorer::battery_state::ConstPtr &msg)
    {
        ROS_DEBUG("Received battery state");
        battery_charge = (int) (msg->soc * 100);
        charge_time = msg->remaining_time_charge;
        next_available_distance = msg->remaining_distance;
//        ROS_INFO("SOC: %d%%; available distance: %.2f; conservative av. distance: %.2f; time: %.2f", battery_charge, next_available_distance, conservative_available_distance(available_distance), msg->remaining_time_run);

        if (battery_charge == 100 && charge_time == 0)
            recharge_cycles++;  // TODO(minor) hmm... soc, charge, ...
        
        conservative_maximum_available_distance = msg->maximum_traveling_distance;

    }
    
    void shutdown() {
        ros::Duration(10).sleep();
        ros::shutdown();
    }
    
    void finish_callback(const std_msgs::Empty &msg) {
        ROS_INFO("finish_callback");
//        robot_state_next = finished_next;
    }
    
    void abort() {
        ROS_ERROR("Exploration is going to be gracefully terminated for this robot...");
        ros::Duration(3).sleep();
        this->indicateSimulationEnd();
        
        shutdown();
        
    }
    
    

    bool robot_pose_callback(fake_network::RobotPositionSrv::Request &req, fake_network::RobotPositionSrv::Response &res)
    {
        
        //tf::Stamped<tf::Pose> robotPose;
        if(!exploration->getRobotPose(robotPose))
            return false;
        res.x = robotPose.getOrigin().getX();
        res.y = robotPose.getOrigin().getY();
        return true;

    }
       
    bool reachable_target_callback(explorer::DistanceFromRobot::Request &req,
                                      explorer::DistanceFromRobot::Response &res)
    {
        res.reachable = exploration->reachable_target(req.x, req.y);
        return true;
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
        
        ROS_DEBUG("x: %.1f; y: %.1f; angle: %.1f", pose_x, pose_y, pose_angle);
        //if(robot_id == 1)
        //    ROS_ERROR("%.2f: %.1f, %.1f", pose_angle, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);
        if(robot_id == 0)
            ; //ROS_ERROR("x: %.1f; y: %.1f", pose_x, pose_y);
        
        print_new_position();
        
        traveled_distance += sqrt( (last_x-pose_x)*(last_x-pose_x) + (last_y-pose_y)*(last_y-pose_y) );
    
        last_x = pose_x;
        last_y = pose_y;
    }
    
    void print_new_position() {
        if( fabs(last_printed_pose_x - pose_x) >= 0.1 && fabs(last_printed_pose_y - pose_y) >= 0.1) {
            ROS_DEBUG("x: %.1f; y: %.1f", pose_x, pose_y);
            last_printed_pose_x = pose_x;
            last_printed_pose_y = pose_y;
        }
    }

    void feedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr &msg)
    {
        feedback_value = msg.get()->status.status;
        feedback_succeed_value = msg.get()->feedback.base_position.pose.orientation.w;
    }
    
    bool robot_is_moving() {
        if(robot_state == robot_state::MOVING_TO_FRONTIER || robot_state == robot_state::GOING_CHARGING || robot_state == robot_state::GOING_CHECKING_VACANCY || robot_state == robot_state::GOING_IN_QUEUE || robot_state == robot_state::LEAVING_DS)
            return true;
        return false;
    
    }
    
    void free_cells_count_callback(const std_msgs::Int32 msg) {
        //ROS_ERROR("YESS!!!");
        free_cells_count = msg.data;
        //ROS_ERROR("received count: %d", free_cells_count);
    }
    
    void discovered_free_cells_count_callback(const std_msgs::Int32 msg) {
        //ROS_ERROR("YESS!!!");
        discovered_free_cells_count = msg.data;
        //ROS_ERROR("received count: %d", discovered_free_cells_count);
    }
    
    void safety_checks() {
    
//        bool already_perfomed_recovery_procedure = false;
        double sleeping_time = 10.0;
        while(exploration == NULL)
            ros::Duration(sleeping_time).sleep();

        int starting_value_moving = TIMEOUT_CHECK_1 * 60; //seconds
//        int starting_value_countdown_2 = 3 * TIMEOUT_CHECK_1 * 60; //seconds
        int starting_value_countdown_2 = 30 * 60; //seconds
        ros::Duration countdown = ros::Duration(starting_value_moving);
        ros::Duration countdown_2 = ros::Duration(starting_value_countdown_2);
        
        float prev_robot_x = 0, prev_robot_y = 0, prev_robot_x_2 = 0, prev_robot_y_2 = 0, stuck_x = 0, stuck_y = 0;
        
        int prints_count = 0;
        
        //tf::Stamped<tf::Pose> robotPose;
//        state_t prev_robot_state = robot_state::CHARGING_COMPLETED;
            
        ros::Time prev_time = ros::Time::now();   
        while(ros::ok() && !exploration_finished) {
        
            if(approximate_success >= 3)
                log_major_error("too many approximated successes");
            
            //ROS_DEBUG("Checking...");
            //if(exploration->getRobotPose(robotPose)) {
            //    robot_x = robotPose.getOrigin().getX();
            //    robot_y = robotPose.getOrigin().getY();
            //}
            
            //IMPORTANT: be careful that a robot could change state while it is stucked, since it may continuosly change between 'moving_to_fonrtier' to 'robot_state::COMPUTING_NEXT_GOAL' to compute and try rearching a new goal!!!
            //if((int) prev_robot_state == (int) robot_state && pose_x == prev_robot_x && pose_y == prev_robot_y) 
            //if( ((int) prev_robot_state == (int) robot_state) && ((int) pose_x == (int) prev_robot_x) && ((int) pose_y == (int) prev_robot_y)) 
            if(robot_is_moving()) {
                if (fabs(pose_x - prev_robot_x) < 0.1 && fabs(pose_y - prev_robot_y) < 0.1 ) 
                { 
                    //if(robot_state == robot_state::MOVING_TO_FRONTIER || robot_state == robot_state::GOING_CHARGING || robot_state == robot_state::GOING_CHECKING_VACANCY) {
                    //if(countdown <= ros::Duration(starting_value_moving - 60 * prints_count))
                    if(countdown < ros::Duration(60))
                    {
                        ROS_ERROR("Countdown to shutdown at %ds...", (int) countdown.toSec() );
                        ROS_DEBUG("Countdown to shutdown at %ds...", (int) countdown.toSec() );
                        //prints_count++;   
                    }
                    //}
                    //else {
                    //    ROS_DEBUG("Countdown to shutdown at %ds...", (int) countdown.toSec() );  
                    //}
                    
                    countdown -= ros::Time::now() - prev_time;
                    
                    
                    if(countdown < ros::Duration(0)) {
                        if(robot_state == robot_state::GOING_CHARGING) {
                            log_minor_error("Force the robot to think that it has reached the target DS");
                            update_robot_state_2(robot_state::CHARGING);
                        }
                        else if(robot_state == robot_state::LEAVING_DS)
                        {
                            log_minor_error("Force the robot to think that it has left the target DS");
                            update_robot_state_2(robot_state::CHOOSING_ACTION);
                        }
                        else if(robot_state == robot_state::GOING_IN_QUEUE) {
                            log_minor_error("Force the robot to think that it reached the queue");
                            update_robot_state_2(robot_state::IN_QUEUE);
                        }
                        else if(robot_state == robot_state::GOING_CHECKING_VACANCY) {
                            log_minor_error("Force the robot to think that it reached the DS to check vavancy");
                            update_robot_state_2(robot_state::CHECKING_VACANCY);
                        }
                        else {
                            if(retries_moving < 3) {
                                log_minor_error("Robot is not moving anymore... retrying");
                                retries_moving++;
                                update_robot_state_2(robot_state::CHOOSING_ACTION);
                            }
                            else {
                                log_major_error("Robot is not moving anymore");
                                //abort();
                                log_stucked();
                            }
                        }
                    }
                }
                
                else {
                    //ROS_DEBUG("Robot is moving");
                    //if(robot_state == robot_state::MOVING_TO_FRONTIER || robot_state == robot_state::GOING_CHARGING || robot_state == robot_state::GOING_CHECKING_VACANCY) //TODO complete
                        countdown = ros::Duration(starting_value_moving);
                    //else
                    //    countdown = ros::Duration(starting_value_standing);
                        
                    prev_robot_x = pose_x;
                    prev_robot_y = pose_y;
    //                prev_robot_state = robot_state;
                    //prints_count = 1;  
                    retries_moving = 0;
                }
            }
            
            else if( robot_state != robot_state::IN_QUEUE && robot_state != robot_state::CHARGING) {
                if( fabs(pose_x - prev_robot_x_2) < 0.1 && fabs(pose_y - prev_robot_y_2) < 0.1 )
                {
                    countdown_2 -= ros::Time::now() - prev_time;
                    
                    if(countdown_2 < ros::Duration(0)) {
                    
                        /*
                        if(!already_perfomed_recovery_procedure) {
                            ROS_ERROR("Trying to recover from stuck...");
                            stuck_x = pose_x;
                            stuck_y = pose_y;
                            geometry_msgs::Twist msg;
                            msg.linear.y = -1;
                            ros::NodeHandle nh;
                            ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
                            pub.publish(msg);
                            already_perfomed_recovery_procedure = true;
                            countdown = ros::Duration(starting_value_moving);
                            countdown_2 = ros::Duration(starting_value_countdown_2);
                        }
                        else {
                        */
                            ROS_ERROR("Robot is not moving from %d minutes!", starting_value_countdown_2 / 60);
                            ROS_INFO("Robot is not moving from %d minutes!", starting_value_countdown_2 / 60);
                            if(percentage > 90)
                                log_minor_error("deadlock / slow execution / waiting for auction result? BUT with percentage>90%!");
                            else
                                log_major_error("deadlock / slow execution / waiting for auction result??? and at <90%!!!");
                                
                            //abort();
                            if(!robot_is_moving())
                                log_major_error("robot has to be stopped even if it is not moving!");
                                
                            log_stopped();
                        //}
                    }
//                    starting_value_countdown_2 - 20*60
                    else if(countdown_2 < ros::Duration(starting_value_countdown_2 - 20*60) && prints_count == 2)
                    {
                        prints_count++;
                        ROS_ERROR("Robot is not moving from 20 minutes!");
                    }
                    else if(countdown_2 < ros::Duration(starting_value_countdown_2 - 10*60) && prints_count == 1)
                    {
                        prints_count++;
                        ROS_ERROR("Robot is not moving from 10 minutes!");
                    } 
                    else if(countdown_2 < ros::Duration(starting_value_countdown_2 -  5*60) && prints_count == 0)
                    {
                        ROS_ERROR("Robot is not moving from 5 minutes!");
                        prints_count++;   
                    }
                }
                else
                {
                    prev_robot_x_2 = pose_x;
                    prev_robot_y_2 = pose_y;
                    countdown_2 = ros::Duration(starting_value_countdown_2);
                    prints_count = 0;
                }
            }

            if( (stuck_x - pose_x) * (stuck_x - pose_x) + (stuck_y - pose_y) * (stuck_y - pose_y) >= 5*5 ) //pose_x and pose_y are in cells, not meters
                ;
                // already_perfomed_recovery_procedure = false;
            
            prev_time = ros::Time::now();
            ros::Duration(sleeping_time).sleep();

        }
        
        ROS_INFO("safety checks have been stopped");
    
    }
    
    void update_distances() {
        while(!explorer_ready)
            ros::Duration(10).sleep();
            
        while(!exploration_finished) {
            exploration->updateDistances(conservative_maximum_available_distance);
            exploration->sendListDssWithEos();
            exploration->available_distance_for_reply = next_available_distance;
            ros::Duration(1).sleep();
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
    int robot_id;
    int frontier_selection, global_costmap_iteration, number_unreachable_frontiers_for_cluster;
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

    std::string csv_file, csv_state_file, log_file, exploration_start_end_log, revocery_log, major_errors_file, lock_file, computation_time_log;
    std::string log_path, original_log_path;
    std::fstream fs_csv, fs_csv_state, fs, fs_exp_se_log, major_errors_fstream, lock_fstream, fs_computation_time;

    int number_of_recharges = 0;

    ros::Publisher pub_check_vacancy;
    ros::Subscriber sub_check_vacancy;

    ros::Subscriber sub_lost_own_auction, sub_won_auction, sub_lost_other_robot_auction;

    robot_state::robot_state_t robot_state, previous_state;

    std::vector<std::string> enum_string;

    std::string get_text_for_enum(int enumVal)
    {
        if((unsigned int)enumVal >= enum_string.size()) {
            log_major_error("segmenv in get_text_for_enum");
            ROS_ERROR("enumVal: %d", enumVal);
            ROS_INFO("enumVal: %d", enumVal);
            return "";
        }
        else
            return enum_string[enumVal];
    }

    enum state_next_t
    {
        exploring_next,
        going_charging_next,
        going_queue_next,
        current_state,
        finished_next
    };
    state_next_t robot_state_next;

  private:

    ros::Publisher pub_move_base;
    ros::Publisher pub_Point;
    ros::Publisher pub_home_Point;
    ros::Publisher pub_frontiers;

    ros::ServiceClient mm_log_client;

    ros::NodeHandle nh;
    ros::Time time_start;
    ros::WallTime wall_time_start;

    // Create a move_base_msgs to define a goal to steer the robot to
    move_base_msgs::MoveBaseActionGoal action_goal_msg;
    move_base_msgs::MoveBaseActionFeedback feedback_msgs;

    geometry_msgs::PointStamped goalPoint;
    geometry_msgs::PointStamped homePoint;

    std::vector<geometry_msgs::PoseStamped> goals;
    tf::Stamped<tf::Pose> robotPose;

    explorationPlanner::ExplorationPlanner *exploration;

    double pose_x, pose_y, pose_angle, prev_pose_x, prev_pose_y, prev_pose_angle, last_printed_pose_x, last_printed_pose_y, starting_x, starting_y;

    double x_val, y_val, home_point_x, home_point_y, optimal_ds_x, optimal_ds_y;
    int feedback_value, feedback_succeed_value, rotation_counter, home_point_message, goal_point_message;
    int counter;
    bool recharging;
    int w1, w2, w3, w4;
};

/********
 * MAIN *
 ********/

void initializingState() {
    ROS_INFO("end initialization");
    update_robot_state_2(robot_state::COMPUTING_NEXT_GOAL);
}

bool movingToFrontierState() {
    ROS_INFO("Navigating to Goal");

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
            ROS_ERROR("Failed to find backoff goal: move to original goal (%.2f,%.2f)",
                      final_goal.at(0), final_goal.at(1)); // which is done later, when the robot noticed that it couldn't reach a goal
            ROS_INFO("Navigate to goal");
            navigate_to_goal = navigate(final_goal);
        }
    }
    return navigate_to_goal;
}

void auctioningState() {
    move_robot_away_before_auctioning();

    ros::Time auction_start_time = ros::Time::now();
    
    if(moving_along_path) {
        log_major_error("'moving_along_path' should be false!!!");
        moving_along_path = false;
    }
    
    while ( (robot_state == robot_state::AUCTIONING || robot_state == auctioning_2 || robot_state == auctioning_3) && 
        ros::Time::now() - auction_start_time < ros::Duration(5*60) )
    {
        ROS_INFO("Auctioning...");
        ROS_INFO("ros::Time::now(): %f", ros::Time::now().toSec());
        ROS_INFO("auction_start_time: %f", auction_start_time.toSec());
        ros::Duration(1).sleep();
        ros::spinOnce();
        update_robot_state();
    }
    
    if(ros::Time::now() - auction_start_time >= ros::Duration(5*60)) {
        log_major_error("auctioning was forced to stop!");
        ROS_INFO("ros::Time::now() - auction_start_time: %f", (ros::Time::now() - auction_start_time).toSec());
        continue;   
    }
    
    ROS_INFO("Auction completed");
}

void inQueueState() {
    if(moving_along_path) {
        // it could be interesting to try to move to the next DS... but when moving toward it the robot could consume a lot of energy, and if when it reaches the next DS it has to go in queue because there are many robots with a lower battery life, it could be "dangerous"
    
    }
    while (robot_state == robot_state::IN_QUEUE)
    {
        ROS_DEBUG("Waiting in queue...");
        ros::Duration(0.1).sleep();  // TODO(minor) are all these sleeps necessary? and do they have the
                                     // corrent sleep time???
        ros::spinOnce();  // TODO(minor) is spin necessary? isn't it called by update_robot_State already?

        if (exploration->distance_from_robot(optimal_ds_x, optimal_ds_y) < min_distance_queue_ds) {
            ROS_INFO("robot too close to DS for being in queue...");
            counter++;
            move_robot_away(counter);
        }
        update_robot_state();
    }
    ROS_DEBUG("No more in queue");
}

void checkingVacancyState() {
    ROS_DEBUG("Start checking for DS vacancy (timeout: %.1fs)", checking_vacancy_timeout);

    ros::Time start_check_time = ros::Time::now();

    int i = 0;
    while (i < 100 && robot_state == robot_state::CHECKING_VACANCY && ((ros::Time::now() - start_check_time) < ros::Duration(checking_vacancy_timeout + 1)) )
    {
        ros::Duration(0.2).sleep();
        ros::spinOnce();
        update_robot_state();
        i++;
    }
    if(i >= 100)
        log_major_error("robot was saved from stucking in robot_state::CHECKING_VACANCY");
    state_mutex.lock();
    ROS_INFO("finished vacancy check");
    if(robot_state == CHECKING_VACANCY)
        update_robot_state_2(robot_state::GOING_CHARGING);
    state_mutex.unlock();
}

bool goingState() {
    if (robot_state == robot_state::GOING_CHECKING_VACANCY)
        ROS_INFO("Approaching ds%d (%f, %f) to check if it is free", -1, optimal_ds_x, optimal_ds_y);
    else if (robot_state == robot_state::GOING_IN_QUEUE)
        ROS_INFO("Travelling to DS to go in queue");
    else
        if(failures_going_to_DS != 0)
            ROS_INFO("Robot can finally prepare itself to recharge");
        else
            ROS_INFO("tying to reach DS");

    counter++;
    moving_to_ds = true;          
    navigate_to_goal = move_robot(counter, optimal_ds_x, optimal_ds_y);
    moving_to_ds = false;
    return navigate_to_goal;
}

void createLogFiles() {
    initLogPath();
    csv_file = log_path + std::string("periodical.log");
    csv_state_file = log_path + std::string("robot_state.log");
    log_file = log_path + std::string("exploration.log");
    exploration_start_end_log = log_path + std::string("exploration_start_end.log");
    major_errors_file = original_log_path + std::string("_errors.log");
    computation_time_log =  log_path + std::string("computation_times.log");
     
    fs_csv_state.open(csv_state_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs_csv_state << "#elapsed_sim_time,robot_state,moving_along_path" << std::endl;
    fs_csv_state.close();
    
    fs_computation_time.open(computation_time_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs_computation_time << "#success,number_of_frontiers,sort_time,selection_time,selection_strategy" << std::endl;
    fs_computation_time.close();
}

void visualizeFirstGoal() {
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

    // transmit three times, since rviz need at least 1 to buffer before
    // visualizing the point
    for (int i = 0; i <= 2; i++)
    {
        visualize_home_point();
    }
}

void initialize() {
    // instantiate the planner
    exploration = new explorationPlanner::ExplorationPlanner(robot_id, robot_prefix_empty, robot_name);

    robot_home_position_x = robotPose.getOrigin().getX();
    robot_home_position_y = robotPose.getOrigin().getY();

    exploration->next_auction_position_x = robotPose.getOrigin().getX();
    exploration->next_auction_position_y = robotPose.getOrigin().getY();

    exploration->storeVisitedFrontier(robot_home_position_x, robot_home_position_y, robot_id, robot_name, -1);
    exploration->storeFrontier(robot_home_position_x, robot_home_position_y, robot_id, robot_name, -1);

    exploration->setRobotConfig(robot_id, robot_home_position_x, robot_home_position_y, move_base_frame);
}

void fillEnumString() {
    enum_string.push_back("INITIALIZING");
    enum_string.push_back("CHOOSING_ACTION");
    enum_string.push_back("COMPUTING_NEXT_GOAL");
    enum_string.push_back("MOVING_TO_FRONTIER");
    enum_string.push_back("GOING_CHECKING_VACANCY");
    enum_string.push_back("CHECKING_VACANCY");
    enum_string.push_back("GOING_CHARGING");
    enum_string.push_back("CHARGING");
    enum_string.push_back("CHARGING_COMPLETED");
    enum_string.push_back("CHARGING_ABORTED");
    enum_string.push_back("LEAVING_DS");
    enum_string.push_back("GOING_IN_QUEUE");
    enum_string.push_back("IN_QUEUE");
    enum_string.push_back("AUCTIONING");
    enum_string.push_back("auctioning_2");
    enum_string.push_back("exploring_for_graph_navigation");
    enum_string.push_back("stopped");
    enum_string.push_back("stuck");
    enum_string.push_back("auctioning_3");
    enum_string.push_back("finished");
}

void rosariaInitialization() {
    geometry_msgs::Twist twi;
   // should be a parameter!! only for testing on Wed/17/9/14 //TODO(minor)
    ros::Publisher twi_publisher = nh.advertise<geometry_msgs::Twist>("/Rosaria/cmd_vel", 1); 
    twi.angular.z = 0.75;
    twi_publisher.publish(twi);
    ros::Duration(5.0).sleep();
    twi_publisher.publish(twi); 
}
