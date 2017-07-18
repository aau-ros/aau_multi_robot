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

//#define PROFILE

#ifdef PROFILE
#include "google/profiler.h"
#include "google/heap-profiler.h"
#endif

#define OPERATE_ON_GLOBAL_MAP true
#define OPERATE_WITH_GOAL_BACKOFF false
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
#define TIMEOUT_CHECK_1 3
#define DS_GRAPG_NAVIGATION_ALLOWED true
#define COEFF_A 0.9
#define COEFF_B 0.2

#define LOG_STATE_TRANSITION true

bool exploration_finished;

boost::mutex costmap_mutex;
boost::mutex log_mutex;

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
    ros::Publisher pub_robot, pub_wait, pub_finished_exploration, pub_finished_exploration_id;
    ros::Subscriber sub_wait, sub_free_cells_count, sub_discovered_free_cells_count, sub_force_in_queue;
    int path[2][2];
    std::vector<adhoc_communication::MmPoint> complex_path;
    ros::ServiceServer ss_robot_pose, ss_distance_from_robot, ss_distance, ss_reachable_target;
    ros::ServiceClient sc_get_robot_state;
    bool created;
    float queue_distance, max_av_distance;
    float safety_coeff, min_distance_queue_ds;
    float stored_robot_x, stored_robot_y;
    float auction_timeout, checking_vacancy_timeout;
    bool already_navigated_DS_graph;
    int explorations;
    int free_cells_count, discovered_free_cells_count;
    float percentage;
    int failures_going_to_DS;
    int approximate_success;
    bool going_home, checked_percentage;
    int major_errors, minor_errors;
    bool skip_findFrontiers;
    int retry_recharging_current_ds;
    int increase;
    ros::Timer checking_vacancy_timer;
    bool ds_graph_navigation_allowed;
    double conservative_maximum_available_distance;
    bool moving_to_ds, home_point_set;
    float coeff_a, coeff_b;
    unsigned int retries, retries2, retries3, retries4, retries5, retries6, retries_moving;
    double next_available_distance;
    double moving_time;
    bool received_battery_info;
    ros::Publisher pub_next_ds;
    bool full_battery;

    /*******************
     * CLASS FUNCTIONS *
     *******************/
    Explorer(tf::TransformListener &tf)  // TODO(minor) put comments (until CREATE LOG PATH)
        : nh("~")
    {
        if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
           ros::console::notifyLoggerLevelsChanged();
        }
        
//        ros::NodeHandle nh("~");
        rotation_counter = 0;
        number_of_robots = 1;
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
        already_navigated_DS_graph = false;
        need_to_recharge = false;
        explorations = 0;
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
        increase = 0;
        ds_graph_navigation_allowed = DS_GRAPG_NAVIGATION_ALLOWED;
        conservative_maximum_available_distance = -1;
        moving_to_ds = false;
        home_point_set = false;
        coeff_a = COEFF_A;
        coeff_b = COEFF_B;
        received_battery_info = false;
        full_battery = false;

        /* Initial robot state */
        robot_state = exploring;  // TODO(minor) what if instead it is not fully charged?
//        robot_state = fully_charged; //fully_charged is then the robot is still occupying the DS
        robot_state_next = current_state;

        /* Robot state publishers */
        //pub_check_vacancy = nh.advertise<std_msgs::Empty>("check_vacancy", 1);  // to publish vacancy check requests
        
        ros::NodeHandle nh2;
        sub_free_cells_count = nh2.subscribe("free_cells_count", 10, &Explorer::free_cells_count_callback, this);
        sub_discovered_free_cells_count = nh2.subscribe("discovered_free_cells_count", 10, &Explorer::discovered_free_cells_count_callback, this);
        
        //ROS_ERROR("%s", sub_free_cells_count.getTopic().c_str());

        /* Robot state subscribers */

        sub_check_vacancy =
            nh.subscribe("adhoc_communication/reply_for_vacancy", 10, &Explorer::reply_for_vacancy_callback,
                         this);  // to receive replies for vacancy checks

        // TODO(minor) improve this
        sub_lost_own_auction = nh.subscribe("lost_own_auction", 10, &Explorer::lost_own_auction_callback,
                                            this);  // to know when a robot lost its own auction
        sub_won_auction =
            nh.subscribe("won_auction", 1, &Explorer::won_callback, this);  // to know when a robot won an auction
        sub_lost_other_robot_auction = nh.subscribe("lost_other_robot_auction", 10, &Explorer::lost_other_robot_callback,
                                                    this);  // to know when a robot lost another robot auction

        pub_robot = nh.advertise<adhoc_communication::EmRobot>("robot", 10);  // to publish robot state updates
        
        sub_wait = nh.subscribe("are_you_ready", 10, &Explorer::wait_for_explorer_callback, this);
        pub_wait = nh.advertise<std_msgs::Empty>("im_ready", 10);
        
        sub_force_in_queue = nh.subscribe("force_in_queue", 10, &Explorer::force_in_queue_callback, this);
        
        pub_finished_exploration = nh2.advertise<std_msgs::Empty>("finished_exploration", 10);
        pub_finished_exploration_id = nh2.advertise<adhoc_communication::EmRobot>("finished_exploration_id", 10);
        pub_next_ds = nh2.advertise<std_msgs::Empty>("next_ds", 1);

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
        nh.param<float>("auction_timeout", auction_timeout, 3);
        nh.param<float>("checking_vacancy_timeout", checking_vacancy_timeout, 3);

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

            ROS_INFO("move_base_frame: %s", move_base_frame.c_str());
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
        exploration_start_end_log = log_path + std::string("exploration_start_end.log");
        major_errors_file = original_log_path + std::string("_errors.log");
        computation_time_log =  log_path + std::string("computation_times.log");
         
        fs_csv_state.open(csv_state_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        fs_csv_state << "#time,robot_state,moving_along_path" << std::endl;
        fs_csv_state.close();
        
        fs_computation_time.open(computation_time_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        fs_computation_time << "#success,number_of_frontiers,sort_time,selection_time,selection_strategy" << std::endl;
        fs_computation_time.close();


        ROS_INFO_NAMED("start", "*********************************************");
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
        enum_string.push_back("dead");
        enum_string.push_back("moving_away_from_ds");
        enum_string.push_back("auctioning_3");
        enum_string.push_back("stopped");
        enum_string.push_back("exploring_for_graph_navigation");
        
        checking_vacancy_timer = nh.createTimer(ros::Duration(checking_vacancy_timeout), &Explorer::vacancy_callback, this, true, false);

    }

    void explore()  // TODO(minor) comments
    {   
        
        ROS_INFO("STARTING EXPLORATION");
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
        wall_time_start = ros::WallTime::now();

        ros::NodeHandle nh;

        ros::Subscriber sub, sub2, sub3, pose_sub, sub_finish;

        ros::Subscriber my_sub =
            nh.subscribe("charging_completed", 10, &Explorer::battery_charging_completed_callback, this);

        ros::Subscriber sub_new_optimal_ds = nh.subscribe("explorer/new_optimal_ds", 10,
                                                         &Explorer::new_optimal_docking_station_selected_callback, this);

        ros::Subscriber sub_moving_along_path =
            nh.subscribe("moving_along_path", 10, &Explorer::moving_along_path_callback, this);
            
        //ROS_ERROR("%s", pub_next_ds.getTopic().c_str());

        ros::Publisher pub_occupied_ds = nh.advertise<std_msgs::Empty>("occupied_ds", 1);
        
        ros::Publisher pub_path = nh.advertise<std_msgs::String>("error_path", 1);
        
        


        /* Subscribe to battery management topic */
        sub = nh.subscribe("battery_state", 10, &Explorer::bat_callback, this);

        /* Subscribe to robot pose to check if robot is stuck */
        pose_sub = nh.subscribe("amcl_pose", 10, &Explorer::poseCallback, this);
        
        sub_finish = nh.subscribe("explorer/finish", 10, &Explorer::finish_callback, this);
        
        exploration->set_auction_timeout(auction_timeout);

        ROS_INFO("STARTING EXPLORATION");
        
        ros::Time start_time = ros::Time::now();

        /* Start main loop (it loops till the end of the exploration) */
        while (!exploration_finished)
        {       
        
            std::vector<double> final_goal;
            std::vector<double> backoffGoal;
            std::vector<std::string> robot_str;

            bool navigate_to_goal = false;
            bool negotiation;
            int count = 0;
        
//            if(robot_id == 0) {
//                finalize_exploration();
//                continue;
//            }
        
            /* Update robot state */
            update_robot_state();
            
            std_msgs::String msg;
            msg.data = ros::package::getPath("multi_robot_analyzer");
            pub_path.publish(msg); //TODO put in better place

            ros::Time time_2 = ros::Time::now();
            ros::Time time = ros::Time::now();
            if(robot_state != going_checking_vacancy && robot_state != checking_vacancy && robot_state != charging && robot_state != going_charging && robot_state != leaving_ds) {
                
                /**************************
                 * FRONTIER DETERMINATION *
                 **************************/

                ROS_INFO("****************** FRONTIER DETERMINATION ******************");

                fs_exp_se_log.open(exploration_start_end_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
                fs_exp_se_log << "0" << ": " << "Start new explore() loop iteration" << std::endl;
                fs_exp_se_log.close();

                /*
                * Use mutex to lock the critical section (access to the costmap)
                * since rosspin tries to update the costmap continuously
                */
                ROS_DEBUG("COSTMAP STUFF");
                print_mutex_info("explore()", "acquiring");
                costmap_mutex.lock();
                ROS_DEBUG("COSTMAP STUFF, lock aquired");
                print_mutex_info("explore()", "lock");

                exploration->transformToOwnCoordinates_frontiers();
                exploration->transformToOwnCoordinates_visited_frontiers();

                //ROS_ERROR("initialize planner");
                exploration->initialize_planner("exploration planner", costmap2d_local, costmap2d_global);
                //ROS_ERROR("planner initialized");
             
        //                if(skip_findFrontiers) {
        //                    ROS_INFO("skipping findFrontiers");
        //                } else {
                fs_exp_se_log.open(exploration_start_end_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
                fs_exp_se_log << "0" << ": " << "Find frontiers" << std::endl;
                fs_exp_se_log.close();
                    
                exploration->transformToOwnCoordinates_frontiers();
                exploration->transformToOwnCoordinates_visited_frontiers();
                  
                exploration->findFrontiers();
                exploration->clearVisitedFrontiers();
                exploration->clearUnreachableFrontiers();
                exploration->clearSeenFrontiers(costmap2d_global);
                
        //                }   
        //            fs_exp_se_log.open(exploration_start_end_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        //            fs_exp_se_log << ros::Time::now() - time << std::endl; //<< ": " << "Clear visisited/unreachable/seen frontiers" << std::endl;
        //            fs_exp_se_log.close();

        //                if(skip_findFrontiers)
//                    exploration->clearVisitedFrontiers();
//                exploration->clearUnreachableFrontiers(); //should remove frontiers that are marked as unreachable from 'frontiers' vector
        //                if(skip_findFrontiers)
//                    exploration->clearSeenFrontiers(costmap2d_global);

                costmap_mutex.unlock();
                print_mutex_info("explore()", "unlock");
                ROS_DEBUG("COSTMAP STUFF, lock released");
                
                //tf::Stamped<tf::Pose> robotPose;
                
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
//                ros::Duration(2).sleep();
                ros::Duration(10).sleep();
                
                explorer_ready = true;
            
            }

            

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
            
            store_current_position();
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
            
            // if (robot_state == exploring || robot_state == fully_charged)
            if (robot_state == exploring || robot_state == fully_charged || robot_state == leaving_ds)
            {
                if(!received_battery_info && next_available_distance <= 0) {
                    ROS_DEBUG("waiting battery info");
                    ros::Duration(3).sleep();
                    ros::spinOnce();
                    continue;
                   }
                   
                received_battery_info = true;
                   
                if(conservative_maximum_available_distance < 0 || conservative_maximum_available_distance < conservative_available_distance(next_available_distance)) {
                    ROS_INFO("setting maximum distance");
//                    conservative_maximum_available_distance = conservative_available_distance(next_available_distance); 
                    conservative_maximum_available_distance = conservative_available_distance(next_available_distance);
                }
                   
                if(robot_state == fully_charged) {
                    ROS_INFO("using max distance");
                    available_distance = conservative_maximum_available_distance;
                } else
                    available_distance = next_available_distance;
                   
                if(robot_state == leaving_ds || robot_state == fully_charged )
//                if(robot_state == leaving_ds)
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
                                    //update_robot_state_2(moving_away_from_ds);
//                                    fs_csv_state.open(csv_state_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//                                    fs_csv_state << time << "," << "moving_away_from_ds" << std::endl; //TODO make real state
//                                    fs_csv_state.close();
                    counter++;
                    move_robot_away(counter);  // TODO(minor) move robot away also if in queue and too close...
//                                    ROS_INFO("Now it is ok...");
//                                }
//                    if(robot_state == leaving_ds) { //this is because at the moment in the code fully_charged in considered a state of computation with the idea that the computation is performed with full battery life, differently from 'exploring' state, and this information is used sometimes, so we should modifying the transition function to be "charging -> leaving_ds -> exploring_fully_charged | exploring_not_fully_charged"
                        update_robot_state_2(exploring);
                        continue;
//                    }
                }
                
                
                    
                //this is done because the robot, actually , can never use the full battery to move, since it has to perform some pre-processing operations with the costmap before selecting a frontier, and so meanwhile the battery life is decreased; so, we store the first available_distance that we have when performing the first frontier selection

                    
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
                * 4 ... Navigate to random frontier
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
                * 5 ... (missing description)
                * 6 ... (missing description)
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
                                for (unsigned int i = 0; i < exploration->clusters.size(); i++)
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
                else if (frontier_selection == 17) //TODO
                    ;
                
                else if (frontier_selection == 7 || frontier_selection == 10)
                {

                    
                    if(moving_along_path) {
                        ROS_INFO("moving along DS path");
//                        if(OPP_ONLY_TWO_DS)
//                            if (ds_path_counter < 2)
//                            {
//                                optimal_ds_x = path[ds_path_counter][0];
//                                optimal_ds_y = path[ds_path_counter][1];
//                                ds_path_counter++;
//                                update_robot_state_2(going_checking_vacancy);
//                            }
//                            else
//                            {
//                                moving_along_path = false;
//                                update_robot_state_2(leaving_ds);
//                            }
//                        else {
                            if(ds_path_counter < ds_path_size - 1)
                            {
                                ros::spinOnce();
                                ROS_INFO("Trying to reach next DS");
                                ROS_INFO("ds_path_counter: %d; ds_path_size: %d", ds_path_counter, ds_path_size);
                                //double next_ds_x = path[ds_path_counter+1][0];
                                //double next_ds_y = path[ds_path_counter+1][0];
                                double next_ds_x = complex_path[ds_path_counter+1].x;
                                double next_ds_y = complex_path[ds_path_counter+1].y;
                                double dist = -1;
                                ROS_INFO("Compute distance");
                                for(int i=0; i<5 && dist < 0; i++) {
                                    dist = exploration->distance_from_robot(next_ds_x, next_ds_y); //TODO(minor) very bad way to check... -> parameter...
                                    if(dist<0)
                                        ros::Duration(1).sleep();
                                }
                                ROS_INFO("Distance computed");
                                
                                if(dist < 0) {
                                    log_major_error("unable to compute distance to reach next DS!!!");
                                    if(retry_recharging_current_ds < 3) {
                                        ROS_INFO("unable to compute distance");
                                        ROS_ERROR("reauction for current one");
                                        ROS_INFO("reauction for current one");
                                        counter++;
                                        move_robot_away(counter);
                                        update_robot_state_2(auctioning);
                                        retry_recharging_current_ds++;
                                    }
                                    else
                                        log_major_error("finished because cannot reach next ds in path");
                                        update_robot_state_2(stopped);
                                        finalize_exploration();
                                }

//                                else if(dist > available_distance * safety_coeff) 
                                else if(dist > conservative_available_distance(available_distance) )  {
                                    //robot cannot reach next next DS, it must recharge at current one
//                                    if(robot_state == fully_charged)
                                    if(full_battery)
                                    {
                                        log_major_error("ERROR WITH DS GRAPH");
                                        
                                        // we force to move to next ds
                                        retry_recharging_current_ds = 0;
                                        ds_path_counter++;
                                        optimal_ds_x = complex_path[ds_path_counter].x; 
                                        optimal_ds_y = complex_path[ds_path_counter].y; 
    //                                    double world_x, world_y;
    //                                    map_to_world(optimal_ds_x, optimal_ds_y, &world_x, &world_y); //TODO to be implemented
    //                                    ROS_INFO("Going to next DS in path, which is at (%f, %f)", world_x, world_y);
                                        ROS_INFO("Going to next DS in path, which is at (%f, %f)", optimal_ds_x, optimal_ds_y);
                                        std_msgs::Empty msg;
                                        pub_next_ds.publish(msg);
                                        update_robot_state_2(going_checking_vacancy); //TODO(minor) maybe it should start an auction before, but in that case we must check that it is not too close to the last optimal_ds (in fact optimal_ds is the next one)
                                        
                                    }
                                    else {
                                        ROS_INFO("Cannot reach next DS on the path: reauction for current one");
                                        counter++;
                                        move_robot_away(counter);
                                        update_robot_state_2(auctioning);
                                    }
                                } else {
                                    retry_recharging_current_ds = 0;
                                    ds_path_counter++;
                                    optimal_ds_x = complex_path[ds_path_counter].x; 
                                    optimal_ds_y = complex_path[ds_path_counter].y; 
//                                    double world_x, world_y;
//                                    map_to_world(optimal_ds_x, optimal_ds_y, &world_x, &world_y); //TODO to be implemented
//                                    ROS_INFO("Going to next DS in path, which is at (%f, %f)", world_x, world_y);
                                    ROS_INFO("Going to next DS in path, which is at (%f, %f)", optimal_ds_x, optimal_ds_y);
                                    std_msgs::Empty msg;
                                    pub_next_ds.publish(msg);
                                    update_robot_state_2(going_checking_vacancy); //TODO(minor) maybe it should start an auction before, but in that case we must check that it is not too close to the last optimal_ds (in fact optimal_ds is the next one)
                                }
                                
                                continue;
                                
                            }
                            else 
                            {
                                ROS_INFO("Finished path traversal");
                                moving_along_path = false;
                                std_msgs::Empty msg;
                                pub_next_ds.publish(msg);
                                
                                if(going_home) {
                                    //move_home_if_possible();
                                    move_home();
                                } else
                                    continue;
                                
                            }
//                        }
                    }


                    {
                        // TODO(minor) do those sorting works correclty?
                        /* Sort frontiers, firstly from nearest to farthest and then by
                         * efficiency */
                        ROS_INFO("SORTING FRONTIERS...");
                        
//                        fs_exp_se_log.open(exploration_start_end_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//                        fs_exp_se_log << ros::Time::now() - time << ": " << "Sort (and possibly cluster) frontiers with sort()" << std::endl;
//                        fs_exp_se_log.close();
                        
                        //exploration->sort(2);
                        //exploration->sort(3);
                        //exploration->clusterFrontiers();
                        //exploration->sort(4);
                        
//                        fs_exp_se_log.open(exploration_start_end_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//                        fs_exp_se_log << ros::Time::now() - time << ": " << "Sort frontiers with sort_cost()" << std::endl;
//                        fs_exp_se_log.close();
                        
                        //exploration->sort_cost_with_approach(battery_charge > 50, w1, w2, w3, w4);

                        /* Look for a frontier as goal */
                        ROS_INFO("DETERMINE GOAL...");
                        
                        fs_exp_se_log.open(exploration_start_end_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
                        fs_exp_se_log << ros::Time::now() - time << ": " << "Compute goal" << std::endl;
                        fs_exp_se_log.close();
                        
                        // goal_determined = exploration->determine_goal_staying_alive(1, 2,
                        // available_distance, &final_goal, count, &robot_str, -1);
                        //ROS_ERROR("available_distance: %f", available_distance);
//                        if(DEBUG)
//                            goal_determined = exploration->determine_goal_staying_alive_2(
//                                1, 2, available_distance * SAFETY_COEFF +
//                                          available_distance * SAFETY_COEFF * INCR * number_of_recharges,
//                                &final_goal, count, &robot_str, -1);
//                        else
                            //goal_determined = exploration->determine_goal_staying_alive_2(1, 2, available_distance, &final_goal, count, &robot_str, -1);
 
                        ros::spinOnce(); // to update available_distance

                        if(robot_state != fully_charged)
                            available_distance = next_available_distance;    
                        
//                        if(conservative_available_distance(available_distance) <= 0)
//                            goal_determined = false;
//                        else {
//                            goal_determined = exploration->my_determine_goal_staying_alive(1, 2, available_distance, &final_goal, count, &robot_str, -1, battery_charge > 50, w1, w2, w3, w4);
                            print_mutex_info("explore()", "acquiring");
                            costmap_mutex.lock();
                            print_mutex_info("explore()", "lock");
                            
                            if(exploration->updateRobotPose()) {
                                exploration->updateOptimalDs();
                                goal_determined = exploration->my_determine_goal_staying_alive(1, 2, conservative_available_distance(available_distance), &final_goal, count, &robot_str, -1, battery_charge > 50, w1, w2, w3, w4);
                            }
                            else {
                                log_major_error("robot pose not updated");
                                goal_determined = false;
                            }
//                        }
                        
                        ROS_INFO("GOAL DETERMINED: %s; counter: %d", (goal_determined ? "yes" : "no"), count);
                        
                        fs_exp_se_log.open(exploration_start_end_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
                        fs_exp_se_log << ros::Time::now() - time << ": " << "Finished" << std::endl;
                        fs_exp_se_log.close();
                        
                        fs_computation_time.open(computation_time_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
                        fs_computation_time << exploration->frontier_selected << "," << exploration->number_of_frontiers << "," << exploration->sort_time << "," << exploration->selection_time << std::endl;
                        fs_computation_time.close();
                        
                        ros::Duration d = ros::Time::now() - time_2;
                        if(d > ros::Duration(5 * 60)) {
                            log_minor_error("very slow...");
                        }
                        
                        if(DEBUG && IMM_CHARGE && number_of_recharges == 0 ) {
                            goal_determined  = false;
                            update_robot_state_2(exploring);
                            ros::Duration(5).sleep();
                        }
                        
//                        fs_exp_se_log.open(exploration_start_end_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//                        fs_exp_se_log << std::endl;
//                        fs_exp_se_log.close();

                        /* Check if the robot has found a reachable frontier */
                        if (goal_determined == true)
                        {
                        
                            //this can be true if we are at the end of the DS path (i.e., we have reached the last DS and we have 
//                            if(moving_along_path) {
//                                ROS_INFO("Finished path traversal");
//                                moving_along_path = false;
//                                std_msgs::Empty msg;
//                                pub_next_ds.publish(msg);
//                            }
                        
                            /* The robot has found a reachable frontier: it can move toward it */
                            
                            //update_robot_state_2(coordinated_exploration); //TODO
                            //ROS_ERROR("STARTING NEGOTIATION");
                            
                            //ROS_ERROR("start frontier negotiation!");
//                            exploration->my_negotiate();
//                     
//                            for(int i = 0; i < auction_timeout/0.1; i++) {
//                                ros::Duration(0.1).sleep();
//                                ros::spinOnce();
//                            }
                            
                            //ROS_ERROR("End of negotiation");
                            //ros::spinOnce();
                            
//                            if(exploration->winner_of_auction)
//                            {
                                explorations++;
//                                ROS_ERROR("%d", explorations);
//                                if(explorations == 3 && robot_id == 0) {
//                                    move_robot(counter, 0, 12);    
//                                    update_robot_state_2(leaving_ds);
//                                    continue;
//                                }
//                                if( ((explorations == 3 || explorations == 4)&& robot_id == 0) || (explorations == 4 && robot_id == 1) ) {
//                                    ROS_ERROR("auctioning");
//                                    update_robot_state_2(auctioning);    
//                                }
//                                else {
//                                    ROS_ERROR("%d", explorations);
                                    update_robot_state_2(moving_to_frontier);
                                    retries3 = 0;
                                    retries5 = 0;
//                                }
                                    
                                //exploration->clean_frontiers_under_auction();
                                
//                            } 
//                            else {
//                                ROS_INFO("lost auction, ");
//                                update_robot_state_2(exploring); //TODO(minor) not very good, i could be already in explorer (i could be in fully_charged, ...)...
//                                continue;
//                            }
                            
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
                            
                            ROS_DEBUG("reset retries counter");
                            retries = 0;
                        }

                        else
                        {
                        
//                            if(exploration->recomputeGoal() && retries < 7) { //TODO(IMPORTANT)
//                                ROS_ERROR("Goal not found due to some computation failure, trying to recompute goal...");
//                                ROS_INFO("Goal not found due to some computation failure, trying to recompute goal...");
////                                ROS_ERROR("Goal not found due to some computation failure, start auction...");
////                                ROS_INFO("Goal not found due to some computation failure, start auction...");
//                                retries++;
//                                ROS_DEBUG("%d", retries);
//                                
//                                ros::Duration(3).sleep();
//                                costmap_mutex.unlock();
//                                print_mutex_info("explore()", "unlock");
////                                update_robot_state_2(auctioning);
//                                continue;                             
//                            }
                            ROS_INFO("goal not determined: selecting next action...");
                            
//                            if(moving_along_path) {
//                                ROS_INFO("moving_along_path: reauctiong for current DS (the last of the path)");
//                                counter++;
//                                move_robot_away(counter);
//                                update_robot_state_2(auctioning);
//                                std_msgs::Empty msg;
//                                pub_next_ds.publish(msg);
//                            }
                            
                            if(retries >= 4)
                                log_major_error("too many retries, this shouldn't happend");
                            
                            //if the robot is not fully charged, recharge it, since the checks to detect if there is a reachable frontier can be very computational expensive
//                            if(robot_state != fully_charged) {
//                                update_robot_state_2(auctioning);
//                                costmap_mutex.unlock();
//                                print_mutex_info("explore()", "unlock");
//                                continue;
//                            } 
                            
                            //TODO we could think aabout moving this part in docking.cpp.. but then we have to be sure that explorer won't continue to think that there are reachable frontiers in another part of the enviroment while energy_mgmt will publish the path for another
//                            if(retries2 < 4 && retries3 < 5 && retries5 < 10) {
                            if(retries2 < 1 && retries3 < 5 && retries5 < 10) {
//                                retries5++;
                                bool error = false;
                                ros::spinOnce(); //to udpate available_distance
                                if( !exploration->existFrontiers() ) {
                                    ROS_INFO("No more frontiers: moving home...");
                                    move_home_if_possible();
                                
                                //TODO use 0.99 as coefficient?
                                } 
                                else
                                {
                                    if(exploration->discovered_new_frontier)
                                        retries6 = 0;
                                    else
                                        retries6++;
                                    ROS_INFO("retries6: %d", retries6);
                                    if(retries6 >= 3) {
                                        log_major_error("tried too many times to navigate graph: retries6 >= 3");
                                        move_home_if_possible();
                                    }
                                    else
                                    {
                                        exploration->discovered_new_frontier = false;

                                        if( exploration->existFrontiersReachableWithFullBattery(conservative_maximum_available_distance, &error) ) {
                                            ROS_INFO("There are still frontiers that can be reached from the current DS: start auction for this DS...");
                                            counter++;
                                            move_robot_away(counter);
                                            update_robot_state_2(auctioning);
                                            retries4 = 0;
                                        }
                                        else {
                                        
                                            update_robot_state_2(exploring_for_graph_navigation);
                                            ros::Duration(1).sleep();
                                        
                                            if( ds_graph_navigation_allowed && exploration->existReachableFrontiersWithDsGraphNavigation(0.999*conservative_maximum_available_distance, &error) ) {
                                                ROS_INFO("There are frontiers that can be reached from other DSs: start moving along DS graph...");
                                                
    //                                            std_msgs::Empty msg;
    //                                            pub_next_ds.publish(msg);
    //                                            ros::Duration(3).sleep();
                                                
                                                int result = -1;
                                                exploration->compute_and_publish_ds_path(conservative_maximum_available_distance, &result);
                                                if(result == 0) //TODO very very orrible idea, using result...
                                                {
                                                    ROS_INFO("path successfully found");
                                                    
                                                    counter++;
                                                    move_robot_away(counter);
                                                    update_robot_state_2(auctioning_2);
                                                    retries2 = 0;
                                                    retries4 = 0;
                                                }
                                                else {
                                                    retries2++;
                                                    
                                                    if(result == 1)
                                                        log_major_error("No DS with EOs was found");
                                                    else if(result == 2)
                                                        log_major_error("impossible, no closest ds found...");
                                                    else if(result == 3) {
                                                        log_minor_error("closest_ds->id == min_ds->id, this should not happen...");
                                                        counter++;
                                                        move_robot_away(counter);
                                                        update_robot_state_2(auctioning_2);
                                                        retries3++;
                                                    }
                                                    else
                                                        log_major_error("invalid result value");
                                                }
                                            }
                                            else {
                                                ROS_DEBUG("errors: %s", (error ? "yes" : "no") );
                                                if(error) {
                                                        ROS_ERROR("Failure in checking if reachable frontiers still exists: retrying...");
                                                        ROS_INFO("Failure in checking if reachable frontiers still exists: retrying...");
                                                        ros::Duration(3).sleep();
                                                        retries2++;
                                                        costmap_mutex.unlock();
                                                        print_mutex_info("explore()", "unlock");
                                                        update_robot_state_2(exploring);
                                                        continue;
                                                }
                                                else {
                                                    retries4++;
                                                    if(retries4 < 3) {
                                                        ROS_INFO("retrying to search if one of the remaining frontiers is reachable");
                                                        costmap_mutex.unlock();
                                                        print_mutex_info("explore()", "unlock");
                                                        update_robot_state_2(exploring);
                                                        continue;
                                                    }
                                                    else
                                                    {
                                                        log_minor_error("There are still unvisited frontiers, but the robot cannot reach them even with full battery: exploration can be concluded; robot will go home...");
                                                        move_home_if_possible();
                                                    }
                                                }
                                            }
                                        }
                                     }
                                 }
                            }
                            else {
                                if(retries2 >= 4)
                                    log_major_error("tried too many times to navigate graph: retries2 >= 4");
                                else if(retries3 >=5)
                                    log_major_error("tried too many times to navigate graph: retries3 >= 5");
                                else
                                    log_major_error("tried too many times to navigate graph: retries5 >= 10");
                                move_home_if_possible();
                            }
                                
//                                if(!ds_graph_navigation_allowed)
//                                    ROS_INFO("There are no more frontiers that can be reached from the current DS and Ds graph navigation is not allowed: start moving along DS graph...");
                                
//                            else if (robot_state == fully_charged)
//                            {
//                                /* The robot wasn't able to find a reachable frontier even if it is fully charged: this means
//                                 * that it will never be able to reach a frontier: exploration is over */
//                                // TODO(minor) we shoudl better check if there are still unvisited
//                                // frontier and the robot is not able to reach any of them even if
//                                // fully charged, because clearly this is a problem... which could be solved by navigating the
//                                // ds graph to reach one of this frontiers
//                                //if(!already_navigated_DS_graph || exploration->existFrontiers()) //TODO(minor) are we sure taht it works correctly? theoretically yes, but the "error" message below is printed too ofter...
//                                if(exploration->existFrontiers())
//                                {
//                                    ROS_INFO("There are still unvisited frontiers, but the robot cannot reach them even with full battery: try searching for a DS with EOs (if the DS selection strategy allows it...)"); //notice that it is true that the existing frontiers are unreachable with full battery at the moment, or the execution flow wouldn't be here...//TODO(minor) bad text... 
//                                    update_robot_state_2(auctioning_2);
//                                    already_navigated_DS_graph = true;
//                                    
//                                    if(ros::Time::now() - start_time < ros::Duration(5*60)) {
//                                        log_major_error("trying to navigate graph!!!");     
//                                    }
//                                        
//                                    
//                                }
//                                else
//                                {
//                                    ROS_INFO("No more frontiers to be visited: exploration is completed");
//                                    /* Start countdown to finalize exploration */
//                                    // TODO(minor)
//                                    // exit_countdown--;
//                                    // ROS_ERROR("Shutdown in: %d", exit_countdown);
//                                    // if (exit_countdown <= 0)
//                                    ros::spinOnce();
//                                    move_home_if_possible();
//                                }                                
//                            }

//                            else if(robot_state == leaving_ds) {
//                                if (robot_state_next != going_charging_next && robot_state_next != going_queue_next) {
//                                    // check the existence of reachable frontiers with full battery life
//                                    //if(!already_navigated_DS_graph || exploration->existFrontiers()) {
//                                    if(exploration->existFrontiers()) {
//                                        //if(exploration->existFrontiersReachableWithFullBattery(max_av_distance)) { //TODO(minor) really necessary?
//                                            ROS_INFO("There are still unvisited frontiers, but the robot cannot reach them even with full battery: try searching for a DS with EOs (if the DS selection strategy allows it...)"); //TODO(minor) bad text...
//                                            update_robot_state_2(auctioning_2);
//                                            already_navigated_DS_graph = true;
//                                         }
//                                     else {
//                                        ROS_ERROR("Charging was interrupted and there are still unvisited frontiers, but they cannot reach even when the robot has full battery: finalize exploration...");
//                                        ros::spinOnce();
//                                        move_home_if_possible();
//                                     }
////                                    } else {
////                                        ROS_INFO("Charging was interrupted, but there are no more frontiers that can be reached byt the robot");
////                                        finalize_exploration();
////                                    }
//                                }
//                            }
//                            else
//                            {
//                                /* Robot cannot reach any frontier but it is not fully charged, so a recharging is in order */
//                                // TODO(minor) check if the robot has not just won an auction meanwhile... but should be solved by docking now...
//                                // TODO(minor) call udpate_robot_state() instead!!!
//                                // TODO(minor) but also calling update_robot_state does not solve the problem, because maybe the
//                                // docking node knows that all the auctions have been concluded, but it has not notified the
//                                // explorer node about it yet, which means that update_robot_state hase nothing to update at the
//                                // moment... it could be solved using a service or using robot_state node
//                                if (robot_state_next != going_charging_next && robot_state_next != going_queue_next)
//                                {
//                                    //ROS_ERROR("Robot cannot reach any frontier: starting auction to "
//                                    //          "acquire access to a DS to recharge");  // TODO(minor) this message could be misleading
//                                                                                      // if the robot does not really start a
//                                                                                      // new auction...
//                                    update_robot_state_2(auctioning);
//                                }
//                            }
                        }
                        
                        costmap_mutex.unlock();
                        print_mutex_info("explore()", "unlock");
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
             
            ROS_INFO("FRONTIER COORDINATION");

            /* Produce frontier/cluster points for rviz */
            //exploration->visualize_Cluster_Cells();
            //exploration->visualize_Frontiers();

            /* Navigate robot to next frontier */
            
            
            if(robot_state == exploring) //happens when the robot lost the auction for the negotiation of the frontier
            {
                ROS_INFO("continue in state 'exploring'");
                continue;
            }
            
            
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
//                    ROS_INFO("final_goal size: %lu", final_goal.size());
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
                                  final_goal.at(0), final_goal.at(1)); // which is done later, when the robot noticed that it couldn't reach a goal
                        navigate_to_goal = false;  // navigate(final_goal);
//                        fs_exp_se_log.open(exploration_start_end_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//                        fs_exp_se_log << ros::Time::now() - time << ": " << "Failed to find backoff goal, mark original goal (" << final_goal.at(0) << "," << final_goal.at(1) << ") as unreachable" << std::endl;
//                        fs_exp_se_log.close();
                    }
                }
                else
                {
                    if (!exploration->smartGoalBackoff(final_goal.at(0), final_goal.at(1), costmap2d_global, &backoffGoal))
                        ROS_ERROR("Failed to find backoff goal for goal (%.2f,%.2f)", final_goal.at(0), final_goal.at(1));
                    ROS_INFO("Navigate to goal");
                    navigate_to_goal = navigate(final_goal);
                }
            }

            // TODO(minor) hmm... here??
            if (robot_state == auctioning || robot_state == auctioning_2 || robot_state == auctioning_3 )
            {
//                double distance = -1;
//                int i = 0;
//                while(distance < 0 && i < 10) {
//                    exploration->distance_from_robot(optimal_ds_x, optimal_ds_y);
//                    i++;
//                    ros::Duration(2).sleep();
//                }
//                if(distance < 0)
//                    ROS_INFO("cannot comptue distance between robot and DS: leaving robot where it is");
//                else
//                    if (distance <
//                            min_distance_queue_ds)  // TODO could the DS change meanwhile???
//                            {
//                                ROS_INFO("ROBOT TOO CLOSE TO DS to start an auction: moving a little bit farther...");
//                                ROS_DEBUG("distance: %.2f; min_distance_queue_ds: %.2f", distance, min_distance_queue_ds);
//                                update_robot_state_2(moving_away_from_ds);
//                                fs_csv_state.open(csv_state_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//                                fs_csv_state << time << "," << "moving_away_from_ds" << std::endl; //TODO make real state
//                                fs_csv_state.close();
//                                move_robot_away();  // TODO(minor) move robot away also if in queue and too close...
//                                ROS_INFO("NOW it is ok...");
//                            }
                
                int auctioning_counter = 0;
                while ( (robot_state == auctioning || robot_state == auctioning_2 || robot_state == auctioning_3 ) && auctioning_counter < 10 * 60 * 5)  // TODO(minor) better management of the while loop
                {
                    
                    ROS_INFO("Auctioning...");
                    ros::Duration(1).sleep();
                    ros::spinOnce();  // TODO(minor) is spin necessary? isn't it called by update_robot_State or in main() already?
                    auctioning_counter++;
                    update_robot_state();
                }
                
                if((robot_state == auctioning && auctioning_counter >= 60) || auctioning_counter >= 180) {
                    log_major_error("auctioning was forced to stop!");
                    update_robot_state_2(going_in_queue);
                    continue;   
                }
                
                ROS_INFO("Auction completed");
            }

            // TODO(minor) hmm... here??
            if (robot_state == in_queue)
            {
                if(moving_along_path) {
                    // it could be interesting to try to move to the next DS... but when moving toward it the robot could consume a lot of energy, and if when it reaches the next DS it has to go in queue because there are many robots with a lower battery life, it could be "dangerous"
                
                }
            
                int i = 0;
                while (robot_state == in_queue && i < 10)  // TODO(minor) better management of the while loop
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
//                    i++; //if we use the "idle" mode when the robot is in queue, we don't have to perform recomputations...
                }
                if(robot_state == in_queue)
                    continue; //to force the recomputations of the frontiers
                else 
                    ROS_DEBUG("No more in queue");
            }

            if (robot_state == checking_vacancy)
            {
                /* Robot reached frontier */
                //ROS_ERROR("\n\t\e[1;34mchecking_for_vacancy...\e[0m");
                
                ROS_DEBUG("Start checking for DS vacancy (timeout: %.1fs)", checking_vacancy_timeout);
                checking_vacancy_timer.stop();
                checking_vacancy_timer.setPeriod(ros::Duration(checking_vacancy_timeout), true);
                checking_vacancy_timer.start();

                // TODO(minor) use a bterr way!!!
                int i = 0; //just for safety
                while (robot_state == checking_vacancy && i < 30)
                {
                    ros::Duration(1).sleep();
                    ros::spinOnce();
                    update_robot_state();
                    i++;
                }
                if(i >= 30) {
                    log_minor_error("robot was saved from stucking in checking_vacancy");
                    update_robot_state_2(in_queue);   
                }
                
                // Stop the timer, since I could have exited the while loop above due to a occupancy message
                checking_vacancy_timer.stop();
                
            }

            // navigate robot home for recharging
            // IMPORTANT: do not put an else-if here, because when I exit from the
            // above else if(robot_state == auctioning), I may need to go home / to ds
            // since now I may be in state going_charging because I won an auction!!!!
            // TODO(minor) am I sure???
            if (robot_state == going_in_queue || robot_state == going_checking_vacancy || robot_state == going_charging)
            {
                if (robot_state == going_checking_vacancy)
                    ROS_INFO("Approaching ds%d (%f, %f) to check if it is free", -1, optimal_ds_x, optimal_ds_y);
                else if (robot_state == going_in_queue)
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
            }

            /* NAVIGATION COMPLETED */ //F
            /* Check if the robot was able to reach the selected goal */
            if (navigate_to_goal == true)  // IMPORTANT do not put an else-if //TODO(minor) sure??
            {
                /* Result of navigation successful */
                ROS_INFO("navigation to goal succeeded");
                failures_going_to_DS = 0;
                
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

                    // do not store travelled distance here... the move_robot called at the previous iteration has done it...
                    //exploration->trajectory_plan_store(optimal_ds_x, optimal_ds_y);
                }

                else if (robot_state == moving_to_frontier || robot_state == moving_to_frontier_before_going_charging)
                {
                    /* Robot reached frontier */
                    if (robot_state == moving_to_frontier)
                        update_robot_state_2(exploring);

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
                if (robot_state == going_charging)
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
                    
                    
                    //exit_countdown--;
                    //ROS_ERROR("Shutdown in: %d", exit_countdown);
                    //if (exit_countdown <= 0)
                    //    finalize_exploration();
                }

                else if (robot_state == moving_to_frontier || robot_state == moving_to_frontier_before_going_charging)
                {
                    if (robot_state == moving_to_frontier)
                    {
                        ROS_ERROR("Robot could not reach goal: mark goal as unreachable and explore again");
                        ROS_INFO("Robot could not reach goal: mark goal as unreachable and explore again");
//                        if( fabs(starting_x - pose_x) < 1 && fabs(starting_y - pose_y) < 1 )
//                            skip_findFrontiers = true;
//                        else
//                            skip_findFrontiers = false;
                        update_robot_state_2(exploring);
                    }
                    else
                        update_robot_state_2(going_charging);  // TODO(minor) if
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
    
    void store_current_position() {
        int i=0;
        while(!exploration->getRobotPose(robotPose) && i < 10) {
            ROS_ERROR("Failed getting current position... retrying in a moment");
            ros::Duration(2).sleep();
            i++;
        }
//        if(i >= 10)
//            ROS_FATAL("unable got get robot pose")
        stored_robot_x = robotPose.getOrigin().getX();
        stored_robot_y = robotPose.getOrigin().getY();
    }
    
    void update_robot_state_2(int new_state)
    {  // TODO(minor) comments in the update_blabla functions, and lso in the other callbacks
        ROS_INFO_COND(LOG_STATE_TRANSITION, "State transition: %s -> %s", get_text_for_enum(robot_state).c_str(),
                  get_text_for_enum(new_state).c_str());
        adhoc_communication::EmRobot msg;
        msg.id = robot_id;
        msg.state = new_state;
        previous_state = robot_state;
        robot_state = static_cast<state_t>(new_state);
        pub_robot.publish(msg);

        if (robot_state == auctioning || robot_state == auctioning_2 || robot_state == auctioning_3) {
            need_to_recharge = true;
            ROS_INFO("setting need_to_recharge to true");   
        }
        // else if(robot_state == exploring || robot_state == fully_charged)
        else if (robot_state == leaving_ds || robot_state == fully_charged) {
            ROS_INFO("setting need_to_recharge to false");   
            need_to_recharge = false;
        }
        
        if(robot_state == fully_charged)
            full_battery = true;
            
        if(robot_state == moving_to_frontier || robot_state == auctioning || robot_state == auctioning_2 || robot_state == auctioning_3 || robot_state == in_queue)
            full_battery = false;

        if(robot_state == moving_to_frontier) 
            already_navigated_DS_graph = false;
            
        //robot_state::GetRobotState srv;
        // sc_get_robot_state.call(srv);
        
        ros::Duration time = ros::Time::now() - time_start;

        fs_csv_state.open(csv_state_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        fs_csv_state << time << "," << get_text_for_enum(robot_state).c_str() << "," << moving_along_path << std::endl;
        fs_csv_state.close();
        
        if(robot_state == stuck && (previous_state == auctioning || previous_state == auctioning_2 || robot_state == auctioning_3) )
            log_major_error("stucked after auction!!!");
        if(robot_state == stuck &&  previous_state == going_charging)
            log_major_error("stuck when going_charging!!!");
            
        
        //TODO move this in update_robot_state, where the state is set to finished
//        if(robot_state == finished || robot_state == dead || robot_state == stuck) {
//            std_msgs::Empty msg;
//            pub_finished_exploration.publish(msg);
//            exploration_finished = true;
//        }            
        
    }

    void update_robot_state()
    {
        ROS_DEBUG("Updating robot state...");
        
        // TODO(minor) do we need the spin?
        ros::spinOnce();
     
        if(percentage >= 100 && !checked_percentage && robot_state_next != finished_next && robot_state != finished) {
            if(percentage > 100.1) {
                log_major_error("Exploration percentage is higher that 100.0");
                ROS_ERROR("percentage: %.1f", percentage);
            }
            //ROS_INFO("100%% of the environment explored: the robot can conclude its exploration");
            //robot_state_next = finished_next;
            checked_percentage = true;
        }
        
        if(robot_state_next == finished_next) {
            ROS_INFO("Have to finish...");
            //robot_state == auctioning_3;
            move_home_if_possible();
            finalize_exploration();
        }
        
        /* If the next state is equal to the current one, do nothing */
        else if (robot_state_next == current_state)
        {
            ROS_DEBUG("Next state is equal to the current state: do nothing");
        } 
       

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
        
        // happens if the robot is changing and meanwhile it looses an auction started by another robot
        else if (robot_state_next == going_queue_next && robot_state == charging)
        {
            /*
            if (DS_SELECTION_POLICY == 2 && moving_along_path)  
                if(OPP_ONLY_TWO_DS)
                    if (ds_path_counter < 2)
                    {
                        optimal_ds_x = path[ds_path_counter][0];
                        optimal_ds_y = path[ds_path_counter][1];
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
                        optimal_ds_x = path[ds_path_counter][0];
                        optimal_ds_y = path[ds_path_counter][1];
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
            //{
//                if(robot_id != 0)
                    update_robot_state_2(leaving_ds);
            //}
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
                        optimal_ds_x = path[ds_path_counter][0];
                        optimal_ds_y = path[ds_path_counter][1];
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
                        optimal_ds_x = path[ds_path_counter][0];
                        optimal_ds_y = path[ds_path_counter][1];
                        ds_path_counter++;
                        update_robot_state_2(going_checking_vacancy);
                    }
                    else {
                         moving_along_path = false;
                        update_robot_state_2(fully_charged);
                    }               

            else
            */
            //{
                ROS_DEBUG("prearing for fully_charged");
                update_robot_state_2(fully_charged);
//                if(moving_along_path) {
//                    ROS_INFO("pub_next_ds");
////                    moving_along_path = false;
//                    std_msgs::Empty msg;
//                    pub_next_ds.publish(msg);
//                }
            //}
        }

        /* */
        else if (robot_state_next == going_charging_next)
        {
            if (robot_state != charging && robot_state != going_charging && robot_state != going_checking_vacancy &&
                robot_state != checking_vacancy)
            {
                ROS_INFO("preparing for going_checking_vacancy");
                update_robot_state_2(going_checking_vacancy);
            }
            else if(robot_state == checking_vacancy)
                update_robot_state_2(going_charging);
            else
                ROS_INFO("already charging (or approaching charging, etc.)");
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
                ROS_INFO("already in_queue...");
                update_robot_state_2(in_queue); //VERY IMPORTANT: necessary to force restarting of periodic auction timer!!!!!!!!
            }

            /* If the robot is already preparing to enter in a queue, do nothing */
            else if (robot_state == going_in_queue)
            {
                ROS_INFO("already going_in_queue...");
            }

            /* If the robot is going to charge, let it charge at least a little: the other robot will start later a new
             * auction */
            // TODO(minor) something probably went wrong in this case...
            else if (robot_state == going_charging)
            {
                ROS_INFO("i want to charge a little, first...");
            }

            /* If the robot was participating to an auction, prepare the robot to go in queue */
            else if (robot_state == auctioning || robot_state == auctioning_2 || robot_state == auctioning_3)
            {
                ROS_DEBUG("prearing for going_in_queue");
                update_robot_state_2(going_in_queue);
            }
            
            /* If the robot is going to check if the target DS is free or it is already checking, do nothing (the robot will receive messages from the other robots telling it that the DS is not vacant) */ //TODO what if all these messages are lost
            else if (robot_state == going_checking_vacancy)
            {
                ROS_INFO("robot is going_checking_vacancy... ignore going_in_queue then...");
                update_robot_state_2(going_in_queue);
            }
            
            else if (robot_state == checking_vacancy)
            {
                ROS_INFO("discovered that the DS is occupied!");
                update_robot_state_2(going_in_queue);
            }
            
            /* Otherwise, something strange happened */
            else {
                log_major_error("robot would like to go in queue even if it should not!");
                ROS_DEBUG("robot state in fact is: %s", get_text_for_enum(robot_state).c_str());
                ROS_INFO("ignoring going_queue_next");
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
                            optimal_ds_x = path[ds_path_counter][0];
                            optimal_ds_y = path[ds_path_counter][1];
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
                //{
                    ROS_DEBUG("prearing for leaving_ds");
//                    if(robot_id != 0)
                        update_robot_state_2(leaving_ds);
                //}
            } else {
                /* This happens when a robot lost another robot auction, but it doesn't need to recharge */
                ROS_INFO("keep exploring..."); 
                //update_robot_state_2(exploring); 
            }
        }
        else {
            ROS_ERROR("Invalid next state");
            ROS_DEBUG("Invalid next state");
        }

        /* Reset next state */
        robot_state_next = current_state;
    }

    void frontiers()
    {
        while(!explorer_ready)
            ros::Duration(10).sleep();
    
        ros::Duration(10).sleep();
        ROS_INFO("Can start periodical recomputation and publishing of frontiers"); //TODO probably now the findFrontiers, etc., in explorer are useless... but we have to be sure that when we search for a frontier we have an updated map
        
        while (ros::ok())
        {
            ros::Rate(0.1).sleep();
            
            if(robot_state == in_queue) //idle mode
                continue;

            //ROS_DEBUG("frontiers(): acquiring lock");
            print_mutex_info("frontiers()", "acquiring");
            costmap_mutex.lock();
            //ROS_DEBUG("frontiers(): lock acquired");
            print_mutex_info("frontiers()", "lock");
            
            exploration->transformToOwnCoordinates_frontiers();
            exploration->transformToOwnCoordinates_visited_frontiers();

            exploration->findFrontiers();
            exploration->clearVisitedFrontiers();
            exploration->clearUnreachableFrontiers();
            exploration->clearSeenFrontiers(costmap2d_global);
            
            //ROS_INFO("publish_frontier_list and visualize them in rviz");
            exploration->publish_frontier_list();
//            exploration->publish_visited_frontier_list();  // TODO(minor) this doesn0t work really well since it publish
                                                           // only the frontier visited by this robot...

            /* Publish frontier points for rviz */
            exploration->visualize_Frontiers();
            //exploration->visualize_Clusters();
            //visualize

            costmap_mutex.unlock();
            //ROS_DEBUG("frontiers(): lock released");
            print_mutex_info("frontiers()", "unlock");

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
        fs_csv << "#time,wall_time,global_map_progress_percentage,exploration_travel_path_global_meters," //TODO(minor) maybe there is a better way to obtain exploration_travel_path_global_meters without modifying ExplorationPlanner...
                  "global_map_explored_cells,discovered_free_cells_count,local_map_explored_cells,total_number_of_free_cells"
//                  ",battery_state,"
//                  "recharge_cycles,energy_consumption,frontier_selection_strategy,coeff_a,coeff_b"
               << std::endl;
        fs_csv.close();
        
        double last_moving_instant = 0;

        while (ros::ok() && !exploration_finished)
        {
            // double angle_robot = robotPose.getRotation().getAngle();
            // ROS_ERROR("angle of robot: %.2f\n", angle_robot);

            print_mutex_info("map_info()", "acquiring");
            costmap_mutex.lock();
            print_mutex_info("map_info()", "lock");

            double time = ros::Time::now().toSec() - time_start.toSec();
            double wall_time = ros::WallTime::now().toSec() - wall_time_start.toSec();

            map_progress.global_freespace = global_costmap_size();
            //map_progress.global_freespace = discovered_free_cells_count;
            map_progress.local_freespace = local_costmap_size();
            map_progress.time = time;
            
            if(robot_is_moving()) { //notice that since the loop is executed every N seconds, we won't have a very precise value, but given that we use approximation in computing the remaining battery life it is ok...
                double elapsed_time_in_moviment = ros::Time::now().toSec() - last_moving_instant;
                moving_time += elapsed_time_in_moviment;
            }
            last_moving_instant = ros::Time::now().toSec();
            
            map_progress_during_exploration.push_back(map_progress);
            if(free_cells_count <= 0 || discovered_free_cells_count <= 0)
                percentage = -1;
            else
                percentage = (float) (discovered_free_cells_count * 100) / free_cells_count; //this makes sense only if the environment has no cell that are free but unreachable (e.g.:if there is rectangle in the environment, if it's surface is not completely black, the cells inside its perimeters are considered as free cells but they are obviously unreachable...); to solve this problem we would need a smart way to exclude cells that are free but unreachable...

            //ROS_ERROR("%.0f", map_progress.global_freespace);
            //ROS_ERROR("%d", free_cells_count);
            //ROS_ERROR("%f", percentage);
            double exploration_travel_path_global =
                //F
                //(double)exploration->exploration_travel_path_global * costmap_resolution;
                exploration->exploration_travel_path_global_meters;

            if (battery_charge_temp >= battery_charge)
                energy_consumption += battery_charge_temp - battery_charge;
            battery_charge_temp = battery_charge;

            fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
            fs_csv << map_progress.time << "," << wall_time << "," << percentage << "," << exploration_travel_path_global << ","
                   << map_progress.global_freespace << "," << discovered_free_cells_count << ","
                   << map_progress.local_freespace << "," << free_cells_count //<< "," 
//                   << battery_charge << "," << recharge_cycles << "," << energy_consumption << "," << frontier_selection << "," << coeff_a << "," << coeff_b
                   << std::endl;
            fs_csv.close();

            costmap_mutex.unlock();
            print_mutex_info("map_info()", "unlock");

            ROS_DEBUG("Saving progress...");
            save_progress();
            ROS_DEBUG("Progress have been saved");

            // publish average speed
            explorer::Speed speed_msg;
            
//            speed_msg.avg_speed = exploration_travel_path_global / map_progress.time;
            speed_msg.avg_speed = exploration_travel_path_global / moving_time;

            publisher_speed.publish(speed_msg);

            ros::Duration(5.0).sleep();
        }
    }
    
    void log_map() {
        while (ros::ok() && !exploration_finished) {
            // call map_merger to log data
            map_merger::LogMaps log;
            log.request.log = 12;  /// request local and global map progress
            ROS_INFO("Calling map_merger service logOutput");
            if (!mm_log_client.call(log))
                ROS_ERROR("Could not call map_merger service to store log.");
            ROS_DEBUG("Finished service call.");
            ros::Duration(30).sleep();
        }
    }

    int global_costmap_size()
    {
        //occupancy_grid_global = costmap2d_global->getCostmap()->getCharMap();
        //int num_map_cells_ =
        //    costmap2d_global->getCostmap()->getSizeInCellsX() * costmap2d_global->getCostmap()->getSizeInCellsY();
        int free = 0;

        /*
        for (unsigned int i = 0; i < num_map_cells_; i++)
        {
            if ((int) occupancy_grid_global[i] == costmap_2d::FREE_SPACE)
            getCost(cell_x, cell_y)
            {
                free++;
            }
        }
        */
        
        //ROS_ERROR("%d", costmap2d_global->getCostmap()->getSizeInCellsX() * costmap2d_global->getCostmap()->getSizeInCellsY());
        
        for (unsigned int i = 0; i < costmap2d_global->getCostmap()->getSizeInCellsX(); i++)
            for (unsigned int j = 0; j < costmap2d_global->getCostmap()->getSizeInCellsY(); j++)
            {
                if (costmap2d_global->getCostmap()->getCost(i,j) == costmap_2d::FREE_SPACE)
                    free++;
            }
        
        //ROS_ERROR("%d", free);
        return free;
    }
    
    int total_size()
    {
        occupancy_grid_global = costmap2d_global->getCostmap()->getCharMap();
        return costmap2d_global->getCostmap()->getSizeInCellsX() * costmap2d_global->getCostmap()->getSizeInCellsY();
    }

    int local_costmap_size()
    {
        if (OPERATE_ON_GLOBAL_MAP == false)
        {
            occupancy_grid_local = costmap2d_local->getCostmap()->getCharMap();
            unsigned int num_map_cells_ =
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
            unsigned int num_map_cells_ = costmap2d_local_size->getCostmap()->getSizeInCellsX() *
                                 costmap2d_local_size->getCostmap()->getSizeInCellsY();
            unsigned int free = 0;

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

        original_log_path = log_path;

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
//        double size_global_map =
//            map_progress_during_exploration.at(map_progress_during_exploration.size() - 1).global_freespace;

//        double efficiency_value = (exploration_time) / (number_of_robots * navigation_goals_required);

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
        fs << "w1                               = " << w1 << std::endl;
        fs << "w2                               = " << w2 << std::endl;
        fs << "w3                               = " << w3 << std::endl;
        fs << "w4                               = " << w4 << std::endl;
        fs << "queue_distance                   = " << queue_distance << std::endl;
        fs << "auction_timeout                  = " << auction_timeout << std::endl;
        fs << "checking_vacancy_timeout         = " << checking_vacancy_timeout << std::endl;

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

    void exploration_has_finished()
    {
        ros::Duration ros_time = ros::Time::now() - time_start;

        double exploration_time = ros_time.toSec();
        int navigation_goals_required = counter;
        double exploration_travel_path = (double)exploration->exploration_travel_path_global * 0.02;
        double size_global_map =
            map_progress_during_exploration.at(map_progress_during_exploration.size() - 1).global_freespace;
        ROS_INFO("overall freespace in the global map: %f", size_global_map);

        for (unsigned int i = 0; i < map_progress_during_exploration.size(); i++)
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

//        double efficiency_value = (exploration_time) / (number_of_robots * navigation_goals_required);

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
        
        ROS_INFO("Creating file %s to indicate end of exploration.", status_file.c_str());
        std::ofstream outfile(status_file.c_str());
        outfile.close();
        if(!boost::filesystem::exists(status_file)) {
            log_major_error("cannot create status file!!!");
            //exit(-1);   
        } else
            ROS_INFO("Status file created successfully");
        
        if(percentage < 90 && robot_state != stuck && robot_state != dead) {
            log_major_error("low percentage (<90%)!!!");
        }
        
        if(percentage < 95 && robot_state != stuck && robot_state != dead) {
            log_minor_error("percentage < 95%");
        }
        
        ros::Duration(10).sleep();
        
        adhoc_communication::EmRobot msg;
        msg.id = robot_id;
        pub_finished_exploration_id.publish(msg);
        
        std_msgs::Empty msg2;
        pub_finished_exploration.publish(msg2);
        
        exploration_finished = true;
        
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

        exploration->initialize_planner("exploration planner", costmap2d_global, costmap2d_global);
        
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
    
    double conservative_available_distance(double available_distance) {
        //return available_distance * coeff_a - max_av_distance * coeff_b;
        return available_distance;
    }

    bool move_robot(int seq, double position_x, double position_y)
    {
        ROS_INFO("Preparing to move toward goal (%.1f, %.1f)...", position_x, position_y);

        exploration->next_auction_position_x = position_x;
        exploration->next_auction_position_y = position_y;
//        int stuck_countdown = EXIT_COUNTDOWN;
        ros::Duration my_stuck_countdown = ros::Duration( (TIMEOUT_CHECK_1 - 2) * 60);
        ros::Duration my_fallback_countdown = ros::Duration(30);
        bool timer_started = false;
        ros::Time start_time_fallback;

        /* Move the robot with the help of an action client. Goal positions are transmitted to the robot and feedback is
         * given about the actual driving state of the robot. */
        //if (!costmap2d_local->getRobotPose(robotPose))
        //{
        //    ROS_ERROR("Failed to get RobotPose");  // TODO(minor) so what???
        //}

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
        if (remaining_distance > 0 && remaining_distance < queue_distance && (robot_state == going_in_queue || robot_state == going_checking_vacancy) )
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

        ros::Time time_before = ros::Time::now();
        prev_pose_x = pose_x;
        prev_pose_y = pose_y;
        while (robot_is_moving() && ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            // robot seems to be stuck
            if ( fabs(prev_pose_x - pose_x) < 0.1 && fabs(prev_pose_y - pose_y) < 0.1 && fabs(prev_pose_angle - pose_angle) < 0.1 )  // TODO(minor) ...
            {
                //stuck_countdown--; //TODO(minor)
                // if(stuck_countdown <= 5){

                // TODO(minor) if STUCK_COUNTDOWN is too low, even when the robot is
                // computing the frontier, it is believed to be stucked...
//                if (stuck_countdown <= 10)
//                {
//                    ROS_ERROR("Robot is not moving anymore, shutdown in: %d", stuck_countdown);
//                }

                if (my_stuck_countdown <= ros::Duration(0))
                {
                    store_travelled_distance();

                    ac.cancelGoal();
                    exploration->next_auction_position_x = robotPose.getOrigin().getX();
                    exploration->next_auction_position_y = robotPose.getOrigin().getY();
                    approximate_success++;
                
                    if( fabs(position_x - pose_x) < 1 && fabs(position_y - pose_y) < 1 ) {
                        ROS_ERROR("robot seems unable to received ACK from actionlib even if the goal have been reached");
                        ROS_INFO("robot seems unable to received ACK from actionlib even if the goal have been reached");
                        return true;
                    } else
                        return false;
                }
                
                my_stuck_countdown -= ros::Time::now() - time_before;
                ROS_DEBUG("%.1f", my_stuck_countdown.toSec());
            }
            else
            {
                //ROS_ERROR("(%f, %f; %f) : (%f, %f; %f)", prev_pose_x, prev_pose_y, prev_pose_angle, pose_x, pose_y, pose_angle);
//                stuck_countdown = STUCK_COUNTDOWN;  // robot is moving again
                prev_pose_x = pose_x;
                prev_pose_y = pose_y;
                prev_pose_angle = pose_angle;
            }

            if(robot_state == going_checking_vacancy || robot_state == going_in_queue || robot_state == going_charging) {
                remaining_distance = exploration->distance_from_robot(position_x, position_y);

                /* Print remaining distance to be travelled to reach goal if the goal is a DS */
                if (robot_state == going_checking_vacancy || robot_state == going_in_queue)
                    ROS_DEBUG("Remaining distance: %.3f\e[0m", remaining_distance);

                /* If the robot is approaching a DS to queue or to check if it is free, stop it when it is close enough to
                 * the DS */
                if ((robot_state == going_checking_vacancy || robot_state == going_in_queue) && remaining_distance > 0 && remaining_distance < queue_distance)
                {
                    ac.cancelGoal();
                    exploration->next_auction_position_x = robotPose.getOrigin().getX();
                    exploration->next_auction_position_y = robotPose.getOrigin().getY();
                    
                    store_travelled_distance();
                    
                    return true;
                }
                if( (robot_state == going_charging && remaining_distance < 3.0) || (robot_state == going_in_queue && remaining_distance < 6.0) ) {
                    if(!timer_started) {
                        timer_started = true;
                        start_time_fallback = ros::Time::now();
                    }
                    my_fallback_countdown -= ros::Time::now() - time_before;
                    if(my_fallback_countdown <= ros::Duration(0)) {
                        ac.cancelGoal();
                        exploration->next_auction_position_x = robotPose.getOrigin().getX();
                        exploration->next_auction_position_y = robotPose.getOrigin().getY();
                        log_minor_error("consider that robot reached the DS");
                        
                        store_travelled_distance();
                        return true;
                    }
                }
            }
            
            time_before = ros::Time::now();
            ros::Duration(1).sleep(); //TODO(minor)
        }

        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            if(!robot_is_moving()) {
                log_minor_error("robot was forced to stop from another thread");
                ac.cancelGoal();
                store_travelled_distance();
                exploration->next_auction_position_x = robotPose.getOrigin().getX();
                exploration->next_auction_position_y = robotPose.getOrigin().getY();
                return true;
            }
            
            if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
            {
                
                store_travelled_distance();
                exploration->next_auction_position_x = robotPose.getOrigin().getX();
                exploration->next_auction_position_y = robotPose.getOrigin().getY();
                
                if( (position_x - pose_x) * (position_x - pose_x) + (position_y - pose_y) * (position_y - pose_y) < 5*5 ) {
                      ROS_ERROR("Robot seems unable to closely reach the goal, but it is close enough to consider the goal reached... ");
                      if(moving_to_ds || going_home) 
                        log_minor_error("Robot didn't properly reach home/DS");
                        return true;
                }
                else  {
                    ROS_INFO("ABORTED: goal not reached (robot is farther than 3 meters from goal)");
                    return false;
                }                

            }
        }

        ROS_INFO("Goal reached");

        exploration->next_auction_position_x = robotPose.getOrigin().getX();
        exploration->next_auction_position_y = robotPose.getOrigin().getY();

        approximate_success = 0;
        store_travelled_distance();
        return true;
    }
    
    bool move_robot_away(int seq)
    {
        ROS_INFO("Preparing to move away...");
//        int stuck_countdown = EXIT_COUNTDOWN;

        /* Move the robot with the help of an action client. Goal positions are transmitted to the robot and feedback is
         * given about the actual driving state of the robot. */
        //if (!costmap2d_local->getRobotPose(robotPose))
        //{
        //    ROS_ERROR("Failed to get RobotPose");  // TODO(minor) so what???
        //}
        
        double remaining_distance = exploration->distance_from_robot(optimal_ds_x, optimal_ds_y);
        
        if(remaining_distance > queue_distance / 2.0 || exploration->distance_from_robot(home_point_x, home_point_y) < 1 ) {
        
            ROS_INFO("alreayd away from DS"); // althought this could be false... in the sense that maybe the movement was simply aborted

            exploration->next_auction_position_x = robotPose.getOrigin().getX();
            exploration->next_auction_position_y = robotPose.getOrigin().getY();
//            store_travelled_distance();
            return true;
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
        
        ros::Duration my_stuck_countdown = ros::Duration( (TIMEOUT_CHECK_1 - 2) * 60);
        ros::Time time_before = ros::Time::now();
        prev_pose_x = pose_x;
        prev_pose_y = pose_y;
        while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            // robot seems to be stuck
            if ( fabs(prev_pose_x - pose_x) < 0.1 && fabs(prev_pose_y - pose_y) < 0.1 && fabs(prev_pose_angle - pose_angle) < 0.1 )  // TODO(minor) ...
            {
                //stuck_countdown--; //TODO(minor)
                // if(stuck_countdown <= 5){

                // TODO(minor) if STUCK_COUNTDOWN is too low, even when the robot is
                // computing the frontier, it is believed to be stucked...
//                if (stuck_countdown <= 10)
//                {
//                    ROS_ERROR("Robot is not moving anymore, shutdown in: %d", stuck_countdown);
//                }

                if (my_stuck_countdown <= ros::Duration(0))
                {
//                    if( fabs(home_point_x - pose_x) < 1 && fabs(home_point_y - pose_y) < 1 ) {
//                        ROS_ERROR("robot seems unable to received ACK from actionlib even if the goal have been reached");
//                        ROS_INFO("robot seems unable to received ACK from actionlib even if the goal have been reached");
//                        ac.cancelGoal();
                        exploration->next_auction_position_x = robotPose.getOrigin().getX();
                        exploration->next_auction_position_y = robotPose.getOrigin().getY();
//                        approximate_success++;
                        store_travelled_distance();
                        return true;
//                    } else
//                        return false;
                }

                ros::Duration(1).sleep();
                
                my_stuck_countdown -= ros::Time::now() - time_before;
                time_before = ros::Time::now();

            }
            else
            {
                //ROS_ERROR("(%f, %f; %f) : (%f, %f; %f)", prev_pose_x, prev_pose_y, prev_pose_angle, pose_x, pose_y, pose_angle);
//                stuck_countdown = STUCK_COUNTDOWN;  // robot is moving again
                prev_pose_x = pose_x;
                prev_pose_y = pose_y;
                prev_pose_angle = pose_angle;
            }

            remaining_distance = exploration->distance_from_robot(optimal_ds_x, optimal_ds_y);

            /* Print remaining distance to be travelled to reach goal if the goal is a DS */
            if (remaining_distance > queue_distance / 2.0) {
//                ROS_DEBUG("Remaining distance: %.3f\e[0m", remaining_distance);

            /* If the robot is approaching a DS to queue or to check if it is free, stop it when it is close enough to
             * the DS */
                ac.cancelGoal();

                //ROS_ERROR("\n\t\e[1;34mOK!!!\e[0m");

                break;
            }

            // ros::Duration(0.5).sleep(); //TODO(minor)
        }

        ROS_INFO("DS left"); // althought this could be false... in the sense that maybe the movement was simply aborted

        exploration->next_auction_position_x = robotPose.getOrigin().getX();
        exploration->next_auction_position_y = robotPose.getOrigin().getY();
        store_travelled_distance();
        return true;
    }

    bool turn_robot(int seq)
    {
        double angle = 45;

        //if (!costmap2d_local->getRobotPose(robotPose))
        //{
        //    ROS_ERROR("Failed to get RobotPose");
        //}

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
        ROS_INFO("lost_own_auction_callback");
//        if(robot_id == 0) {
//            robot_state_next = going_charging_next;
//            return;
//        } 
//        else if(robot_id == 1) {
//            increase++;
//            ROS_ERROR("%d", increase);
//            if(increase > 3) {
//                robot_state_next = going_charging_next;
//                return;
//            }
//        }
        robot_state_next = going_queue_next;
    }

    void won_callback(const std_msgs::Empty::ConstPtr &msg)
    {
        ROS_INFO("won_callback");
        robot_state_next = going_charging_next;
    }

    void lost_other_robot_callback(const std_msgs::Empty::ConstPtr &msg)
    {
        ROS_INFO("lost_other_robot_callback");
//        if(robot_id == 0) {
//            robot_state_next = going_charging_next;
//            return;
//        } 
//        else if(robot_id == 1) {
//            increase++;
//            ROS_ERROR("%d", increase);
//            if(increase > 3) {
//                robot_state_next = going_charging_next;
//                return;
//            }
//        }
        if(robot_state == in_queue) //to force the resetting of the timer to restart an auction
            robot_state_next = going_queue_next;
        
        else if (robot_state_next != fully_charged_next) { //TODO what about leaving_ds? but maybe it is already handled later, since here the check is on the *next* state...
            if (need_to_recharge)
                robot_state_next = going_queue_next;
            else
                robot_state_next = exploring_next;           
        } else
            ROS_INFO("ignoring");
    }

    // TODO(minor) use this instead than all the other auction callbacks
//    void lost_callback()
//    {
//        if (need_to_recharge)
//            robot_state_next = going_queue_next;
//        else
//            robot_state_next = exploring_next;
//    }

    void reply_for_vacancy_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg)
    {
        ROS_INFO("Target DS is (going to be) occupied by robot %d", msg.get()->used_by_robot_id);
        //checking_vacancy_timer.stop(); //TODO it doesn't work here... why???
        if(robot_state == checking_vacancy) //TODO this check should be already in update_robot_state() probably...
            robot_state_next = going_queue_next;
    }
    
    void vacancy_callback(const ros::TimerEvent &event) {
        ROS_INFO("Timeout for vacancy check");
        if(robot_state_next != going_queue_next && robot_state == checking_vacancy)
            robot_state_next = going_charging_next;
        else
            robot_state_next = going_queue_next;
    }

    void battery_charging_completed_callback(const std_msgs::Empty::ConstPtr &msg)
    {
        ROS_INFO("Recharging completed");
        if (robot_state != moving_to_frontier)
            robot_state_next = fully_charged_next;
    }

    void new_optimal_docking_station_selected_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg)
    {
//        if (robot_state != charging && robot_state != going_charging && robot_state != going_checking_vacancy &&
//                robot_state != checking_vacancy) {
//            optimal_ds_id = msg.get()->id;
            optimal_ds_x = msg.get()->x;
            optimal_ds_y = msg.get()->y;
            ROS_DEBUG("Storing new optimal DS position, which is placed at (%f, %f)", optimal_ds_x, optimal_ds_y);
//            exploration->new_optimal_ds(optimal_ds_x, optimal_ds_y);
//        } else {
//            //it could happen that the node is moving to a DS for checking if it is vacant, and that meanwhile it wins an auction for another DS, but the robot should ignore this other auction (in fact, even if it won the auction, we cannot be sure that the auctioned DS is really vacant, so there is no point in changing the target DS)
//             ROS_INFO("The robot is already approaching a DS, so I cannot change it now...");
//        }
       

        // TODO(minor)
        /*
        map_merger::TransformPoint point;
        if (robot_prefix == "/robot_1")
            point.request.point.src_robot = "robot_0";
        else if (robot_prefix == "/robot_0")
            point.request.point.src_robot = "robot_1";
        else
            ;  // ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        point.request.point.x = optimal_ds_x;
        point.request.point.y = optimal_ds_y;

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
        ROS_INFO("Moving along path");
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

    void bat_callback(const explorer::battery_state::ConstPtr &msg)
    {
        ROS_DEBUG("Received battery state");
        battery_charge = (int) (msg->soc * 100);
        charge_time = msg->remaining_time_charge;
        next_available_distance = msg->remaining_distance;
        ROS_INFO("SOC: %d%%; available distance: %.2f; conservative av. distance: %.2f; time: %.2f", battery_charge, available_distance, conservative_available_distance(available_distance), msg->remaining_time_run);

        if (msg->charging == false && battery_charge == 100 && charge_time == 0)
            recharge_cycles++;  // TODO(minor) hmm... soc, charge, ...

        /* If the robot has run out of energy, it cannot move anymore: terminate exploration... */
//        if (available_distance <= 0 && robot_state != charging)
//        {
//            log_major_error("Robot has run out of energy!");
//            abort();
//        }
        
        if(max_av_distance < available_distance)
            max_av_distance = available_distance;
    }
    
    void shutdown() {
        ros::Duration(10).sleep();
        ros::shutdown();
    }
    
    void finish_callback(const std_msgs::Empty &msg) {
        ROS_INFO("finish_callback");
        robot_state_next = finished_next;
    }
    
    void abort() {
        ROS_ERROR("Exploration is going to be gracefully terminated for this robot...");
        ros::Duration(3).sleep();
        update_robot_state_2(dead);
        this->indicateSimulationEnd();
        
        shutdown();
        
    }
    
    void log_stucked() {
    
        update_robot_state_2(stuck);
        this->indicateSimulationEnd();
        
        std::stringstream robot_number;
        robot_number << robot_id;

        std::string prefix = "/robot_";
        
        std::string status_directory = "/simulation_status_stuck";
        std::string robo_name = prefix.append(robot_number.str());
        std::string file_suffix(".stuck");

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
        
        shutdown();
        
    }
    
    void log_stopped() {
    
        update_robot_state_2(stopped);
        
        std::stringstream robot_number;
        robot_number << robot_id;

        std::string prefix = "/robot_";
        
        std::string status_directory = "/simulation_status_stopped";
        std::string robo_name = prefix.append(robot_number.str());
        std::string file_suffix(".stuck");

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
        
        this->indicateSimulationEnd();
        ros::Duration(10).sleep();
        
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
        
        ROS_DEBUG("x: %.1f; y: %.1f; angle: %.1f", pose_x, pose_y, pose_angle);
        //if(robot_id == 1)
        //    ROS_ERROR("%.2f: %.1f, %.1f", pose_angle, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);
        if(robot_id == 0)
            ; //ROS_ERROR("x: %.1f; y: %.1f", pose_x, pose_y);
        
        print_new_position();
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

    bool target_reached(void)
    {
//        ros::NodeHandle nh_sub_move_base;
//        sub_move_base = nh_sub_move_base.subscribe("feedback", 1000, &Explorer::feedbackCallback, this);

//        while (feedback_value != feedback_succeed_value)
//            ;

        return true;
    }
    
    bool robot_is_moving() {
        if(robot_state == moving_to_frontier || robot_state == going_charging || robot_state == going_checking_vacancy || robot_state == going_in_queue || robot_state == leaving_ds)
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
//        state_t prev_robot_state = fully_charged;
            
        ros::Time prev_time = ros::Time::now();   
        while(ros::ok() && !exploration_finished) {
        
            if(approximate_success >= 3)
                log_major_error("too many approximated successes");
            
            //ROS_DEBUG("Checking...");
            //if(exploration->getRobotPose(robotPose)) {
            //    robot_x = robotPose.getOrigin().getX();
            //    robot_y = robotPose.getOrigin().getY();
            //}
            
            //IMPORTANT: be careful that a robot could change state while it is stucked, since it may continuosly change between 'moving_to_fonrtier' to 'exploring' to compute and try rearching a new goal!!!
            //if((int) prev_robot_state == (int) robot_state && pose_x == prev_robot_x && pose_y == prev_robot_y) 
            //if( ((int) prev_robot_state == (int) robot_state) && ((int) pose_x == (int) prev_robot_x) && ((int) pose_y == (int) prev_robot_y)) 
            if(robot_is_moving()) {
                if (fabs(pose_x - prev_robot_x) < 0.1 && fabs(pose_y - prev_robot_y) < 0.1 ) 
                { 
                    //if(robot_state == moving_to_frontier || robot_state == going_charging || robot_state == going_checking_vacancy) {
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
                        if(robot_state == going_charging) {
                            log_minor_error("Force the robot to think that it has reached the target DS");
                            update_robot_state_2(charging);
                        }
                        else if(robot_state == leaving_ds)
                        {
                            log_minor_error("Force the robot to think that it has left the target DS");
                            update_robot_state_2(exploring);
                        }
                        else if(robot_state == going_in_queue) {
                            log_minor_error("Force the robot to think that it reached the queue");
                            update_robot_state_2(in_queue);
                        }
                        else if(robot_state == going_checking_vacancy) {
                            log_minor_error("Force the robot to think that it reached the DS to check vavancy");
                            update_robot_state_2(checking_vacancy);
                        }
                        else {
                            if(retries_moving < 3) {
                                log_minor_error("Robot is not moving anymore... retrying");
                                retries_moving++;
                                update_robot_state_2(exploring);
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
                    //ROS_ERROR("state: %d - %d", prev_robot_state, robot_state);
                    //ROS_ERROR("x: %f - %f", prev_robot_x, pose_x);
                    //ROS_ERROR("y: %f - %f", prev_robot_y, pose_y);
                    //if(robot_state == moving_to_frontier || robot_state == going_charging || robot_state == going_checking_vacancy) //TODO complete
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
            
            else if( robot_state != in_queue && robot_state != charging) {
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
            
            //ROS_ERROR("%f, %f", pose_x, pose_y);

            if( (stuck_x - pose_x) * (stuck_x - pose_x) + (stuck_y - pose_y) * (stuck_y - pose_y) >= 5*5 ) //pose_x and pose_y are in cells, not meters
                ;
                // already_perfomed_recovery_procedure = false;
            
            prev_time = ros::Time::now();
            ros::Duration(sleeping_time).sleep();

        }
        
        ROS_INFO("safety checks have been stopped");
    
    }
    
    void log_major_error(std::string text) {
        ROS_FATAL("%s", text.c_str());
        ROS_INFO("%s", text.c_str());
        
        major_errors++;
        
        major_errors_fstream.open(major_errors_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        major_errors_fstream << "[MAJOR] " << robot_id << ": " << text << std::endl;
        major_errors_fstream.close();

        std::stringstream robot_number;
        std::stringstream error_counter;
        robot_number << robot_id;
        error_counter << major_errors;
        std::string prefix = "/robot_";
        
        std::string status_directory = "/simulation_status_error";
        std::string robo_name = prefix.append(robot_number.str());
        std::string file_name = robo_name.append(error_counter.str());
        std::string file_suffix(".major_error");

        std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
        std::string status_path = ros_package_path + status_directory;
        std::string status_file = status_path + file_name + file_suffix;

        // TODO(minor): check whether directory exists
        boost::filesystem::path boost_status_path(status_path.c_str());
        if(!boost::filesystem::exists(boost_status_path))
            if(!boost::filesystem::create_directories(boost_status_path))
                ROS_ERROR("Cannot create directory %s.", status_path.c_str());
        std::ofstream outfile(status_file.c_str());
        outfile.close();
        ROS_INFO("Creating file %s to indicate error",
        status_file.c_str());
        
    }
    
    void log_minor_error(std::string text) {
        ROS_FATAL("%s", text.c_str());
        ROS_INFO("%s", text.c_str());
        
        minor_errors++;
        
        major_errors_fstream.open(major_errors_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        major_errors_fstream << "[minor] " << robot_id << ": " << text << std::endl;
        major_errors_fstream.close();

        std::stringstream robot_number;
        std::stringstream error_counter;
        robot_number << robot_id;
        error_counter << minor_errors;
        std::string prefix = "/robot_";
        
        std::string status_directory = "/simulation_status_minor_error";
        std::string robo_name = prefix.append(robot_number.str());
        std::string file_name = robo_name.append(error_counter.str());
        std::string file_suffix(".minor_error");

        std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
        std::string status_path = ros_package_path + status_directory;
        std::string status_file = status_path + file_name + file_suffix;

        // TODO(minor): check whether directory exists
        boost::filesystem::path boost_status_path(status_path.c_str());
        if(!boost::filesystem::exists(boost_status_path))
            if(!boost::filesystem::create_directories(boost_status_path))
                ROS_ERROR("Cannot create directory %s.", status_path.c_str());
        std::ofstream outfile(status_file.c_str());
        outfile.close();
        ROS_INFO("Creating file %s to indicate error",
        status_file.c_str());
        
    }
    
    void move_home_if_possible() {
        ros::spinOnce();
        if(exploration->home_is_reachable(available_distance)) {
            bool completed_navigation = false;
            for (int i = 0; i < 5; i++)
            {
                if (completed_navigation == false)
                {
                    counter++;
                    going_home = true;
                    completed_navigation = move_robot(counter, home_point_x, home_point_y);
                    going_home = false;
                }
                else
                {
                    break;
                }
            }
            finalize_exploration();
        }
        else {
            going_home = true;
            counter++;
            move_robot_away(counter);
            update_robot_state_2(auctioning_3);
        }
    }
    
   void move_home() {
        ros::spinOnce();
        if(exploration->home_is_reachable(available_distance)) {
            bool completed_navigation = false;
            for (int i = 0; i < 5; i++)
            {
                if (completed_navigation == false)
                {
                    counter++;
                    going_home = true;
                    completed_navigation = move_robot(counter, home_point_x, home_point_y);
                    going_home = false;
                }
                else
                {
                    break;
                }
            }
            finalize_exploration();
        }
        else {
            log_major_error("robot has finished the exploration but cannot reach home!");
            finalize_exploration();
        }
    }
    
    void print_mutex_info(std::string function_name, std::string action) {
        log_mutex.lock();
        lock_file = log_path + std::string("lock.log");
        lock_fstream.open(lock_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        lock_fstream << ros::Time::now() - time_start << ": " << function_name << ": " << action << std::endl;
        lock_fstream.close();
        log_mutex.unlock();
    }
    
    void store_travelled_distance() {
        ROS_INFO("Storing travelled distance");
        exploration->trajectory_plan_store(starting_x, starting_y);
        ROS_INFO("Stored");
        starting_x = pose_x;
        starting_y = pose_y;
    }
    
    void force_in_queue_callback(const std_msgs::Empty msg) {
        ROS_INFO("forced to go idle by another node");
        update_robot_state_2(in_queue);
    }
    
    void update_distances() {
        while(!explorer_ready)
            ros::Duration(10).sleep();
            
        while(!exploration_finished) {
            exploration->updateDistances(conservative_maximum_available_distance);
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

    std::string csv_file, csv_state_file, log_file, exploration_start_end_log, revocery_log, major_errors_file, lock_file, computation_time_log;
    std::string log_path, original_log_path;
    std::fstream fs_csv, fs_csv_state, fs, fs_exp_se_log, major_errors_fstream, lock_fstream, fs_computation_time;

    int number_of_recharges = 0;

    bool test;
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
        leaving_ds,                                // the robot was recharging, but another robot stopped
        dead,
        moving_away_from_ds,
        auctioning_3,
        stopped,
        exploring_for_graph_navigation
    };
    state_t robot_state, previous_state;

    // TODO

    std::vector<std::string> enum_string;

    std::string get_text_for_enum(int enumVal)
    {
        if((unsigned int)enumVal >= enum_string.size()) {
            log_major_error("segmenv in get_text_for_enum");
            return "";
        }
        else
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
        finished_next
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

    exploration_finished = false;

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
    
    boost::thread thr_log_map(boost::bind(&Explorer::log_map, &explorer));

    /* Create thread to periodically publish unexplored frontiers */
    boost::thread thr_frontiers(boost::bind(&Explorer::frontiers, &explorer));
    
    boost::thread thr_safety_checks(boost::bind(&Explorer::safety_checks, &explorer));
    
    boost::thread thr_update_distances(boost::bind(&Explorer::update_distances, &explorer));

    /*
     * FIXME
     * Which rate is required in order not to oversee
     * any callback data (frontiers, negotiation ...)
     */
    while (ros::ok())
    {
        if(!exploration_finished) { //TODO actually we should termine the thread when the exploration is over...
            //explorer.print_mutex_info("main()", "acquiring");
            //ROS_DEBUG("acquiring");
            //costmap_mutex.lock();  
            //explorer.print_mutex_info("main()", "lock");
            //ROS_DEBUG("lock");
            ros::spinOnce();
            //costmap_mutex.unlock();
            //explorer.print_mutex_info("main()", "unlock");
            //ROS_DEBUG("unlock");
        }
        ros::Duration(0.1).sleep();
    }

    /* Terminate threads */
    thr_explore.interrupt();
    thr_map.interrupt();
    thr_explore.join();
    thr_map.join();
    thr_log_map.interrupt();
    thr_log_map.join();
    thr_update_distances.interrupt();
    thr_update_distances.join();
    thr_safety_checks.interrupt();
    thr_safety_checks.join();
    thr_frontiers.interrupt();
    thr_frontiers.join();
    // TODO thr_frontiers...

#ifdef PROFILE
    HeapProfilerStop();
    ProfilerStop();
#endif

    return 0;
}

