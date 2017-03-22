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

//#define PROFILE

#ifdef PROFILE
    #include "google/profiler.h"
    #include "google/heap-profiler.h"
#endif

#define OPERATE_ON_GLOBAL_MAP true
#define OPERATE_WITH_GOAL_BACKOFF true
//#define EXIT_COUNTDOWN 5
#define EXIT_COUNTDOWN 500 //F
//#define STUCK_COUNTDOWN 10
#define STUCK_COUNTDOWN 1000 //F

#define SAFETY_COEFF 0.015

boost::mutex costmap_mutex;

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        ros::Duration(1.0).sleep();
}

class Explorer {

public:
    
    //F
    bool test, winner_of_auction;
    ros::Publisher pub_robot_pos;
    //ros::Subscriber sub_auction_completed;
    enum state_t {exploring, going_charging, charging, finished, fully_charged, stuck, in_queue, auctioning, reaching_frontier_before_going_charging};
    state_t robot_state;

    Explorer(tf::TransformListener& tf) : counter(0), rotation_counter(0), nh("~"), number_of_robots(1), accessing_cluster(0), cluster_element_size(0), cluster_flag(false), cluster_element(-1), cluster_initialize_flag(false), global_iterations(0), global_iterations_counter(0), counter_waiting_for_clusters(0), global_costmap_iteration(0), robot_prefix_empty(false), robot_id(0), battery_charge(100), recharge_cycles(0), battery_charge_temp(100), energy_consumption(0), available_distance(0),
    //robot_state(fully_charged),
    charge_time(0), pose_x(0), pose_y(0), pose_angle(0), prev_pose_x(0), prev_pose_y(0), prev_pose_angle(0)
    {
    
         if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
            ros::console::notifyLoggerLevelsChanged();
        
        //available_distance = 100; //F
        //F
        test = true;
        winner_of_auction = false;
        robot_state == fully_charged;
        pub_robot_pos = nh.advertise<move_base_msgs::MoveBaseGoal>("robot_position", 1);
        //sub_auction_completed = nh.subscribe("auction_completed", 1, &Explorer::auction_completed_callback, this);
        
        

        nh.param("frontier_selection",frontier_selection,1);
        nh.param("local_costmap/width",costmap_width,0);
        nh.param<double>("local_costmap/resolution",costmap_resolution,0);
        nh.param("number_unreachable_for_cluster", number_unreachable_frontiers_for_cluster,3);
        nh.param("recharging", recharging, false);

        nh.param("w1", w1, 1);
        nh.param("w2", w2, 0);
        nh.param("w3", w3, 0);
        nh.param("w4", w4, 0);

        ROS_INFO("Costmap width: %d", costmap_width);
        ROS_INFO("Frontier selection is set to: %d", frontier_selection);

        srand((unsigned)time(0));

        nh.param<std::string>("move_base_frame",move_base_frame,"map");
        nh.param<int>("wait_for_planner_result",waitForResult,3);

        // determine host name
        nh.param<std::string>("robot_prefix",robot_prefix,"");
        ROS_INFO("robot prefix: \"%s\"", robot_prefix.c_str());

        // create map_merger service
        std::string service = robot_prefix + std::string("/map_merger/logOutput");
        mm_log_client = nh.serviceClient<map_merger::LogMaps>(service.c_str());

        if(robot_prefix.empty())
        {
            char hostname_c[1024];
            hostname_c[1023] = '\0';
            gethostname(hostname_c, 1023);
            robot_name = std::string(hostname_c);
            ROS_INFO("NO SIMULATION! Robot name: %s", robot_name.c_str());

            /*
             * THIS IS REQUIRED TO PERFORM COORDINATED EXPLORATION
             * Assign numbers to robot host names in order to make
             * auctioning and frontier selection UNIQUE !!!!!
             * To use explorer node on a real robot system, add your robot names
             * here and at ExplorationPlanner::lookupRobotName function ...
             */
            std::string bob = "bob";
            std::string marley = "marley";
            std::string turtlebot = "turtlebot";
            std::string joy = "joy";
            std::string hans = "hans";

            if(robot_name.compare(turtlebot) == 0)
                robot_id = 0;
            if(robot_name.compare(joy) == 0)
                robot_id = 1;
            if(robot_name.compare(marley) == 0)
                robot_id = 2;
            if(robot_name.compare(bob) == 0)
                robot_id = 3;
            if(robot_name.compare(hans) == 0)
                robot_id = 4;

            robot_prefix_empty = true;
            ROS_INFO("Robot name: %s    robot_id: %d", robot_name.c_str(), robot_id);
        }else
        {
            robot_name = robot_prefix;
            ROS_INFO("Move_base_frame: %s",move_base_frame.c_str());
            robot_id = atoi(move_base_frame.substr(7,1).c_str());

            ROS_INFO("Robot name: %s    robot_id: %d", robot_name.c_str(), robot_id);
        }

        
        
        

       /*
        *  CREATE LOG PATH
        * Following code enables to write the output to a file
        * which is localized at the log_path
        */
        initLogPath();
        csv_file = log_path + std::string("periodical.log");
        log_file = log_path + std::string("exploration.log");

		ROS_DEBUG("*********************************************");
		ROS_INFO("******* Initializing Simple Navigation ******");
		ROS_DEBUG("                                             ");

		ROS_DEBUG("Creating global costmap ...");
		costmap2d_global = new costmap_2d::Costmap2DROS("global_costmap", tf);
		ROS_DEBUG("Global costmap created ... now performing costmap -> pause");
		costmap2d_global->pause();
		ROS_DEBUG("Pausing performed");

		ROS_DEBUG("                                             ");

        if(OPERATE_ON_GLOBAL_MAP == true)
        //if(false)
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
        
        //ROS_ERROR("\n\t\e[1;34m Global costmap origin: (%f, %f) \e[0m\n", costmap2d_global->getCostmap()->getOriginX(), costmap2d_global->getCostmap()->getOriginY());
        //ROS_ERROR("\n\t\e[1;34m Local costmap origin: (%f, %f) \e[0m\n", costmap2d_local->getCostmap()->getOriginX(), costmap2d_local->getCostmap()->getOriginY());
        

		ROS_INFO("---------------- COSTMAP DONE ---------------");

		/*
		 * Set the first goal as PointStamped message to visualize in RVIZ.
		 * RVIZ requires a minimal history length of 1, which means that at least
		 * one entry has to be buffered, before the first Goal is able to be
		 * visualized. Therefore set the "first" goal to the point of origin
		 * (home position).
		 */

		if (!costmap2d_local->getRobotPose(robotPose)) {
			ROS_ERROR("Failed to get RobotPose");
		}
		visualize_goal_point(robotPose.getOrigin().getX(),
				robotPose.getOrigin().getY());
				
		//ROS_ERROR("\n\t\e[1;34m Robot starting position in local map: (%f, %f) \e[0m\n", robotPose.getOrigin().getX(), robotPose.getOrigin().getY());

		// transmit three times, since rviz need at least 1 to buffer before visualizing the point
		for (int i = 0; i <= 2; i++) {
			visualize_home_point();
		}

		ROS_INFO("---------- SET HOME/ GOAL POINT DONE --------");

		// instantiate the planner
		exploration = new explorationPlanner::ExplorationPlanner(robot_id, robot_prefix_empty, robot_name);

		/*
		 * Define the first Goal. This is required to have at least one entry
		 * within the vector. Therefore set it to the home position.
		 */

        robot_home_position_x = robotPose.getOrigin().getX();
        robot_home_position_y = robotPose.getOrigin().getY();

        exploration->next_auction_position_x = robotPose.getOrigin().getX();
        exploration->next_auction_position_y = robotPose.getOrigin().getY();

        exploration->storeVisitedFrontier(robot_home_position_x,robot_home_position_y, robot_id, robot_name, -1);
        exploration->storeFrontier(robot_home_position_x,robot_home_position_y, robot_id, robot_name, -1);

        exploration->setRobotConfig(robot_id, robot_home_position_x, robot_home_position_y, move_base_frame);

		ROS_INFO("                                             ");
		ROS_INFO("************* INITIALIZING DONE *************");

	}
	
	
	//void battery_charging_completed_callback(const std_msgs::Empty::ConstPtr& msg)  { //F
	void battery_charging_completed_callback(const std_msgs::Empty::ConstPtr& msg)  { //F
        ROS_ERROR("\n\t\e[1;34m Received charging complete! \e[0m\n");
        robot_state = fully_charged;
    }
    
    
    void docking_station_detected_callback(const std_msgs::Empty::ConstPtr& msg) {
        //ROS_ERROR("4444444");
        if(false) {
            //ROS_ERROR("Storing new DS position!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            //ROS_ERROR("home_point_x = %f; home_point_y: %f", home_point_x, home_point_y);
            //home_point_x = home_point_x + 1;
            //home_point_y = home_point_y + 1;
            test = false;
        }
        
    }
    
    
    void new_best_docking_station_selected_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        //ROS_ERROR("4444444");
        //ROS_ERROR("Storing new best DS position");
        ROS_INFO("Storing new best DS position");
        //ROS_ERROR("home_point_x = %f; home_point_y: %f", home_point_x, home_point_y);
        home_point_x = msg.get()->point.x;
        home_point_y = msg.get()->point.y;
        
        
        map_merger::TransformPoint point;
        if(robot_prefix == "/robot_1")
            point.request.point.src_robot = "robot_0";
        else if(robot_prefix == "/robot_0")
            point.request.point.src_robot = "robot_1";
        else
            ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        point.request.point.x = home_point_x;
        point.request.point.y = home_point_y;
        
        
        ros::ServiceClient sc_trasform = nh.serviceClient<map_merger::TransformPoint>("map_merger/transformPoint");
        if(!sc_trasform.call(point))
            ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
       //ROS_ERROR("\e[1;34mCalling: %s\e[0m", sc_trasform.getService().c_str());sc_trasform.call(point);
               //ROS_ERROR("\e[1;34mOriginal point:   (%f, %f)\e[0m", point.request.point.x, point.request.point.y);
        //ROS_ERROR("\e[1;34mTansformed point: (%f, %f)\e[0m", point.response.point.x, point.response.point.y);
       
        
    }
	
	
    void bat_callback(const energy_mgmt::battery_state::ConstPtr& msg)
    {
        //ROS_ERROR("Received battery state");
        battery_charge = (int) msg->soc;
        charge_time = msg->remaining_time_charge;
        available_distance = msg->remaining_distance;
        //ROS_ERROR("%f", available_distance);

        if(msg->charging == false && battery_charge == 100 && charge_time == 0){
            //robot_state = fully_charged;
            //ROS_ERROR("22222222222222222222222222222222222222222222222222222222");
            recharge_cycles++;
        }

        // robot is out of energy, exit
        if(available_distance <= 0 && robot_state != charging)
        {
            ROS_ERROR("Robot has run out of energy!");
            finalize_exploration();
        }
    }

    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
    {
      // Get rotation of robot relative to starting position
      tf::Quaternion orientation = tf::Quaternion(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);
      pose_angle = orientation.getAngle();

      // Robot position
      pose_x = pose->pose.pose.position.x;
      pose_y = pose->pose.pose.position.y;
    }

	void explore()
    {
        
		/*
		 * Sleep is required to get the actual a
		 * costmap updated with obstacle and inflated
		 * obstacle information. This is rquired for the
		 * first time explore() is called.
		 */
        ROS_DEBUG("Sleeping 5s for costmaps to be updated.");
        geometry_msgs::Twist twi;
        //should be a parameter!! only for testing on Wed/17/9/14
        ros::Publisher twi_publisher = nh.advertise<geometry_msgs::Twist>("/Rosaria/cmd_vel",1);
        twi.angular.z = 0.75;
        twi_publisher.publish(twi);
        ros::Duration(5.0).sleep();
        twi_publisher.publish(twi);
        
        

        int exit_countdown = EXIT_COUNTDOWN;
        int charge_countdown = EXIT_COUNTDOWN;

        /*
         * START TAKING THE TIME DURING EXPLORATION
         */
        time_start = ros::Time::now();
        ros::NodeHandle nh;
        std_msgs::Empty msg;
       // msg.data = "traveling home to recharge";
        ros::Publisher publisher_re = nh.advertise<std_msgs::Empty>("going_to_recharge",1);
        ros::Publisher pub_going_charging = nh.advertise<std_msgs::Empty>("going_charging", 1);
        //ROS_ERROR("\n\t\e[1;34m%s\e[0m\n", publisher_re.getTopic().c_str());
        ros::Subscriber sub, sub2, sub3, pose_sub;
        
        ros::Subscriber my_sub = nh.subscribe("charging_completed", 1, &Explorer::battery_charging_completed_callback, this);   ;//F
        ros::Subscriber sub_ds = nh.subscribe("docking_station_detected", 1, &Explorer::docking_station_detected_callback, this);  //F
        ros::Subscriber sub_new_best_ds = nh.subscribe("new_best_docking_station_selected", 1, &Explorer::new_best_docking_station_selected_callback, this);  //F
        ros::Subscriber sub_auction_completed = nh.subscribe("auction_completed", 1, &Explorer::auction_completed_callback, this);
        ros::Subscriber sub_auction_winner = nh.subscribe("auction_winner", 1, &Explorer::auction_winner_callback, this);
        ros::Subscriber sub_auction_loser = nh.subscribe("auction_loser", 1, &Explorer::auction_loser_callback, this);
        
        

        // subscribe to battery management topics
        sub = nh.subscribe("battery_state",1,&Explorer::bat_callback,this);

        // Subscribe to robot pose to check if robot is stuck
        pose_sub = nh.subscribe("amcl_pose", 1, &Explorer::poseCallback, this);

		while (robot_state != finished)
        {
            ROS_INFO("EXPLORING");
            // do nothing while recharging
            if(robot_state == charging)
            {
                ROS_ERROR("Waiting for battery to charge...");
                if(charge_time > 0)
                    ros::Duration(charge_time).sleep();
                else
                    ros::Duration(1).sleep();
                ros::spinOnce(); // update charge_time
                continue;
            }

            /*
            * *****************************************************
            * FRONTIER DETERMINATION
            * *****************************************************
            */
            std::vector<double> final_goal;
            std::vector<double> backoffGoal;
            std::vector<std::string> robot_str;

            bool navigate_to_goal = false;
            bool negotiation;
            int count = 0;

            ROS_INFO("****************** EXPLORE ******************");

            /*
            * Use mutex to lock the critical section (access to the costmap)
            * since rosspin tries to update the costmap continuously
            */
            ROS_INFO("COSTMAP STUFF");
            costmap_mutex.lock();

            exploration->transformToOwnCoordinates_frontiers();
            exploration->transformToOwnCoordinates_visited_frontiers();

            exploration->initialize_planner("exploration planner", costmap2d_local, costmap2d_global);
            exploration->findFrontiers();

            exploration->clearVisitedFrontiers();
            exploration->clearUnreachableFrontiers();
            exploration->clearSeenFrontiers(costmap2d_global);

            costmap_mutex.unlock();
            ROS_INFO("PUBLISHING FRONTIERS");

            /*exploration->publish_frontier_list();
            exploration->publish_visited_frontier_list();

            ros::Duration(1).sleep();
            exploration->publish_frontier_list();
            exploration->publish_visited_frontier_list();*/

            /*
            * Sleep to ensure that frontiers are exchanged
            */
            ros::Duration(1).sleep();

            if(robot_state == exploring || robot_state == fully_charged)
            {

                /*
                * *****************************************************
                * FRONTIER SELECTION
                * *****************************************************
                */

                /*********** EXPLORATION STRATEGY ************
                * 0 ... Navigate to nearest frontier TRAVEL PATH
                * 1 ... Navigate using auctioning with cluster selection using NEAREST selection (Kuhn-Munkres)
                * 2 ... Navigate to furthest frontier
                * 3 ... Navigate to nearest frontier EUCLIDEAN DISTANCE
                * 4 ... Navigate to random Frontier
                * 5 ... Cluster frontiers, then navigate to nearest cluster using EUCLIDEAN DISTANCE (with and without negotiation)
                * 6 ... Cluster frontiers, then navigate to random cluster (with and without negotiation)
                *
                * ENERGY AWARE STRATEGIES:
                * 7 ... Navigate to frontier that satisfies energy efficient cost function with staying-alive path planning
                * 8 ... Navigate to leftmost frontier (Mei et al. 2006) with staying-alive path planning
                * 9 ... Just like strategy 0 but with staying-alive path planning
                */

                /******************** SORT *******************
                * Choose which strategy to take.
                * 1 ... Sort the buffer from furthest to nearest frontier
                * 2 ... Sort the buffer from nearest to furthest frontier, normalized to the
                *       robots actual position (EUCLIDEAN DISTANCE)
                * 3 ... Sort the last 10 entries to shortest TRAVEL PATH
                * 4 ... Sort all cluster elements from nearest to furthest (EUCLIDEAN DISTANCE)
                * 5 ...
                * 6 ...
                * 7 ... Sort frontiers in sensor range clock wise (starting from left of robot)
                *       and sort the remaining frontiers from nearest to furthest (EUCLIDEAN DISTANCE)
                */

                if(frontier_selection == 0)
                {
                    exploration->sort(2);
                    exploration->sort(3);

                    while(true)
                    {
                        goal_determined = exploration->determine_goal(2, &final_goal, count, 0, &robot_str);
                        ROS_DEBUG("Goal_determined: %d   counter: %d",goal_determined, count);
                        if(goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            //negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
                            negotiation = true;
                            if(negotiation == true)
                            {
                                break;
                            }
                            count++;
                        }
                    }
                }
                else if(frontier_selection == 1)
                {
                    costmap_mutex.lock();
                    if(cluster_initialize_flag == true)
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

                    while(true)
                    {
                        final_goal.clear();
                        robot_str.clear();
                        goal_determined = exploration->determine_goal(5, &final_goal, 0, cluster_element, &robot_str);

                        ROS_ERROR("Cluster element: %d", cluster_element);

                        int cluster_vector_position = -1;

                        if(cluster_element != -1)
                        {
                            if(exploration->clusters.size() > 0)
                            {
                                for (int i = 0; i < exploration->clusters.size(); i++)
                                {
                                    if(exploration->clusters.at(i).id == cluster_element)
                                    {
                                        if(exploration->clusters.at(i).cluster_element.size() > 0)
                                        {
                                            cluster_vector_position = i;
                                        }
                                        break;
                                    }
                                }
                            }
                        }
                        ROS_ERROR("Cluster vector position: %d", cluster_vector_position);

                        if(cluster_vector_position >= 0)
                        {
                            if(exploration->clusters.at(cluster_vector_position).unreachable_frontier_count >= number_unreachable_frontiers_for_cluster)
                            {
                                goal_determined = false;
                                ROS_ERROR("Cluster inoperateable");
                            }else
                            {
                                ROS_ERROR("Cluster operateable");
                            }
                        }

                        if(goal_determined == false)
                        {

                            ROS_INFO("No goal was determined, cluster is empty. Bid for another one");

                            final_goal.clear();
                            robot_str.clear();
                            clusters_available_in_pool.clear();

                            bool auctioning = exploration->auctioning(&final_goal, &clusters_available_in_pool, &robot_str);
                            if(auctioning == true)
                            {
                                goal_determined = true;
                                cluster_element = final_goal.at(4);
                                counter_waiting_for_clusters = 0;
                                break;
                            }
                            else
                            {
                                if(exploration->clusters.size() > 0) //clusters_available_in_pool.size() > 0)
                                {
                                    ROS_INFO("No cluster was selected but other robots are operating ... waiting for new clusters");
                                    counter_waiting_for_clusters++;
                                    break;
                                }else
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
                else if(frontier_selection == 2)
                {

                    exploration->sort(1);

                    /*
                    * Choose which strategy to take.
                    * 1 ... least to first goal in list
                    * 2 ... first to last goal in list
                    *
                    * Determine_goal() takes world coordinates stored in
                    * the frontiers list(which contain every found frontier with sufficient
                    * spacing in between) and returns the most attractive one, based on the
                    * above defined strategies.
                    */
                    while(true)
                    {
                        goal_determined = exploration->determine_goal(1, &final_goal, count, 0, &robot_str);
                        if(goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            //negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
                            negotiation = true;
                            if(negotiation == true)
                            {
                                break;
                            }
                            count++;
                        }
                    }
                }
                else if(frontier_selection == 3)
                {
                    exploration->sort(2);

                    while(true)
                    {
                        goal_determined = exploration->determine_goal(2, &final_goal, count, 0, &robot_str);
                        if(goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            //negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
                            negotiation = true;
                            if(negotiation == true)
                            {
                                break;
                            }
                            count++;
                        }
                    }
                }
                else if(frontier_selection == 4)
                {
                    while(true)
                    {
                        goal_determined = exploration->determine_goal(3, &final_goal, count, 0, &robot_str);
                        ROS_DEBUG("Goal_determined: %d   counter: %d",goal_determined, count);
                        if(goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            //negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
                            negotiation = true;
                            if(negotiation == true)
                            {
                                break;
                            }
                            count++;
                        }
                    }

                }
                else if(frontier_selection == 5)
                {
                    if(cluster_initialize_flag == true)
                    {
                        exploration->clearVisitedAndSeenFrontiersFromClusters();
                    }
                    else
                    {
                        /*
                        * This is necessary for the first run
                        */
                        if(robot_id == 0)
                            ros::Duration(10).sleep();

                        exploration->sort(2);
                    }

                    exploration->clusterFrontiers();
                    exploration->sort(4);
                    exploration->sort(5);

                    clusters_available_in_pool.clear();
                    while(true)
                    {
                        final_goal.clear();
                        goal_determined = exploration->determine_goal(4, &final_goal, count, cluster_element, &robot_str);

                        if(goal_determined == false)
                        {
                            ROS_INFO("Another cluster is not available, no cluster determined");
                            break;
                        }
                        else
                        {
                            if(cluster_initialize_flag == false)
                            {
                                /*
                                *  TODO ... just to reduce the selection of the same
                                * clusters since no auctioning is implemented jet.
                                */
                            if(robot_id == 1)
                            {
                                ros::Duration(5).sleep();
                            }

                            cluster_initialize_flag = true;
                            }

                            /*
                            * If negotiation is not needed, simply uncomment
                            * and set the negotiation to TRUE.
                            */
                            negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),final_goal.at(4));
                            if(negotiation == true)
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
                            * another cluster to operate in, one have to fill the clusters_available_in_pool
                            * vector if count is increased at least once which determines
                            * that at least one cluster
                            */
                            clusters_available_in_pool.push_back(1);
                        }
                    }

                }
                else if(frontier_selection == 6)
                {
                    if(cluster_initialize_flag == true)
                    {
                        exploration->clearVisitedAndSeenFrontiersFromClusters();
                    }

                    exploration->clusterFrontiers();
                    exploration->sort(6);
                    exploration->visualizeClustersConsole();

                    if(cluster_initialize_flag == false)
                    {
                        ros::Duration(2).sleep();
                    }
                    cluster_initialize_flag = true;

                    while(true)
                    {
                        goal_determined = exploration->determine_goal(4, &final_goal, count, cluster_element, &robot_str);

                        if(goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            /*
                            * If negotiation is not needed, simply uncomment
                            * and set the negotiation to TRUE.
                            */
                            negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),final_goal.at(4));

                            if(negotiation == true)
                            {
                                cluster_element = final_goal.at(4);
                                break;
                            }
                            else
                            {
                                cluster_element = int(exploration->clusters.size()*rand()/(RAND_MAX));
                                ROS_INFO("Random cluster_element: %d  from available %lu clusters", cluster_element, exploration->clusters.size());
                            }
                            count++;
                        }
                    }
                }
                else if(frontier_selection == 7)
                {
                    // first sort the frontiers from near to far and then by efficiency
                    ROS_INFO("SORTING FRONTIERS...");
                    exploration->sort(2);
                    exploration->sort(3);
                    exploration->sort_cost(battery_charge > 50, w1, w2, w3, w4);

                    // look for a frontier as goal
                    ROS_INFO("DETERMINE GOAL...");
                    //goal_determined = exploration->determine_goal_staying_alive(1, 2, available_distance, &final_goal, count, &robot_str, -1);
                    goal_determined = exploration->determine_goal_staying_alive(1, 2, available_distance*SAFETY_COEFF, &final_goal, count, &robot_str, -1);
                    //ROS_INFO("Goal_determined: %d   counter: %d",goal_determined, count);
                    //ROS_ERROR("Goal_determined: %d   counter: %d",goal_determined, count);

                    // found a frontier, go there
                    if(goal_determined == true)
                    {
                        robot_state = exploring;
                        exit_countdown = EXIT_COUNTDOWN;
                        charge_countdown = EXIT_COUNTDOWN;
                    }

                    // robot cannot reach any frontier, even if fully charged
                    // simulation is over
                    else if(recharging == false || robot_state == fully_charged)
                    {
                        exit_countdown--;
                        ROS_ERROR("Shutdown in: %d", exit_countdown);
                        if(exit_countdown <= 0)
                            finalize_exploration();
                        continue;
                    }

                    // robot cannot reach any frontier
                    // go charging
                    else
                    {
                        if(charge_countdown > 0) //F
                            charge_countdown = 0; //F
                        charge_countdown--;
                        if(charge_countdown <= 0){
                            //ROS_INFO("Could not determine goal, need to recharge!");
                            ROS_ERROR("Could not determine goal, need to recharge!");
                            //robot_state = going_charging;
                            robot_state = auctioning;
                            std_msgs::Empty msg;
                            pub_going_charging.publish(msg);
                        }
                        else
                            continue;
                    }
                }
                else if(frontier_selection == 8)
                {
                    // sort frontiers clock wise starting from left
                    exploration->sort(7);

                    // look for a frontier as goal
                    ROS_INFO("DETERMINE GOAL...");
                    goal_determined = exploration->determine_goal_staying_alive(1, 2, available_distance, &final_goal, count, &robot_str, -1);
                    ROS_INFO("Goal_determined: %d   counter: %d",goal_determined, count);

                    // found a frontier, go there
                    if(goal_determined == true)
                    {
                        robot_state = exploring;
                        exit_countdown = EXIT_COUNTDOWN;
                        charge_countdown = EXIT_COUNTDOWN;
                    }

                    // robot cannot reach any frontier, even if fully charged
                    // simulation is over
                    else if(recharging == false || robot_state == fully_charged)
                    {
                        exit_countdown--;
                        ROS_ERROR("Shutdown in: %d", exit_countdown);
                        if(exit_countdown <= 0)
                            finalize_exploration();
                        continue;
                    }

                    // robot cannot reach any frontier
                    // go charging
                    else
                    {
                        charge_countdown--;
                        if(charge_countdown <= 0){
                            ROS_INFO("Could not determine goal, need to recharge!");
                            robot_state = going_charging;
                        }
                        else
                            continue;
                    }
                }
                else if(frontier_selection == 9)
                {
                    // sort the frontiers from near to far
                    exploration->sort(2);
                    exploration->sort(3);

                    // look for a frontier as goal
                    goal_determined = exploration->determine_goal_staying_alive(1, 2, available_distance, &final_goal, count, &robot_str, -1);
                    ROS_DEBUG("Goal_determined: %d   counter: %d",goal_determined, count);

                    // found a frontier
                    // go there
                    if(goal_determined == true)
                    {
                        robot_state = exploring;
                        exit_countdown = EXIT_COUNTDOWN;
                        charge_countdown = EXIT_COUNTDOWN;
                    }

                    // robot cannot reach any frontier, even if fully charged
                    // simulation is over
                    else if(recharging == false || robot_state == fully_charged)
                    {
                        exit_countdown--;
                        ROS_ERROR("Shutdown in: %d", exit_countdown);
                        if(exit_countdown <= 0)
                            finalize_exploration();
                        continue;
                    }

                    // robot cannot reach any frontier
                    // go charging
                    else
                    {
                        charge_countdown--;
                        if(charge_countdown <= 0){
                            ROS_INFO("Could not determine goal, need to recharge!");
                            robot_state = going_charging;
                        }
                        else
                            continue;
                    }
                }
            }


             /*
             * *****************************************************
             * FRONTIER COORDINATION
             * *****************************************************
             */

            exploration->visualize_Cluster_Cells();
            exploration->visualize_Frontiers();

            ROS_INFO("Navigating to Goal");

            // navigate robot to next frontier
            if(robot_state == exploring)
            {
                if(OPERATE_WITH_GOAL_BACKOFF == true)
                {
                    ROS_INFO("Doing smartGoalBackoff");
                    if(exploration->smartGoalBackoff(final_goal.at(0),final_goal.at(1), costmap2d_global, &backoffGoal))
                    {
                        ROS_INFO("Navigate to backoff goal (%.2f,%.2f) --> (%.2f,%.2f)", final_goal.at(0), final_goal.at(1), backoffGoal.at(0), backoffGoal.at(1));
                        navigate_to_goal = navigate(backoffGoal);
                    }else
                    {
                        ROS_ERROR("Failed to find backoff goal, mark original goal (%.2f,%.2f) as unreachable", final_goal.at(0), final_goal.at(1));
                        navigate_to_goal = false; // navigate(final_goal);
                    }
                }else
                {
                    ROS_INFO("Navigate to goal");
                    navigate_to_goal = navigate(final_goal);
                }
            }
            
            else if(robot_state == auctioning) { 
                ROS_ERROR("\n\t\e[1;34mAuctioning...\e[0m\n");
                while(robot_state == auctioning) {
                    //ROS_ERROR("\n\t\e[1;34mAuctioning...\e[0m\n");
                    ros::Duration(0.1).sleep();
                    ros::spinOnce();
                }
                ROS_ERROR("\n\t\e[1;34mAuction completed\e[0m\n");
            }

            // navigate robot home for recharging
            if(robot_state == going_charging)
            {
                    ROS_ERROR("Traveling home for recharging");
                    //ROS_ERROR("home_point_x = %f; home_point_y: %f", home_point_x, home_point_y);
                    counter++;
                    
                    
                    navigate_to_goal = move_robot(counter, home_point_x, home_point_y);
            }
            //F: I exit from move_robot only when the goal has been reached
            
            
                
            
            

            // result of navigation successful
            if(navigate_to_goal == true)
            {
                // robot reached home point, start recharging
                if(robot_state == going_charging)
                {
                    ROS_ERROR("At home for recharging");
                    // compute path length
                    exploration->trajectory_plan_store(home_point_x, home_point_y);

                    publisher_re.publish(msg); //F
                    
                    robot_state = charging; //F
                    
                }

                // robot reached frontier
                else if(robot_state == exploring)
                {
                    ROS_INFO("STORING PATH");
                    // compute path length
                    exploration->trajectory_plan_store(exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).x_coordinate, exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).y_coordinate);

                    ROS_DEBUG("Storing visited...");
                    exploration->storeVisitedFrontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),robot_str.at(0),final_goal.at(3));
                    ROS_DEBUG("Stored Visited frontier");
                }
            }

            // robot could not reach goal
            else
            {
                if(robot_state == going_charging)
                {
                    ROS_ERROR("Robot cannot reach home for recharging!");
                    exit_countdown--;
                    ROS_ERROR("Shutdown in: %d", exit_countdown);
                    if(exit_countdown <= 0)
                        finalize_exploration();
                }
                else if(robot_state == exploring)
                {
                    ROS_INFO("Storing unreachable...");
                    exploration->storeUnreachableFrontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),robot_str.at(0),final_goal.at(3));
                    ROS_DEBUG("Stored unreachable frontier");
                }
            }

			ROS_DEBUG("                                             ");
            ROS_DEBUG("                                             ");

            ROS_INFO("DONE EXPLORING");
		}
	}
	
	
	    

        void frontiers()
        {
            ros::Rate r(5);
            while(ros::ok())
            {

                costmap_mutex.lock();

                exploration->clearSeenFrontiers(costmap2d_global);
                exploration->clearVisitedFrontiers();
                exploration->clearUnreachableFrontiers();

                exploration->publish_frontier_list();
                exploration->publish_visited_frontier_list();

                costmap_mutex.unlock();

                r.sleep();
            }
        }


        void map_info()
        {
            /*
            * Publish average speed of robot
            */
            ros::NodeHandle nh_pub_speed;
            ros::Publisher publisher_speed = nh_pub_speed.advertise<explorer::Speed>("avg_speed",1);

            fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
            fs_csv << "#time,exploration_travel_path_global,available_distance,global_map_progress,locbal_map_progress,battery_state,recharge_cycles,energy_consumption,frontier_selection_strategy" << std::endl;
            fs_csv.close();

            while(ros::ok() && robot_state != finished)
            {
                //double angle_robot = robotPose.getRotation().getAngle();
                //ROS_ERROR("angle of robot: %.2f\n", angle_robot);

                costmap_mutex.lock();

                ros::Duration time = ros::Time::now() - time_start;

                map_progress.global_freespace = global_costmap_size();
                map_progress.local_freespace = local_costmap_size();
                map_progress.time = time.toSec();
                map_progress_during_exploration.push_back(map_progress);

                double exploration_travel_path_global = (double)exploration->exploration_travel_path_global * costmap_resolution;

                if(battery_charge_temp >= battery_charge)
                    energy_consumption += battery_charge_temp - battery_charge;
                battery_charge_temp = battery_charge;

                fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
                fs_csv << map_progress.time << "," << exploration_travel_path_global << "," << available_distance << "," << map_progress.global_freespace << "," << map_progress.local_freespace << "," << battery_charge << "," << recharge_cycles << "," << energy_consumption << "," << frontier_selection << std::endl;
                fs_csv.close();

                costmap_mutex.unlock();

                // call map_merger to log data
                map_merger::LogMaps log;
                log.request.log = 12;    /// request local and global map progress
                ROS_DEBUG("Calling map_merger service logOutput");
                if(!mm_log_client.call(log))
                    ROS_WARN("Could not call map_merger service to store log.");
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
           int num_map_cells_ = costmap2d_global->getCostmap()->getSizeInCellsX() * costmap2d_global->getCostmap()->getSizeInCellsY();
           int free = 0;

           for (unsigned int i = 0; i < num_map_cells_; i++)
           {
                if ((int) occupancy_grid_global[i] == costmap_2d::FREE_SPACE)
                {
                     free++;
                }
            }
           return free;
        }

        int local_costmap_size()
        {
            if(OPERATE_ON_GLOBAL_MAP == false)
            {
                occupancy_grid_local = costmap2d_local->getCostmap()->getCharMap();
                int num_map_cells_ = costmap2d_local->getCostmap()->getSizeInCellsX() * costmap2d_local->getCostmap()->getSizeInCellsY();
                int free = 0;

                for (unsigned int i = 0; i < num_map_cells_; i++)
                {
                     if ((int) occupancy_grid_local[i] == costmap_2d::FREE_SPACE)
                     {
                          free++;
                     }
                 }
                return free;
            }else
            {
                occupancy_grid_local = costmap2d_local_size->getCostmap()->getCharMap();
                int num_map_cells_ = costmap2d_local_size->getCostmap()->getSizeInCellsX() * costmap2d_local_size->getCostmap()->getSizeInCellsY();
                int free = 0;

                for (unsigned int i = 0; i < num_map_cells_; i++)
                {
                     if ((int) occupancy_grid_local[i] == costmap_2d::FREE_SPACE)
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
            *  CREATE LOG PATH
            * Following code enables to write the output to a file
            * which is localized at the log_path
            */

            nh.param<std::string>("log_path",log_path,"");

            std::stringstream robot_number;
            robot_number << robot_id;
            std::string prefix = "/robot_";
            std::string robo_name = prefix.append(robot_number.str());

            log_path = log_path.append("/explorer");
            log_path = log_path.append(robo_name);
            ROS_INFO("Logging files to %s", log_path.c_str());

            boost::filesystem::path boost_log_path(log_path.c_str());
            if(!boost::filesystem::exists(boost_log_path))
                try{
                    if(!boost::filesystem::create_directories(boost_log_path))
                        ROS_ERROR("Cannot create directory %s.", log_path.c_str());
                }catch(const boost::filesystem::filesystem_error& e)
                {
                    ROS_ERROR("Cannot create path %s.", log_path.c_str());
                }

            log_path = log_path.append("/");

        }

        void save_progress(bool final=false)
        {
            ros::Duration ros_time = ros::Time::now() - time_start;

            double exploration_time = ros_time.toSec();
            int navigation_goals_required = counter;
            double exploration_travel_path = (double)exploration->exploration_travel_path_global * 0.02;
            double size_global_map = map_progress_during_exploration.at(map_progress_during_exploration.size()-1).global_freespace;

            double efficiency_value = (exploration_time) / (number_of_robots * navigation_goals_required);

            std::string tmp_log;
	    if(!final)
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
             *  WRITE LOG FILE
             * Write all the output to the log file
             */
            time_t raw_time;
            struct tm* timeinfo;
            time (&raw_time);
            timeinfo = localtime (&raw_time);

            fs << "[Exploration]" << std::endl;
            fs << "time_file_written    			= " << asctime(timeinfo); // << std::endl;
            fs << "start_time           			= " << time_start << std::endl;
            fs << "end_time             			= " << ros::Time::now() << std::endl;
            fs << "exploration_time    			= " << exploration_time << std::endl;
            fs << "required_goals       			= " << navigation_goals_required << std::endl;
            fs << "unreachable_goals                       = " << exploration->unreachable_frontiers.size() << std::endl;
            fs << "travel_path_overall  			= " << exploration_travel_path << std::endl;
            fs << "number_of_completed_auctions            = " << exploration->number_of_completed_auctions << std::endl;
            fs << "number_of_uncompleted_auctions          = " << exploration->number_of_uncompleted_auctions << std::endl;
            fs << "frontier_selection_strategy             = " << frontier_selection << std::endl;
            fs << "costmap_size                            = " << costmap_width << std::endl;
            fs << "global costmap iterations               = " << global_costmap_iteration << std::endl;
            fs << "number of recharges                     = " << recharge_cycles << std::endl;
            fs << "energy_consumption                      = " << energy_consumption << std::endl;
            fs << "available_distance                      = " << available_distance << std::endl;

	    double param_double;
        int param_int;
	    std::string param;
        /*param = robot_prefix + "/explorer/local_costmap/height";
            ros::param::get(param,param_double);*/
        nh.param<int>("local_costmap/height",param_int,-1);
        fs << "explorer_local_costmap_height 		= " << param_int << std::endl;

        /*param = robot_prefix + "/explorer/local_costmap/width";
            ros::param::get(param,param_double);*/
        nh.param<int>("local_costmap/width",param_int,-1);
        fs << "explorer_local_costmap_width 		= " << param_int << std::endl;

	    param = robot_prefix + "/move_base/local_costmap/height";
            ros::param::get(param,param_double);
	    fs << "move_base_local_costmap_height 		= " << param_double << std::endl;

	    param = robot_prefix + "/move_base/local_costmap/width";
            ros::param::get(param,param_double);
	    fs << "move_base_local_costmap_width 		= " << param_double << std::endl;

	    param = robot_prefix + "/move_base/global_costmap/obstacle_layer/raytrace_range";
            ros::param::get(param,param_double);
	    fs << "move_base_raytrace_range 		= " << param_double << std::endl;

	    param = robot_prefix + "/move_base/global_costmap/obstacle_layer/obstacle_range";
            ros::param::get(param,param_double);
	    fs << "move_base_obstacle_range 		= " << param_double << std::endl;

//	    param = robot_prefix + "/navigation/global_costmap/obstacle_layer/raytrace_range";
        nh.getParam("/global_costmap/obstacle_layer/raytrace_range",param_double);
        ros::param::get(param,param_double);
	    fs << "explorer_raytrace_range 		= " << param_double << std::endl;

        //param = robot_prefix + "/navigation/global_costmap/obstacle_layer/obstacle_range";
        //    ros::param::get(param,param_double);
        nh.getParam("/global_costmap/obstacle_layer/obstacle_range",param_double);
	    fs << "explorer_obstacle_range 		= " << param_double << std::endl;

	    if(final)
            	fs << "complete             			= " << "1" << std::endl;
	    else
            	fs << "complete             			= " << "0" << std::endl;

            fs.close();
//            ROS_INFO("Wrote file %s\n", log_file.c_str());


            /*
             * Inform map_merger to save maps
             */

           if(final)
	    {
		    map_merger::LogMaps log;
		    log.request.log = 3;    /// request local and global map
		    if(!mm_log_client.call(log))
			ROS_WARN("Could not call map_merger service to store log.");
	    }

        }

        void finalize_exploration()
        {
            // finished exploration
            robot_state = finished;

            // finish log files
            exploration_has_finished();

            visualize_goal_point(home_point_x, home_point_y);

            bool completed_navigation = false;
            for(int i = 0; i< 5; i++)
            {
                if(completed_navigation == false)
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
            // When the multi_robot_simulation/multiple_exploration_runs.sh script is run, this kills all processes and starts a new run
            this->indicateSimulationEnd();

            ROS_ERROR("Shutting down...");
            ros::shutdown();
        }

        void exploration_has_finished()
        {
            ros::Duration ros_time = ros::Time::now() - time_start;

            double exploration_time = ros_time.toSec();
            int navigation_goals_required = counter;
            double exploration_travel_path = (double)exploration->exploration_travel_path_global * 0.02;
            double size_global_map = map_progress_during_exploration.at(map_progress_during_exploration.size()-1).global_freespace;
            ROS_INFO("overall freespace in the global map: %f", size_global_map);

            for(int i = 0; i< map_progress_during_exploration.size(); i++)
            {
                ROS_INFO("map progress: %f",(map_progress_during_exploration.at(i).global_freespace/size_global_map)*100);
            }

            ROS_DEBUG("******************************************");
            ROS_DEBUG("******************************************");
            ROS_DEBUG("TIME: %f sec  GOALS: %d  PATH: %f  COMPLETED AUCTIONS: %d  UNCOMPLETED AUCTIONS: %d", exploration_time, navigation_goals_required, exploration_travel_path, exploration->number_of_completed_auctions, exploration->number_of_uncompleted_auctions);
            ROS_DEBUG("******************************************");
            ROS_DEBUG("******************************************");


            double efficiency_value = (exploration_time) / (number_of_robots * navigation_goals_required);

            /*
             *  WRITE LOG FILE
             * Write all the output to the log file
             */
            save_progress(true);
            ROS_INFO("Wrote file %s\n", log_file.c_str());


            /*
             * Inform map_merger to save maps
             */

            map_merger::LogMaps log;
            log.request.log = 3;    /// request local and global map
            if(!mm_log_client.call(log))
                ROS_WARN("Could not call map_merger service to store log.");


//#ifdef PROFILE
//HeapProfilerStop();
//ProfilerStop();
//#endif
        }

        void indicateSimulationEnd()
        {
            /// FIXME: remove this stuff once ported to multicast
            
            /*
            std::stringstream robot_number;
            robot_number << robot_id;

            std::string prefix = "/robot_";
            std::string status_directory = "/simulation_status";
            std::string robo_name = prefix.append(robot_number.str());
            std::string file_suffix(".finished");

            std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
            std::string status_path = ros_package_path + status_directory;
            std::string status_file = status_path + robo_name + file_suffix;

            /// TODO: check whether directory exists
            boost::filesystem::path boost_status_path(status_path.c_str());
            if(!boost::filesystem::exists(boost_status_path))
                if(!boost::filesystem::create_directories(boost_status_path))
                    ROS_ERROR("Cannot create directory %s.", status_path.c_str());
            std::ofstream outfile(status_file.c_str());
            outfile.close();
            ROS_INFO("Creating file %s to indicate end of exploration.", status_file.c_str());
            */

        }


    /**
     * Iterate over the whole map to see if there are any remaining frontiers
     */
    bool iterate_global_costmap(std::vector<double> *global_goal, std::vector<std::string> *robot_str)
    {
        global_costmap_iteration++;
        int counter = 0;
        bool exploration_flag;

        costmap_mutex.lock();

        exploration->transformToOwnCoordinates_frontiers();
        exploration->transformToOwnCoordinates_visited_frontiers();

        exploration->initialize_planner("exploration planner", costmap2d_global, costmap2d_global);
        exploration->findFrontiers();

        exploration->clearVisitedFrontiers();
        exploration->clearUnreachableFrontiers();
        exploration->clearSeenFrontiers(costmap2d_global);

        costmap_mutex.unlock();

        exploration->visualize_Frontiers();

        if(frontier_selection < 5 && frontier_selection != 1)
        {
            exploration->sort(2);

            while(true)
            {
                exploration_flag = exploration->determine_goal(2, global_goal, counter, -1, robot_str);

                if(exploration_flag == false)
                {
                    break;
                }
                else
                {
                    bool negotiation = true;
                    negotiation = exploration->negotiate_Frontier(global_goal->at(0),global_goal->at(1),global_goal->at(2),global_goal->at(3), -1);

                    if(negotiation == true)
                    {
                        return true;
                    }
                    counter++;
                }
            }
        }

        else if(frontier_selection == 1 || frontier_selection == 6 || frontier_selection == 5)
        {
            costmap_mutex.lock();
            exploration->clearVisitedAndSeenFrontiersFromClusters();

            exploration->clusterFrontiers();

            exploration->sort(4);
            exploration->sort(5);

            costmap_mutex.unlock();

            cluster_element = -1;

            while(true)
            {
                std::vector<double> goal_vector;
                std::vector<std::string> robot_str_name;
                std::vector<int> clusters_used_by_others;

                goal_determined = exploration->determine_goal(5, &goal_vector, 0, cluster_element, robot_str);

                if(goal_determined == false)
                {
                    ROS_INFO("No goal was determined, cluster is empty. Bid for another one");

                    goal_vector.clear();
                    bool auctioning = exploration->auctioning(&goal_vector, &clusters_used_by_others, &robot_str_name);
                    if(auctioning == true)
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
            
            //F
            // publish goal position
            
            
        }

        // no valid goal found
        else
        {
            rotation_counter++;
            ROS_INFO("In navigation .... cluster_available: %lu     counter: %d", exploration->clusters.size(), counter_waiting_for_clusters);

            // ???
            if(exploration->clusters.size() == 0 || counter_waiting_for_clusters > 10)
            {
                // check the whole map for any remaining frontiers
                ROS_INFO("Iterating over GLOBAL COSTMAP to find a goal!!!!");
                std::vector<double> global_goal;
                std::vector<std::string> robot_str;
                bool global_costmap_goal = iterate_global_costmap(&global_goal, &robot_str);

                // no frontiers available anymore, exploration finished
                if(global_costmap_goal == false)
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
                    bool backoff_sucessfull = exploration->smartGoalBackoff(global_goal.at(0),global_goal.at(1), costmap2d_global, &backoffGoal);

                    if(backoff_sucessfull == true)
                    {
                        ROS_DEBUG("doing navigation to back-off goal");
                        visualize_goal_point(backoffGoal.at(0), backoffGoal.at(1));
                        completed_navigation = move_robot(counter, backoffGoal.at(0), backoffGoal.at(1));
                        rotation_counter = 0;
                        if(completed_navigation == true)
                        {
                            // compute path length
                            exploration->trajectory_plan_store(exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).x_coordinate, exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).y_coordinate);

                            ROS_INFO("Storing visited...");
                            exploration->storeVisitedFrontier(global_goal.at(0),global_goal.at(1),global_goal.at(2),robot_str.at(0), global_goal.at(3));
                            ROS_INFO("Stored Visited frontier");
                        }
                        else
                        {
                            ROS_INFO("Storing unreachable...");
                            exploration->storeUnreachableFrontier(global_goal.at(0),global_goal.at(1),global_goal.at(2),robot_str.at(0), global_goal.at(3));
                            ROS_INFO("Stored unreachable frontier");
                        }
                    }
                    else if(backoff_sucessfull == false)
                    {
                        ROS_ERROR("Navigation to global costmap back-off goal not possible");
                        ROS_INFO("Storing as unreachable...");
                        exploration->storeUnreachableFrontier(global_goal.at(0),global_goal.at(1),global_goal.at(2), robot_str.at(0), global_goal.at(3));
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
        return(completed_navigation);
    }

	void visualize_goal_point(double x, double y) {

		goalPoint.header.seq = goal_point_message++;
		goalPoint.header.stamp = ros::Time::now();
                goalPoint.header.frame_id = move_base_frame; //"map"
		goalPoint.point.x = x;// - robotPose.getOrigin().getX();
		goalPoint.point.y = y;// - robotPose.getOrigin().getY();

		ros::NodeHandle nh_Point("goalPoint");
        pub_Point = nh_Point.advertise < geometry_msgs::PointStamped
				> ("goalPoint", 100, true);
		pub_Point.publish < geometry_msgs::PointStamped > (goalPoint);
	}

	void visualize_home_point() {

		homePoint.header.seq = home_point_message++;
		homePoint.header.stamp = ros::Time::now();
        homePoint.header.frame_id = move_base_frame; //"map";
		home_point_x = robotPose.getOrigin().getX();
		home_point_y = robotPose.getOrigin().getY();
		homePoint.point.x = home_point_x;
		homePoint.point.y = home_point_y;

		ros::NodeHandle nh("homePoint");
		pub_home_Point = nh.advertise < geometry_msgs::PointStamped > ("homePoint", 100, true);
		pub_home_Point.publish < geometry_msgs::PointStamped > (homePoint);

	}

    bool move_robot(int seq, double position_x, double position_y)
    {
        exploration->next_auction_position_x = position_x;
        exploration->next_auction_position_y = position_y;
        int stuck_countdown = EXIT_COUNTDOWN;

        /*
         * Move the robot with the help of an action client. Goal positions are
         * transmitted to the robot and feedback is given about the actual
         * driving state of the robot.
         */
        if (!costmap2d_local->getRobotPose(robotPose)) {
            ROS_ERROR("Failed to get RobotPose");
        }

        actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> ac("move_base", true);

        while (!ac.waitForServer(ros::Duration(10.0)))
        ;

        move_base_msgs::MoveBaseGoal goal_msgs;

        goal_msgs.target_pose.header.seq = seq;	// increase the sequence number
        goal_msgs.target_pose.header.frame_id = move_base_frame; //"map";
        goal_msgs.target_pose.pose.position.x = position_x;
        goal_msgs.target_pose.pose.position.y = position_y;
        goal_msgs.target_pose.pose.position.z = 0;
        goal_msgs.target_pose.pose.orientation.x = 0;
        goal_msgs.target_pose.pose.orientation.y = 0;
        goal_msgs.target_pose.pose.orientation.z = 0;
        goal_msgs.target_pose.pose.orientation.w = 1;
        
        
        //F
        
        pub_robot_pos.publish(goal_msgs);

        ac.sendGoal(goal_msgs);

        ac.waitForResult(ros::Duration(waitForResult));
        while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
        {
            ros::Duration(0.5).sleep();
        }
        ROS_INFO("Not longer PENDING");

        while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            // robot seems to be stuck
            if(prev_pose_x == pose_x && prev_pose_y == pose_y && prev_pose_angle == pose_angle)
            {
                stuck_countdown--;
                if(stuck_countdown <= 5){
                    ROS_ERROR("Robot is not moving anymore, shutdown in: %d", stuck_countdown);
                }
                if(stuck_countdown <= 0) {
                    exit(0);
                }
            }
            else
            {
                stuck_countdown = STUCK_COUNTDOWN; // robot is moving again
                prev_pose_x = pose_x;
                prev_pose_y = pose_y;
                prev_pose_angle = pose_angle;
            }
            
            int remaining_distance = exploration->trajectory_plan(position_x, position_y);
            if(robot_state == going_charging)
                ;//ROS_ERROR("\n\t\e[1;34mRemaining distance: %d\e[0m\n", remaining_distance);
            if(remaining_distance < 50 && robot_state == going_charging && winner_of_auction == false) {
            //if(remaining_distance < 10 && winner_of_auction == false) {
                ac.cancelGoal();
                ROS_ERROR("\n\t\e[1;34m!!!!!!!!!!!!!!!!!!!!!!!\nSTOP!!!!!\e[0m\n");
                robot_state = in_queue;
            }

            ros::Duration(0.5).sleep();
            
        }
        ROS_INFO("Not longer ACTIVE");
        //ROS_ERROR("\n\t\e[1;34m%s\e[0m\n", ac.getState().toString().c_str());

        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
                ROS_INFO("ABORTED");

                exploration->next_auction_position_x = robotPose.getOrigin().getX();
                exploration->next_auction_position_y = robotPose.getOrigin().getY();
                return false;
            }
        }
        ROS_INFO("TARGET REACHED");

        exploration->next_auction_position_x = robotPose.getOrigin().getX();
        exploration->next_auction_position_y = robotPose.getOrigin().getY();
        return true;
    }

	bool turn_robot(int seq) {

                double angle = 45;

                if (!costmap2d_local->getRobotPose(robotPose)) {
			ROS_ERROR("Failed to get RobotPose");
		}

		actionlib::SimpleActionClient < move_base_msgs::MoveBaseAction
				> ac("move_base", true);
		while (!ac.waitForServer(ros::Duration(10.0)))
			;

		move_base_msgs::MoveBaseGoal goal_msgs;

		goal_msgs.target_pose.header.seq = seq;	// increase the sequence number
		goal_msgs.target_pose.header.stamp = ros::Time::now();

                goal_msgs.target_pose.header.frame_id = move_base_frame;//"map";
		goal_msgs.target_pose.pose.position.x = robotPose.getOrigin().getX();
		goal_msgs.target_pose.pose.position.y = robotPose.getOrigin().getY();
		goal_msgs.target_pose.pose.position.z = 0;
		goal_msgs.target_pose.pose.orientation.x = 0; //sin(angle/2); // goals[0].pose.orientation.x;
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

		while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
			if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
				ROS_INFO("ABORTED");
				return false;
			}
		}
		ROS_INFO("ROTATION ACCOMBLISHED");
		return true;
	}

	void feedbackCallback(
			const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
		feedback_value = msg.get()->status.status;
		feedback_succeed_value =
				msg.get()->feedback.base_position.pose.orientation.w;

	}

	bool target_reached(void) {

		ros::NodeHandle nh_sub_move_base;
		sub_move_base = nh_sub_move_base.subscribe("feedback", 1000,
				&Explorer::feedbackCallback, this);

		while (feedback_value != feedback_succeed_value)
			;

		return true;
    }
    
    void auction_completed_callback(const std_msgs::Empty::ConstPtr &msg) {
        ROS_ERROR("\n\t\e[1;34mAuction completed: going to charging or going in queue?\e[0m\n");
        robot_state = going_charging;
    }
    
    void auction_winner_callback(const std_msgs::Empty::ConstPtr &msg) {
        ROS_ERROR("\n\t\e[1;34mGoing to charge!\e[0m\n");
        robot_state = reaching_frontier_before_going_charging; //F WRONG in the general case!!!!
        winner_of_auction = true;
    }
    
    void auction_loser_callback(const std_msgs::Empty::ConstPtr &msg) {
        ROS_ERROR("\n\t\e[1;34mGoing in queue...\e[0m\n");
        winner_of_auction = false;
    }

    public:

        struct map_progress_t
        {
            double local_freespace;
            double global_freespace;
            double time;
        } map_progress;

        ros::Subscriber sub_move_base, sub_obstacle;

        // create a costmap
        costmap_2d::Costmap2DROS* costmap2d_local;
        costmap_2d::Costmap2DROS* costmap2d_local_size;
        costmap_2d::Costmap2DROS* costmap2d_global;
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
        std::string robot_prefix;               /// The prefix for the robot when used in simulation
        std::string robot_name;
        const unsigned char* occupancy_grid_global;
        const unsigned char* occupancy_grid_local;

        std::string csv_file, log_file;
        std::string log_path;
        std::fstream fs_csv, fs;

    private:

        //enum state_t {exploring, going_charging, charging, finished, fully_charged, stuck, in_queue};
        //state_t robot_state;

        ros::Publisher pub_move_base;
        ros::Publisher pub_Point;
        ros::Publisher pub_home_Point;
        ros::Publisher pub_frontiers;

        ros::ServiceClient mm_log_client;

        ros::NodeHandle nh;
        ros::Time time_start;

        //Create a move_base_msgs to define a goal to steer the robot to
        move_base_msgs::MoveBaseActionGoal action_goal_msg;
        move_base_msgs::MoveBaseActionFeedback feedback_msgs;

        geometry_msgs::PointStamped goalPoint;
        geometry_msgs::PointStamped homePoint;

        std::vector<geometry_msgs::PoseStamped> goals;
        tf::Stamped<tf::Pose> robotPose;

        explorationPlanner::ExplorationPlanner *exploration;

        double pose_x, pose_y, pose_angle, prev_pose_x, prev_pose_y, prev_pose_angle;

        double x_val, y_val, home_point_x, home_point_y;
        int seq, feedback_value, feedback_succeed_value, rotation_counter, home_point_message, goal_point_message;
        int counter;
        bool recharging;
        bool pioneer;
        int w1, w2, w3, w4;
};

int main(int argc, char **argv)
{
#ifdef PROFILE
    const char  fname[3] = "TS";
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
	 * Create instance of Simple Navigation
	 */
	tf::TransformListener tf(ros::Duration(10));
	Explorer simple(tf);

	/*
	 * The ros::spin command is needed to wait for any call-back. This could for
	 * example be a subscription on another topic. Do this to be able to receive a
	 * message.
	 */
	boost::thread thr_explore(boost::bind(&Explorer::explore, &simple));

    /*
     * The following thread is only necessary to log simulation results.
     * Otherwise it produces unused output.
     */
    boost::thread thr_map(boost::bind(&Explorer::map_info, &simple));

    /*
     * FIXME
     * Which rate is required in order not to oversee
     * any callback data (frontiers, negotiation ...)
     */
    while (ros::ok())
    {
        costmap_mutex.lock();
        ros::spinOnce();
        costmap_mutex.unlock();

        ros::Duration(0.1).sleep();
	}

	thr_explore.interrupt();
    thr_map.interrupt();
    thr_explore.join();
    thr_map.join();

#ifdef PROFILE
    HeapProfilerStop();
    ProfilerStop();
#endif

    return 0;
}
