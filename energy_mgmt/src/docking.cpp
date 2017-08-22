#include <docking.h>

//TODO(minor) ConstPtr

#pragma GCC diagnostic ignored "-Wenum-compare"

using namespace std;
using namespace robot_state;

int counter;

//DONE (DONE without any + means that the function is correct, but it's missing comments, debug outputs, ...)
docking::docking()  // TODO(minor) create functions; comments here and in .h file; check if the topics uses the correct messages...
{
    ROS_INFO("Creating instance of Docking class...");
   
    /* Load parameters */  // TODO(minor) checks if these params exist...
    ros::NodeHandle nh_tilde("~");
    nh_tilde.getParam("x", origin_absolute_x);
    nh_tilde.getParam("y", origin_absolute_y);
    nh_tilde.param<string>("robot_prefix", robot_prefix, "");
    nh_tilde.param<std::string>("log_path", log_path, "");
    nh_tilde.param<float>("fiducial_signal_range", fiducial_signal_range, 30.0); //m
    nh_tilde.param<bool>("fiducial_sensor_on", fiducial_sensor_on, true);         // not used at the moment...

    /* Initialize robot name */
    if (robot_prefix.empty())
    {
        /* Empty prefix: we are on an hardware platform (i.e., real experiment) */
        ROS_INFO("Real robot");

        /* Set robot name and hostname */
        char hostname[1024];
        hostname[1023] = '\0';
        gethostname(hostname, 1023);
        robot_name = string(hostname);

        /* Set robot ID based on the robot name */
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

        my_prefix = "robot_" + SSTR(robot_id) + "/";  // TODO(minor)
    }
    else
    {
        /* Prefix is set: we are in a simulation */
        ROS_INFO("Simulation");

        robot_name = robot_prefix;  // TODO(minor) we need this? and are we sure that
                                    // it must be equal to robot_refix
                                    // (there is an unwanted '/' maybe...)

        /* Read robot ID number: to do this, it is required that the robot_prefix is
         * in the form "/robot_<id>", where
         * <id> is the ID of the robot */
        // TODO(minor) what if there are more than 10 robots and so we have
        // robot_10: this line of code will fail!!!
        robot_id = atoi(robot_prefix.substr(7, 1).c_str());

        /* Since we are in simulation and we use launch files with the group tag,
         * prefixes to topics are automatically
         * added: there is no need to manually specify robot_prefix in the topic
         * name */
        // my_prefix = "docking/"; //TODO(minor)
        my_prefix = "";
        // my_node = "energy_mgmt/"
        my_node = "";

        ROS_DEBUG("Robot prefix: %s; robot id: %d", robot_prefix.c_str(), robot_id);
    }

    /* Initialize robot struct */
    robot = NULL;
    robot = new robot_t;
    if(robot == NULL)
        ROS_FATAL("Allocation failure!");

    ros::Duration(10).sleep();
    robot->id = robot_id;
    robot->home_world_x = origin_absolute_x;
    robot->home_world_y = origin_absolute_y;
    abs_to_rel(origin_absolute_x, origin_absolute_y, &(robot->x), &(robot->y));
    
    robots.push_back(*robot);

    // TODO(minor) names (robot_0 end dockign)
    // TODO(minor) save names in variables
    // TODO(minor) queues length?

    /* Adhoc communication services */
    sc_send_docking_station = nh.serviceClient<adhoc_communication::SendEmDockingStation>(
        my_prefix + "adhoc_communication/send_em_docking_station");
    sc_send_robot = nh.serviceClient<adhoc_communication::SendEmRobot>(my_prefix + "adhoc_communication/send_em_robot");

    /* General services */
    sc_robot_pose = nh.serviceClient<fake_network::RobotPositionSrv>(my_prefix + "explorer/robot_pose");
    sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target");
    sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance");

    /* Subscribers */
    sub_battery = nh.subscribe(my_prefix + "battery_state", 1000, &docking::cb_battery, this);
    sub_robots = nh.subscribe(my_prefix + "robots", 1000, &docking::cb_robots, this);
    sub_jobs = nh.subscribe(my_prefix + "frontiers", 1000, &docking::cb_jobs, this);
    sub_docking_stations = nh.subscribe(my_prefix + "docking_stations", 1000, &docking::cb_docking_stations, this);
    sub_check_vacancy = nh.subscribe("adhoc_communication/check_vacancy", 1000, &docking::check_vacancy_callback, this);
    sub_next_ds = nh.subscribe("next_ds", 1000, &docking::next_ds_callback, this);
	sub_goal_ds_for_path_navigation = nh.subscribe("goal_ds_for_path_navigation", 1000, &docking::goal_ds_for_path_navigation_callback, this);
    sub_wait = nh.subscribe("explorer/im_ready", 1000, &docking::wait_for_explorer_callback, this);
    sub_path = nh.subscribe("error_path", 1000, &docking::path_callback, this);
    sub_ds_with_EOs = nh.subscribe("ds_with_EOs", 1000, &docking::ds_with_EOs_callback, this);
    
    /* Publishers */
    pub_ds = nh.advertise<std_msgs::Empty>("docking_station_detected", 1000);
    pub_adhoc_new_best_ds =
        nh.advertise<adhoc_communication::EmDockingStation>("adhoc_new_best_docking_station_selected", 1000);
    pub_moving_along_path = nh.advertise<adhoc_communication::MmListOfPoints>("moving_along_path", 1000);
    pub_finish = nh.advertise<std_msgs::Empty>("explorer/finish", 10);
	pub_new_optimal_ds = nh.advertise<adhoc_communication::EmDockingStation>("explorer/new_optimal_ds", 1000);
	pub_robot_absolute_position = nh.advertise<fake_network::RobotPosition>("fake_network/robot_absolute_position", 100);
	pub_new_ds_on_graph = nh.advertise<adhoc_communication::EmDockingStation>("new_ds_on_graph", 1000);
	pub_ds_count = nh.advertise<std_msgs::Int32>("ds_count", 100);
    pub_wait = nh.advertise<std_msgs::Empty>("explorer/are_you_ready", 10);

    /* Variable initializations */
    robot_state = robot_state::INITIALIZING;  // TODO(minor) param
    moving_along_path = false;
    recompute_graph = false;
    recompute_llh = false;
    going_to_ds = false;
    explorer_ready = false;
    optimal_ds_set = false;
    finished_bool = false;
    no_jobs_received_yet = true;
    group_name = "mc_robot_0";
    //group_name = "";
    my_counter = 0;
    maximum_travelling_distance = -1;
    old_optimal_ds_id = -100;
    next_optimal_ds_id = old_optimal_ds_id;
    old_optimal_ds_id_for_log = old_optimal_ds_id;
    time_start = ros::Time::now();
    path_navigation_tries = 0;
    next_remaining_distance = 0, current_remaining_distance = 0;
    has_to_free_optimal_ds = false;
    id_ds_to_be_freed = -1;
    already_printed_matching_error = false;
    wait_for_ds = 0;
    index_of_ds_in_path = -1;
    two_robots_at_same_ds_printed = false;
    invalid_ds_count_printed = false;
    ds_appears_twice_printed = false;
    waiting_to_discover_a_ds = false;
    changed_state_time = ros::Time::now();
    test_mode = false;

    /* Function calls */
    preload_docking_stations();
    
    ROS_INFO("Instance of Docking class correctly created");
    
//    sub_finalize_exploration = nh.subscribe("finalize_exploration", 10 , &docking::finalize_exploration_callback, this);
    
    DsGraph mygraph;
    mygraph.addEdge(1,2,10);
    //mygraph.print();
    
    nh_tilde.param<std::string>("log_path", original_log_path, "");
    major_errors_file = original_log_path + std::string("_errors.log");
    
//    ROS_ERROR("%s", major_errors_file.c_str());
    
    graph_navigation_allowed = GRAPH_NAVIGATION_ALLOWED;
    
    pub_ds_position = nh.advertise <visualization_msgs::Marker> ("energy_mgmt/ds_positions", 1000, true);
    pub_this_robot = nh.advertise<adhoc_communication::EmRobot>("this_robot", 10, true);
    
    major_errors = 0, minor_errors = 0;
    
    timer_recompute_ds_graph = nh.createTimer(ros::Duration(60), &docking::timer_callback_recompute_ds_graph, this, false, true); //TODO(minor) timeout
    
    starting_time = ros::Time::now();

    fs_info.open(info_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs_info << "#robot_id,num_robots,ds_selection_policy,starting_absolute_x,"
               "starting_absolute_y" << std::endl;
    fs_info << robot_id << "," << num_robots << "," << ds_selection_policy << "," << origin_absolute_x << ","
            << origin_absolute_y << "," << std::endl;
    fs_info.close();
    
    get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");
    
}

void docking::wait_for_explorer() {
    std_msgs::Empty msg;
    ros::Duration(5).sleep();
    while(!explorer_ready) {
        ROS_INFO("Waiting for the explorer services to be ready...");
        pub_wait.publish(msg);
        ros::Duration(5).sleep();
        ros::spinOnce();
    }
    ros::Duration(5).sleep();
    
    ros::service::waitForService("explorer/distance");
    //sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance", true);
    sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance");
    for(int i = 0; i < 10 && !sc_distance; i++) {
        ROS_ERROR("No connection to service 'explorer/distance': retrying...");
        ros::Duration(3).sleep();
        //sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance", true);
    }
    ROS_INFO("Established persistent connection to service 'explorer/distance'");
    //ros::Duration(0.1).sleep();
    
    ros::service::waitForService("explorer/reachable_target");
    //sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target", true);
    sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target");
    for(int i = 0; i < 10 && !sc_reachable_target; i++) {
        ROS_ERROR("No connection to service 'explorer/reachable_target': retrying...");
        ros::Duration(3).sleep();
        //sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target", true);
    }
    ROS_INFO("Established persistent connection to service 'explorer/reachable_target'");
    //ros::Duration(0.1).sleep();
    
    ros::service::waitForService("explorer/robot_pose");
    //sc_robot_pose = nh.serviceClient<fake_network::RobotPosition>(my_prefix + "explorer/robot_pose", true);
    sc_robot_pose = nh.serviceClient<fake_network::RobotPositionSrv>(my_prefix + "explorer/robot_pose");      
    for(int i = 0; i < 10 && !sc_robot_pose; i++) {
        ROS_ERROR("No connection to service 'explorer/robot_pose': retrying...");
        ros::Duration(3).sleep();
        //sc_robot_pose = nh.serviceClient<fake_network::RobotPosition>(my_prefix + "explorer/robot_pose", true);   
    }
    ROS_INFO("Established persistent connection to service 'explorer/robot_pose'");    
    //ros::Duration(0.1).sleep();
    
//    ros::Duration(5).sleep();
    send_robot();
}


template <class T>
void establishPersistenServerConnection(ros::ServiceClient &sc, std::string service_name) {
/*
    ros::service::waitForService(service_name);
    for(int i = 0; i < 10 && !(*sc); i++) {
        ROS_FATAL("NO MORE CONNECTION!");
        ros::Duration(1).sleep();
        *sc = nh.serviceClient<T>(service_name, true);
    } 
    */
}


void docking::wait_for_explorer_callback(const std_msgs::Empty &msg) {
    ROS_INFO("Explorer is ready");
    explorer_ready = true;
}


void docking::handle_robot_state()
{
    // TODO(minor) better update...
        
    if(robot_state == robot_state::CHARGING && (next_robot_state != robot_state::LEAVING_DS && next_robot_state != finished)) {
        log_major_error("invalid state after robot_state::CHARGING!!!");
        ROS_INFO("current state: robot_state::CHARGING");   
        ROS_INFO("next state: %d", next_robot_state);
    }
    
    if(has_to_free_optimal_ds && next_robot_state == robot_state::LEAVING_DS) {
//        set_optimal_ds_vacant(true);
        free_ds(id_ds_to_be_freed);
        has_to_free_optimal_ds = false;
    }
        
    if (next_robot_state != robot_state::GOING_CHECKING_VACANCY) //TODO(minor) very bad... maybe in if(... == robot_state::CHECKING_VACANCY) would be better...
        going_to_ds = false;
        
//    if (has_to_free_optimal_ds && next_robot_state != fully_charged && next_robot_state != robot_state::LEAVING_DS) //TODO maybe since we put the DS as occupied only when we start charging, we could put it as free when we leave it already (put are we sure that this doens't cause problems somewhere else?)... although the robot_state::LEAVING_DS state is so short that it makes almost no different
//     {
//            has_to_free_optimal_ds = false;
//            set_optimal_ds_vacant(true);
//    }

    if (next_robot_state == robot_state::IN_QUEUE)
    {
        ;
    }
    else if (next_robot_state == robot_state::GOING_CHARGING)
    {
        ;  // ROS_ERROR("\n\t\e[1;34m Robo t going charging!!!\e[0m");
    }
    else if (next_robot_state == robot_state::CHARGING)
    {
        id_ds_to_be_freed = get_optimal_ds_id();
        has_to_free_optimal_ds = true;
        set_optimal_ds_vacant(false); // we could thing of doing it ealrly... but this would mean that the other robots will think that a DS is occupied even if it is not, which means that maybe one of them could have a high value of the llh and could get that given DS, but instead the robot will give up and it will try with another DS (this with the vacant stragety), so it could be disadvantaging...
    }
    else if (next_robot_state == robot_state::GOING_CHECKING_VACANCY)
    {
        ;  // ROS_ERROR("\n\t\e[1;34m going checking vacancy!!!\e[0m");
    }
    else if (next_robot_state == robot_state::CHECKING_VACANCY)
    {
//        if(get_target_ds_id() < 0 || get_target_ds_id() >= num_ds) {
//            log_major_error("sending invalid DS id 5!!!");
//            ROS_ERROR("%d",  get_target_ds_id());  
//        }
    
        ;  // ROS_ERROR("\n\t\e[1;34m checking vacancy!!!\e[0m");
        adhoc_communication::SendEmDockingStation srv_msg;
        srv_msg.request.topic = "adhoc_communication/check_vacancy";
        srv_msg.request.dst_robot = group_name;
//        srv_msg.request.docking_station.id = get_target_ds_id();  // target_ds, not best_ds!!!!!
        srv_msg.request.docking_station.id = get_optimal_ds_id();  // target_ds, not best_ds!!!!!
        srv_msg.request.docking_station.header.message_id = getAndUpdateMessageIdForTopic("adhoc_communication/check_vacancy");
        srv_msg.request.docking_station.header.sender_robot = robot_id;
        sc_send_docking_station.call(srv_msg);
    }
    else if (next_robot_state == robot_state::AUCTIONING)
    {
        ROS_INFO("Robot needs to recharge");
        if(!going_to_ds) //TODO(minor) very bad check... to be sure that only if the robot has not just won
                                  // another auction it will start its own (since maybe explorer is still not aware of this and so will communicate "robot_state::AUCTIONING" state...); do we have other similar problems?
            ROS_ERROR("calling start_new_auction()");
//            start_new_auction(); 

        if(!optimal_ds_is_set())
            log_major_error("!optimal_ds_is_set");
                                  // TODO(minor) only if the robot has not been just interrupted from recharging
    }
    else if (next_robot_state == auctioning_2) {
        if(ds_selection_policy == 2)
		{
			if(finished_bool) {
                ROS_ERROR("No more frontiers..."); //TODO(minor) probably this checks are reduntant with the ones of explorer
                std_msgs::Empty msg;
                pub_finish.publish(msg);
            } else {
		        ROS_INFO("Robot needs to recharge");
		        if(!going_to_ds) //TODO(minor) very bad check... to be sure that only if the robot has not just won
		                                  // another auction it will start its own (since maybe explorer is still not aware of this and so will communicate "robot_state::AUCTIONING" state...); do we have other similar problems?
		        {
		            ros::Duration(1).sleep();
		            ROS_ERROR("calling start_new_auction()");
//		            start_new_auction();
		        }
			}
	    } else if(graph_navigation_allowed) {
//	        moving_along_path = true;
//	        compute_and_publish_path_on_ds_graph();
//            if(finished_bool) {
//                std_msgs::Empty msg;
//                pub_finish.publish(msg);
//            } else {
//                ROS_INFO("Robot needs to recharge");
//                if(!going_to_ds) //TODO(minor) very bad check... to be sure that only if the robot has not just won
//                                          // another auction it will start its own (since maybe explorer is still not aware of this and so will communicate "robot_state::AUCTIONING" state...); do we have other similar problems?
//                {
//                    ros::Duration(10).sleep();
//                    start_new_auction();
//                }
//            }
        } else {
            ROS_ERROR("DS graph cannot be navigated with this strategy...");
            ROS_INFO("DS graph cannot be navigated with this strategy...");
            std_msgs::Empty msg;
            pub_finish.publish(msg);
        }   
    }
    else if (next_robot_state == auctioning_3) {
        if(graph_navigation_allowed) {
	        compute_and_publish_path_on_ds_graph_to_home();
            if(finished_bool) {
                ROS_ERROR("No more frontiers..."); //TODO(minor) probably this checks are reduntant with the ones of explorer
                std_msgs::Empty msg;
                pub_finish.publish(msg);
            } else {
                ROS_INFO("Robot needs to recharge");
                if(!going_to_ds) //TODO(minor) very bad check... to be sure that only if the robot has not just won
                                          // another auction it will start its own (since maybe explorer is still not aware of this and so will communicate "robot_state::AUCTIONING" state...); do we have other similar problems?
                {
                    ros::Duration(10).sleep();
                    ROS_ERROR("calling start_new_auction()");
//                    start_new_auction();
                }
            }
        } else {
            ROS_ERROR("DS graph cannot be navigated with this strategy...");
            ROS_INFO("DS graph cannot be navigated with this strategy...");
            std_msgs::Empty msg;
            pub_finish.publish(msg);
        }   
    }
    else if (next_robot_state == robot_state::LEAVING_DS)
    {
        going_to_ds = false;
        //free_ds(id_ds_to_be_freed); //it is better to release the DS when the robot has exited the fully_charged or robot_state::LEAVING_DS state, but sometimes (at the moment for unknown reasones) this takes a while, even if the robot has already phisically released the DS...
    }
    else if (next_robot_state == robot_state::MOVING_TO_FRONTIER || next_robot_state == robot_state::GOING_IN_QUEUE)
        ;
    else if(next_robot_state == robot_state::COMPUTING_NEXT_GOAL)
    {
        ;
    }
    else if (next_robot_state == finished)
    {
        finalize();
    }
    else if (next_robot_state == stuck)
    {
        finalize();
    }
    else if (next_robot_state == stopped)
    {
        ;
    }
    else if (next_robot_state == exploring_for_graph_navigation)
    {
        ;
    }
    else
    {
        ROS_FATAL("\n\t\e[1;34m none of the above!!!\e[0m");
        return;
    }
        
    robot_state = next_robot_state;
    robot->state = robot_state;
    
    changed_state_time = ros::Time::now();

    send_robot();

}

void docking::get_robot_state() {
    robot_state::GetRobotState msg;
    while(!get_robot_state_sc.call(msg))
        ROS_ERROR("get state failed");
    next_robot_state = static_cast<robot_state::robot_state_t>(msg.response.robot_state);
}

void docking::check_vacancy_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg)  // TODO(minor) explain
                                                                                                  // very well the
                                                                                                  // choices
{
    // Safety check on the received DS
    if(msg.get()->id < 0) {
        ROS_ERROR("Invalid DS id: %d", msg.get()->id);
        log_major_error("received invalid DS from a robot...");
        return;
    }

    if(!checkAndUpdateReceivedMessageId("check_vacancy", msg.get()->header.message_id, msg.get()->header.sender_robot))
        log_major_error("missed a vacancy check message!!");
        

    ROS_INFO("Received request for vacancy check for ds%d", msg.get()->id); //TODO(minor) complete

    /* If the request for vacancy check is not about the target DS of the robot,
     * for sure the robot is not occupying it
     */
//    if (target_ds_is_set() && msg.get()->id == get_target_ds_id())
    if (optimal_ds_is_set() && msg.get()->id == get_optimal_ds_id())

        /* If the robot is going to or already charging, or if it is going to check
         * already checking for vacancy, it is
         * (or may be, or will be) occupying the DS */
        if (robot_state == robot_state::CHARGING || robot_state == robot_state::GOING_CHARGING || robot_state == robot_state::GOING_CHECKING_VACANCY ||
            robot_state == robot_state::CHECKING_VACANCY || robot_state == robot_state::LEAVING_DS)
        {
            /* Print some debut text */
            if (robot_state == robot_state::CHARGING || robot_state == robot_state::GOING_CHARGING)
                ROS_INFO("I'm using / going to use ds%d!!!!", msg.get()->id);
            else if (robot_state == robot_state::GOING_CHECKING_VACANCY || robot_state == robot_state::CHECKING_VACANCY)
                ROS_INFO("I'm approachign ds%d too!!!!", msg.get()->id);
            else if (robot_state == robot_state::LEAVING_DS)
                ROS_INFO("I'm leaving ds%d, just wait a sec...", msg.get()->id);

            /* Reply to the robot that asked for the check, telling it that the DS is
             * occupied */
            adhoc_communication::SendEmDockingStation srv_msg;
            srv_msg.request.topic = "explorer/adhoc_communication/reply_for_vacancy";
            srv_msg.request.dst_robot = group_name;
//            srv_msg.request.docking_station.id = get_target_ds_id();
            srv_msg.request.docking_station.id = get_optimal_ds_id();
            srv_msg.request.docking_station.used_by_robot_id = robot_id;
            sc_send_docking_station.call(srv_msg);
            ROS_INFO("Notified other robot that ds%d is occupied by me", msg.get()->id);
        }
        else
            ROS_DEBUG("target ds, but currently not used by the robot");
    else
        ROS_DEBUG("robot is not targetting that ds");
}

void docking::create_log_files()
{
    ROS_INFO("Creating log files...");

    /* Create directory */
    log_path = log_path.append("/energy_mgmt");
    log_path = log_path.append(robot_name);
    boost::filesystem::path boost_log_path(log_path.c_str());
    if (!boost::filesystem::exists(boost_log_path))
    {
        ROS_INFO("Creating directory %s", log_path.c_str());
        try
        {
            if (!boost::filesystem::create_directories(boost_log_path))
            {
                ROS_ERROR("Cannot create directory %s: aborting node...", log_path.c_str());
                exit(-1);
            }
        }
        catch (const boost::filesystem::filesystem_error &e)
        {
            ROS_ERROR("Cannot create path %saborting node...", log_path.c_str());
            exit(-1);
        }
    }
    else
    {
        ROS_INFO("Directory %s already exists: log files will be saved there", log_path.c_str());
    }

    std::fstream fs;

    /* Create file names */
    log_path = log_path.append("/");
    csv_file = log_path + std::string("optimal_ds.csv"); //TODO(minor) .log or .csv?
    csv_file_2 = log_path + std::string("position.csv");
    csv_file_3 = log_path + std::string("connectivity.log");
    info_file = log_path + std::string("metadata.csv");
    graph_file = log_path + std::string("graph.log");
    ds_filename = log_path + std::string("ds.csv");

    /* Create and initialize files */
    fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs_csv << "#sim_time,wall_time,optimal_ds" << std::endl;
    fs_csv.close();

    fs2_csv.open(csv_file_2.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs2_csv << "#sim_time,wall_time,x,y" << std::endl;
    fs2_csv.close();

    fs3_csv.open(csv_file_3.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs3_csv << "#sim_time,wall_time,sender_robot_id" << std::endl;
    fs3_csv.close();
    
    ds_fs.open(ds_filename.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    ds_fs << "#id,x,y" << std::endl;
    ds_fs.close();

    fs.open(info_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs << "#robot_id,num_robots,ds_selection_policy,starting_absolute_x,"
               "starting_absolute_y" << std::endl;
    fs << robot_id << "," << num_robots << "," << ds_selection_policy << "," << origin_absolute_x << "," << origin_absolute_y << std::endl;
    fs.close();

}

void docking::send_robot()
{    
    ROS_INFO("send_robot");
    
    adhoc_communication::SendEmRobot robot_msg;
    robot_msg.request.dst_robot = group_name;
    robot_msg.request.topic = "robots";
    robot_msg.request.robot.id = robot_id;
    robot_msg.request.robot.x = robot->x;
    robot_msg.request.robot.y = robot->y;
    robot_msg.request.robot.home_world_x = robot->home_world_x;
    robot_msg.request.robot.home_world_y = robot->home_world_y;
    robot_msg.request.robot.state = robot->state;
    robot_msg.request.robot.header.message_id = getAndUpdateMessageIdForTopic("robots");
    robot_msg.request.robot.header.sender_robot = robot_id;
    robot_msg.request.robot.header.timestamp = ros::Time::now().toSec();
    
    if (optimal_ds_is_set())
        robot_msg.request.robot.selected_ds = get_optimal_ds_id();
    else
        robot_msg.request.robot.selected_ds = -10;
    sc_send_robot.call(robot_msg);
    
    adhoc_communication::EmRobot robot_msg_2;
    robot_msg_2.home_world_x = robot->home_world_x;
    robot_msg_2.home_world_y = robot->home_world_y;
    pub_this_robot.publish(robot_msg_2);

    ros::Duration time = ros::Time::now() - time_start;
    fs2_csv.open(csv_file_2.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs2_csv << ros::Time::now().toSec() << "," << ros::WallTime::now().toSec() << "," << robot->x << "," << robot->y << std::endl;
    fs2_csv.close();
    
    ROS_INFO("robot sent");
}

void docking::finalize() //TODO(minor) do better
{
    ROS_INFO("Close log files");
    
    ros::Duration time = ros::Time::now() - time_start;

    fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out); //TODO(minor) avoid continusouly open-close...
    fs_csv << "#end" << std::endl;
    fs_csv << time.toSec() << ","
           << "-1"
           << "," 
           << "-1"
           << "," 
           << std::endl;
    fs_csv << "#end";
    fs_csv.close();

    fs3_csv.open(csv_file_3.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs3_csv << time.toSec() << ","
            << "-1"
            << "," 
            << std::endl;
    fs3_csv << "#end";
    fs3_csv.close();

    fs2_csv.open(csv_file_2.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs2_csv << "#end";
    fs2_csv.close();

    ros::Duration(5).sleep();

    // exit(0); //TODO(minor)
    
    ROS_INFO("finished_bool = true");
    finished_bool = true;
    
    std_msgs::Empty msg;
    pub_finish.publish(msg);
    
}

void docking::spin()
{
    ROS_INFO("Start thread to receive callbacks"); //TODO(minor) remove spin in other points of the code
    double rate = 10.0; // Hz
    ros::Rate loop_rate(rate);
    while (ros::ok() && !finished_bool)
    {
        ros::spinOnce();   
        loop_rate.sleep();  // sleep for 1/rate seconds
    }
}

void docking::update_robot_position()
{   
    ROS_INFO("update_robot_position");
    
    /* Get current robot position (once the service required to do that is ready) by calling explorer's service */
    //ros::service::waitForService("explorer/robot_pose");
    fake_network::RobotPositionSrv srv_msg;
    for(int i = 0; i < 10 && !sc_robot_pose; i++) {
        ROS_FATAL("NO MORE CONNECTION!");
        ros::Duration(1).sleep();
        //sc_robot_pose = nh.serviceClient<fake_network::RobotPosition>(my_prefix + "explorer/robot_pose", true);   
    }
    if (sc_robot_pose.call(srv_msg))
    {
        robot->x = srv_msg.response.x;
        robot->y = srv_msg.response.y;
        ROS_DEBUG("Robot position: (%f, %f)", robot->x, robot->y);
    }
    else
    {
        ROS_ERROR("Call to service %s failed; not possible to compute optimal DS "
                  "for the moment",
                  sc_robot_pose.getService().c_str());
        return;
    }
    
    fake_network::RobotPosition msg;
    msg.id = robot_id;
    double x, y;
    
    rel_to_abs(robot->x, robot->y, &x, &y);
    
    msg.world_x = x;
    msg.world_y = y;    
    
    pub_robot_absolute_position.publish(msg);
}

void docking::update_reamining_distance() {
    current_remaining_distance = next_remaining_distance;
}

float docking::conservative_remaining_distance_with_return() {
    return current_remaining_distance / (double) 2.0;
}

float docking::conservative_maximum_distance_with_return() {
    return maximum_travelling_distance / (double) 2.0;
}

float docking::conservative_remaining_distance_one_way() {
    return current_remaining_distance;
}

float docking::conservative_maximum_distance_one_way() {
    return maximum_travelling_distance;
}

void docking::wait_battery_info() {
    while(maximum_travelling_distance <= 0) {
        ROS_ERROR("Waiting battery information...");
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
}

void docking::runtime_checks() {
//    for(unsigned int i=0; i<robots.size()-1; i++)
//        for(unsigned int j=i+1; j<robots.size(); j++)
//            if(!two_robots_at_same_ds_printed && robots[i].selected_ds == robots[j].selected_ds && robots[i].state == robot_state::CHARGING && robots[j].state == robot_state::CHARGING) {
//                log_major_error("two robots recharging at the same DS!!!");
//                ROS_DEBUG("robots are: %d, %d; ds is ds%d", robots.at(i).id, robots.at(j).id, robots[i].selected_ds);
//                two_robots_at_same_ds_printed = true;
//            }
    
    ds_mutex.lock();
    if(num_ds > 0)          
        if(!invalid_ds_count_printed && (ds.size() + undiscovered_ds.size() + discovered_ds.size() > (unsigned int)num_ds) ){
            log_major_error("invalid number of DS!");
            invalid_ds_count_printed = true;
            
            ROS_DEBUG("ds.size(): %lu; content:", (long unsigned int)ds.size());
            for(unsigned int i=0; i < ds.size(); i++)
                ROS_DEBUG("%d", ds.at(i).id);
                
            ROS_DEBUG("undiscovered_ds.size(): %lu; content:", (long unsigned int)undiscovered_ds.size());
            for(unsigned int i=0; i < undiscovered_ds.size(); i++)
                ROS_DEBUG("%d", undiscovered_ds.at(i).id);
                
            ROS_DEBUG("discovered_ds.size(): %lu; content: ", (long unsigned int)discovered_ds.size());
            for(unsigned int i=0; i < discovered_ds.size(); i++)
                ROS_DEBUG("%d", discovered_ds.at(i).id);
        }
    
    if(!ds_appears_twice_printed)
        for(unsigned int i=0; i < ds.size()-1; i++)
            for(unsigned int j=i+1; j < ds.size(); j++)
                if(ds.at(i).id == ds.at(j).id) {
                    log_major_error("a DS appears twice!!");
                    ROS_ERROR("ds.size(): %lu, undiscovered_ds.size(): %lu, discovered_ds.size(): %lu", (long unsigned int)ds.size(), (long unsigned int)undiscovered_ds.size(), (long unsigned int)discovered_ds.size());
                    ds_appears_twice_printed = true;
                }
                
    ds_mutex.unlock();
}

void docking::path_callback(const std_msgs::String msg) {
    ros_package_path = msg.data;
}

void docking::log_major_error(std::string text) {
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

//        std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
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

void docking::log_minor_error(std::string text) {
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

//        std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
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

void docking::send_ds() {
    ROS_INFO("send_ds");
    adhoc_communication::SendEmDockingStation srv_msg;
    srv_msg.request.topic = "docking_stations";
    srv_msg.request.dst_robot = group_name;
    
    ds_mutex.lock();
    
//    boost::shared_lock< boost::shared_mutex > lock(ds_mutex);
    for(unsigned int i=0; i < ds.size(); i++) {
        double x, y;
        rel_to_abs(ds.at(i).x, ds.at(i).y, &x, &y);
        srv_msg.request.docking_station.x = x;  // it is necessary to fill also this fields because when a Ds is
                                                // received, robots perform checks on the coordinates
        srv_msg.request.docking_station.y = y;
        srv_msg.request.docking_station.id = ds.at(i).id;
        srv_msg.request.docking_station.vacant = ds.at(i).vacant;
        srv_msg.request.docking_station.header.timestamp = ds.at(i).timestamp; //TODO should timestamp be part of ds_t or not ???
        srv_msg.request.docking_station.header.sender_robot = robot_id;
        srv_msg.request.docking_station.header.message_id = getAndUpdateMessageIdForTopic("docking_stations");
        sc_send_docking_station.call(srv_msg);
    }
    
    ds_mutex.unlock();
}

unsigned int docking::getAndUpdateMessageIdForTopic(std::string topic) {
    mutex_message.lock();
    auto search = topic_ids.find(topic);
    if(search == topic_ids.end()) {
        topic_ids.insert({topic, 1});
        search = topic_ids.find(topic);
    }
    unsigned int return_value = search->second * pow(10, (ceil(log10(num_robots)))) + robot_id;
    search->second++;
    mutex_message.unlock();
    return return_value;
}

bool docking::checkAndUpdateReceivedMessageId(std::string topic, unsigned int message_id, unsigned int sender_robot_id) {
    bool return_value = false;
    auto search = received_topic_ids.find(topic);
    unsigned int local_id = (unsigned int)(floor(message_id / pow(10, (ceil(log10(num_robots))))));
    if(search == received_topic_ids.end()) {
//        ROS_FATAL("invalid topic"); //TODO insert automatically? but in this case we cannot avoid errors due to wrong topics
        received_topic_ids.insert({topic, {}});
        search = received_topic_ids.find(topic);
        ROS_INFO("inserted topic %s", topic.c_str());
    }
    if(search != received_topic_ids.end())
    {
        auto search2 = search->second.find(sender_robot_id);
        if(search2 == search->second.end()) {
//            ROS_FATAL("invalid robot");
            search->second.insert({sender_robot_id, local_id - 1});
            search2 = search->second.find(sender_robot_id);
            ROS_INFO("inserted robot %u for topic %s", sender_robot_id, topic.c_str());
        }
        if(search2 != search->second.end()) 
        {
            unsigned int expected_id = search2->second + 1;
            return_value = (local_id == expected_id);
            search2->second = local_id;
            if(!return_value) {
                ROS_DEBUG("message_id: %u", message_id);
                ROS_DEBUG("local_id: %u", local_id);
                ROS_DEBUG("expected_id: %u", expected_id);
            }
        }            
    }

    return return_value;
}

void docking::send_optimal_ds() {
    adhoc_communication::EmDockingStation msg_optimal;
    msg_optimal.id = get_optimal_ds_id();
    msg_optimal.x = get_optimal_ds_x();
    msg_optimal.y = get_optimal_ds_y();
    pub_new_optimal_ds.publish(msg_optimal);
}

void docking::ds_management() {
    ROS_INFO("ds_management started");

    ros::Time last_sent = ros::Time::now();
    while(ros::ok() && !finished_bool){
            
        discover_docking_stations();    
        check_reachable_ds();   
        compute_optimal_ds();
        runtime_checks();
        log_optimal_ds();
        if(ros::Time::now() - last_sent > ros::Duration(5)) {
            send_ds();
            send_robot();
            send_optimal_ds();
            last_sent = ros::Time::now();   
        }
        ros::Duration(1).sleep();
    }
}
