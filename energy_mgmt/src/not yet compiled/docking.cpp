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

void docking::check_vacancy_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg)  // TODO(minor) explain
                                                                                                  // very well the
                                                                                                  // choices
{
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
//        if (robot_state == robot_state::CHARGING || robot_state == robot_state::GOING_CHARGING || robot_state == robot_state::GOING_CHECKING_VACANCY ||
//            robot_state == robot_state::CHECKING_VACANCY)
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
            srv_msg.request.docking_station.id = get_optimal_ds_id();
            srv_msg.request.docking_station.used_by_robot_id = robot_id;
            srv_msg.request.docking_station.request_by_robot_id = msg.get()->request_by_robot_id;
            srv_msg.request.docking_station.request_id = msg.get()->request_id;
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
