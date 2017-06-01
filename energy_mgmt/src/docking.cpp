#include <docking.h>
#define DEBUG false
#define RESOLUTION 0.05

//TODO(minor) ConstPtr

using namespace std;

//DONE (DONE without any + means that the function is correct, but it's missing comments, debug outputs, ...)
docking::docking()  // TODO(minor) create functions; comments here and in .h file; check if the topics uses the correct messages...
{
    ROS_INFO("Creating instance of Docking class...");
    
    /* Load parameters */  // TODO(minor) checks if these params exist...
    ros::NodeHandle nh_tilde("~");
    nh_tilde.param("num_robots", num_robots, -1);
    nh_tilde.param("w1", w1, 0.25);
    nh_tilde.param("w2", w2, 0.25);
    nh_tilde.param("w3", w3, 0.25);
    nh_tilde.param("w4", w4, 0.25);
    nh_tilde.getParam("x", origin_absolute_x);
    nh_tilde.getParam("y", origin_absolute_y);
    nh_tilde.param<string>("move_base_frame", move_base_frame, "map");  // TODO(minor) remove this (used only to call map_merger translation)
    nh_tilde.param<string>("robot_prefix", robot_prefix, "");
    nh_tilde.param<std::string>("log_path", log_path, "");
    nh_tilde.param<int>("ds_selection_policy", ds_selection_policy, -1);
    nh_tilde.param<int>("auction_duration", auction_timeout, 5); //TODO(minor) int?
    nh_tilde.param<int>("extra_auction_time", extra_time, 3);
    nh_tilde.param<int>("reauctioning_timeout", reauctioning_timeout, 10); //s
    nh_tilde.param<float>("fiducial_signal_range", fiducial_signal_range, 10.0); //m
    nh_tilde.param<bool>("fiducial_sensor_on", fiducial_sensor_on, true);         // not used at the moment...
    nh_tilde.param<float>("safety_coeff", safety_coeff, 0.8);

    // TODO(minor) other checks
    if (num_robots < 1)
        ROS_FATAL("Invalid number of robots (%d)!", num_robots);
    if (ds_selection_policy < 0 || ds_selection_policy > 4)
        ROS_FATAL("Invalid docking station selection policy (%d)!", ds_selection_policy);

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
    /*
    if(robot == NULL)
        ROS_ERROR("NULL!!!!!");
    else {
        robot->id = 10;
        ROS_ERROR("%d", robot->id);
    }
    */
    ros::Duration(10).sleep();
    robot->id = robot_id;
    robot->state = active;
    abs_to_rel(origin_absolute_x, origin_absolute_y, &(robot->x), &(robot->y));
    robots.push_back(*robot);
    
    best_ds = NULL;
    target_ds == NULL;
    best_ds = new ds_t;
    if(best_ds == NULL)
        ROS_FATAL("Allocation failure!");
    target_ds = new ds_t;
    if(target_ds == NULL)
        ROS_FATAL("Allocation failure!");
    target_ds->id = -1;
    
    /*
    temp_robot.x = 10;
    ROS_ERROR("%f, %f", temp_robot.x, robots[0].x); //TODO(minor) they are different because with push_back I'm coping the whole structure, I'm not storing it's pointer!!!
    robots[0].x = 20;
    ROS_ERROR("%f, %f", temp_robot.x, robots[0].x);
    robot = &robots[0];
    robot->x = 30;
    ROS_ERROR("%f, %f, %f", temp_robot.x, robot->x, robots[0].x);
    robots[0].x = 40;
    ROS_ERROR("%f, %f, %f", temp_robot.x, robot->x, robots[0].x);

    robot_t temp2_robot;
    temp2_robot.id = 100;
    temp2_robot.state = active;
    robots.push_back(temp2_robot);
    robot = &robots[1];
    */

    // TODO(minor) names (robot_0 end dockign)
    // TODO(minor) save names in variables
    // TODO(minor) queues length?


    /* Adhoc communication services */
    sc_send_auction =
        nh.serviceClient<adhoc_communication::SendEmAuction>(my_prefix + "adhoc_communication/send_em_auction");
    sc_send_docking_station = nh.serviceClient<adhoc_communication::SendEmDockingStation>(
        my_prefix + "adhoc_communication/send_em_docking_station");
    sc_send_robot = nh.serviceClient<adhoc_communication::SendEmRobot>(my_prefix + "adhoc_communication/send_em_robot");


    /* General services */
    //sc_trasform = nh.serviceClient<map_merger::TransformPoint>("map_merger/transformPoint");  // TODO(minor)
    sc_robot_pose = nh.serviceClient<fake_network::RobotPosition>(my_prefix + "explorer/robot_pose", true);
    //sc_robot_pose = nh.serviceClient<fake_network::RobotPosition>(my_prefix + "explorer/robot_pose");
    //sc_distance_from_robot = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/distance_from_robot", true);
    sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target", true);
    sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance", true);

    ss_distance_robot_frontier_on_graph = nh.advertiseService("energy_mgmt/distance_on_graph", &docking::distance_robot_frontier_on_graph_callback, this);
    //ROS_ERROR("%s", ss_distance_robot_frontier_on_graph.getService().c_str());

    /* Subscribers */
    sub_battery = nh.subscribe(my_prefix + "battery_state", 10, &docking::cb_battery, this);
    sub_robots = nh.subscribe(my_prefix + "robots", 10, &docking::cb_robots, this);
    sub_jobs = nh.subscribe(my_prefix + "frontiers", 1000, &docking::cb_jobs, this);
    sub_robot = nh.subscribe(my_prefix + "explorer/robot", 10, &docking::cb_robot, this);
    sub_docking_stations = nh.subscribe(my_prefix + "docking_stations", 10, &docking::cb_docking_stations, this);

    sub_auction_starting = nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/new_auction", 10,
                                        &docking::cb_new_auction, this);
    sub_auction_reply =
        nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/reply", 10, &docking::cb_auction_reply, this);
    sub_auction_winner_adhoc =
        nh.subscribe(my_prefix + "adhoc_communication/auction_winner", 10, &docking::cb_auction_result, this);

    sub_adhoc_new_best_ds = nh.subscribe("adhoc_new_best_docking_station_selected", 10, &docking::adhoc_ds, this);

    sub_charging_completed = nh.subscribe("charging_completed", 10, &docking::cb_charging_completed, this);

    sub_translate = nh.subscribe("translate", 10, &docking::cb_translate, this);
    sub_all_points = nh.subscribe("all_positions", 10, &docking::points, this);

    sub_check_vacancy = nh.subscribe("adhoc_communication/check_vacancy", 10, &docking::check_vacancy_callback, this);
    
    sub_resend_ds_list = nh.subscribe("adhoc_communication/resend_ds_list", 10, &docking::resend_ds_list_callback, this);
    sub_full_battery_info = nh.subscribe("full_battery_info", 10, &docking::full_battery_info_callback, this);
    
    
    /* Publishers */
    pub_ds = nh.advertise<std_msgs::Empty>("docking_station_detected", 10);
    pub_new_target_ds =
        nh.advertise<geometry_msgs::PointStamped>("new_target_docking_station_selected", 10);  // to tell explorer...
    pub_adhoc_new_best_ds =
        nh.advertise<adhoc_communication::EmDockingStation>("adhoc_new_best_docking_station_selected", 10);
    pub_lost_own_auction = nh.advertise<std_msgs::Empty>("explorer/lost_own_auction", 10);
    pub_won_auction = nh.advertise<std_msgs::Empty>("explorer/won_auction", 10);
    pub_lost_other_robot_auction = nh.advertise<std_msgs::Empty>("explorer/lost_other_robot_auction", 10);

    pub_auction_result = nh.advertise<std_msgs::Empty>("explorer/auction_result", 10);

    pub_moving_along_path = nh.advertise<adhoc_communication::MmListOfPoints>("moving_along_path", 10);
    sub_next_ds = nh.subscribe("next_ds", 10, &docking::next_ds_callback, this);
    
    pub_finish = nh.advertise<std_msgs::Empty>("explorer/finish", 10);
    
    pub_test = nh.advertise<std_msgs::Empty>("test", 10);
    sub_test = nh.subscribe("test", 10, &docking::test_2, this);
	pub_new_optimal_ds = nh.advertise<adhoc_communication::EmDockingStation>("explorer/new_optimal_ds", 10);
	
	pub_robot_absolute_position = nh.advertise<adhoc_communication::EmRobot>("fake_network/robot_absolute_position", 10);
	
	pub_new_ds_on_graph = nh.advertise<adhoc_communication::EmDockingStation>("new_ds_on_graph", 10);
	pub_ds_count = nh.advertise<std_msgs::Int32>("ds_count", 10);

    /* Timers */
    timer_finish_auction = nh.createTimer(ros::Duration(auction_timeout), &docking::timerCallback, this, true, false);
    timer_restart_auction =
        nh.createTimer(ros::Duration(reauctioning_timeout), &docking::timer_callback_schedure_auction_restarting, this, true, false);

    /* Variable initializations */
    robot_state = fully_charged;  // TODO(minor) param
    auction_winner = false;
    started_own_auction = false;
    update_state_required = false;
    moving_along_path = false;
    managing_auction = false;
    need_to_charge = false;  // TODO(minor) useless variable (if we use started_own_auction)
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
    llh = 0;
    local_auction_id = 0;
    participating_to_auction = 0;
    maximum_travelling_distance = -100;
    old_optimal_ds_id = -100;
    time_start = ros::Time::now();

    /* Function calls */
    preload_docking_stations();
    create_log_files();

    // TODO(minor) vary bad if we don't know the number of DS a-priori
    /* Initialize docking station graph */
    for (int i = 0; i < num_ds; i++)
    {
        std::vector<int> temp;
        std::vector<float> temp_f;
        for (int j = 0; j < undiscovered_ds.size(); j++) {
            temp.push_back(-1);
            temp_f.push_back(-1);
        }
        ds_mst.push_back(temp);
        ds_graph.push_back(temp_f);
    }

    if (DEBUG)
    {
        ros::Timer timer0 = nh.createTimer(ros::Duration(20), &docking::debug_timer_callback_0, this, false, true);
        debug_timers.push_back(timer0);
        ros::Timer timer1 = nh.createTimer(ros::Duration(20), &docking::debug_timer_callback_1, this, false, true);
        debug_timers.push_back(timer1);
        ros::Timer timer2 = nh.createTimer(ros::Duration(20), &docking::debug_timer_callback_2, this, false, true);
        debug_timers.push_back(timer2);

        debug_timers[robot_id].stop();
    }
    
    ROS_INFO("Instance of Docking class correctly created");
    
    pub_wait = nh.advertise<std_msgs::Empty>("explorer/are_you_ready", 10);
    sub_wait = nh.subscribe("explorer/im_ready", 10, &docking::wait_for_explorer_callback, this);
    
    DsGraph mygraph;
    mygraph.addEdge(1,2,10);
    //mygraph.print();
    
}

void docking::test() {
    std_msgs::Empty msg;
    //pub_test.publish(msg);
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
    sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance", true);
    while(!sc_distance) {
        ROS_ERROR("No connection to service 'explorer/distance': retrying...");
        ros::Duration(3).sleep();
        sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance", true);
    }
    ROS_INFO("Established persistent connection to service 'explorer/distance'");
    //ros::Duration(0.1).sleep();
    
    ros::service::waitForService("explorer/reachable_target");
    sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target", true);
    for(int i = 0; i < 10 && !sc_reachable_target; i++) {
        ROS_ERROR("No connection to service 'explorer/reachable_target': retrying...");
        ros::Duration(3).sleep();
        sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target", true);
    }
    ROS_INFO("Established persistent connection to service 'explorer/reachable_target'");
    //ros::Duration(0.1).sleep();
    
    ros::service::waitForService("explorer/robot_pose");
    sc_robot_pose = nh.serviceClient<fake_network::RobotPosition>(my_prefix + "explorer/robot_pose", true);   
    for(int i = 0; i < 10 && !sc_robot_pose; i++) {
        ROS_ERROR("No connection to service 'explorer/robot_pose': retrying...");
        ros::Duration(3).sleep();
        sc_robot_pose = nh.serviceClient<fake_network::RobotPosition>(my_prefix + "explorer/robot_pose", true);   
    }
    ROS_INFO("Established persistent connection to service 'explorer/robot_pose'");    
    //ros::Duration(0.1).sleep();
    
    ros::Duration(5).sleep();
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

//DONE+
void docking::preload_docking_stations()
{
    ROS_INFO("Preload DSs and mark them as undiscovered");

    int index = 0;  // index of the DS, used to loop among all the docking stations
                    // inserted in the file
    double x, y;    // DS coordinates

    /* If the x-coordinate of a DS with index <index> is found, it means that that DS
     * is present in the file and must be loaded. Notice that we assume that if there is a x-coordinate, also the corresponding y is present */
    ros::NodeHandle nh_tilde("~");
    while (nh_tilde.hasParam("d" + SSTR(index) + "/x")) //TODO(minor) maybe ds would be nicer
    {
        /* Load coordinates of the new DS */
        nh_tilde.param("d" + SSTR(index) + "/x", x, 0.0);
        nh_tilde.param("d" + SSTR(index) + "/y", y, 0.0);

        /* Store new DS */
        ds_t new_ds;
        new_ds.id = index;
        abs_to_rel(x, y, &(new_ds.x), &(new_ds.y));
        new_ds.vacant = true;  // TODO(minor) param...
        undiscovered_ds.push_back(new_ds);

        /* Delete the loaded parameters (since they are not used anymore) */
        nh_tilde.deleteParam("d" + SSTR(index) + "/x");
        nh_tilde.deleteParam("d" + SSTR(index) + "/y");

        /* Prepare to search for next DS (if it exists) in the file */
        index++;
    }
    
    /* Store the number of DSs in the environment */
    num_ds = undiscovered_ds.size(); //TODO(minor) a problem in general!!

    /* Print loaded DSs with their coordinates relative to the local reference system of the robot; only for debugging
     */  // TODO(minor) or better in global system?
    std::vector<ds_t>::iterator it;
    for (it = undiscovered_ds.begin(); it != undiscovered_ds.end(); it++)
        ROS_DEBUG("ds%d: (%f, %f)", (*it).id, (*it).x, (*it).y);
        
}

void docking::compute_optimal_ds() //TODO(minor) best waw to handle errors in distance computation??
//TODO(minor) maybe there are more efficient for some steps; for instance, i coudl keep track somewhere of the DS with EOs and avoid recomputing them two times in the same call of this funcion...
{   
    ROS_DEBUG("Compute optimal DS");

    /* Compute optimal DS only if at least one DS is reachable (just for efficiency and debugging) */
    if (ds.size() > 0)
    {
        /* Store currently optimal DS (for debugging ans safety checks)
         *
         * NB: even if the robot position does not change during the execution of
         * this function, the distance between the robot and a fixed point computed
         * using distance_from_robot() could change, because it calls a function in
         * explorer node that uses a costmap to compute the distance, and since the
         * costmap could be updated during the execution of compute_optimal_ds() (or,
         * more simply, the algorithm used to compute the distance is not 100%
         * precise), the distance could change: this means that we need to check at
         * the end if the optimal DS that we have computed is really a
         * different DS from the previous one; this can happens for instance because
         * the other robots are moving. Of course the differences between two
         * different calls of distance_from_robot with the same point as argument
         * will be very little, but they are enough to make us think that we have
         * found a DS that is better than the one already selected, even if the new
         * DS is exactly the old one. Another problems is that we could select a DS
         * that is not really the closest at the end of the execution of
         * compute_optimal_ds() because the distances could have changed, but we
         * accept this. */

        if(old_optimal_ds_id > 10000) //can happen sometimes... why??
            ROS_ERROR("WHAT?????????????????????????");

        // TODO(minor) functions
        /* "Closest" policy */
        if (ds_selection_policy == 0)  // TODO(minor) switch-case
            compute_closest_ds();

        /* "Vacant" policy */
        else if (ds_selection_policy == 1)
        {
            /* Check if there are vacant DSs. If there are, check also which one is the closest to the robot */
            bool found_vacant_ds = false;
            double min_dist = numeric_limits<int>::max();
            for (std::vector<ds_t>::iterator it = ds.begin(); it != ds.end(); it++)
            {
                if ((*it).vacant)
                {
                    /* We have just found a vacant DS (possibly the one already selected as optimal DS before calling
                     * this function).
                     *
                     * Notice that is is important to consider also the already selected optimal DS when we loop on all
                     *the DSs, since we have to update variable 'found_vacant_ds' to avoid falling back to "closest"
                     *policy, i.e., we cannot use a check like best_ds->id != (*it).id here */
                    found_vacant_ds = true;

                    /* Check if that DS is also the closest one */
                    double dist = distance_from_robot((*it).x, (*it).y);
                    if (dist < 0) {
                        ROS_ERROR("Distance computation failed: skipping this DS in the computation of the optimal DS..."); //TODO(minor) place everywhere, and write it better...
                        continue;
                    }
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        set_optimal_ds_given_id(it->id);
                    }
                }
            }

            /* If no DS is vacant at the moment, use "closest" policy to update the
             * optimal DS */
            if (!found_vacant_ds)
            {
                ROS_DEBUG("No vacant DS found: fall back to 'closest' policy");
                compute_closest_ds();
            }
        }

        /* "Opportune" policy */
        else if (ds_selection_policy == 2)
        {
            if (!moving_along_path)
            {
                /* Check if there are reachable DSs (i.e., DSs that the robot can reach with the remaining battery life) with EOs */
                double min_dist = numeric_limits<int>::max();
                bool found_reachable_ds_with_eo = false, found_ds_with_eo = false;
                for (int i = 0; i < ds.size(); i++)
                    for (int j = 0; j < jobs.size(); j++)
                    {
                        double dist = distance(ds.at(i).x, ds.at(i).y, jobs.at(j).x_coordinate, jobs.at(j).y_coordinate);
                        if (dist < 0)
                            continue;
                        if (dist < conservative_maximum_distance_with_return())
                        {
                            /* We have found a DS with EOs */
                            found_ds_with_eo = true;

                            //TODO(minor) maybe it will be more efficient to invert the checks? but am I sure taht it works considering the code later?
                            /* Check if that DS is also directly reachable (i.e., without
                             * recharging at intermediate DSs) */
                            double dist2 = distance_from_robot(ds.at(i).x, ds.at(i).y);
                            if (dist2 < 0)
                                continue;
                            if (dist2 < conservative_remaining_distance_one_way())
                            {
                                /* We have found a DS that is directly reachable and with EOs */
                                found_reachable_ds_with_eo = true;

                                /* Check if it also the closest reachable DS with EOs */
                                if (dist2 < min_dist)  // TODO(minor) maybe another heuristics would be better...
                                {
                                    /* Update optimal DS */
                                    min_dist = dist2;
                                    set_optimal_ds_given_index(i);
                                }
                            }
                        }
                    }

                /* If there are no reachable DSs with EOs, check if there are DSs with
                 * EOs: if there are, compute a path on the graph of the DSs to reach
                 * one of these DSs, otherwise jsut use "closest" policy */
                if (!found_reachable_ds_with_eo)
                {
                    if (found_ds_with_eo)
                    {
                        /* Compute a path formed by DSs that must be used for recharging, to
                         * reach a DS with EOs */
                        bool ds_found_with_mst = false;

                        
                        //if (!OPP_ONLY_TWO_DS)  // TODO(minor)
                        {
                            //TODO(minor) is it better to reacharge at each intermediate DS as soon as one Ds is found, or just when it is strictly necessary???
                            // compute closest DS with EOs // TODO(minor) are we sure that this is
                            // what the paper asks?
                            double min_dist = numeric_limits<int>::max();
                            ds_t *min_ds = NULL;
                            for (int i = 0; i < ds.size(); i++)
                            {
                                for (int j = 0; j < jobs.size(); j++)
                                {
                                    double dist = distance(ds.at(i).x, ds.at(i).y, jobs.at(j).x_coordinate, jobs.at(j).y_coordinate);
                                    if (dist < 0)
                                        continue;

                                    if (dist < conservative_maximum_distance_with_return())
                                    {
                                        double dist2 = distance_from_robot(ds.at(i).x, ds.at(i).y);
                                        if (dist2 < 0)
                                            continue;

                                        if (dist2 < min_dist)
                                        {
                                            min_dist = dist2;
                                            min_ds = &ds.at(i);
                                        }
                                        
                                        break;
                                    }
                                }
                            }
                            if (min_ds == NULL)
                                return;  // this could happen if distance() always fails... //TODO(IMPORTANT) what happen if I return and the explorer node needs to reach a frontier?

                            // compute closest DS
                            min_dist = numeric_limits<int>::max();
                            ds_t *closest_ds;
                            for (int i = 0; i < ds.size(); i++)
                            {
                                double dist = distance_from_robot(ds.at(i).x, ds.at(i).y);
                                if (dist < 0)
                                    continue;

                                if (dist < min_dist)
                                {
                                    min_dist = dist;
                                    closest_ds = &ds.at(i);
                                }
                            }

                            path.clear();
                            index_of_ds_in_path = 0;
                            ds_found_with_mst = find_path_2(closest_ds->id, min_ds->id, path);

                            if (ds_found_with_mst)
                            {
                                int closest_ds_id;

                                moving_along_path = true;

                                adhoc_communication::MmListOfPoints msg_path;  // TODO(minor)
                                                                               // maybe I can
                                                                               // pass directly
                                                                               // msg_path to
                                                                               // find_path...
                                for (int i = 0; i < path.size(); i++)
                                    for (int j = 0; j < ds.size(); j++)
                                        if (ds[j].id == path[i])
                                        {
                                            msg_path.positions[i].x = ds[j].x;
                                            msg_path.positions[i].y = ds[j].y;
                                        }

                                pub_moving_along_path.publish(msg_path);

                                for (int j = 0; j < ds.size(); j++)
                                    if (path[0] == ds[j].id)
                                    {
                                        //TODO(minor) it should be ok... but maybe it would be better to differenciate an "intermediate target DS" from "target DS": moreover, are we sure that we cannot compute the next optimal DS when moving_along_path is true?
                                        set_optimal_ds_given_index(j);
                                        target_ds = &ds[j];
                                    }
                            }
                            else
                                // closest policy //TODO(minor) when could this happen?
                                compute_closest_ds();
                        }

                        /*
                        else  // only two ds... not really implemented...
                        {
                            if (!moving_along_path)
                            {
                                double first_step_x, first_step_y, second_step_x, second_step_y;
                                for (int i = 0; i < ds.size(); i++)
                                {
                                    bool existing_eo;
                                    for (int j = 0; j < jobs.size(); j++)
                                    {
                                        double dist = distance(ds.at(i).x, ds.at(i).y, jobs.at(j).x, jobs.at(j).y);
                                        if (dist < 0)
                                            continue;
                                        if (dist < battery.remaining_distance / 2)
                                        {
                                            existing_eo = true;
                                            for (int k = 0; k < ds.size(); k++)
                                            {
                                                dist = distance(ds.at(i).x, ds.at(i).y, ds.at(k).x, ds.at(k).y);
                                                if (dist < 0)
                                                    continue;
                                                if (k != i && dist < battery.remaining_distance / 2)
                                                {
                                                    double dist2 = distance_from_robot(ds.at(k).x, ds.at(k).y);
                                                    if (dist2 < 0)
                                                        continue;
                                                    if (dist2 < battery.remaining_distance / 2)
                                                    {
                                                        moving_along_path = true;
                                                        ds_found_with_mst = true;
                                                        first_step_x = ds.at(k).x;
                                                        first_step_y = ds.at(k).y;
                                                        second_step_x = ds.at(i).x;
                                                        second_step_y = ds.at(i).y;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                if (ds_found_with_mst)
                                {
                                    moving_along_path = true;

                                    adhoc_communication::MmListOfPoints msg_path;
                                    msg_path.positions[0].x = first_step_x;
                                    msg_path.positions[0].y = first_step_y;
                                    msg_path.positions[1].x = second_step_x;
                                    msg_path.positions[1].y = second_step_y;

                                    pub_moving_along_path.publish(msg_path);
                                }
                            }
                        }
                        */
                    }
                    else 
						if(no_jobs_received_yet)
	                        compute_closest_ds();  // TODO(minor) although probably the robot will think
	                                               // that the exploration is over given how
	                                               // explorer works for the moment...
						else 
                        	finished_bool = true;
                }
            }
            else
                ROS_ERROR("Already moving along path...");
        }

        /* "Current" policy */
        else if (ds_selection_policy == 3)
        {
            /* If no optimal DS has been selected yet, use "closest" policy, otherwise use "current" policy */ //TODO(minor) assumption
            if (!optimal_ds_is_set()) {
                ROS_DEBUG("No optimal docking station selected yet: fall back to 'closest' policy");
                compute_closest_ds();
            }
            else
            {
                /* If the currently optimal DS has still EOs, keep using it, otherwise use
                 * "closest" policy */
                bool existing_eo = false;
                for (int i = 0; i < jobs.size(); i++)
                {
                    double dist = distance(best_ds->x, best_ds->y, jobs.at(i).x_coordinate, jobs.at(i).y_coordinate);
                    if (dist < 0) {
                        //ROS_ERROR("Computation of DS-frontier distance failed: ignore this frontier (i.e., do not consider it an EO)");
                        continue;
                    }
                    if (dist < conservative_maximum_distance_with_return())
                    {
                        existing_eo = true;
                        break;
                    }
                }
                if (!existing_eo) {
                    ROS_DEBUG("Current optimal DS has no more EOs: use 'closest' policy to compute new optimal DS");
                    compute_closest_ds();
                }
            }
        }

        /* "Flocking" policy */
        else if (ds_selection_policy == 4) //TODO(minor) comments and possibly use functions...
        {   
            /* Compute DS with minimal cost */
            double min_cost = numeric_limits<int>::max();
            for (int d = 0; d < ds.size(); d++)
            {
                /* n_r */
                int count = 0;
                for (int i = 0; i < robots.size(); i++)
                    if (optimal_ds_is_set() &&
                        robots.at(i).selected_ds == best_ds->id)  // TODO(minor) best_ds or target_ds??? optimal_ds_is_set()?
                        count++;
                double n_r = (double)count / (double)num_robots;

                /* d_s */
                int sum_x = 0, sum_y = 0;
                for (int i = 0; i < robots.size(); i++)
                {
                    sum_x += robots.at(i).x;
                    sum_y += robots.at(i).y;
                }
                double flock_x = (double)sum_x / (double)num_robots;
                double flock_y = (double)sum_y / (double)num_robots;
                double max_distance = numeric_limits<int>::min();
                for(int h=0; h < ds.size(); h++) {
                    double dist = distance(ds[d].x, ds[d].y, ds[h].x, ds[h].y);
                    if(dist < 0)
                        continue; //TODO(minor) hmm...
                    if(dist < conservative_maximum_distance_one_way())
                        if(dist > max_distance)
                            max_distance = dist;
                    
                }
                if(max_distance == numeric_limits<int>::min())
                    continue; //TODO(minor) hmm
                double d_s = distance(ds.at(d).x, ds.at(d).y, flock_x, flock_y) / max_distance;
                if (d_s < 0)
                    continue;

                /* theta_s */
                double swarm_direction_x = 0, swarm_direction_y = 0;
                for (int i = 0; i < robots.size(); i++)
                {
                    double robot_i_target_ds_x, robot_i_target_ds_y;

                    for (int k = 0; k < ds.size(); k++)
                        if (robots.at(i).selected_ds == ds.at(k).id)
                        {
                            robot_i_target_ds_x = ds.at(k).x;
                            robot_i_target_ds_y = ds.at(k).y;
                        }

                    swarm_direction_x += robot_i_target_ds_x - robots.at(i).x;
                    swarm_direction_y += robot_i_target_ds_y - robots.at(i).y;
                }
                double rho = atan2(swarm_direction_y, swarm_direction_x) * 180 / PI; //degree; e.g., with atan2(1,1), rho is 45.00
                                                                              //To compute the value, the function takes into account the sign of both arguments in order to determine the quadrant.
                double alpha = atan2((ds.at(d).y - robot->y), (ds.at(d).x - robot->x)) * 180 / PI;
                double theta_s = fabs(alpha - rho) / (double)180;

                /* d_f */
                double d_f = numeric_limits<int>::max();
                for (int i = 0; i < jobs.size(); i++)
                {
                    double dist = distance(ds[d].x, ds[d].y, jobs[i].x_coordinate, jobs[i].y_coordinate);
                    if (dist < 0)
                        continue;
                    if (dist < d_f)
                        d_f = dist;
                }

                double cost = n_r + d_s + theta_s + d_f;
                if (cost < min_cost)
                {
                    min_cost = cost;
                    set_optimal_ds_given_index(d);
                }
            }
        }

        /* If a new optimal DS has been found, parameter l4 of the charging likelihood function must be updated. Notice that the other robots will be informed about this when the send_robot_information() function is called */
        if (!optimal_ds_is_set())
            ROS_DEBUG("No optimal DS has been selected yet");
        else if (old_optimal_ds_id != best_ds->id)
        {
            finished_bool = false; //TODO(minor) find better place...
            
            /* Debug output */
            if (old_optimal_ds_id >= 0) //TODO bad way to check if a ds has been already selected...
                ROS_INFO("Change optimal DS: ds%d -> ds%d", old_optimal_ds_id, best_ds->id);
            else
                ROS_INFO("Change optimal DS: (none) -> ds%d", best_ds->id);
            if(best_ds->id > 10000) //can happen sometimes... buffer overflow somewhere?
                ROS_ERROR("OH NO!!!!!!!!!!!!");
                
            old_optimal_ds_id = best_ds->id;

            /* Keep track of the new optimal DS in log file */
            ros::Duration time = ros::Time::now() - time_start;
            fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
            fs_csv << time.toSec() << "," << best_ds->id << std::endl;
            fs_csv.close();

            /* Update parameter l4 */
            //update_l4();
            
            /* Notify explorer about the optimal DS change */
            adhoc_communication::EmDockingStation msg_optimal;
            msg_optimal.id = best_ds->id;
            msg_optimal.x = best_ds->x;
            msg_optimal.y = best_ds->y;
            pub_new_optimal_ds.publish(msg_optimal);

        }
        else
            ROS_DEBUG("Optimal DS unchanged");
    }
    else
        ROS_DEBUG("No DS has been discovered for the moment: optimal DS has not "
                  "been computed");
}

void docking::points(const adhoc_communication::MmListOfPoints::ConstPtr &msg)  // TODO(minor)
{
    ;
}

void docking::adhoc_ds(const adhoc_communication::EmDockingStation::ConstPtr &msg)  // TODO(minor)
{
    ;  // ROS_ERROR("adhoc_ds!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}

double docking::get_llh()
{
    ROS_DEBUG("Get current value of the charging likelihood function");
    
    /* Recompute parameters if necessary */ //TODO(minor) maybe there is a better way?
    while (recompute_llh)
        update_llh();

    /* The likelihood can be updated only if the robot is not participating to an auction */  // TODO(minor) really
    // necessary???
    if (participating_to_auction == 0)
        llh = w1 * l1 + w2 * l2 + w3 * l3 + w4 * l4;
    
    return llh;
}

void docking::update_llh() {
    ROS_INFO("update_llh");
    update_l1();
    update_l2();
    update_l3();
    update_l4();
}

void docking::update_l1() //TODO(minor) would be better to update them only when get_llh() is called, for efficiency... the problem is that the check participating == 0 would not allow it..
{
    ROS_DEBUG("Update l1");
    
    /* Count vacant docking stations */
    int num_ds_vacant = 0;
    for (int i = 0; i < ds.size(); ++i)
    {
        if (ds[i].vacant == true)
            ++num_ds_vacant;
    }
    //ROS_DEBUG("Number of vacant DS: %d", num_ds_vacant);

    /* Count active robots */
    int num_robots_active = 0;
    for (int i = 0; i < robots.size(); ++i)
    {
        if (robots[i].state == active)
            ++num_robots_active;
    }
    //ROS_DEBUG("Number of active robots DS: %d", num_robots_active);

    /* Sanity checks */
    if (num_ds_vacant < 0)
    {
        ROS_ERROR("Invalid number of vacant docking stations: %d!", num_ds_vacant);
        l1 = 0;
        return;
    }
    if (num_robots_active < 0)
    {
        ROS_ERROR("Invalid number of active robots: %d!", num_robots_active);
        l1 = 1;
        return;
    }

    /* Compute l1, considerign the boundaries */
    if (num_ds_vacant > num_robots_active)
    
        l1 = 1;
    
    else if (num_robots_active == 0)
    
        l1 = 0;
    
    else
    
        l1 = num_ds_vacant / num_robots_active;
   
}

void docking::update_l2()
{
    ROS_DEBUG("Update l2");
    
    double time_run = battery.remaining_time_run;
    //ROS_DEBUG("Remaining running time: %.3fs", time_run);
    
    double time_charge = battery.remaining_time_charge;
    //ROS_DEBUG("Remaining time until recharge completion: %.3fs", time_charge);

    /* Sanity checks */
    if (time_charge < 0)
    {
        ROS_ERROR("Invalid charging time: %.2f!", time_charge);
        l2 = 0;
        return;
    }
    if (time_run < 0)
    {
        ROS_ERROR("Invalid run time: %.2f!", time_run);
        l2 = 1;
        return;
    }
    if (time_run == 0 && time_charge == 0)
    {
        ROS_ERROR("Invalid run and charging times. Both are zero!");
        l2 = 1;
        return;
    }

    /* Compute l2 */
    l2 = time_charge / (time_charge + time_run);
}

void docking::update_l3()
{
    ROS_DEBUG("Update l3");
    
    /* Count number of jobs: count frontiers and reachable frontiers */
    int num_jobs = 0;
    int num_jobs_close = 0;
    for (int i = 0; i < jobs.size(); ++i)
    {
        ++num_jobs;
        double dist = distance_from_robot(jobs[i].x_coordinate, jobs[i].y_coordinate, true); // use euclidean distance to make it faster //TODO(minor) sure? do the same somewhere else?
        
        /* If it is not possible to compute the distance from this job, it is better to restart the computation of l3 later */
        if (dist < 0)
        {
            ROS_INFO("Computation of l3 failed: it will be recomputed later...");
            recompute_llh = true;
            return;
        }
        
        if (dist <= conservative_maximum_distance_with_return()) //NB I'm considering the frontiers that are reachable, possibly, with a recharging, whereare previopusly I've count just the unvisited frontiers, not matter if they are reachable or not...
            ++num_jobs_close;
    }
    //ROS_DEBUG("Number of frontiers: %d", num_jobs);
    //ROS_DEBUG("Number of reachable frontiers: %d", num_jobs_close);
    
    /* If the execution flow reaches this point, the (re)computation of l3 succeeded */
    recompute_llh = false;

    /* Sanity checks */
    if (num_jobs < 0)
    {
        ROS_ERROR("Invalid number of jobs: %d", num_jobs);
        l3 = 1;
        return;
    }
    if (num_jobs_close < 0)
    {
        ROS_ERROR("Invalid number of jobs close by: %d", num_jobs_close);
        l3 = 1;
        return;
    }
    if (num_jobs_close > num_jobs)
    {
        ROS_ERROR("Number of jobs close by greater than total number of jobs: %d > %d", num_jobs_close, num_jobs);
        l3 = 0;
        return;
    }

    /* Compute l3, considering boundaries */
    if (num_jobs == 0)
        l3 = 1;
    else
        l3 = (num_jobs - num_jobs_close) / num_jobs;
        
}

void docking::update_l4() //TODO(minor) comments
{
    ROS_DEBUG("Update l4");
    
    if (!optimal_ds_is_set() || ds.size() == 0 || jobs.size() == 0)
    {
        ROS_DEBUG("No optimal DS and/or frontiers");
        l4 = 0;
        return;
    }
    
    // get distance to docking station
    double dist_ds = -1;
    for (int i = 0; i < ds.size(); i++)
    {       
        if (ds[i].id == best_ds->id)
        {
            dist_ds = distance_from_robot(ds[i].x, ds[i].y);
            //ROS_ERROR("%f, %f", ds[i].x, ds[i].y);
            if (dist_ds < 0)
            {
                ROS_ERROR("Computation of l4 failed: it will be recomputed later...");
                recompute_llh = true;
                return;
            }
            break;
        }
    }
    //ROS_DEBUG("Distance to optimal DS: %.3f", dist_ds);

    // get distance to closest job
    double dist_job = numeric_limits<int>::max();
    for (int i = 0; i < jobs.size(); i++)
    {
        double dist_job_temp = distance_from_robot(jobs[i].x_coordinate, jobs[i].y_coordinate,
                                                true);  // use euclidean distance to make it faster //TODO(minor) sure? do the same somewhere else?
        if (dist_job_temp < 0)
        {
            recompute_llh = true;
            return;
        }
        if (dist_job_temp < dist_job)
            dist_job = dist_job_temp;
    }
    //ROS_DEBUG("Distance to closest frontier: %.3f", dist_job);
    
    recompute_llh = false;

    // sanity checks
    if (dist_job < 0 || dist_job >= numeric_limits<int>::max())
    {
        ROS_ERROR("Invalid distance to closest job: %.3f", dist_job);
        l4 = 0;
        return;
    }
    if (dist_job == 0 && dist_ds == 0)
    {
        ROS_ERROR("Invalid distances to closest job and docking station. Both are zero!");
        l4 = 0;
        return;
    }      

    // compute l4
    l4 = dist_job / (dist_job + dist_ds);

    recompute_llh = false;
}



bool docking::auction_send_multicast(string multicast_group, adhoc_communication::EmAuction auction,
                                     string topic)  // TODO(minor) useless?
{
    adhoc_communication::SendEmAuction auction_service;

    string destination_name = multicast_group + robot_name;

    ROS_INFO("Sending auction to multicast group %s on topic %s", destination_name.c_str(), topic.c_str());
    auction_service.request.dst_robot = destination_name;
    auction_service.request.auction = auction;
    auction_service.request.topic = topic;

    if (sc_send_auction.call(auction_service))
    {
        if (auction_service.response.status)
        {
            ROS_INFO("Auction was transmitted successfully.");
            return true;
        }
        else
        {
            ROS_WARN("Failed to send auction to multicast group %s!", destination_name.c_str());
            return false;
        }
    }
    else
    {
        ROS_WARN("Failed to call service %s/adhoc_communication/send_auction [%s]", robot_name.c_str(),
                 sc_send_auction.getService().c_str());
        return false;
    }
}

double docking::distance_from_robot(double goal_x, double goal_y, bool euclidean)
{
    return distance(robot->x, robot->y, goal_x, goal_y, euclidean);
}

double docking::distance(double start_x, double start_y, double goal_x, double goal_y, bool euclidean)
{
    /* Use euclidean distance if required by the caller */
    if (euclidean)
    {
        double dx = (goal_x - start_x) * RESOLUTION; //TODO bad...
        double dy = (goal_y - start_y) * RESOLUTION;
        
        return sqrt(dx * dx + dy * dy);
    }

    /* Otherwise, use actual distance: ask explorer node to compute it (using its costmap) */    
    explorer::Distance srv_msg;
    srv_msg.request.x1 = start_x;
    srv_msg.request.y1 = start_y;
    srv_msg.request.x2 = goal_x;
    srv_msg.request.y2 = goal_y;
    
    //ros::service::waitForService("explorer/distance");   
    for(int i = 0; i < 10 && !sc_distance; i++) {
        ROS_FATAL("NO MORE CONNECTION!");
        ros::Duration(1).sleep();
        sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance", true);
    }
    for (int i = 0; i < 10; i++)
        if (sc_distance.call(srv_msg) && srv_msg.response.distance >= 0) {
            return srv_msg.response.distance;
        } else
            ros::Duration(1).sleep();
    
    
    /* If the service is not working at the moment, return invalid value */ //TODO(minor) raise exception?
    ROS_ERROR("Unable to compute distance at the moment...");
    return -1;
}

void docking::cb_battery(const energy_mgmt::battery_state::ConstPtr &msg)
{
    //ROS_DEBUG("Received battery state");

    /* Store new battery state */
    battery.charging = msg.get()->charging;
    battery.soc = msg.get()->soc;
    battery.remaining_time_charge = msg.get()->remaining_time_charge;
    battery.remaining_time_run = msg.get()->remaining_time_run;
    battery.remaining_distance = msg.get()->remaining_distance;

    /* Update parameter l2 of charging likelihood function */
    //update_l2();
    
    //TODO(minor) very bad way to be sure to set maximum_travelling_distance...
    if(maximum_travelling_distance < msg.get()->remaining_distance)
        maximum_travelling_distance = msg.get()->remaining_distance;
}

void docking::cb_robot(const adhoc_communication::EmRobot::ConstPtr &msg)  // TODO(minor) better name and do better
{
    // TODO(minor) better update...
    for(int i=0; i<robots.size(); i++)
        if(robots[i].id == robot_id)
            if (msg.get()->state == exploring || msg.get()->state == fully_charged || msg.get()->state == moving_to_frontier ||
                msg.get()->state == leaving_ds)
                robots[i].state = active;
            else
                robots[i].state = idle;
        
    if (msg.get()->state != going_checking_vacancy) //TODO(minor) very bad... maybe in if(... == checking_vacancy) would be better...
        going_to_ds = false;

    if (msg.get()->state == in_queue)
    {
        ROS_DEBUG("Starting timer_restart_auction");
        /* Schedule next auction (a robot goes in queue only if it has lost an
         * auction started by itself? NOOOO!!! it could win is auction, then
         * immediately lose a following one that was sovrapposing with the one
         * started by it, so when both auctions are completed, the robot will
         * seem to be lost only an auction started by another robot!!!*/
        timer_restart_auction.stop(); //reduntant ?
        timer_restart_auction.setPeriod(ros::Duration(reauctioning_timeout), true);
        timer_restart_auction.start();
        
        //ROS_ERROR("Robot in queue!!!");
    }
    else if (msg.get()->state == going_charging)
    {
        ;  // ROS_ERROR("\n\t\e[1;34m Robo t going charging!!!\e[0m");
    }
    else if (msg.get()->state == charging)
    {
        ; //ROS_ERROR("\n\t\e[1;34mRechargin!!!\e[0m");
        need_to_charge = false;  // TODO(minor) useless

        set_target_ds_vacant(false);
    }
    else if (msg.get()->state == going_checking_vacancy)
    {
        ;  // ROS_ERROR("\n\t\e[1;34m going checking vacancy!!!\e[0m");
    }
    else if (msg.get()->state == checking_vacancy)
    {
        ;  // ROS_ERROR("\n\t\e[1;34m checking vacancy!!!\e[0m");
        adhoc_communication::SendEmDockingStation srv_msg;
        srv_msg.request.topic = "adhoc_communication/check_vacancy";
        srv_msg.request.dst_robot = group_name;
        srv_msg.request.docking_station.id = target_ds->id;  // target_ds, not best_ds!!!!!
        sc_send_docking_station.call(srv_msg);
    }
    else if (msg.get()->state == auctioning)
    {
        ROS_INFO("Robot needs to recharge");
        need_to_charge = true;
        if(!going_to_ds) //TODO(minor) very bad check... to be sure that only if the robot has not just won
                                  // another auction it will start its own (since maybe explorer is still not aware of this and so will communicate "auctioning" state...); do we have other similar problems?
            start_new_auction();  
                                  // TODO(minor) only if the robot has not been just interrupted from recharging
    }
    else if (msg.get()->state == auctioning_2) {
        if(ds_selection_policy == 2)
		{
			if(finished_bool) {
                ROS_ERROR("No more frontiers..."); //TODO(minor) probably this checks are reduntant with the ones of explorer
                std_msgs::Empty msg;
                pub_finish.publish(msg);
            } else {
		        ROS_INFO("Robot needs to recharge");
		        need_to_charge = true;
		        if(!going_to_ds) //TODO(minor) very bad check... to be sure that only if the robot has not just won
		                                  // another auction it will start its own (since maybe explorer is still not aware of this and so will communicate "auctioning" state...); do we have other similar problems?
		        {
		            ros::Duration(10).sleep();
		            start_new_auction();
		        }
			}
        } else {
            ROS_ERROR("DS graph cannot be navigated with this strategy...");
            ROS_INFO("DS graph cannot be navigated with this strategy...");
            std_msgs::Empty msg;
            pub_finish.publish(msg);
        }   
    }
    else if (msg.get()->state == fully_charged || msg.get()->state == leaving_ds)
    {
        set_target_ds_vacant(true);
    }
    else if (msg.get()->state == moving_to_frontier || msg.get()->state == going_in_queue ||
             msg.get()->state == exploring)
    {
        ;  // ROS_ERROR("\n\t\e[1;34midle!!!\e[0m");
    }
    else if (msg.get()->state == finished)
    {
        finalize();
    }
    else if (msg.get()->state == stuck)
    {
        finalize();
    }
    else if (msg.get()->state == dead)
    {
        finalize();
    }
    else if (msg.get()->state == moving_away_from_ds)
    {
        ;
    }
    else
    {
        ROS_FATAL("\n\t\e[1;34m none of the above!!!\e[0m");
        return;
    }

    robot_state = static_cast<state_t>(msg.get()->state);
}

void docking::cb_robots(const adhoc_communication::EmRobot::ConstPtr &msg)
{
    //ROS_DEBUG("Received information from robot %d", msg.get()->id);
    //ROS_ERROR("(%.1f, %.1f)", msg.get()->x, msg.get()->y);
    if (DEBUG) //TODO(minor) move away...
    {
        debug_timers[msg.get()->id].stop();
        debug_timers[msg.get()->id].setPeriod(ros::Duration(20), true);
        debug_timers[msg.get()->id].start();
        return;
    }

    /* Log information */ //TODO(minor) better log file...
    ros::Duration time = ros::Time::now() - time_start;
    fs3_csv.open(csv_file_3.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs3_csv << time.toSec() << "," << msg.get()->id << std::endl;
    fs3_csv.close();

    /* Check if robot is in list already */
    bool new_robot = true;
    for (int i = 0; i < robots.size(); ++i)
    {
        if (robots[i].id == msg.get()->id)
        {
            /* The robot is not new, update its information */

            new_robot = false;

            if (msg.get()->state == exploring || msg.get()->state == fully_charged ||
                msg.get()->state == moving_to_frontier)
                robots[i].state = active;
            else
                robots[i].state = idle;
            robots[i].x = msg.get()->x;
            robots[i].y = msg.get()->y;
            robots[i].selected_ds = msg.get()->selected_ds;

            break;
        }
    }

    /* If it is a new robot, add it */
    if (new_robot)
    {
        /* Store robot information */    
        robot_t new_robot;
        new_robot.id = msg.get()->id;
        if (msg.get()->state == exploring || msg.get()->state == fully_charged ||
            msg.get()->state == moving_to_frontier)
            new_robot.state = active;
        else
            new_robot.state = idle;
        new_robot.x = msg.get()->x;
        new_robot.y = msg.get()->y;
        new_robot.selected_ds = msg.get()->selected_ds;
        robots.push_back(new_robot);

        /* Recompute number of robots */
        int count = 0;
        for (int i = 0; i < robots.size(); i++)
            count++;
        num_robots = count;  // TODO(minor) also works for real exp?
    }

    /* Update parameter l1 of charging likelihood function */
    //update_l1();
}

void docking::cb_jobs(const adhoc_communication::ExpFrontier::ConstPtr &msg)
{
    /* NB. Here we use a surely inefficient approach: every time we clear the frontier vector and we store the whole set of unvisited frontiers from skratch. A better approach would be of course to just remove the visited frontiers and add the new discovered ones: notice however that a frontier could have been visited by another robot, so it is not easy to implement a service in explorer that returns the frontiers that have been visited since the last call of this service */ //TODO(minor) do it...

    /* Clear previous information about frontiers */
    //jobs.clear();

    /* Store new information about frontiers */
    /*
    for (int i = 0; i < msg.get()->frontier_element.size(); ++i)
    {
        adhoc_communication::ExpFrontierElement frontier_element = msg.get()->frontier_element.at(i);
        job_t job;
        job.id = frontier_element.id;
        job.x = frontier_element.x_coordinate;
        job.y = frontier_element.y_coordinate;
        jobs.push_back(job);
    }
    */
    
    //boost::mutex frontiers_mutex.lock();
    jobs = msg.get()->frontier_element;
    //frontiers_mutex.lock();
    
    if(jobs.size() > 0)
        no_jobs_received_yet = false;
    
    /* Update parameters l3 and l4 of charging likelihood function */
    //update_l3();
    //update_l4();
}


void docking::cb_docking_stations(const adhoc_communication::EmDockingStation::ConstPtr &msg)
{
    /* Check if DS is in list already */
    bool new_ds = true;
    for (int i = 0; i < ds.size(); ++i)
    {
        if (ds[i].id == msg.get()->id)
        {
            // coordinates don't match //TODO(minor) do it...
            double x, y;
            abs_to_rel(msg.get()->x, msg.get()->y, &x, &y);
            if (ds[i].x != x || ds[i].y != y)
                ROS_ERROR("Coordinates of docking station %d do not match: (%.2f, "
                          "%.2f) != (%.2f, %.2f)",
                          ds[i].id, ds[i].x, ds[i].y, x, y);

            /* DS is already in list, update its state; notice that, due to message loss, it could be possible that the robot has received a new state (vacant or occupied) for the DS that it is no different from the one already stored by the robot */

            new_ds = false;

            if (ds[i].vacant != msg.get()->vacant)
            {
                ds[i].vacant = msg.get()->vacant;
                ROS_INFO("ds%d is now %s", msg.get()->id,
                          (msg.get()->vacant ? "vacant" : "occupied"));
            }
            else
                ROS_DEBUG("State of ds%d is unchanged (%s)", msg.get()->id,
                          (msg.get()->vacant ? "vacant" : "occupied"));
            
            break;
        }
    }

    /* If the DS is new, add it */
    if (new_ds)
    {
        //TODO(minor) use a function...
        ds_t s;
        s.id = msg.get()->id;
        abs_to_rel(msg.get()->x, msg.get()->y, &s.x, &s.y);
        s.vacant = msg.get()->vacant;
        discovered_ds.push_back(s); //discovered, but not reachable, since i'm not sure if it is reachable for this robot...
        ROS_INFO("\e[1;34mNew docking station received: ds%d (%f, %f) \e[0m", s.id, s.x, s.y);

        /* Remove DS from the vector of undiscovered DSs */
        for (std::vector<ds_t>::iterator it = undiscovered_ds.begin(); it != undiscovered_ds.end(); it++)
            if ((*it).id == s.id)
            {
                undiscovered_ds.erase(it);
                break;
            }
    }

    /* Update parameter l1 of charging likelihood function */
    //update_l1();
}

void docking::cb_new_auction(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    ROS_INFO("Received bid for a new auction (%d)", msg.get()->auction);
    
    // TODO(minor) should do this ckeck also when the robot receive the result of an auction
    bool already_known_ds = false;
    for(std::vector<ds_t>::iterator it = discovered_ds.begin(); it != discovered_ds.end(); it++) //TODO(minor) create list discovered_ds + ds
        if(msg.get()->docking_station == (*it).id) {
            already_known_ds = true;
            break;
        }
    for(std::vector<ds_t>::iterator it = ds.begin(); it != ds.end(); it++)
        if(msg.get()->docking_station == (*it).id) {
            already_known_ds = true;
            break;
        }  
    if(!already_known_ds) {
        adhoc_communication::SendEmDockingStation srv_msg;
        srv_msg.request.topic = "adhoc_communication/resend_ds_list";
        srv_msg.request.dst_robot = group_name; //TODO(minor) use a string everywhere...
        sc_send_docking_station.call(srv_msg);
        return;
    }
        

    /*
    // set auction id
    if(id > auction_id) // it is a new action from another robot, respond
        auction_id = id;
    else if(id > 0){    // it is an old auction from another robot, ignore
        ROS_ERROR("Bad auction ID, it should be greater than %d!", auction_id);
        return false;
    }
    else                // it is a new auction by this robot
        ; //++auction_id;
    */

    /* Check if the robot has some interested in participating to this auction,
     * i.e., if the auctioned DS is the one
     * currently targetted by the robot */
    if (!optimal_ds_is_set() || msg.get()->docking_station != best_ds->id)
    {
        /* Robot received a bid of an auction whose auctioned docking station is not
         * the one the robot is interested in
         * at the moment, so it won't participate to the auction */
        ROS_INFO("Robot has no interested in participating to this auction");
    }
    else
    {
        if (get_llh() > msg.get()->bid)
        {
            /* The robot is interested in participating to the auction */
            ROS_INFO("The robot can place an higher bid than the one received, so it is going to participate to the auction");
            participating_to_auction++;

            /* Start timer to force the robot to consider the auction concluded after
             * some time, even in case it does not
             * receive the result of the auction.
             * To keep the timer active, we must store it on the heap; the timer is
             * just pushed back in vector 'timers',
             * which is cleared when no auction is pending (and so no timer is
             * active). A better management should be
             * possible, for instance using insert(), at(), etc., but in previous
             * tries the node always crashed when using
             * these functions... */
            ros::Timer timer = nh.createTimer(ros::Duration(auction_timeout + extra_time),
                                              &docking::end_auction_participation_timer_callback, this, true, false);
            timer.start();
            timers.push_back(timer);

            adhoc_communication::SendEmAuction srv;
            srv.request.dst_robot = group_name;
            srv.request.topic = "adhoc_communication/send_em_auction/reply";
            srv.request.auction.auction = msg.get()->auction;
            srv.request.auction.robot = robot_id;
            srv.request.auction.docking_station = best_ds->id;
            srv.request.auction.bid = get_llh();
            ROS_DEBUG("Calling service: %s", sc_send_auction.getService().c_str());
            sc_send_auction.call(srv);
        }
        else
        {
            ROS_INFO("The robot has no chance to win, so it won't place a bid for "
                     "this auction");
        }
    }
}

void docking::cb_auction_reply(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    if (!managing_auction)
    {
        ROS_INFO("Received a bid that has arrived too late, since the associated auction has already finished: ignore it");
        return;
    }
    
    if (auction_id != msg.get()->auction)
    {
        ROS_INFO("Received a bid that is not for the auction recently started by this robot: ignore it");
        return;
    }

    // TODO(minor) probably this is unnecessary, but jsut to be safe...
    for (std::vector<auction_bid_t>::iterator it = auction_bids.begin(); it != auction_bids.end(); it++)
        if ((*it).robot_id == msg.get()->robot)
        {
            ROS_INFO("Received a bid that was already received before: ignore it");
            return;
        }
        
    ROS_INFO("Received bid for auction %d", auction_id);
    ROS_DEBUG("Store bid (%f) of robot %d for this auction", msg.get()->bid, msg.get()->robot);
    
    auction_bid_t bid;
    bid.robot_id = msg.get()->robot;
    bid.bid = msg.get()->bid;
    auction_bids.push_back(bid);
}

void docking::end_auction_participation_timer_callback(const ros::TimerEvent &event)  // TODO(minor) what if instead the
                                                                                      // auction result are received
                                                                                      // after this timer? what if
                                                                                      // instead it is received a lot
                                                                                      // early?
{
    ROS_DEBUG("Force to consider auction concluded");
    participating_to_auction--;
}

void docking::timerCallback(const ros::TimerEvent &event)
{
    /* The auction is concluded; the robot that started it has to compute who is
     * the winner and must inform all the
     * other robots */
    ROS_INFO("Auction timeout: compute auction winner");

    // ??? //TODO(minor)
    managing_auction = false;

    /* Compute auction winner: loop through all the received bids and find the
     * robot that sent the highest one */
    int winner;
    float winner_bid = numeric_limits<int>::min();
    std::vector<auction_bid_t>::iterator it = auction_bids.begin();
    for (; it != auction_bids.end(); it++)
    {
        ROS_INFO("robot_%d placed %f", (*it).robot_id, (*it).bid);
        if ((*it).bid > winner_bid)
        {
            winner = (*it).robot_id;
            winner_bid = (*it).bid;
        }
    }

    ROS_DEBUG("The winner is robot_%d", winner);

    /* Delete stored bids to be able to start another auction in the future */
    auction_bids.clear();  // TODO(minor) inefficient!!!!

    /* Check if the robot that started the auction is the winner of */
    if (winner == robot_id)
    {
        /* The robot won its own auction */
        ROS_INFO("Winner of the auction");  // TODO(minor) specify which auction

        auction_winner = true;
        timer_restart_auction.stop();  // TODO(minor) i'm not sure that this follows the
                                       // idea in the paper... jsut put a
                                       // check in the timer callback...
    }
    else
    {
        /* The robot lost its own auction */
        ROS_INFO("Robot lost its own auction");
        auction_winner = false;
    }

    adhoc_communication::SendEmAuction srv_msg;
    srv_msg.request.topic = "adhoc_communication/auction_winner";
    srv_msg.request.dst_robot = group_name;
    srv_msg.request.auction.auction = auction_id;
    srv_msg.request.auction.robot = winner;

    srv_msg.request.auction.docking_station = best_ds->id;
    srv_msg.request.auction.bid = get_llh();

    // ROS_ERROR("\n\t\e[1;34m%s\e[0m", sc_send_auction.getService().c_str());
    sc_send_auction.call(srv_msg);

    /* Computation completed */
    participating_to_auction--;
}

void docking::cb_charging_completed(const std_msgs::Empty &msg)  // TODO(minor)
{
    ;
}

void docking::timer_callback_schedure_auction_restarting(const ros::TimerEvent &event)
{
    ROS_INFO("Periodic re-auctioning");
    
    //start auction only if no other one for teh same ds is on going: this is to avoid an "infinite loop" of auctions"
    if (participating_to_auction == 0)  // Notice that it is still possible that
                                        // two robots start an auction at the same
                                        // time...
        start_new_auction();
    else
    {
        update_state_required = true;
        started_own_auction = true;  // otherwise a robot could not start the auction
                                     // because the following if is true, then win
                                     // another robot auction and stop the time, then
                                     // lost another and not be reset in queue... //TODO(minor) not very clean...
        ROS_INFO("Robot is already participating to an auction: let's wait "
                  "instead of starting another one...");
        timer_restart_auction.stop(); //reduntant?
        timer_restart_auction.setPeriod(ros::Duration(reauctioning_timeout), true);
        timer_restart_auction.start();
    }
}

void docking::start_new_auction()
{
    if (!optimal_ds_is_set())
    {
        ROS_FATAL("The robot needs to recharge, but it doesn't know about any "
                  "existing DS!");  // TODO(minor) improve...
        return;
    }

    ROS_INFO("Starting new auction");
    
    /* Keep track of robot bid */
    auction_bid_t bid;
    bid.robot_id = robot_id;
    bid.bid = get_llh();
    auction_bids.push_back(bid);

    /* The robot is starting an auction */
    managing_auction = true;  // TODO(minor) reduntant w.r.t started_own_auction???
    started_own_auction = true;
    update_state_required = true;
    participating_to_auction++; //must be done after get_llh(), or the llh won't be computed correctly //TODO(minor) very bad in this way...

    /* Start auction timer to be notified of auction conclusion */
    timer_finish_auction.stop();
    timer_finish_auction.setPeriod(ros::Duration(auction_timeout), true);
    timer_finish_auction.start();

    /* Send broadcast message to inform all robots of the new auction */
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = "adhoc_communication/send_em_auction/new_auction";
    srv.request.dst_robot = group_name;
    srv.request.auction.auction = next_auction_id();
    srv.request.auction.robot = robot_id;
    srv.request.auction.docking_station = best_ds->id;
    srv.request.auction.bid = get_llh();
    ROS_DEBUG("Calling service: %s", sc_send_auction.getService().c_str());
    sc_send_auction.call(srv);

}

void docking::cb_translate(const adhoc_communication::EmDockingStation::ConstPtr &msg)  // TODO(minor)
{
    if (robot_prefix == "/robot_1")  // TODO(minor) useless ...
    {
        // ROS_ERROR("\n\t\e[1;34mReply to translation\e[0m");
        map_merger::TransformPoint point;

        point.request.point.src_robot = "robot_0";
        point.request.point.x = msg.get()->x;
        point.request.point.y = msg.get()->y;
        // if(sc_trasform.call(point)) ROS_ERROR("\e[1;34mTransformation
        // succeded:\n\t\tOriginal point: (%f,
        // %f)\n\t\tObtained point: (%f, %f)\e[0m",point.request.point.x,
        // point.request.point.y, point.response.point.x,
        // point.response.point.y);
    }
}

void docking::cb_auction_result(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    if (!optimal_ds_is_set())
    {
        ROS_ERROR("The robot does not know about any existing DS!");  // TODO(minor) it
                                                                      // means that
                                                                      // it missed
                                                                      // some
                                                                      // messages!!
        return;
    }

    // TODO(minor) the robot must check for its participation to the auction!!!

    // ROS_INFO("Received result of auction ... //TODO(minor) complete!!

    /* Check if the robot is interested in the docking station that was object of
     * the auction whose result has been just
     * received */
    if (msg.get()->docking_station == best_ds->id)  // TODO check if the robot already knows this DS! //TODO what if the robot changes best_ds between the start of this auction and the 
    {
        ROS_INFO("Received result of an auction to which the robot participated");  // TODO(minor)
                                                                                    // acutally
                                                                                    // maybe
                                                                                    // it
                                                                                    // didn't participate because its
                                                                                    // bid was lost OR because only now its best_ds is the one of the auction...

        /* Since the robot received the result of an auction to which it took part,
         * explorer node must be informed of a
         * possible change in the robot state */
        update_state_required = true; //TODO maybe it would be better when it receive the bid?

        /* Check if the robot is the winner of the auction */
        if (robot_id == msg.get()->robot)
        {
            /* The robot won the auction */
            ROS_INFO("Winner of the auction started by another robot");
            auction_winner = true;
            lost_other_robot_auction = false;  // TODO(minor) redundant?

            // TODO(minor) and what if best_ds is updated just a moment before starting the
            // auction??? it shoul be ok because (in the sense that it couldn't be the winner of this auction, since it didn't participate
            // the if above would be false
            timer_restart_auction.stop();  // TODO(minor)  i'm not sure that this follows the
                                           // idea in the paper... but probably
                                           // this is needed otherwise when i have many pendning auction and with a
                                           // timeout enough high, i could have an inifite loop of restarting
                                           // auctions... or I could but a control in the timer_callback!!!
        }
        else
        {
            /* The robot has lost an auction started by another robot (because the
             * robot that starts an auction does not
             * receive the result of that auction with this callback */
            // TODO(minor) should check if the robto took part to the auction
            ROS_INFO("Robot didn't win this auction started by another robot");
            auction_winner = false;
            lost_other_robot_auction = true;
        }
    }
    else
        ROS_DEBUG("Received result of an auction the robot was not interested in: "
                  "ignoring");
}

//DONE++
void docking::abs_to_rel(double absolute_x, double absolute_y, double *relative_x, double *relative_y)
{
    *relative_x = absolute_x - origin_absolute_x;
    *relative_y = absolute_y - origin_absolute_y;
}

//DONE++
void docking::rel_to_abs(double relative_x, double relative_y, double *absolute_x, double *absolute_y)
{
    *absolute_x = relative_x + origin_absolute_x;
    *absolute_y = relative_y + origin_absolute_y;
}

void docking::check_vacancy_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg)  // TODO(minor) explain
                                                                                                  // very well the
                                                                                                  // choices
{
    // ROS_INFO("Received request for vacancy check for "); //TODO(minor) complete

    /* If the request for vacancy check is not about the target DS of the robot,
     * for sure the robot is not occupying it
     */
    if (target_ds_is_set() && msg.get()->id == target_ds->id)

        /* If the robot is going to or already charging, or if it is going to check
         * already checking for vacancy, it is
         * (or may be, or will be) occupying the DS */
        if (robot_state == charging || robot_state == going_charging || robot_state == going_checking_vacancy ||
            robot_state == checking_vacancy || robot_state == fully_charged || robot_state == leaving_ds)
        {
            /* Print some debut text */
            if (robot_state == charging || robot_state == going_charging)
                ROS_INFO("\n\t\e[1;34mI'm using / going to use that DS!!!!\e[0m");
            else if (robot_state == going_checking_vacancy || robot_state == checking_vacancy)
                ROS_INFO("\n\t\e[1;34mI'm approachign that DS too!!!!\e[0m");
            else if (robot_state == fully_charged || robot_state == leaving_ds)
                ROS_INFO("\n\t\e[1;34mI'm leaving the DS, jsut wait a sec...\e[0m");

            /* Reply to the robot that asked for the check, telling it that the DS is
             * occupied */
            adhoc_communication::SendEmDockingStation srv_msg;
            srv_msg.request.topic = "explorer/adhoc_communication/reply_for_vacancy";
            srv_msg.request.dst_robot = group_name;
            srv_msg.request.docking_station.id = target_ds->id;
            sc_send_docking_station.call(srv_msg);
        }
        else
            ROS_DEBUG("target ds, but currently not used by the robot");
    else
        ROS_DEBUG("robot is not targetting that ds");
}

void docking::update_robot_state()  // TODO(minor) simplify
{
    ROS_INFO("update_robot_state");
    
    /*
     * Check if:
     * - there are no more pending auctions: this is to avoid to communicate
     *contradicting information to the explorer
     *   node about the next state of the robot, so it is better to wait that all
     *the auctions have been finished and
     *   then make a "global analisys" of the sistuation;
     * - an update of the state is required, i.e., that at least one auction was
     *recently performed and that the robot
     *   took part to it (i.e., that it placed a bid).
     *
     * Notice that it may happen that the energy_mgmt node thinks that an update
     *of the state is required, but maybe it
     * is not necessary: it is the explorer node that has to make some ckecks and,
     *only if necessary, update the state;
     * the energy_mgmt node just informs the explorer node that something has
     *recently happened.
     */
    if (update_state_required && participating_to_auction == 0)
    {
        /* An update of the robot state is required and it can be performed now */
        ROS_INFO("Sending information to explorer node about the result of recent "
                 "auctions");

        /* Create the empty message to be sent */
        std_msgs::Empty msg;

        /* If the robot is not the winner of the most recent auction, notify
         * explorer.
         * Notice that for sure it took part to at least one auction, or
         * 'update_state_required' would be false and we
         * wouldn't be in this then-branch */
        if (!auction_winner)
        {
            if (started_own_auction) {  // TODO(minor) should be better to use a variqable that
                                      // keep track of teh fact that the robot started
                                      // its own auction, since if !auction_winner is
                                      // true, it is already enough to know that the
                                      // robot needs to recharge (i.e., lost its own
                                      // auction)...
                                      /* Notify explorer node about the lost of an auction started by the
                                       * robot itself */
                pub_lost_own_auction.publish(msg);  
                ROS_INFO("pub_lost_own_auction");
            }

            /* If the robot has lost an auction that was not started by it, notify
             * explorer (because if the robot was
             * recharging, it has to leave the docking station) */
            else {
                pub_lost_other_robot_auction.publish(msg);
                ROS_INFO("pub_lost_other_robot_auction");
            }
        }

        /* Robot is the winner of at least one auction, notify explorer */
        else
        {
            going_to_ds = true; //TODO(minor) very bas way to solve the problem of a robot that has jsut won anotehr robot auction, but the explorer node is still not aware of this fact, and so will communicate to docking that the robot is now in "auctioning" state ...
            
            /* If the robot is already approaching a DS to recharge, if it is already
             *charging, etc., ignore the fact
             *that meanwhile it has won another auction.
             *
             * This check (i.e., the if condition) is necessary because, while moving
             *toward the DS, the robot could
             *have changed its currently optimal DS and it could have also taken part
             *to an auction for this new DS and
             *won it; without the check the explorer node would be notified for a
             *change of target DS even if the robot
             *is still approaching the old one, which could cause problems when the
             *robot state changes from
             *checking_vacancy to going_charging, since it would move to the new DS
             *without doing again the vacancy
             *check (this is because the move_robot() function in explorer node uses
             *the coordinates of the currently
             *set target DS when the robot wants to reach a DS). */
            if (robot_state != charging || robot_state != going_charging || robot_state != going_checking_vacancy ||
                robot_state != checking_vacancy)
            {
                //safety check
                if(!optimal_ds_is_set())
                    ROS_FATAL("THIS SHOULD NOT HAPPEN!");
                    
                /* Notify explorer node about the new target DS.
                 * Notice that it is important that target_ds is updated only here,
                 * because otherwise there could be problem when the robot communicates
                 * to the other robot that the DS that is currently targettting is now
                 * vacant/occupied. */
                target_ds = best_ds;
                geometry_msgs::PointStamped msg1;
                msg1.point.x = target_ds->x;
                msg1.point.y = target_ds->y;
                pub_new_target_ds.publish(msg1);

                /* Notify explorer node about the victory */
                pub_won_auction.publish(msg);  // TODO(minor) it is important that this is after the other pub!!!! can we do better?
                ROS_INFO("pub_won_auction");
            }
            else
                ROS_INFO("The robot has already won an auction: ignore the result of "
                          "the most recent auction");
        }

        /* Reset all the variables that are used to keep information about the
         * auctions results (i.e., about the next
         * robot state) */
        update_state_required = false;
        auction_winner = false;
        started_own_auction = false;
        timers.clear();  // TODO(minor) inefficient!!
    }
    else
    {
        /* Do nothing, just print some debug text */
        if (participating_to_auction > 0 && !update_state_required)
            ROS_DEBUG("There are still pending auctions, and moreover no update is "
                      "necessary for the moment");
        else if (!update_state_required)
            ROS_DEBUG("No state update required");
        else if (participating_to_auction > 0)
            ROS_DEBUG("There are still pending auctions, cannot update robot state");
        else {
            ROS_FATAL("ERROR: the number of pending auctions is negative: %d", participating_to_auction);
            ROS_DEBUG("ERROR: the number of pending auctions is negative: %d", participating_to_auction);
        }
    }
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

    /* Create file names */
    log_path = log_path.append("/");
    csv_file = log_path + std::string("optimal_ds.log"); //TODO(minor) .log or .csv?
    csv_file_2 = log_path + std::string("position.log");
    csv_file_3 = log_path + std::string("connectivity.log");
    info_file = log_path + std::string("metadata.csv");

    /* Create and initialize files */
    fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs_csv << "#time,optimal_ds" << std::endl;
    fs_csv.close();

    fs2_csv.open(csv_file_2.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs2_csv << "#time,x,y" << std::endl;
    fs2_csv.close();

    fs3_csv.open(csv_file_3.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs3_csv << "#time,sender_robot_id" << std::endl;
    fs3_csv.close();

    fs_info.open(info_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs_info << "#robot_id,num_robots,ds_selection_policy,starting_absolute_x,"
               "starting_absolute_y" << std::endl;
    fs_info << robot_id << "," << num_robots << "," << ds_selection_policy << "," << origin_absolute_x << ","
            << origin_absolute_y << std::endl;
    fs_info.close();
}

void docking::set_target_ds_vacant(bool vacant)
{
    adhoc_communication::SendEmDockingStation srv_msg;
    srv_msg.request.topic = "docking_stations";
    srv_msg.request.dst_robot = group_name;
    srv_msg.request.docking_station.id = target_ds->id;
    double x, y;
    rel_to_abs(target_ds->x, target_ds->y, &x, &y);
    srv_msg.request.docking_station.x = x;  // it is necessary to fill also this fields because when a Ds is
                                            // received, robots perform checks on the coordinates
    srv_msg.request.docking_station.y = y;
    srv_msg.request.docking_station.vacant = vacant;
    sc_send_docking_station.call(srv_msg);

    target_ds->vacant = vacant;

    ROS_INFO("Updated own information about ds%d state", target_ds->id);

    //update_l1();
}

void docking::compute_MST()  // TODO(minor) check all functions related to MST
{
    int V = ds_graph.size();
    //int N = ds.size();
    //int graph_2[V][V];
    int parent[V];   // Array to store constructed MST
    float key[V];      // Key values used to pick minimum weight edge in cut
    bool mstSet[V];  // To represent set of vertices not yet included in MST
    
    // Initialize all keys as INFINITE
    for (int i = 0; i < V; i++) {
        key[i] = INT_MAX;
        mstSet[i] = false;
    }
    
    /*
    for(int i=0; i<V; i++)
        for(int j=0; j<V; j++)
            graph_2[i][j] = 0;
    for(int i=0; i<V-1; i++)
        for(int j=0; j<V-1; j++)
            graph_2[i][j] = graph[i][j];
            
    graph_2[1][2] = 100;
    graph_2[2][1] = 100;
    graph_2[2][4] = 20;
    graph_2[4][2] = 20;
    
*/    
    
    for (int i = 0; i < V; i++)
        for (int j = 0; j < V; j++)
            ROS_ERROR("(%d, %d): %f", i, j, ds_graph[i][j]);


    // Always include first 1st vertex in MST.
    key[0] = 0;      // Make key 0 so that this vertex is picked as first vertex
    parent[0] = -1;  // First node is always root of MST

    // The MST will have V vertices
    for (int count = 0; count < V - 1; count++)
    {

        // Pick the minimum key vertex from the set of vertices
        // not yet included in MST
        //int u = minKey(key, mstSet, V);
        
        
        int min = INT_MAX, u = -1;
        /*
        for(int u=0; u < V; u++)
            if(mstSet[u] == false) {
                min_index = u;
                break;
            }
        */
        for (int v = 0; v < V; v++) {
            //ROS_ERROR("%d", mstSet[v]);        
            //ROS_ERROR("%d", key[v]);
            if (mstSet[v] == false && key[v] < min) {
                min = key[v];
                u = v;
            }
        }

        if(u < 0) {
            ROS_ERROR("DS graph has unconnected components! Not implemented for the moment...");
            return;
            
            /*
            for(u=0; u < V; u++)
            if(mstSet[u] == false)
                break;
            if(u >= V) {
                ROS_FATAL("Strange DS graph... abort computation of MST");
                return;
            }
            parent[u] = -1;
            */
        }
        

        // Add the picked vertex to the MST Set
        mstSet[u] = true;

        // Update key value and parent index of the adjacent vertices of
        // the picked vertex. Consider only those vertices which are not yet
        // included in MST
        for (int v = 0; v < V; v++)

            // graph[u][v] is non zero only for adjacent vertices of m
            // mstSet[v] is false for vertices not yet included in MST
            // Update the key only if graph[u][v] is smaller than key[v]
            if (ds_graph[u][v] > 0 && mstSet[v] == false && ds_graph[u][v] < key[v]) {
                parent[v] = u;
                key[v] = ds_graph[u][v];
            }
    }

    // print the constructed MST
    // printMST(parent, V, graph);
    //int ds_mst_2[V][V];
    for (int i = 0; i < V; i++)
        for (int j = 0; j < V; j++)
            ds_mst[i][j] = 0;

    // TODO(minor) does not work if a DS is not connected to any other DS
    for (int i = 1; i < V; i++)
    {
        if(parent[i] < 0) {
            ROS_ERROR("This node is not connected with other nodes");
            for(int j=0; j < V; j++) {
                ds_mst[i][j] = -10;
                ds_mst[j][i] = -10;
            }
            continue;
        }
        if(i >= ds_mst.size() || parent[i] >= V || parent[i] >= ds_mst[i].size() || parent[i] < 0) {
            ROS_FATAL("SIZE!!!");
            ROS_ERROR("%d", i);   
            ROS_ERROR("%d", parent[i]);
        }
        ds_mst[i][parent[i]] = 1;  // parent[i] is the node closest to node i
        ds_mst[parent[i]][i] = 1;
    }

    
    for (int i = 0; i < V; i++)
        for (int j = 0; j < V; j++)
            ; //ROS_ERROR("(%d, %d): %d ", i, j, ds_mst[i][j]);
    

    /*
    int k = 0;              // index of the closest recheable DS
    int target = 4;         // the id of the DS that we want to reach
    std::vector<int> path;  // sequence of nodes that from target leads to k;
    i.e., if the vector is traversed in the
                            // inverse order (from end to begin), it contains the
    path to go from k to target

    find_path(ds_mst, k, target, path);

    std::vector<int>::iterator it;
    for (it = path.begin(); it != path.end(); it++)
        ROS_ERROR("%d - ", *it);
        */
}


int docking::minKey(int key[], bool mstSet[], int V)
{
// A utility function to find the vertex with minimum key value, from
// the set of vertices not yet included in MST
    // Initialize min value
    int min = INT_MAX, min_index;
/*
    for(int u=0; u < V; u++)
        if(mstSet[u] == false) {
            min_index = u;
            break;
        }
        */
    for (int v = 0; v < V; v++) {
        if (mstSet[v] == false && key[v] < min) {
            min = key[v];
            min_index = v;
        }
    }

    return min_index;
}

int docking::printMST(int parent[], int n, std::vector<std::vector<int> > graph)
{
// A utility function to print the constructed MST stored in parent[]
    printf("Edge   Weight\n");
    for (int i = 1; i < graph.size(); i++)
        ROS_ERROR("%d - %d    %d \n", parent[i], i, graph[i][parent[i]]);
}

//DONE++
bool docking::find_path(std::vector<std::vector<int> > tree, int start, int end, std::vector<int> &path)
{
    ROS_INFO("Searching for path from ds%d to ds%d...", start, end);
    /* Temporary variable to store the path from 'end' to 'start' */
    std::vector<int> inverse_path;

    /* Call auxiliary function find_path_aux() to perform the search */
    bool path_found = find_path_aux(tree, start, end, inverse_path, start);

    /* Push also the starting node (as last node of the inverse path), since find_path_aux() does not push it automatically */
    inverse_path.push_back(start);

    /* If a path was found, reverse the constructed path to obtain the real path
     * that goes from 'start' to 'end' */
    if (path_found) {
        ROS_INFO("Path found!");
        for (int i = inverse_path.size() - 1; i >= 0; i--)
            path.push_back(inverse_path.at(i));
        for(int i=0; i < path.size(); i++)
            ROS_DEBUG("\t%d", path[i]);
    }
    else
        ROS_INFO("No path found");

    /* Tell the caller if a path was found or not */
    return path_found;
}

//DONE++
bool docking::find_path_aux(std::vector<std::vector<int> > tree, int start, int end, std::vector<int> &path,
                            int prev_node)
{
    /* Loop on all tree nodes */
    for (int j = 0; j < tree.size(); j++)

        /* Check if there is an edge from node 'start' to node 'j', and check if node 'j' is not the node just visited in the previous step of the recursive descending */
        if (tree[start][j] > 0 && j != prev_node)

            /* If 'j' is the searched node ('end'), or if from 'j' there is a path that leads to 'end', we have to store 'j' as one of the node that must be visited to reach 'end' ('j' could be 'end' itself, but of course this is not a problem, since we have to visit 'end' to reach it), and then return true to tell the caller that a path to 'end' was found */
            if (j == end || find_path_aux(tree, j, end, path, start))
            {
                path.push_back(j);
                return true;
            }

    /* If, even after considering all the nodes of the tree, we have not found a node that is connected to 'start' and from which it is possible to find a path leading to 'end', it means that no path from 'start' to 'end' exists: return false and, just for "cleaness", clear variable path */
    path.clear();
    return false;
}

bool docking::find_path_2(int start, int end, std::vector<int> &path) {
    compute_MST_2(start);
    return find_path(ds_mst, start, end, path);
}

void docking::compute_MST_2(int root)  // TODO(minor) check all functions related to MST
{
    int V = ds_graph.size();
    //int N = ds.size();
    //int graph_2[V][V];
    int parent[V];   // Array to store constructed MST
    float key[V];      // Key values used to pick minimum weight edge in cut
    bool mstSet[V];  // To represent set of vertices not yet included in MST
    
    // Initialize all keys as INFINITE
    for (int i = 0; i < V; i++) {
        key[i] = INT_MAX;
        mstSet[i] = false;
        parent[i] = -1;
    }
    
    /*
    for(int i=0; i<V; i++)
        for(int j=0; j<V; j++)
            graph_2[i][j] = 0;
    for(int i=0; i<V-1; i++)
        for(int j=0; j<V-1; j++)
            graph_2[i][j] = graph[i][j];
            
    graph_2[1][2] = 100;
    graph_2[2][1] = 100;
    graph_2[2][4] = 20;
    graph_2[4][2] = 20;
    
*/    
    
    for (int i = 0; i < V; i++)
        for (int j = 0; j < V; j++)
            ; //ROS_ERROR("(%d, %d): %f", i, j, ds_graph[i][j]);


    // 'root' is our root of the MST
    key[root] = 0;      // Make key 0 so that this vertex is picked as first vertex
    parent[root] = -1;  // First node is always root of MST
    
    //for(int i=0; i<V; i++)
    //    parent[i] = -1;
    //ROS_INFO("Computing...");
    // The MST will have V vertices
    bool can_continue = true;
    for (int count = 0; count < V - 1 && can_continue; count++)
    {

        // Pick the minimum key vertex from the set of vertices
        // not yet included in MST
        //int u = minKey(key, mstSet, V);
        
        
        int min = INT_MAX, u = -1;
        /*
        for(int u=0; u < V; u++)
            if(mstSet[u] == false) {
                min_index = u;
                break;
            }
        */
        for (int v = 0; v < V; v++) {
            //ROS_ERROR("%d", mstSet[v]);        
            //ROS_ERROR("%d", key[v]);
            if (mstSet[v] == false && key[v] < min) {
                min = key[v];
                u = v;
            }
        }
        //ROS_INFO("u: %d", u);
        if(u < 0) {
            //ROS_INFO("DS graph has unconnected components!");
            can_continue = false;
            continue;
            
            /*
            for(u=0; u < V; u++)
            if(mstSet[u] == false)
                break;
            if(u >= V) {
                ROS_FATAL("Strange DS graph... abort computation of MST");
                return;
            }
            parent[u] = -1;
            */
        }

        // Add the picked vertex to the MST Set
        mstSet[u] = true;
        
        // Update key value and parent index of the adjacent vertices of
        // the picked vertex. Consider only those vertices which are not yet
        // included in MST
        for (int v = 0; v < V; v++)

            // graph[u][v] is non zero only for adjacent vertices of m
            // mstSet[v] is false for vertices not yet included in MST
            // Update the key only if graph[u][v] is smaller than key[v]
            if (ds_graph[u][v] > 0 && mstSet[v] == false && (key[u] + ds_graph[u][v]) < key[v]) {
                parent[v] = u;
                key[v] = key[u] + ds_graph[u][v];
                //ROS_ERROR("%d has %d as parent; cost of %d is %f", v, parent[v], v, key[v]); 
            }
    }

    // print the constructed MST
    // printMST(parent, V, graph);
    //int ds_mst_2[V][V];
    for (int i = 0; i < V; i++)
        for (int j = 0; j < V; j++)
            ds_mst[i][j] = 0;

    // TODO(minor) does not work if a DS is not connected to any other DS
    for (int i = 0; i < V; i++)
    {
        if(parent[i] < 0) {
            //ROS_INFO("node %d is not connected with other nodes", i);
            /*
            for(int j=0; j < V; j++) {
                ds_mst[i][j] = -10;
                ds_mst[j][i] = -10;
            }
            */
            continue;
        }
        if(i >= ds_mst.size() || parent[i] >= V || parent[i] >= ds_mst[i].size()) {
            ROS_FATAL("SIZE!!!");
            ROS_ERROR("%d", i);   
            ROS_ERROR("%d", parent[i]);
        }
        //ROS_INFO("%d has %d as parent", i, parent[i]); 
        ds_mst[i][parent[i]] = 1;  // parent[i] is the node closest to node i
        ds_mst[parent[i]][i] = 1;
    }

    
    for (int i = 0; i < V; i++)
        for (int j = 0; j < V; j++)
            ; //ROS_INFO("(%d, %d): %d ", i, j, ds_mst[i][j]);
    

    /*
    int k = 0;              // index of the closest recheable DS
    int target = 4;         // the id of the DS that we want to reach
    std::vector<int> path;  // sequence of nodes that from target leads to k;
    i.e., if the vector is traversed in the
                            // inverse order (from end to begin), it contains the
    path to go from k to target

    find_path(ds_mst, k, target, path);

    std::vector<int>::iterator it;
    for (it = path.begin(); it != path.end(); it++)
        ROS_ERROR("%d - ", *it);
        */
}


//DONE+
void docking::compute_closest_ds()
{
    //ROS_DEBUG("Using 'closest' strategy to compute optimal DS");
    double min_dist = numeric_limits<int>::max();
    for (std::vector<ds_t>::iterator it = ds.begin(); it != ds.end(); it++)
    {
        double dist = distance_from_robot((*it).x, (*it).y);
        //ROS_ERROR("ds%d: %f, %f", it->id, dist, distance_from_robot((*it).x, (*it).y, true));
        if (dist < 0)
            continue; //TODO(minor) sure?
        if (dist < min_dist)
        {
            min_dist = dist;
            set_optimal_ds_given_id(it->id);
        }
    }
}

//DONE+
void docking::discover_docking_stations() //TODO(minor) comments
{
    ROS_INFO("discover_docking_stations");
    
    /* Check if there are DSs that can be considered discovered (a DS is considered discovered if the euclidean distance between it and the robot is less than the range of the "simulated" fiducial signal emmitted by the DS */
    for (std::vector<ds_t>::iterator it = undiscovered_ds.begin(); it != undiscovered_ds.end(); it++)
    {
        double dist = distance_from_robot((*it).x, (*it).y, true);
        if (dist > 0 && dist < fiducial_signal_range)
        {
            /* Store new DS in the vector of known DSs, and remove it from the vector
             * of undiscovered DSs */
            ROS_INFO("Found new DS ds%d at (%f, %f). Currently, no path for this DS is known...", (*it).id, (*it).x,
                     (*it).y);  // TODO(minor) index make sense only in simulation (?, not sure...)
            discovered_ds.push_back(*it); //TODO(minor) change vector name, from discovered_ds to unreachable_dss
            undiscovered_ds.erase(it);

            /* Inform other robots about the "new" DS */
            adhoc_communication::SendEmDockingStation send_ds_srv_msg;
            send_ds_srv_msg.request.topic = "docking_stations";
            send_ds_srv_msg.request.docking_station.id = (*it).id;
            double x, y;
            rel_to_abs((*it).x, (*it).y, &x, &y);
            send_ds_srv_msg.request.docking_station.x = x;
            send_ds_srv_msg.request.docking_station.y = y;
            send_ds_srv_msg.request.docking_station.vacant = true;  // TODO(minor) sure???

            /* Since an element from 'undiscovered_ds' was removed, we have to
             * decrease the iterator by one to compensate the future increment of 
             * the for loop, since, after the deletion of the element, all the elements in the vector whose position was after the one of the removed element are shifted by one position, and so we are already pointing to the next element, even without the future increment of the for loop */
            it--;
        }
    }
}

void docking::join_all_multicast_groups() { //TODO(minor) maybe it's enough to join just mc_robot_0
    //ROS_ERROR("Joining");
    ros::ServiceClient sc_join = nh.serviceClient<adhoc_communication::ChangeMCMembership>("adhoc_communication/join_mc_group"); //TODO(minor) move above...
    adhoc_communication::ChangeMCMembership msg;
    msg.request.action=true; //true means that I want to join the group, false that I want to leave it
    //ros::service::waitForService("adhoc_communication/join_mc_group");
    for(int i=0; i < num_robots; i++) {
        std::string group_name = "mc_robot_" + SSTR(i);
        msg.request.group_name=group_name;
        if(sc_join.call(msg))
            ROS_DEBUG("(Re)joined group %s", group_name.c_str());
        else
            ROS_INFO("Unable to (re)join group %s at the moment...", group_name.c_str());
    }
}

void docking::send_robot()
{    
    ROS_INFO("send_robot");
    
    if (robot_id == 0 && DEBUG)
    {
        // ROS_ERROR("%f", distance(robot.x, robot.y, -0.5, -1));
        //ROS_ERROR("(%f, %f)", robot.x, robot.y);
        //ROS_ERROR("%f", distance(robot.x, robot.y, 0, 0));
        ; //ROS_ERROR("%f", distance(robot.x, robot.y, 0, 0, true));
    }
    adhoc_communication::SendEmRobot robot_msg;
    robot_msg.request.dst_robot = group_name;
    robot_msg.request.topic = "robots";
    robot_msg.request.robot.id = robot_id;
    robot_msg.request.robot.x = robot->x;
    robot_msg.request.robot.y = robot->y;
    robot_msg.request.robot.state = robot->state;
    robot_msg.request.robot.state = active;
    if (optimal_ds_is_set())
        robot_msg.request.robot.selected_ds = best_ds->id;
    else
        robot_msg.request.robot.selected_ds = -1;
    sc_send_robot.call(robot_msg);

    ros::Duration time = ros::Time::now() - time_start;
    fs2_csv.open(csv_file_2.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs2_csv << time.toSec() << "," << robot->x << "," << robot->y << std::endl;
    fs2_csv.close();
}

void docking::send_fake_msg()
{
    adhoc_communication::SendEmRobot robot_msg;
    robot_msg.request.topic = "robots";
    robot_msg.request.dst_robot = group_name;
    robot_msg.request.robot.id = robot_id;
    robot_msg.request.robot.x = robot_id;
    robot_msg.request.robot.y = robot_id;
    robot_msg.request.robot.state = robot->state;
    robot_msg.request.robot.selected_ds = robot_id;
    sc_send_robot.call(robot_msg);
}

void docking::debug_timer_callback_0(const ros::TimerEvent &event)
{
    ROS_ERROR("No information received by robot 0 for a certain amount of time!!!");
}

void docking::debug_timer_callback_1(const ros::TimerEvent &event)
{
    ROS_ERROR("No information received by robot 1 for a certain amount of time!!!");
}

void docking::debug_timer_callback_2(const ros::TimerEvent &event)
{
    ROS_ERROR("No information received by robot 2 for a certain amount of time!!!");
}

//DONE+
void docking::next_ds_callback(const std_msgs::Empty &msg)
{
    ROS_INFO("Select next DS on the path in the DS graph to reach the final DS with EOs");
    if (index_of_ds_in_path < path.size() - 1)
    {
        index_of_ds_in_path++;
        for (int i = 0; i < ds.size(); i++)
            if (path[index_of_ds_in_path] == ds[i].id)
            {   
                ROS_INFO("Next DS on path: ds%d", ds[i].id);
                target_ds = &ds[i];  // TODO(minor) probably ok...
                set_optimal_ds_given_index(i);    // TODO(minor) VERY BAD!!!!
                break;
            }
    }
    else {
        ROS_INFO("We have reached the final DS and charged at it");
        moving_along_path = false;
    }
}

void docking::check_reachable_ds()
{
    ROS_INFO("check_reachable_ds");
    
    bool new_ds_discovered = false;
    for (std::vector<ds_t>::iterator it = discovered_ds.begin(); it != discovered_ds.end(); )
    {
        /* If the DS is inside a fiducial laser range, it can be considered
         * discovered */
        bool reachable;
        explorer::DistanceFromRobot srv_msg;
        srv_msg.request.x = (*it).x;
        srv_msg.request.y = (*it).y;
        
        //ros::service::waitForService("explorer/reachable_target");
        for(int i = 0; i < 10 && !sc_reachable_target; i++) {
            ROS_FATAL("NO MORE CONNECTION!");
            ros::Duration(1).sleep();
            sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target", true);
        }
        if (sc_reachable_target.call(srv_msg))
            reachable = srv_msg.response.reachable;
        else
        {
            ROS_ERROR("Unable to check if ds%d is reachable, retrying later...", (*it).id);
            return;
        }

        if (reachable)
        {
            ROS_INFO("ds%d is now reachable", (*it).id);
            
            adhoc_communication::EmDockingStation new_ds_msg;
            new_ds_msg.id = it->id;
            new_ds_msg.x = it->x;
            new_ds_msg.y = it->y;
            //ROS_ERROR("publishing on %s", pub_new_ds_on_graph.getTopic().c_str());
            //TODO publishing here just to avoid to publish the message too early, but of course this is not a good place, and it is not necessary to publish the message every time
            //std_msgs::Int32 ds_count_msg;
            //ds_count_msg.data = num_ds;
            //ROS_ERROR("publishing on topic %s", pub_ds_count.getTopic().c_str());
            //pub_ds_count.publish(ds_count_msg);
            new_ds_msg.total_number_of_ds = num_ds;
            pub_new_ds_on_graph.publish(new_ds_msg);
            
            new_ds_discovered = true;
            ds_t new_ds; 
            new_ds.id = it->id;
            new_ds.x = it->x;
            new_ds.y = it->y;
            new_ds.vacant = it->vacant;
            ds.push_back(new_ds); //otherwise with ds.push_back(*it) valgrind complains...
            int id1 = ds[ds.size()-1].id;
            if(id1 != it->id)
                ROS_ERROR("error");
            int id2 = it->id;
            discovered_ds.erase(it);
            
            it = discovered_ds.begin(); //since it seems that the pointer is invalidated after the erase, so better restart the check... (http://www.cplusplus.com/reference/vector/vector/erase/)
            

            
            if(id1 != id2)
                ROS_ERROR("ERROR");
            
        }
        else {
            ROS_INFO("UNREACHABLE!!!");
            it++;   
        }
    }

    return;
    if (new_ds_discovered || recompute_graph)
    {
        // construct ds graph //TODO(minor) construct graph only when a new DS is found
        for (int i = 0; i < ds.size(); i++) {
            for (int j = 0; j < ds.size(); j++) {
                //safety checks   
                if( ds[i].id >= ds_graph.size() || ds[j].id >= (ds_graph[ds[i].id]).size() || ds[i].id < 0 || ds[j].id < 0)
                    ROS_FATAL("SIZE ERROR!!! WILL CAUSE SEGMENTATION FAULT!!!");          
                if (i == j)
                    ds_graph[ds[i].id][ds[j].id] = 0; //TODO(minor) maybe redundant...
                else
                {
                    double dist;
                    dist = distance(ds.at(i).x, ds.at(i).y, ds.at(j).x, ds.at(j).y);
                    if (dist < 0)
                    {
                        recompute_graph = true;
                        return;
                    }
                    //ROS_ERROR("%f", dist);
                    //ROS_ERROR("%f", conservative_maximum_distance_one_way());
                    if(conservative_maximum_distance_one_way() <= 0){
                        ROS_ERROR("Cannot compute DS graph at the moment...");
                        recompute_graph = true;
                        return;
                    }
                    if (dist < conservative_maximum_distance_one_way())
                    {
                        ds_graph[ds[i].id][ds[j].id] = dist;
                        ds_graph[ds[j].id][ds[i].id] = dist;
                    }
                    else
                    {
                        ds_graph[ds[i].id][ds[j].id] = 0;
                        ds_graph[ds[j].id][ds[i].id] = 0;
                    }
                }
            }
        }
        recompute_graph = false;

        // construct MST starting from ds graph
        //compute_MST_2(1);
        
        //timer_recompute_ds_graph = nh.createTimer(ros::Duration(60), &docking::timer_callback_recompute_ds_graph, this, true, false); //TODO(minor) timeout
    }
    
    
    
}

void docking::finalize() //TODO(minor) do better
{
    ROS_INFO("Close log files");
    
    ros::Duration time = ros::Time::now() - time_start;

    fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out); //TODO(minor) avoid continusouly open-close...
    fs_csv << time.toSec() << ","
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
    
    finished_bool = true;
}

//DONE++
int docking::next_auction_id()
{
    ROS_DEBUG("Compute next auction ID");
    
    /* Increase local auction ID, and then return the global one */
    local_auction_id++;
    auction_id = local_auction_id * pow(10, (ceil(log10(num_robots)))) + robot_id;
    return auction_id;
    
    /*
    Examples:
    23 * pow(10, (ceil(log10(15)))) + 11 = 23 * 10^2 + 11 = 2300 + 11 = 2311;
    23 * pow(10, (ceil(log10(10)))) +  9 = 23 * 10^1 +  9 =  230 +  9 =  239;
    */
}

void docking::spin()
{
    ROS_INFO("Start thread to receive callbacks"); //TODO(minor) remove spin in other points of the code
    while (ros::ok)
    {
        ros::Duration(0.1);
        ros::spinOnce();
    }
}

//DONE+
void docking::update_robot_position()
{   
    ROS_INFO("update_robot_position");
    
    /* Get current robot position (once the service required to do that is ready) by calling explorer's service */
    //ros::service::waitForService("explorer/robot_pose");
    fake_network::RobotPosition srv_msg;
    for(int i = 0; i < 10 && !sc_robot_pose; i++) {
        ROS_FATAL("NO MORE CONNECTION!");
        ros::Duration(1).sleep();
        sc_robot_pose = nh.serviceClient<fake_network::RobotPosition>(my_prefix + "explorer/robot_pose", true);   
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
    /*
    ros::service::waitForService("explorer/robot_pose");  // TODO(minor) string name
    fake_network::RobotPosition srv_msg;
    if (sc_robot_pose.call(srv_msg))
    {
        robot->x = srv_msg.response.x;
        robot->y = srv_msg.response.y;
        ROS_DEBUG("Robot position: (%f, %f)", robot->x, robot->y); //TODO(minor) in the relative fixed reference system
    }
    else
        ROS_ERROR("Call to service %s failed; not possible to update robot position for the moment",
                  sc_robot_pose.getService().c_str());
    */
    
    adhoc_communication::EmRobot msg;
    msg.id = robot_id;
    double x, y;
    //ROS_ERROR("(%f, %f)", robot->x, robot->y);
    rel_to_abs(robot->x, robot->y, &x, &y);
    //ROS_ERROR("(%f, %f)", x, y);
    msg.x = x;
    msg.y = y;
    pub_robot_absolute_position.publish(msg);
}

void docking::resend_ds_list_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg) { //TODO(minor) do better...
    ROS_INFO("Sending complete list of discovered docking stations");
    for(std::vector<ds_t>::iterator it = discovered_ds.begin(); it != discovered_ds.end(); it++) {
        adhoc_communication::SendEmDockingStation srv_msg;
        srv_msg.request.topic = "docking_stations";
        srv_msg.request.docking_station.id = it->id;
        srv_msg.request.dst_robot = group_name;
        double x, y;
        rel_to_abs(it->x, it->y, &x, &y);
        srv_msg.request.docking_station.x = x;
        srv_msg.request.docking_station.y = y;
        srv_msg.request.docking_station.vacant = it->vacant; //TODO(minor) notice that the robot could receive contrasting information!!!
        sc_send_docking_station.call(srv_msg);
    }
        for(std::vector<ds_t>::iterator it = ds.begin(); it != ds.end(); it++) {
        adhoc_communication::SendEmDockingStation srv_msg;
        srv_msg.request.topic = "docking_stations";
        srv_msg.request.docking_station.id = it->id;
        srv_msg.request.dst_robot = group_name;
        double x, y;
        rel_to_abs(it->x, it->y, &x, &y);
        srv_msg.request.docking_station.x = x;
        srv_msg.request.docking_station.y = y;
        srv_msg.request.docking_station.vacant = it->vacant; //TODO(minor) notice that the robot could receive contrasting information!!!
        sc_send_docking_station.call(srv_msg);
    }
        
}

//DONE++
float docking::conservative_remaining_distance_with_return() {
    return (battery.remaining_distance / (double)2) * safety_coeff;
}

//DONE++
float docking::conservative_maximum_distance_with_return() {
    //ROS_ERROR("%f", maximum_travelling_distance);
    return (maximum_travelling_distance / (double)2 ) * safety_coeff;
}

//DONE++
float docking::conservative_remaining_distance_one_way() {
    return battery.remaining_distance * safety_coeff;
}

//DONE++
float docking::conservative_maximum_distance_one_way() {
    //ROS_ERROR("%f", maximum_travelling_distance);
    return maximum_travelling_distance * safety_coeff;
}

void docking::full_battery_info_callback(const energy_mgmt::battery_state::ConstPtr &msg) {
    ROS_INFO("Received!"); //TODO(minor) it seems that this message is not even sent... i think that it's because the instance of battery is created before the isntance of docking, and so when the message is published, there is still no subscriber to receive it... the best way should be to create a una tantum function for abttery, that is called after that both bat and doc objects are created...
    maximum_travelling_distance = msg.get()->remaining_distance;
}

bool docking::optimal_ds_is_set() {
    return optimal_ds_set;
}

bool docking::target_ds_is_set() {
    if(target_ds->id == -1) //TODO(minor) this check is very bad....
        return false;
    return true;
}

void docking::wait_battery_info() {
    while(maximum_travelling_distance <= 0) {
        ROS_ERROR("Waiting battery information...");
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
}

void docking::timer_callback_recompute_ds_graph(const ros::TimerEvent &event) {
    ROS_INFO("Periodic recomputation of DS graph");
    return;
    for (int i = 0; i < ds.size(); i++) {
        for (int j = 0; j < ds.size(); j++) {
            //safety checks   
            if( ds[i].id >= ds_graph.size() || ds[j].id >= (ds_graph[ds[i].id]).size() || ds[i].id < 0 || ds[j].id < 0)
                ROS_FATAL("SIZE ERROR!!! WILL CAUSE SEGMENTATION FAULT!!!");          
            if (i == j)
                ds_graph[ds[i].id][ds[j].id] = 0; //TODO(minor) maybe redundant...
            else
            {
                double dist;
                dist = distance(ds.at(i).x, ds.at(i).y, ds.at(j).x, ds.at(j).y);
                if (dist < 0)
                {
                    recompute_graph = true;
                    return;
                }
                //ROS_ERROR("%f", dist);
                //ROS_ERROR("%f", conservative_maximum_distance_one_way());
                if(conservative_maximum_distance_one_way() <= 0){
                    ROS_ERROR("Cannot compute DS graph at the moment...");
                    return;
                }
                if (dist < conservative_maximum_distance_one_way())
                {
                    ds_graph[ds[i].id][ds[j].id] = dist;
                    ds_graph[ds[j].id][ds[i].id] = dist;
                }
                else
                {
                    ds_graph[ds[i].id][ds[j].id] = 0;
                    ds_graph[ds[j].id][ds[i].id] = 0;
                }
            }
        }
    }
    //compute_MST();
}

void docking::timer_callback_join_all_mc_groups(const ros::TimerEvent &event) {
    //join_all_multicast_groups();
}

void docking::start_join_timer() {
    //ROS_ERROR("Set timer");
    join_timer = nh.createTimer(ros::Duration(30), &docking::timer_callback_join_all_mc_groups, this, false, true);
}

bool docking::set_optimal_ds_given_id(int id) {
    for(int i =0; i < ds.size(); i++)
        if(ds[i].id == id) {
            best_ds = &ds[i];
            optimal_ds_set = true;
            return true;
        }
    return false;
}

bool docking::set_optimal_ds_given_index(int index) {
    if(index < 0 || index >= ds.size())
        return false;
    best_ds = &ds[index];
    optimal_ds_set = true;
    return true;
}

void docking::test_2(const std_msgs::Empty &msg) {
    ROS_ERROR("Test 2");
}

bool docking::distance_robot_frontier_on_graph_callback(explorer::Distance::Request &req, explorer::Distance::Response &res) {
    //ROS_ERROR("called!");
    return true;
    
    int index_closest_ds_to_frontier, index_closest_ds_to_robot;
    double x1, y1, robot_x, robot_y; //TODO
    double min_dist_frontier = numeric_limits<int>::max(), min_dist_robot = numeric_limits<int>::max();
    for(int i; i < ds.size(); i++) {
        double dist = distance(ds[i].x, ds[i].y, x1, y1, true);
        if( dist < min_dist_frontier ) {
            min_dist_frontier = dist;
            index_closest_ds_to_frontier = i;
        }
        dist = distance(ds[i].x, ds[i].y, robot_x, robot_y, true);
        if(dist < min_dist_robot)
        {
            min_dist_robot = dist;
            index_closest_ds_to_robot = i;
        }
    }
    
    //compute_path(index_closest_ds_to_robot, index_closest_ds_to_robot);
    
    return true;
}
