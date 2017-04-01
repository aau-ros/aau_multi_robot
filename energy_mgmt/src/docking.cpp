#include <docking.h>

using namespace std;

docking::docking()
{
    /* Read parameters */
    nh.param("energy_mgmt/num_robots", num_robots, 1);
    nh.param("energy_mgmt/w1", w1, 0.25);
    nh.param("energy_mgmt/w2", w2, 0.25);
    nh.param("energy_mgmt/w3", w3, 0.25);
    nh.param("energy_mgmt/w4", w4, 0.25);
    nh.param("energy_mgmt/x", origin_absolute_x, 0.0);
    nh.param("energy_mgmt/y", origin_absolute_y, 0.0);
    nh.param("energy_mgmt/distance_close", distance_close, 8.0);
    nh.param<string>("energy_mgmt/move_base_frame", move_base_frame, "map");
    nh.param<string>("energy_mgmt/robot_prefix", robot_prefix, "");

    // initialize auction id
    auction_id = 0;

    // initialize navigation function

    // nav.initialize("navigation_path", costmap);

    // initialize robot name

    /*
    if(robot_prefix.empty()){ // hardware platform
        //TODO
        char hostname[1024];
        hostname[1023] = '\0';
        gethostname(hostname, 1023);
        robot_name = string(hostname);
        robot_id = 0;
        exit(0);
        my_prefix = "robot_" + SSTR(robot_id) + "/";
    }
    else
    */
    // TODO
    {  // simulations
        robot_name = robot_prefix;

        // This line makes the program crash if the robot_prefix has not the required format!!!!
        // robot_id = atoi(robot_prefix.substr(7,1).c_str());

        if (robot_prefix == "/robot_0")
            robot_id = 0;
        else if (robot_prefix == "/robot_1")
            robot_id = 1;
        else if (robot_prefix == "/robot_2")
            robot_id = 2;
        else
            ROS_FATAL("\e[1;34mRobot prefix is WRONG!!!!");
        // ROS_ERROR("\e[1;34mRobot prefix: %s, robot id: %d\e[0m", robot_prefix.c_str(), robot_id);
        
        my_prefix = "";
        
    }

    // initialize robot struct
    robot_t robot;
    robot.id = robot_id;
    robot.state = active;
    robots.push_back(robot);

    // initialize service clients
    //TODO names (robot_0 end dockign)
    sc_send_auction = nh.serviceClient<adhoc_communication::SendEmAuction>(my_prefix + "adhoc_communication/send_em_auction");

    sc_send_docking_station = nh.serviceClient<adhoc_communication::SendEmDockingStation>(my_prefix + "adhoc_communication/send_em_docking_station");

    sc_send_robot = nh.serviceClient<adhoc_communication::SendEmRobot>(my_prefix + "adhoc_communication/send_em_robot");


    // subscribe to topics
    sub_battery = nh.subscribe(my_prefix + "battery_state", 1, &docking::cb_battery, this);
    sub_robots = nh.subscribe(my_prefix + "robots", 100, &docking::cb_robots, this);
    sub_jobs = nh.subscribe(my_prefix + "frontiers", 10000, &docking::cb_jobs, this);
    sub_robot = nh.subscribe(my_prefix + "explorer/robot", 100, &docking::cb_robot, this);
    sub_docking_stations =
        nh.subscribe(my_prefix + "adhoc_communication/send_em_docking_station", 100, &docking::cb_docking_stations, this);
    sub_auction_starting =
        nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/new_auction", 100, &docking::cb_auction, this);
    sub_auction_reply =
        nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/reply", 100, &docking::cb_auction_reply, this);
    sub_auction_winner_adhoc =
        nh.subscribe(my_prefix + "adhoc_communication/auction_winner", 100, &docking::cb_auction_result, this);

    // F
    pub_ds = nh.advertise<std_msgs::Empty>("docking_station_detected", 1);
    pub_new_target_ds = nh.advertise<geometry_msgs::PointStamped>("new_target_docking_station_selected", 1); //TODO bad name
    
    sub_robot_position = nh.subscribe("goalPoint/goalPoint", 1, &docking::robot_position_callback, this); //TODO
    pub_adhoc_new_best_ds =
        nh.advertise<adhoc_communication::EmDockingStation>("adhoc_new_best_docking_station_selected", 1);
    sub_adhoc_new_best_ds = nh.subscribe("adhoc_new_best_docking_station_selected", 1, &docking::adhoc_ds, this);

    sub_all_points = nh.subscribe("all_positions", 1, &docking::points, this);

    sc_trasform = nh.serviceClient<map_merger::TransformPoint>("map_merger/transformPoint");

    sub_charging_completed = nh.subscribe("charging_completed", 100, &docking::cb_charging_completed, this);
    sub_vacant_docking_station = nh.subscribe("vacant_docking_station", 100, &docking::cb_vacant_docking_station, this);

    sub_need_charging = nh.subscribe("need_charging", 100, &docking::cb_need_charging, this);

    sub_translate = nh.subscribe("translate", 100, &docking::cb_translate, this);
    

    timer_finish_auction = nh.createTimer(ros::Duration(AUCTION_TIMEOUT), &docking::timerCallback, this, true, false);
    timer_restart_auction =
        nh.createTimer(ros::Duration(10), &docking::timer_callback_schedure_auction_restarting, this, true, false);

    sub_check_vacancy = nh.subscribe("explorer/check_vacancy", 1, &docking::check_vacancy_callback, this);

    sub_ask_for_vacancy =
        nh.subscribe("adhoc_communication/ask_for_vacancy", 1, &docking::ask_for_vacancy_callback, this);
    
    sub_ds_state_update = nh.subscribe("adhoc_communication/ds_state_update", 1, &docking::ds_state_update_callback, this);

    pub_lost_own_auction = nh.advertise<std_msgs::Empty>("explorer/lost_own_auction", 1);
    pub_won_auction = nh.advertise<std_msgs::Empty>("explorer/won_auction", 1);
    pub_lost_other_robot_auction = nh.advertise<std_msgs::Empty>("explorer/lost_other_robot_auction", 1);

    sub_abort_charging = nh.subscribe("explorer/abort_charging", 1, &docking::abort_charging_callback, this);

    sub_robot_pose = nh.subscribe("amcl_pose", 1, &docking::robot_pose_callback, this); //TODO not used
    sc_robot_pose = nh.serviceClient<explorer::RobotPosition>("explorer/robot_pose");
    sc_distance_from_robot = nh.serviceClient<explorer::DistanceFromRobot>("explorer/distance_from_robot");


    recharging = false;
    in_queue = false;
    robot_state = active;
    robot_state_next = stay;
    charging_completed = false;
    going_charging_bool = false;
        auction_winner = false;
    lost_other_robot_auction = false;
    update_state_required = false;
        participating_to_auction = 0;
    managing_auction = false;
    going_to_ds = false;
    going_to_check_if_ds_is_free = false;

    need_to_charge = false;
    test = true;


    
    llh = 0; //TODO 
    
        preload_docking_stations();
    
}

void docking::robot_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
    ROS_DEBUG("Received robot position");
    robot_x = pose->pose.pose.position.x;
    robot_y = pose->pose.pose.position.y;
}

void docking::abort_charging_callback(const std_msgs::Empty &msg)
{
    // ROS_INFO("Charging aborted");
    ROS_ERROR("\e[1;34mABORTED CHARGING!!!!!!!!!!!!!!!!!!!!\e[0m");
    recharging = false;
    going_to_ds = false;
    going_to_check_if_ds_is_free = false;  // TODO hmm...
}

void docking::preload_docking_stations()
{
    int index = 0; //index of the DS: used to loop above all the docking stations inserted in the file
    double x, y; //DS coordinates
    
    /* If the x-coordinate of DS with index <index> is found, it means that that DS is present in the file and must be loaded */
    while (nh.hasParam("energy_mgmt/d" + SSTR(index) + "/x"))
    {
    
        /* Load coordinates */
        nh.param("energy_mgmt/d" + SSTR(index) + "/x", x, 0.0);
        nh.param("energy_mgmt/d" + SSTR(index) + "/y", y, 0.0);
        
        /* Store new DS */
        ds_t new_ds;
        new_ds.id = index;
        new_ds.vacant = true;
        translate_coordinates(x, y, &(new_ds.x), &(new_ds.y));
        ds.push_back(new_ds);
        
        /* Delete the loaded parameters (since they are not used anymore) and prepare to search for next DS */
        nh.deleteParam("energy_mgmt/d" + SSTR(index) + "/x");
        nh.deleteParam("energy_mgmt/d" + SSTR(index) + "/y");
        index++;

    }

    /* Set the initial optimal DS to ds0: this is just to initialize the variable, so that the can compute the distance from the robot and the currently selected DS and compare it with the distance from the other DSs also when the node starts */
    best_ds = ds.at(0); //TODO what if there are not DS?????

    /* Print loaded DSs with their coordinates relative to the local reference system of the robot; only for debug purposes */
    std::vector<ds_t>::iterator it;
    for (it = ds.begin(); it != ds.end(); it++)
        ROS_DEBUG("ds%d: (%f, %f)", (*it).id, (*it).x, (*it).y);
}

void docking::compute_optimal_ds()
{
    // Just to force updating best_ds at the beginning... //TODO useless
    geometry_msgs::PointStamped msg1;
    msg1.point.x = best_ds.x;
    msg1.point.y = best_ds.y;
    //pub_new_target_ds.publish(msg1);


    /* Get current robot position */
    ros::service::waitForService("explorer/robot_pose");
    explorer::RobotPosition srv_msg;
    if (ros::service::exists("explorer/robot_pose", true))
        if (sc_robot_pose.call(srv_msg))
        {
            robot_x = srv_msg.response.x;
            robot_y = srv_msg.response.y;
            ROS_DEBUG("Robot position: (%f, %f)", robot_x, robot_y);
        }
        else
            ROS_ERROR("Call to service %s failed", sc_robot_pose.getService().c_str());
    else
        ROS_ERROR("Service %s is not ready yet", sc_robot_pose.getService().c_str());


    /* VERSION 1 */
    /*
    bool computed_new_optimal_ds = false;
    std::vector<ds_t>::iterator it = ds.begin();
    for (; it != ds.end(); it++)
        // if(optimal_ds_computed_once) {

        //TODO here I should consider that maybe there is a pending update of the DS...
        if ((best_ds.x - x) * (best_ds.x - x) + (best_ds.y - y) * (best_ds.y - y) >
            ((*it).x - x) * ((*it).x - x) + ((*it).y - y) * ((*it).y - y))
        {
            computed_new_optimal_ds = true;
            next_optimal_ds = *it;
        }
        else
            ;  // ROS_ERROR("\e[1;34m!!!!\e[0m");
    //} else {
    //    ROS_ERROR("\e[1;34mFirst computation of optimal DS: ds %d at (%f, %f)\e[0m", best_ds.id, best_ds.x,
    //    best_ds.y);
    //    optimal_ds_computed_once = true;
    //    best_ds = *it;
    //}

   if(computed_new_optimal_ds)
       if(participating_to_auction == 0 && !update_state_required) {
            ROS_ERROR("\e[1;34mNew optimal DS: ds%d (%f, %f)\e[0m", next_optimal_ds.id, next_optimal_ds.x,
   next_optimal_ds.y);
            best_ds = next_optimal_ds;
            geometry_msgs::PointStamped msg1;
            msg1.point.x = best_ds.x;
            msg1.point.y = best_ds.y;
            //pub_new_target_ds.publish(msg1);
        }
        else
            ROS_ERROR("\e[1;34mNext optimal DS: ds%d (%f, %f)\e[0m", next_optimal_ds.id, next_optimal_ds.x,
   next_optimal_ds.y);
     else
        ROS_DEBUG("Optimal DS unchanged");
    */

    /* VERSION 2 */ //TODO
    
    if (participating_to_auction == 0) {
        bool found_new_optimal_ds = false;
        std::vector<ds_t>::iterator it = ds.begin();
        for (; it != ds.end(); it++)
            // if(optimal_ds_computed_once) {

            // TODO Disjkstra!!!
            if ((best_ds.x - robot_x) * (best_ds.x - robot_x) + (best_ds.y -robot_y) * (best_ds.y - robot_y) > ((*it).x - robot_x) * ((*it).x - robot_x) + ((*it).y - robot_y) * ((*it).y - robot_y))
            {
                found_new_optimal_ds = true;
                best_ds = *it;
            }

        /* If a new optimal DS has been found, parameter l4 of the charging likelihood function must be updated */
        if (found_new_optimal_ds)
        {
            ROS_DEBUG("New optimal DS: ds%d (%f, %f)", best_ds.id, best_ds.x, best_ds.y);
            geometry_msgs::PointStamped msg1;
            msg1.point.x = best_ds.x;
            msg1.point.y = best_ds.y;
            //pub_new_target_ds.publish(msg1); //TODO
            
            update_l4();
        }
        else
            ROS_DEBUG("Optimal DS unchanged");
    } else
        ROS_DEBUG("There are still some pending auctions: cannot update optimal DS");
        
    
}

void docking::points(const adhoc_communication::MmListOfPoints::ConstPtr &msg) //TODO 
{
    ;
}

void docking::adhoc_ds(const adhoc_communication::EmDockingStation::ConstPtr &msg) //TODO 
{
    ;  // ROS_ERROR("adhoc_ds!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}

double docking::get_llh()
{
    /* The likelihood can be updated only if the robot is not participating to an auction */ //TODO explain better  
   if(participating_to_auction ==  0)
        llh = w1 * l1 + w2 * l2 + w3 * l3 + w4 * l4;
   return llh;
}

void docking::update_l1() //TODO all update_l*
{
    // count vacant docking stations
    int num_ds_vacant = 0;
    for (int i = 0; i < ds.size(); ++i)
    {
        if (ds[i].vacant == true)
            ++num_ds_vacant;
    }

    // count active robots
    int num_robots_active = 0;
    for (int i = 0; i < robots.size(); ++i)
    {
        if (robots[i].state == active)
            ++num_robots_active;
    }

    // sanity checks
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

    // check boundaries
    if (num_ds_vacant > num_robots_active)
    {
        l1 = 1;
    }
    else if (num_robots_active == 0)
    {
        l1 = 0;
    }

    // compute l1
    else
    {
        l1 = num_ds_vacant / num_robots_active;
    }
}

void docking::update_l2()
{
    double time_run = battery.remaining_time_run;
    double time_charge = battery.remaining_time_charge;

    // sanity checks
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

    // compute l2
    l2 = time_charge / (time_charge + time_run);
}

void docking::update_l3()
{

    // count number of jobs: count frontiers and recheable frontiers
    int num_jobs = 0;
    int num_jobs_close = 0;
    for (int i = 0; i < jobs.size(); ++i)
    {
        ++num_jobs;
        if (distance(jobs[i].x, jobs[i].y, true) <= distance_close)  // use euclidean distance to make it faster //TODO
            ++num_jobs_close;
    }

    // sanity checks
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

    // check boundaries
    if (num_jobs == 0)
    {
        l3 = 1;
    }

    // compute l3
    else
    {
        l3 = (num_jobs - num_jobs_close) / num_jobs;
    }
}

void docking::update_l4()
{

    // get distance to docking station
    int dist_ds = -1;
    for (int i = 0; i < ds.size(); ++i)
    {
        if (ds[i].id == target_ds.id)
        {
            //dist_ds = distance(ds[i].x, ds[i].y);
            dist_ds = distance(ds[i].x, ds[i].y, true); //TODO
            break;
        }
    }

    // get distance to closest job
    int dist_job = numeric_limits<int>::max();
    for (int i = 0; i < jobs.size(); ++i)
    {
        int dist_job_temp = distance(jobs[i].x, jobs[i].y, true);  // use euclidean distance to make it faster //TODO
        if (dist_job_temp < dist_job)
            dist_job = dist_job_temp;
    }

    // sanity checks
    if (dist_job < 0 || dist_job >= numeric_limits<int>::max())
    {
        ROS_ERROR("Invalid distance to closest job: %d", dist_job);
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
}

bool docking::auction(int docking_station, int id, int bid) //TODO remove
{
    return true;
}

bool docking::auction_send_multicast(string multicast_group, adhoc_communication::EmAuction auction, string topic) //TODO
{
    adhoc_communication::SendEmAuction auction_service;

    string destination_name = multicast_group + robot_name;

    ROS_INFO("Sending auction to multicast group %s on topic %s", destination_name.c_str(), topic.c_str());
    ROS_ERROR("Sending auction to multicast group %s on topic %s", destination_name.c_str(), topic.c_str());
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

double docking::distance(double goal_x, double goal_y, bool euclidean)
{
    explorer::DistanceFromRobot srv_msg; //TODO
    srv_msg.request.x = goal_x;
    srv_msg.request.y = goal_y;    
    
    sc_distance_from_robot.call(srv_msg);

    //TODO
    /*
    tf::Stamped<tf::Pose> robotPose;
    if (!costmap->getRobotPose(robotPose))
    {
        ROS_ERROR("Failed to get RobotPose");
        return -1;
    }
    return distance(robotPose.getOrigin().getX(), robotPose.getOrigin().getY(), goal_x, goal_y, euclidean);
    */
    return distance(robot_x, robot_y, goal_x, goal_y, euclidean);
}

double docking::distance(double start_x, double start_y, double goal_x, double goal_y, bool euclidean) //TODO
{
    double distance;

    // use euclidean distance
    if (euclidean)
    {
        double dx = goal_x - start_x;
        double dy = goal_y - start_y;
        distance = sqrt(dx * dx + dy * dy);
    }

    // calculate actual path length
    else
    {
        geometry_msgs::PoseStamped start, goal;

        start.header.frame_id = move_base_frame;
        start.pose.position.x = start_x;
        start.pose.position.y = start_y;
        start.pose.position.z = 0;

        goal.header.frame_id = move_base_frame;
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_y;
        goal.pose.position.z = 0;

        vector<geometry_msgs::PoseStamped> plan;

        // if(nav.makePlan(start, goal, plan))
        //    distance =  plan.size() * costmap->getCostmap()->getResolution();
        // else
        distance = -1;
    }

    return distance;
}

void docking::robot_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg) //TODO
{
    ROS_DEBUG("Received robot position");
    robot_x = msg.get()->point.x;
    robot_y = msg.get()->point.y;
}

void docking::cb_battery(const energy_mgmt::battery_state::ConstPtr &msg)
{
    ROS_DEBUG("Received battery state");

    /* Store new battery state */
    battery.charging = msg.get()->charging;
    battery.soc = msg.get()->soc;
    battery.remaining_time_charge = msg.get()->remaining_time_charge;
    battery.remaining_time_run = msg.get()->remaining_time_run;
    battery.remaining_distance = msg.get()->remaining_distance;

    /* Update parameter l2 of charging likelihood function */
    update_l2();
}


void docking::cb_robot(const adhoc_communication::EmRobot::ConstPtr &msg) {
    adhoc_communication::SendEmRobot srv_msg;
    srv_msg.request.topic = "robots";
    srv_msg.request.robot.id = robot_id;
    srv_msg.request.robot.state = msg.get()->state;
    sc_send_robot.call(srv_msg);
    
    //TODO correctly update the state!!!!
    if(msg.get()-> state == active)
        robots[robot_id].state = active;
    else
        robots[robot_id].state = idle;
    
    if(msg.get()->state == in_queue_enum) {        
        // ROS_INFO("Robot in queue");
        ROS_ERROR("\n\t\e[1;34mRobot in queue!!!\e[0m");
        going_to_check_if_ds_is_free = false;
        recharging = false;
        going_to_ds = false;
        in_queue = true;
        timer_restart_auction.setPeriod(ros::Duration(AUCTION_RESCHEDULING_TIME), true);
        // timer_restart_auction.setPeriod(ros::Duration(10), true);
        timer_restart_auction.start();
        
        adhoc_communication::SendEmDockingStation srv_msg;
        srv_msg.request.topic = "adhoc_communication/ds_state_update";
        srv_msg.request.docking_station.id = target_ds.id;
        srv_msg.request.docking_station.vacant = true;
        sc_send_docking_station.call(srv_msg);
        
        ds[target_ds.id].vacant = true;
    } else if(msg.get()->state == going_charging) {
        ROS_ERROR("\n\t\e[1;34m Robo t going charging!!!\e[0m");
        going_to_ds = true;
        going_to_check_if_ds_is_free = false;
        recharging = false;
    } else if(msg.get()->state == charging) {
        ROS_ERROR("\n\t\e[1;34mRechargin!!!\e[0m");
        recharging = true;
        need_to_charge = false;
        going_to_ds = false;
        going_to_check_if_ds_is_free = false;
        
        adhoc_communication::SendEmDockingStation srv_msg;
        srv_msg.request.topic = "adhoc_communication/ds_state_update";
        srv_msg.request.docking_station.id = target_ds.id;
        srv_msg.request.docking_station.vacant = false;

        sc_send_docking_station.call(srv_msg);
        
        ds[target_ds.id].vacant = false;
    
    }

}

void docking::cb_robots(const adhoc_communication::EmRobot::ConstPtr &msg)
{
    // check if robot is in list already
    bool new_robot = true;
    for (int i = 0; i < robots.size(); ++i)
    {
        // robot is in list, update
        if (robots[i].id == msg.get()->id)
        {
            // update state
            robots[i].state = (state_t)msg.get()->state;

            new_robot = false;
            break;
        }
    }

    // add new robot
    if (new_robot)
    {
        robot_t robot;
        robot.id = msg.get()->id;
        robot.state = (state_t)msg.get()->state;
        robots.push_back(robot);
    }

    // update charging likelihood
    update_l1();
    
}

void docking::cb_jobs(const adhoc_communication::ExpFrontier::ConstPtr &msg)
{
    adhoc_communication::ExpFrontierElement frontier_element;

    // add new jobs
    for (int i = 0; i < msg.get()->frontier_element.size(); ++i)
    {
        frontier_element = msg.get()->frontier_element.at(i);

        // check if it is a new job
        bool new_job = true;
        for (int j = 0; j < jobs.size(); ++j)
        {
            if (frontier_element.id == jobs.at(j).id)
            {
                new_job = false;
                break;
            }
        }

        // store job if it is new
        if (new_job == true)
        {
            job_t job;
            job.id = frontier_element.id;
            job.x = frontier_element.x_coordinate;
            job.y = frontier_element.y_coordinate;
            jobs.push_back(job);
        }
    }

    // remove completed jobs / frontiers
    for (int i = 0; i < jobs.size(); ++i)
    {
        // convert coordinates to cell indices
        unsigned int mx = 0;
        unsigned int my = 0;
        if (false)
        {
            // if(costmap->getCostmap()->worldToMap(jobs[i].x, jobs[i].y, mx, my) == false){
            ROS_ERROR("Could not convert job coordinates: %.2f, %.2f.", jobs[i].x, jobs[i].y);
            continue;
        }

        // get the 4-cell neighborhood with range 6
        vector<int> neighbors_x;
        vector<int> neighbors_y;
        for (int j = 0; j < 6; ++j)
        {
            neighbors_x.push_back(mx + i + 1);
            neighbors_y.push_back(my);
            neighbors_x.push_back(mx - i - 1);
            neighbors_y.push_back(my);
            neighbors_x.push_back(mx);
            neighbors_y.push_back(my + i + 1);
            neighbors_x.push_back(mx);
            neighbors_y.push_back(my - i - 1);
        }

        // check the cost of each neighbor
        bool unknown_found = false;
        bool obstacle_found = false;
        bool freespace_found = false;
        for (int j = 0; j < neighbors_x.size(); ++j)
        {
            int cost = 0;
            // unsigned char cost = costmap->getCostmap()->getCost(neighbors_x[j], neighbors_y[j]);
            if (cost == costmap_2d::NO_INFORMATION)
            {
                unknown_found = true;
            }
            else if (cost == costmap_2d::FREE_SPACE)
            {
                freespace_found = true;
            }
            else if (cost == costmap_2d::LETHAL_OBSTACLE)
            {
                obstacle_found = true;
            }
        }

        // delete frontier if either condition holds
        //  * no neighbor is unknown
        //  * at least one neighbor is an obstacle
        //  * no neighbor is free space
        if (unknown_found == false || obstacle_found == true || freespace_found == false)
        {
            jobs.erase(jobs.begin() + i);
            --i;
        }
    }

    // update charging likelihood
    update_l3();
}

void docking::cb_docking_stations(const adhoc_communication::EmDockingStation::ConstPtr &msg)
{
    // ROS_ERROR("\e[1;34mYESSSSSSSSSSSSSSSSSSS\e[0m");

    // check if docking station is in list already
    bool new_ds = true;
    for (int i = 0; i < ds.size(); ++i)
    {
        // docking station is in list, update
        if (ds[i].id == msg.get()->id)
        {
            // coordinates don't match
            if (ds[i].x != msg.get()->x || ds[i].y != msg.get()->y)
                ROS_ERROR("Coordinates of docking station %d do not match: (%.2f,%.2f) != (%.2f,%.2f)", ds[i].id,
                          ds[i].x, ds[i].y, msg.get()->x, msg.get()->y);

            // update vacancy
            ds[i].vacant = msg.get()->vacant;

            new_ds = false;
            break;
        }
    }

    // add new docking station
    if (new_ds)
    {
        // ROS_ERROR("\e[1;34mNew docking station received\e[0m");
        ds_t s;
        s.id = msg.get()->id;
        s.x = msg.get()->x;
        s.y = msg.get()->y;
        s.vacant = msg.get()->vacant;
        ds.push_back(s);

        // map_merger::TransformPoint point;
        // point.request.point.src_robot = "robot_0";
        // point.request.point.x = s.x;
        // point.request.point.y = s.y;

        // ROS_ERROR("\e[1;34mCalling: %s\e[0m", sc_trasform.getService().c_str());
        // sc_trasform.call(point);
    }

    // update charging likelihood
    update_l1();
}

void docking::cb_auction(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    // ROS_ERROR("\n\t\e[1;34mBid received: (%d, %d)\e[0m", msg.get()->robot, msg.get()->auction);
    




    // TODO
    /*
    // set auction id
    if(id > auction_id) // it is a new action from another robot, respond
        auction_id = id;
    else if(id > 0){    // it is an old auction from another robot, ignore
        ROS_ERROR("Bad auction ID, it should be greater than %d!", auction_id);
        return false;
    }
    else                // it is a new auction by this robot
        ; //++auction_id; //TODO //F
    */
    if (msg.get()->docking_station != best_ds.id)
    {
        /* Robot received a bid of an auction whose auctioned docking station is not the one the robot is interested in
         * at the moment, so it won't participate to the auction */
        ;  // ROS_INFO("Robot has no interested in participating to this auction");
    }
    else
    {
        /* The robot is interested in participating to the auction */
        participating_to_auction++;
        
        /* Start timer to force the robot to consider the auction concluded after some time, even in case it does not
         * receive the result of the auction */
        ros::Timer timer = nh.createTimer(ros::Duration(FORCED_AUCTION_END_TIMEOUT), &docking::end_auction_participation_timer_callback, this, true, false);
        timer.start();
        timers.push_back(timer); //TODO 

        // if(-remaining_time > bid) //TODO
        {
            // ROS_INFO("The robot can bet an higher bid than the one received, so it will participate to the auction");
            adhoc_communication::SendEmAuction srv;
            // srv.request.dst_robot = id;
            srv.request.topic = "adhoc_communication/send_em_auction/reply";
            srv.request.auction.auction = msg.get()->auction;
            srv.request.auction.robot = robot_id;
            srv.request.auction.docking_station = best_ds.id;
            srv.request.auction.bid = get_llh(); //TODO
            //srv.request.auction.bid = -remaining_time;
            ROS_DEBUG("Calling service: %s", sc_send_auction.getService().c_str());
            sc_send_auction.call(srv);
        }
        // TODO
        /*
        else {
            ROS_ERROR("\n\t\e[1;34mIbut i have no chance to win... \e[0m");
            participating_to_auction--;
        }
        */
    }

    //TODO multi-hoop??
    
}

void docking::cb_auction_reply(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    // ROS_INFO("Received reply to auction");

    // TODO //F probably in this version doesn't work with multi-hoop replies!!!!!
    // ROS_ERROR("\n\t\e[1;34mReply received: (%d, %d)\e[0m", msg.get()->robot, msg.get()->auction);
    // if(msg.get()->robot == robot_id) {

    std::vector<auction_bid_t>::iterator it = auction_bids.begin();
    for (; it != auction_bids.end(); it++)
        if ((*it).robot_id == msg.get()->robot)
        {
            // ROS_ERROR("\n\t\e[1;34mBut I have already received it...\e[0m");
            return;
        }

    // if(msg.get()->robot == robot_id && msg.get()->auction == auction_id)
    {
        // TODO it is ok ONLY assuming taht a robot during different auctions do not changes its bid!!!

        // ROS_ERROR("\n\t\e[1;34mReceived back my own auction... store bid of the sending robot\e[0m");

        if (managing_auction)
        {
            // ROS_ERROR("\n\t\e[1;34mreply stored\e[0m");
            auction_bid_t bid;
            bid.robot_id = msg.get()->robot;
            bid.bid = msg.get()->bid;
            auction_bids.push_back(bid);
        }
        else
        {
            // ROS_ERROR("\n\t\e[1;34mbut it was out of time...\e[0m");
            return;  // F ???
        }
    }
    // else
    //    ROS_ERROR("\n\t\e[1;34mbut it is not a reply to an auction of mine...\e[0m");

    /*
    // Multi-hoop reply
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = "adhoc_communication/send_em_auction/reply";
    srv.request.auction.auction = msg.get()->auction;
    srv.request.auction.robot = msg.get()->robot;
    srv.request.auction.docking_station = msg.get()->docking_station;
    srv.request.auction.bid = msg.get()->bid;
    ROS_ERROR("\n\t\e[1;34m Multi-hoop send reply\e[0m");
    sc_send_auction.call(srv);
    */
    
}

void docking::end_auction_participation_timer_callback(const ros::TimerEvent &event)
{
    // ROS_INFO("Force to consider auction concluded");
    participating_to_auction--;
}

void docking::timerCallback(const ros::TimerEvent &event)
{
    /* The auction is concluded; the robot that started it has to compute who is the winner and must inform all the
     * other robots */
    // ROS_INFO("Auction timeout: compute auction winner...");

    // ???
    managing_auction = false;
    // ROS_ERROR("\n\t\e[1;34mdecrementing\e[0m");

    /* Compute auction winner: loop through all the received bids and find the robot that sent the highest one */
    int winner;  // the id of thw winner
    float winner_bid = numeric_limits<int>::min();
    std::vector<auction_bid_t>::iterator it = auction_bids.begin();
    for (; it != auction_bids.end(); it++)
    {
        ROS_DEBUG("Robot %d placed %f", (*it).robot_id, (*it).bid);
        if ((*it).bid > winner_bid)
        {
            winner = (*it).robot_id;
            winner_bid = (*it).bid;
        }
    }

    // ???
    /*
    if (auction_bids.size() < 3 && ds.size() == 1)
        ROS_ERROR("\n\t\e[1;34m OH NO!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \e[0m");
        */

    // ROS_DEBUG("The winner is robot %d", winner);

    /* Delete stored bids to be able to start another auction in the future */
    auction_bids.clear();  // TODO inefficient!!!!

    /* Check if the robot that started the auction is the winner */
    if (winner == robot_id)
    {
        /* The robot won its own auction */
        //ROS_INFO("Winner of the auction"); //TODO specify which auction
        next_target_ds = best_ds;
        // going_to_ds = true;
        // going_to_check_if_ds_is_free = true;
        in_queue = false;

        auction_winner = true;
        lost_other_robot_auction = false;
        robot_state_next = going_charging_next;

        timer_restart_auction.stop();  // F i'm not sure that this follows the idea in the paper...
    }
    else
    {
        /* The robot lost its own auction */
        //ROS_ERROR("\n\t\e[1;34mI lost won my own auction...\e[0m");
        auction_winner = false;

        /* Schedule next auction */
        // timer_restart_auction.setPeriod(ros::Duration(AUCTION_RESCHEDULING_TIME), true);
        // timer_restart_auction.setPeriod(ros::Duration(10), true);
        // timer_restart_auction.start();
    }

    adhoc_communication::SendEmAuction srv_mgs;
    srv_mgs.request.topic = "adhoc_communication/auction_winner";
    srv_mgs.request.auction.auction = auction_id;
    srv_mgs.request.auction.robot = winner;

    srv_mgs.request.auction.docking_station = best_ds.id;
    srv_mgs.request.auction.bid = get_llh();

    // ROS_ERROR("\n\t\e[1;34m%s\e[0m", sc_send_auction.getService().c_str());
    sc_send_auction.call(srv_mgs);

    /* Computation completed */
    participating_to_auction--;  // since the auction is finished
    update_state_required = true;
}

void docking::cb_charging_completed(const std_msgs::Empty &msg)
{

    adhoc_communication::SendEmDockingStation srv_msg;
    srv_msg.request.topic = "adhoc_communication/ds_state_update";
    srv_msg.request.docking_station.id = best_ds.id;
    srv_msg.request.docking_station.vacant = true;
    

    sc_send_docking_station.call(srv_msg);
    
    ds[target_ds.id].vacant = true;
    

    recharging = false;
    going_to_ds = false;                   // TODO but actually until the robot does not leave the ds, it is occupied!!!
    going_to_check_if_ds_is_free = false;  // TODO not here!!!!

    charging_completed = true;
}

void docking::cb_vacant_docking_station(const adhoc_communication::EmDockingStation::ConstPtr &msg)
{
    // ROS_ERROR("\n\t\e[1;34mVACANT !!!!!!!!!!!!!!!!!!!!!!\e[0m");
    std::vector<ds_t>::iterator it = ds.begin();
    for (; it != ds.end(); it++)
        if ((*it).id == msg.get()->id)
            (*it).vacant == true;
}

void docking::timer_callback_schedure_auction_restarting(const ros::TimerEvent &event)
{
    ROS_INFO("Periodic re-auctioning");
    
    //start_new_auction(); //TODO

    /* Starting a new auction */
    managing_auction = true;
    auction_id++;
    participating_to_auction++;

    /* send broadcast information about new auction */
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = "adhoc_communication/send_em_auction/new_auction";
    srv.request.auction.auction = auction_id;
    srv.request.auction.robot = robot_id;
    srv.request.auction.docking_station = best_ds.id;
    srv.request.auction.bid = get_llh();
    ROS_DEBUG("Calling service: %s", sc_send_auction.getService().c_str());
    sc_send_auction.call(srv);

    auction_bid_t bid;
    bid.robot_id = robot_id;
    bid.bid = get_llh();
    auction_bids.push_back(bid);

    timer_finish_auction.setPeriod(ros::Duration(AUCTION_TIMEOUT), true);
    timer_finish_auction.start();
}

void docking::cb_need_charging(const std_msgs::Empty &msg)
{
    // ROS_INFO("Starting new auction");

    /* The robot is starting an auction: in particular, this means that it needs to recharge */
    need_to_charge = true;
    managing_auction = true;
    auction_id++;
    participating_to_auction++;

    /* Start auction timer to be notified of auction conclusion */
    timer_finish_auction.setPeriod(ros::Duration(AUCTION_TIMEOUT), true);
    timer_finish_auction.start();

    /* Send broadcast message to inform all robots of the new auction */
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = "adhoc_communication/send_em_auction/new_auction";
    srv.request.auction.auction = auction_id;
    srv.request.auction.robot = robot_id;
    srv.request.auction.docking_station = best_ds.id;
    srv.request.auction.bid = get_llh();
    ROS_DEBUG("Calling service: %s", sc_send_auction.getService().c_str());
    sc_send_auction.call(srv);

    /* Keep track of robot bid */
    auction_bid_t bid;
    bid.robot_id = robot_id;
    bid.bid = get_llh();
    auction_bids.push_back(bid);
}

void docking::cb_translate(const adhoc_communication::EmDockingStation::ConstPtr &msg)
{
    if (robot_prefix == "/robot_1")  // TODO ...
    {
        // ROS_ERROR("\n\t\e[1;34mReply to translation\e[0m");
        map_merger::TransformPoint point;

        point.request.point.src_robot = "robot_0";
        point.request.point.x = msg.get()->x;
        point.request.point.y = msg.get()->y;
        // if(sc_trasform.call(point)) ROS_ERROR("\e[1;34mTransformation succeded:\n\t\tOriginal point: (%f,
        // %f)\n\t\tObtained point: (%f, %f)\e[0m",point.request.point.x, point.request.point.y, point.response.point.x,
        // point.response.point.y);
    }
}

void docking::cb_auction_result(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    // TODO the robot must check for its participation to the auction!!!

    // ROS_INFO("Received result of auction ... //TODO complete!!

    std_msgs::Empty msg2;

    /* Check if the robot is interested in the docking station that was object of the auction whose result has been just
     * received */
    if (msg.get()->docking_station == best_ds.id)
    {
        // ROS_INFO("Received result of an auction to which the robot participated");
        // participating_to_auction--; //TODO here OR in the timer to force auction conclusion!!!

        /* Since the robot received the result of an auction to which it took part, explorer node must be informed of a possible change in the robot state */
        update_state_required = true;

        /* Check if the robot is the winner of the auction */
        if (robot_id == msg.get()->robot)
        {
            /* The robot won the auction */
            //ROS_INFO("Winner of the auction");
            auction_winner = true;
            
            next_target_ds = best_ds; //TODO this is safe if the best_ds is not modified during auctions!!!
                                      //TODO and what if best_ds is updated just a moment before starting the auction??? it shoul be ok because the if above would be false
            timer_restart_auction.stop();  // TODO  i'm not sure that this follows the idea in the paper... but probably
                                           // this is needed otherwise when i have many pendning auction and with a
                                           // timeout enough high, i could have an inifite loop of restarting
                                           // auctions... or I could but a control in the timer_callback!!!
        }
        else
        {
            /* The robot has lost an auction started by another robot (because the robot that starts an auction does not
             * receive the result of that auction with this callback */
             //TODO should check if the robto took part to the auction
            //ROS_INFO("Robot didn't win this auction"); 
            auction_winner = false;
            lost_other_robot_auction = true;
            // going_to_check_if_ds_is_free = false;

           
        }
    }
    else
        ROS_DEBUG("Received result of an auction the robot was not interested in: ignoring");
}

void docking::translate_coordinates(double starting_x, double starting_y, double *relative_x, double *relative_y)
{
    *relative_x = starting_x - origin_absolute_x;
    *relative_y = starting_y - origin_absolute_y;
}

void docking::check_vacancy_callback(const std_msgs::Empty::ConstPtr &msg)
{
    // ROS_ERROR("\n\t\e[1;34mCHECKING!!!!\e[0m");

    adhoc_communication::SendEmDockingStation srv_msg;
    srv_msg.request.topic = "adhoc_communication/ask_for_vacancy";
    srv_msg.request.docking_station.id = best_ds.id;

    sc_send_docking_station.call(srv_msg);
}

void docking::ds_state_update_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg) {
    //ROS_INFO("New state for ds%d: %s", msg.get()->id, (msg.get()->vacant ? "occupied" : "vacant") );
    ds[msg.get()->id].vacant = msg.get()->vacant;
}


void docking::ask_for_vacancy_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg)
{
    // ROS_ERROR("\n\t\e[1;34mReceived request for vacancy check\e[0m");
    if (msg.get()->id == target_ds.id) //TODO at the beginning no robot has already target_ds set, since it is set only after the end of all auctions!!!!!
        if (recharging || going_to_ds || going_to_check_if_ds_is_free)
        {
            if(recharging)
                ROS_ERROR("\n\t\e[1;34mI'm using that DS!!!!\e[0m");
            else if(going_to_ds || going_to_check_if_ds_is_free)
                ROS_ERROR("\n\t\e[1;34mI'm approachign that DS too!!!!\e[0m");
            adhoc_communication::SendEmDockingStation srv_msg;
            srv_msg.request.topic = "explorer/adhoc_communication/reply_for_vacancy";
            srv_msg.request.docking_station.id = target_ds.id;
            sc_send_docking_station.call(srv_msg);
        }
        else
            ROS_ERROR("\n\t\e[1;34m target ds, but currently not used by the robot \e[0m");
    else
        ROS_ERROR("\n\t\e[1;34m robot is not targetting that ds\e[0m");
}

void docking::update_robot_state()
{
    /*
     * Check if:
     * - there are no more pending auction: this is to avoid to communicate contracdicting information to the explorer
     *   node about the next state of the robot, so it better to wait that all the auctions have been finished and make
     *   a "global analisys" of the sistuation;
     * - an update of the state is required, i.e., that at least one auction was recently performed and that the robot
     *   took part to it.
     *
     * Notice that it may happen that the docking node thinks that an update of the state is required, but maybe it is
     * not necessary: it is the explorer node that has to make some ckecks and, only if necessary, update the state; the
     * docking node just informs the explorer node that something has recently happened
     */
    if (update_state_required && participating_to_auction == 0)
    {
        /* An update of the robot state is required and can be performed now */
        // ROS_INFO("Sending information to explorer node about the next state of the robot");

        /* Create the (empty) message to be sent */
        std_msgs::Empty msg;

        /* If the robot is not the winner of the most recent auction (notice that for sure it took part to at least one
         * auction, of update_state_required would be false) and if it needs to charge, notify explorer */
        if (need_to_charge && !auction_winner) {
            target_ds = next_target_ds;
            geometry_msgs::PointStamped msg1;
            msg1.point.x = target_ds.x;
            msg1.point.y = target_ds.y;
            
            pub_lost_own_auction.publish(msg);
            
        }

        /* If the robot is the winner of at least one auction, notify explorer */
        else if (auction_winner) {
            if( !recharging && !going_to_ds && !going_to_check_if_ds_is_free)  //TODO really necessary??? i don't think so...
            {
                target_ds = next_target_ds; //TODO but what if the robot is already charging at a certain DS which is different from next_target_ds ??????????
                geometry_msgs::PointStamped msg1;
                msg1.point.x = target_ds.x;
                msg1.point.y = target_ds.y;
                pub_new_target_ds.publish(msg1);
                going_to_check_if_ds_is_free = true;  // TODO not here!!!!
                
                pub_won_auction.publish(msg); //TODO it is important that this is after the other pub!!!!
            }

        /* If the robot has lost an auction that was not started by it, notify explorer (because if the robot was
         * recharging, it has to leave the docking station) */
        } else if (lost_other_robot_auction) {
            target_ds = next_target_ds;
            geometry_msgs::PointStamped msg1;
            msg1.point.x = target_ds.x;
            msg1.point.y = target_ds.y;
            
            pub_lost_other_robot_auction.publish(msg);
        
        }

        /* Reset all the variables that are used to keep information about the auctions results (i.e., about the next
         * robot state) */
        update_state_required = false;
        auction_winner = false;
        lost_other_robot_auction = false;
    }
    else
    {
        /* Do nothing, just print some debug text */
        if (participating_to_auction > 0 && !update_state_required)
            ROS_DEBUG("There are still pending auctions, and moreover no update is necessary for the moment");
        else if (!update_state_required)
            ROS_DEBUG("No state update required");
        else if (participating_to_auction > 0)
            ROS_DEBUG("There are still pending auctions, cannot update robot state");
        else
            ROS_FATAL("ERROR: the number of pending auctions is negative: %d", participating_to_auction); //TODO
    }
}
