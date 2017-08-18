#include "concrete_bid_computer.h"

ConcreteBidComputer::ConcreteBidComputer() {
    ros::NodeHandle nh_tilde("~");
    if(!nh_tilde.getParam("w1", w1))
        ROS_FATAL("parameter not found");
    if(!nh_tilde.getParam("w2", w2))
        ROS_FATAL("parameter not found");
    if(!nh_tilde.getParam("w3", w3))
        ROS_FATAL("parameter not found");
    if(!nh_tilde.getParam("w4", w4))
        ROS_FATAL("parameter not found");
    if(!nh_tilde.getParam("x", origin_absolute_x))
        ROS_FATAL("parameter not found");
    if(!nh_tilde.getParam("y", origin_absolute_y))
        ROS_FATAL("parameter not found");
    if(!nh_tilde.getParam("robot_prefix", robot_name))
        ROS_FATAL("parameter not found");
    if(!nh_tilde.getParam("log_path", log_path))
        ROS_FATAL("parameter not found");

    std::string my_prefix = "";
    ros::NodeHandle nh;
    sub_jobs = nh.subscribe(my_prefix + "frontiers", 1000, &ConcreteBidComputer::cb_jobs, this);
    sub_battery = nh.subscribe(my_prefix + "battery_state", 1000, &ConcreteBidComputer::cb_battery, this);
    sub_robots = nh.subscribe(my_prefix + "robots", 1000, &ConcreteBidComputer::cb_robots, this);
    sub_docking_stations = nh.subscribe(my_prefix + "docking_stations", 1000, &ConcreteBidComputer::cb_docking_stations, this);
    sub_new_optimal_ds = nh.subscribe("explorer/new_optimal_ds", 10, &ConcreteBidComputer::newOptimalDsCallback, this);
    pose_sub = nh.subscribe("amcl_pose", 10, &ConcreteBidComputer::poseCallback, this);

    l1 = 0, l2 = 0, l3 = 0, l4 = 0;
    optimal_ds_is_set = false;
}

void ConcreteBidComputer::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {        
    message_mutex.lock();
    ROS_INFO("received robot position");
    next_robot_x = pose->pose.pose.position.x;
    next_robot_y = pose->pose.pose.position.y;
    message_mutex.unlock();
}

void ConcreteBidComputer::newOptimalDsCallback(const adhoc_communication::EmDockingStation::ConstPtr &msg) {
    next_optimal_ds_x = msg.get()->x;
    next_optimal_ds_y = msg.get()->y;
    next_optimal_ds_set = true;
}

void ConcreteBidComputer::updateLlh() {
    update_l1();
    update_l2();
    update_l3();
    update_l4();
}

void ConcreteBidComputer::update_l1() //TODO(minor) would be better to update them only when get_llh() is called, for efficiency... the problem is that the check participating == 0 would not allow it..
{
    message_mutex.lock();
    ROS_DEBUG("Update l1");

    unsigned int num_ds_vacant = countVacantDss();
    unsigned int num_robots_active = countActiveRobots();
    
    if (num_ds_vacant > num_robots_active)
        l1 = 1;
    else if (num_robots_active == 0)
        l1 = 0;
    else
        l1 = (double)num_ds_vacant / (double)num_robots_active;
    ROS_DEBUG("l1: %.1f", l1);

    message_mutex.unlock();   
}

unsigned int ConcreteBidComputer::countVacantDss() {
    ds_mutex.lock();
    unsigned int num_ds_vacant = 0;
    for (unsigned int i = 0; i < ds.size(); i++)
    {
        if (ds[i].vacant)
            num_ds_vacant++;
    }
    ROS_DEBUG("Number of vacant DS: %d", num_ds_vacant);
    ds_mutex.unlock();
    return num_ds_vacant;
}

unsigned int ConcreteBidComputer::countActiveRobots() {
    robot_mutex.lock();
    unsigned int num_robots_active = 0; 
    for (unsigned int i = 0; i < robots.size(); i++)
    {
        if (robots[i].active)
            num_robots_active++;
    }
    ROS_DEBUG("Number of active robots DS: %d", num_robots_active);
    robot_mutex.unlock();
    return num_robots_active;
}

void ConcreteBidComputer::update_l2()
{
    message_mutex.lock();

    ROS_DEBUG("Update l2");

    if(battery == NULL) {
        ROS_WARN("No battery state received yet");
        l2 = 0;
    }
    
    else {
        double time_run = battery->remaining_time_run;
        ROS_DEBUG("Remaining running time: %.2fs", time_run);
        
        double time_charge = battery->remaining_time_charge;
        ROS_DEBUG("Remaining time until recharge completion: %.2fs", time_charge);

        if (time_charge < 0)
        {
            ROS_ERROR("Invalid charging time: %.2f!", time_charge);
            l2 = 0;
        }
        else if (time_run < 0)
        {
            ROS_WARN("Run time is negative: %.2f", time_run);
            l2 = 1;
        }
        else if (time_run == 0 && time_charge == 0)
        {
            ROS_ERROR("Invalid run and charging times. Both are zero!");
            l2 = 1;
        }
        else {
            l2 = time_charge / (time_charge + time_run);
            ROS_DEBUG("l2: %.1f", l2);
        }
    }

    message_mutex.unlock();
}

void ConcreteBidComputer::update_l3()
{
    message_mutex.lock();

    ROS_DEBUG("Update l3");
    
    unsigned int num_jobs, num_jobs_close;
    countJobsAndCloseJobs(num_jobs, num_jobs_close);

    if (num_jobs == 0)
        l3 = 1;
    else
        l3 = (double)(num_jobs - num_jobs_close) / (double)num_jobs;
    ROS_DEBUG("l3: %.1f", l3);

    message_mutex.unlock();
}

void ConcreteBidComputer::countJobsAndCloseJobs(unsigned int &num_jobs, unsigned int &num_jobs_close) {
    num_jobs = 0, num_jobs_close = 0;

    if(battery == NULL)
        ROS_WARN("No battery state received yet");
    else
        for (unsigned int i = 0; i < jobs.size(); i++)
        {
            num_jobs++;

            double dist = distance_from_robot(jobs[i].x_coordinate, jobs[i].y_coordinate); 
            
            double dist2;
            if(optimal_ds_is_set)
                dist2 = distance(jobs[i].x_coordinate, jobs[i].y_coordinate, optimal_ds_x, optimal_ds_y);
            else
                dist2 = distance(jobs[i].x_coordinate, jobs[i].y_coordinate, 0, 0); //TODO 
            
            if (dist + dist2 <= battery->maximum_traveling_distance)
                num_jobs_close++;

        }
    ROS_DEBUG("Number of frontiers: %d", num_jobs);
    ROS_DEBUG("Number of reachable frontiers: %d", num_jobs_close);
}

void ConcreteBidComputer::update_l4()
{
    message_mutex.lock();

    ROS_DEBUG("Update l4");

    if( ds.size() == 0 || jobs.size() == 0)
    {
        ROS_DEBUG("No frontiers");
        l4 = 0;
    }
    else {
        double dist_ds = distanceRobotOptimalDs();
        double dist_job = distanceOptimalDsClosestFrontier();

        if (dist_job < 0 || dist_job >= std::numeric_limits<int>::max())
        {
            ROS_ERROR("Invalid distance to closest job: %.3f", dist_job);
            l4 = 0;
        }
        else if (dist_job == 0 && dist_ds == 0)
        {
            ROS_ERROR("Invalid distances to closest job and docking station. Both are zero!");
            l4 = 0;
        }      
        else {
            l4 = (double)dist_job / (double)(dist_job + dist_ds);
            ROS_DEBUG("l4: %.1f", l4);
        }
    }

    message_mutex.unlock();
}

double ConcreteBidComputer::distanceRobotOptimalDs() {
    double dist;
    if(optimal_ds_is_set)
        dist = distance_from_robot(optimal_ds_x, optimal_ds_y);
    else
        dist = distance_from_robot(0, 0); //TODO
    ROS_DEBUG("Distance to optimal DS: %.2f", dist);
    return dist;
}

double ConcreteBidComputer::distanceOptimalDsClosestFrontier() {
    double dist = std::numeric_limits<int>::max();
    for (unsigned int i = 0; i < jobs.size(); i++)
    {
        double dist_temp = distance_from_robot(jobs[i].x_coordinate, jobs[i].y_coordinate);
        if (dist_temp < dist)
            dist = dist_temp;
    }
    ROS_DEBUG("Distance to closest frontier: %.2f", dist);
    return dist;

}

double ConcreteBidComputer::distance_from_robot(double x, double y) {
    double dx = (x - robot_x);
    double dy = (y - robot_y);
    return sqrt(dx * dx + dy * dy);
}

double ConcreteBidComputer::distance(double start_x, double start_y, double goal_x, double goal_y)
{
    double dx = (goal_x - start_x);
    double dy = (goal_y - start_y);
    return sqrt(dx * dx + dy * dy);
}

void ConcreteBidComputer::cb_battery(const explorer::battery_state::ConstPtr &msg)
{
    ROS_DEBUG("Received battery state");
    next_battery = msg;    
//    ROS_DEBUG("SOC: %d%%; rem. time: %.1f; rem. distance: %.1f", (int) (battery.soc * 100.0), battery.remaining_time_run, battery.remaining_distance);
}

void ConcreteBidComputer::cb_robots(const adhoc_communication::EmRobot::ConstPtr &msg)
{
    robot_mutex.lock();
    ROS_DEBUG("Received information from robot %d", msg.get()->id);
    bool new_robot = true;
    for (unsigned int i = 0; i < robots.size(); ++i)
    {
        if (robots[i].id == msg.get()->id)
        {
            robots[i].active = isActiveState(static_cast<robot_state::robot_state_t>(msg.get()->state));
            robots[i].x = msg.get()->x;
            robots[i].y = msg.get()->y;
            robots[i].selected_ds = msg.get()->selected_ds;               
            break;
        }
    }
    if (new_robot)
    {  
        robot_t new_robot;
        new_robot.id = msg.get()->id;
        new_robot.active = isActiveState(static_cast<robot_state::robot_state_t>(msg.get()->state));
        new_robot.x = msg.get()->x;
        new_robot.y = msg.get()->y;
        new_robot.selected_ds = msg.get()->selected_ds;
        robots.push_back(new_robot);
    }
    
    robot_mutex.unlock();
}

bool ConcreteBidComputer::isActiveState(robot_state::robot_state_t state) {
    return state == robot_state::COMPUTING_NEXT_GOAL || state == robot_state::MOVING_TO_FRONTIER || state == robot_state::CHOOSING_ACTION;
}

void ConcreteBidComputer::cb_jobs(const adhoc_communication::ExpFrontier::ConstPtr &msg)
{
    ROS_INFO("received frontiers");
    next_jobs = msg.get()->frontier_element;
}

void ConcreteBidComputer::cb_docking_stations(const adhoc_communication::EmDockingStation::ConstPtr &msg)
{
    ds_mutex.lock();
    ROS_INFO("received ds%d", msg.get()->id);    

    /* Check if DS is in list already */
    bool new_ds = true;
    for (unsigned int i = 0; i < ds.size(); i++)
    {
        if (ds[i].id == msg.get()->id)
        {
            new_ds = false;

            double x, y;   
            abs_to_rel(msg.get()->x, msg.get()->y, &x, &y); //TODO actually we don't need the DS coordinates...
            
            if(ds.at(i).timestamp > msg.get()->header.timestamp)
                continue;

            ds[i].vacant = msg.get()->vacant;
            
            break;
        }
    }
    
    if (new_ds)
    {       
        ds_t s;
        s.id = msg.get()->id;        
        abs_to_rel(msg.get()->x, msg.get()->y, &s.x, &s.y);
        s.vacant = msg.get()->vacant;
        s.timestamp = msg.get()->header.timestamp;
        ds.push_back(s); //discovered, but not reachable, since i'm not sure if it is reachable for this robot...
        ROS_INFO("New docking station received: ds%d (%f, %f)", s.id, s.x, s.y);
    }
    
    ds_mutex.unlock();
}

void ConcreteBidComputer::processMessages() {
    message_mutex.lock();
    jobs = next_jobs;
    optimal_ds_x = next_optimal_ds_x;
    optimal_ds_y = next_optimal_ds_y;
    optimal_ds_is_set = next_optimal_ds_set;
    robot_x = next_robot_x;
    robot_y = next_robot_y;
    battery = next_battery;
    message_mutex.unlock();
}

void ConcreteBidComputer::abs_to_rel(double absolute_x, double absolute_y, double *relative_x, double *relative_y)
{
    *relative_x = absolute_x - origin_absolute_x;
    *relative_y = absolute_y - origin_absolute_y;
}

void ConcreteBidComputer::logMetadata()
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

    std::string filename;
    std::fstream fs;

    log_path = log_path.append("/");
    filename = log_path + std::string("llh.csv");

    fs.open(filename.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs << "#w1,w2,w3,w4" << std::endl;
    fs << w1 << "," << w2 << "," << w3 << "," << w4 << std::endl;
    fs.close();
}

double ConcreteBidComputer::getBid() {
    llh = l1 * w1 + l2 * w2 + l3 * w3 + l4 * w4;
    return llh;
}
