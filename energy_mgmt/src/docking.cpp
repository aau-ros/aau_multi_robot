#include <docking.h>

using namespace std;

docking::docking()
{
    // read weights for the likelihood values from parameter file
    nh.param("w1", w1, 0.25);
    nh.param("w2", w2, 0.25);
    nh.param("w3", w3, 0.25);
    nh.param("w4", w4, 0.25);

    // initialize auction id
    auction_id = 0;

    // initialize navigation function
    nh.param("distance_close", distance_close, 8);
    nh.param<string>("move_base_frame", move_base_frame, "map");
    nav.initialize("navigation_path", costmap);

    // initialize robot name
    nh.param<string>("robot_prefix", robot_prefix, "");
    if(robot_prefix.empty()){
        //TODO
        char hostname[1024];
        hostname[1023] = '\0';
        gethostname(hostname, 1023);
        robot_name = string(hostname);
        robot_id = 0;
        ROS_ERROR("ONLY SIMULATIONS HAVE BEEN IMPLEMENTED YET, ABORTING");
        exit(0);
    }
    else{
        robot_name = robot_prefix;
        robot_id = atoi(robot_prefix.substr(7,1).c_str());
    }

    // initialize robot struct
    robot_t robot;
    robot.id = robot_id;
    robot.state = active;
    robots.push_back(robot);

    // initialize service clients
    sc_send_auction = nh.serviceClient<adhoc_communication::SendEmAuction>(robot_name+"/adhoc_communication/send_auction");
    sc_send_robot = nh.serviceClient<adhoc_communication::SendEmRobot>(robot_name+"adhoc_communication/send_robot");
    sc_send_ds = nh.serviceClient<adhoc_communication::SendEmDs>(robot_name+"adhoc_communication/send_ds");

    // subscribe to topics
    sub_battery = nh.subscribe(robot_name+"", 1, &docking::cb_battery, this);
    sub_robots = nh.subscribe(robot_name+"/all_positions", 100, &docking::cb_robots, this);
    sub_jobs = nh.subscribe(robot_name+"/frontiers", 10000, &docking::cb_jobs, this);
    sub_docking_stations = nh.subscribe(robot_name+"/docking_stations", 100, &docking::cb_docking_stations, this);
    sub_auction = nh.subscribe(robot_name+"/ds_auction", 100, &docking::cb_auction, this);

}

double docking::get_llh()
{
    return w1*l1 + w2*l2 + w3*l3 + w4*l4;
}

void docking::update_l1()
{
    // count vacant docking stations
    int num_ds_vacant = 0;
    for(int i=0; i<ds.size(); ++i){
        if(ds[i].vacant == true)
            ++num_ds_vacant;
    }

    // count active robots
    int num_robots_active = 0;
    for(int i=0; i<robots.size(); ++i){
        if(robots[i].state == active)
            ++num_robots_active;
    }

    // sanity check
    if(num_ds_vacant < 0){
        ROS_ERROR("Invalid number of vacant docking stations: %d!", num_ds_vacant);
        l1 = 0;
        return;
    }
    if(num_robots_active < 0){
        ROS_ERROR("Invalid number of active robots: %d!", num_robots_active);
        l1 = 1;
        return;
    }

    // check boundaries
    if(num_ds_vacant > num_robots_active){
        l1 = 1;
    }
    else if(num_robots_active == 0){
        l1 = 0;
    }

    // compute l1
    else{
        l1 = num_ds_vacant / num_robots_active;
    }
}

void docking::update_l2()
{
    double time_run = battery.remaining_time_run;
    double time_charge = battery.remaining_time_charge;

    // sanity check
    if(time_charge < 0){
        ROS_ERROR("Invalid charging time: %.2f!", time_charge);
        l2 = 0;
        return;
    }
    if(time_run < 0){
        ROS_ERROR("Invalid run time: %.2f!", time_run);
        l2 = 1;
        return;
    }
    if(time_run == 0 && time_charge == 0){
        ROS_ERROR("Invalid run and charging times. Both are zero!");
        l2 = 1;
        return;
    }

    // compute l2
    l2 = time_charge / (time_charge + time_run);
}

void docking::update_l3()
{
    // count number of jobs
    int num_jobs = 0;
    int num_jobs_close = 0;
    for(int i=0; i<jobs.size(); ++i){
        ++num_jobs;
        if(distance(jobs[i].x, jobs[i].y, true) <= distance_close) // use euclidean distance to make it faster
            ++num_jobs_close;
    }

    // sanity check
    if(num_jobs < 0){
        ROS_ERROR("Invalid number of jobs: %d", num_jobs);
        l3 = 1;
        return;
    }
    if(num_jobs_close < 0){
        ROS_ERROR("Invalid number of jobs close by: %d", num_jobs_close);
        l3 = 1;
        return;
    }
    if(num_jobs_close > num_jobs){
        ROS_ERROR("Number of jobs close by greater than total number of jobs: %d > %d", num_jobs_close, num_jobs);
        l3 = 0;
        return;
    }

    // check boundaries
    if(num_jobs == 0){
        l3 = 1;
    }

    // compute l3
    else{
        l3 = (num_jobs - num_jobs_close) / num_jobs;
    }
}

void docking::update_l4(int docking_station)
{
    // get distance to docking station
    int dist_ds = -1;
    for(int i=0; i<ds.size(); ++i){
        if(ds[i].id == docking_station){
            dist_ds = distance(ds[i].x, ds[i].y);
            break;
        }
    }

    // get distance to closest job
    int dist_job = numeric_limits<int>::max();
    for(int i=0; i<jobs.size(); ++i){
        int dist_job_temp = distance(jobs[i].x, jobs[i].y, true); // use euclidean distance to make it faster
        if(dist_job_temp < dist_job)
            dist_job = dist_job_temp;
    }

    // sanity check
    if(dist_ds < 0){
        ROS_ERROR("Invalid docking station: %d", docking_station);
        l4 = 1;
        return;
    }
    if(dist_job < 0 || dist_job >= numeric_limits<int>::max()){
        ROS_ERROR("Invalid distance to closest job: %d", dist_job);
        l4 = 0;
        return;
    }
    if(dist_job == 0 && dist_ds == 0){
        ROS_ERROR("Invalid distances to closest job and docking station. Both are zero!");
        l4 = 0;
        return;
    }

    // compute l4
    l4 = dist_job / (dist_job + dist_ds);
}

bool docking::auction(int docking_station, int id)
{
    // set auction id
    if(id > auction_id) // it is a new action from another robot, respond
        auction_id = id;
    else if(id > 0){    // it is an old auction from another robot, ignore
        ROS_ERROR("Bad auction ID, it should be greater than %d!", auction_id);
        return false;
    }
    else                // it is a new auction by this robot
        ++auction_id;

    // refresh auction bid for current docking station
    update_l4(docking_station);

    // create an auction message and fill it with data
    adhoc_communication::EmAuction auction_msg;
    auction_msg.auction = auction_id;
    auction_msg.robot = robot_id;
    auction_msg.docking_station = docking_station;
    auction_msg.bid = get_llh();

    // send auction over multicast
    auction_send_multicast("mc_", auction_msg, "ds_auction");
}

bool docking::auction_send_multicast(string multicast_group, adhoc_communication::EmAuction auction, string topic)
{
    adhoc_communication::SendEmAuction auction_service;

    string destination_name = multicast_group + robot_name;

    ROS_INFO("Sending auction to multicast group %s on topic %s", destination_name.c_str(), topic.c_str());
    auction_service.request.dst_robot = destination_name;
    auction_service.request.auction = auction;
    auction_service.request.topic = topic;

    if (sc_send_auction.call(auction_service)){
        if(auction_service.response.status){
            ROS_INFO("Auction was transmitted successfully.");
            return true;
        }
        else{
            ROS_WARN("Failed to send auction to multicast group %s!", destination_name.c_str());
            return false;
        }
    }
    else{
        ROS_WARN("Failed to call service %s/adhoc_communication/send_auction [%s]", robot_name, sc_send_auction.getService().c_str());
        return false;
    }
}

double docking::distance(double goal_x, double goal_y, bool euclidean)
{
    tf::Stamped<tf::Pose> robotPose;
    if (!costmap->getRobotPose(robotPose))
    {
        ROS_ERROR("Failed to get RobotPose");
        return -1;
    }
    return distance(robotPose.getOrigin().getX(), robotPose.getOrigin().getY(), goal_x, goal_y, euclidean);
}

double docking::distance(double start_x, double start_y, double goal_x, double goal_y, bool euclidean)
{
    double distance;

    // use euclidean distance
    if(euclidean){
        double dx = goal_x - start_x;
        double dy = goal_y - start_y;
        distance = sqrt(dx*dx + dy*dy)
    }

    // calculate actual path length
    else{
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

        if(nav.makePlan(start, goal, plan))
            distance =  plan.size() * costmap->getCostmap()->getResolution();
        else
            distance = -1;
    }

    return distance;
}

void docking::cb_battery(const energy_mgmt::battery_state::ConstPtr& msg)
{
    battery = msg.get();

    // update charging likelihood
    update_l2();
}

void docking::cb_robots(const adhoc_communication::MmListOfPoints::ConstPtr& msg)
{
    // check if robot is in list already
    bool new_robot = true;
    for(int i=0; i<robots.size(); ++i){
        // robot is in list, update
        if(robots[i].id == msg.get()->id){
            // update state
            robots[i].state = msg.get()->state;

            new_robot = false;
            break;
        }
    }

    // add new robot
    if(new_robot){
        robot_t robot;
        robot.id = msg.get()->id;
        robot.state = msg.get()->state;
        robots.push_back(robot);
    }

    // update charging likelihood
    update_l1();
}

void docking::cb_jobs(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{
    adhoc_communication::ExpFrontierElement frontier_element;

    // add new jobs
    for(int i=0; i< msg.get()->frontier_element.size(); ++i){
        frontier_element = msg.get()->frontier_element.at(i);

        // check if it is a new job
        bool new_job = true;
        for(int j=0; j<jobs.size(); ++j){
            if(frontier_element.id == jobs.at(j).id){
                new_job = false;
                break;
            }
        }

        // store job if it is new
        if(new_job == true){
            job_t job;
            job.id = frontier_element.id;
            job.x = frontier_element.x_coordinate;
            job.y = frontier_element.y_coordinate;
            jobs.push_back(job);
        }
    }

    // remove completed jobs / frontiers
    for(int i=0; i<jobs.size(); ++i){
        // convert coordinates to cell indices
        int mx = 0;
        int my = 0;
        if(costmap->getCostmap()->worldToMap(jobs[i].x, jobs[i].y, mx, my) == false){
            ROS_ERROR("Could not convert job coordinates: %.2f, %.2f.", jobs[i].x, jobs[i].y);
            continue;
        }

        // get the 4-cell neighborhood with range 6
        vector<array<int, 2>> neighbors;
        for(int j=0; j<6; ++j){
            neighbors.push_back({mx+i+1, my});
            neighbors.push_back({mx-i-1, my});
            neighbors.push_back({mx, my+i+1});
            neighbors.push_back({mx, my-i-1});
        }

        // check the cost of each neighbor
        bool unknown_found = false;
        bool obstacle_found = false;
        bool freespace_found = false;
        for(int j=0; j<neighbors.size(); ++j){
            int new_mx = neighbors[j][0];
            int new_my = neighbors[j][1];
            unsigned char cost = costmap->getCostmap()->getCost(new_mx, new_my);
            if(cost == costmap_2d::NO_INFORMATION){
                unknown_found = true;
            }
            else if(cost == costmap_2d::FREE_SPACE){
                freespace_found = true;
            }
            else if(cost == costmap_2d::LETHAL_OBSTACLE){
                obstacle_found = true;
            }
        }

        // delete frontier if either condition holds
        //  * no neighbor is unknown
        //  * at least one neighbor is an obstacle
        //  * no neighbor is free space
        if(unknown_found == false || obstacle_found == true || freespace_found == false){
            jobs.erase(i);
            --i;
        }
    }

    // update charging likelihood
    update_l3();
}

void docking::cb_docking_stations(const adhoc_communication::EmDockingStation::ConstPtr& msg)
{
    // check if docking station is in list already
    bool new_ds = true;
    for(int i=0; i<ds.size(); ++i){
        // docking station is in list, update
        if(ds[i].id == msg.get()->id){
            // coordinates don't match
            if(ds[i].x != msg.get()->x || ds[i].y != msg.get()->y)
                ROS_ERROR("Coordinates of docking station %d do not match: (%.2f,%.2f) != (%.2f,%.2f)", ds[i].id, ds[i].x, ds[i].y, msg.get()->x, msg.get()->y);

            // update vacancy
            ds[i].vacant = msg.get()->vacant;

            new_ds = false;
            break;
        }
    }

    // add new docking station
    if(new_ds){
        ds_t s;
        s.id = msg.get()->id;
        s.x = msg.get()->x;
        s.y = msg.get()->y;
        s.vacant = msg.get()->vacant;
        ds.push_back(s);
    }

    // update charging likelihood
    update_l1();
}

void docking::cb_auction(const adhoc_communication::EmAuction::ConstPtr& msg)
{
    // only process callback if auction is not initiated by this robot
    if(robot_name.compare(msg.get()->robot_name.c_str()) == 0)
        return;

    // respond to auction
    if(auction(msg.get()->docking_station, msg.get()->auction) == false)
        ROS_ERROR("Failed to respond to auction %d", msg.get()->auction);
}
