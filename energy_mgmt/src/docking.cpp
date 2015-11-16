#include <ros/ros.h>
#include <docking.h>

using namespace std;

docking::docking()
{
    // read weights for the likelihood values from parameter file
    nh.param("w1", w1, 0.25);
    nh.param("w2", w2, 0.25);
    nh.param("w3", w3, 0.25);
    nh.param("w4", w4, 0.25);

    // initialize private variables
    auction_id = 0;
    robots = 1;
    robots_active = 1;
    ds = 0;
    ds_vacant = 0;
    time_charge = 0;
    time_run = 0;
    jobs = 0;
    jobs_close = 0;
    dist_ds = 0;
    dist_job = 0;

    // initialize navigation function
    nav.initialize("navigation_path", costmap_global_ros_);

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

    // initialize service clients
    sc_send_auction = nh.serviceClient<adhoc_communication::SendEmAuction>(robot_name+"/adhoc_communication/send_auction");

    // advertise topics
    //pub_battery = nh.advertise<energy_mgmt::battery_state>("battery_state", 1);

    // subscribe to topics
    sub_robot_positions = nh.subscribe(robot_name+"/all_positions", 1000, &battery::cb_position_robots, this);
    sub_frontiers_positions = nh.subscribe(robot_name+"/frontiers", 10000, &battery::cb_position_frontiers, this);
    sub_docking_positions = nh.subscribe(robot_name+"/docking_stations", 1000, &battery::cb_position_docking_stations, this);
    sub_auction = nh.subscribe(robot_name+"/ds_auction", 1000, &battery::cb_auction, this);

}

void docking::update_llh(energy_mgmt::battery_state battery_state)
{
    time_run = battery_state.remaining_time_run;
    time_charge = battery_state.remaining_time_charge;

    update_l1();
    update_l2();
    update_l3();
    update_l4();

    llh = w1*l1 + w2*l2 + w3*l3 + w4*l4;
}

void docking::update_l1()
{
    // sanity check
    if(ds_vacant < 0){
        ROS_ERROR("Invalid number of vacant docking stations: %d!", ds_vacant);
        l1 = 0;
        return;
    }
    if(robots_active < 0){
        ROS_ERROR("Invalid number of active robots: %d!", robots_active);
        l1 = 1;
        return;
    }

    // check boundaries
    if(ds_vacant > robots_active){
        l1 = 1;
    }
    else if(robots_active == 0){
        l1 = 0;
    }

    // compute l1
    else{
        l1 = ds_vacant / robots_active;
    }
}

void docking::update_l2()
{
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
    // sanity check
    if(jobs < 0){
        ROS_ERROR("Invalid number of jobs: %d", jobs);
        l3 = 1;
        return;
    }
    if(jobs_close < 0){
        ROS_ERROR("Invalid number of jobs close by: %d", jobs_close);
        l3 = 1;
        return;
    }
    if(jobs_close > jobs){
        ROS_ERROR("Number of jobs close by greater than total number of jobs: %d > %d", jobs_close, jobs);
        l3 = 0;
        return;
    }

    // check boundaries
    if(jobs == 0){
        l3 = 1;
    }

    // compute l3
    else{
        l3 = (jobs - jobs_close) / jobs;
    }
}

void docking::update_l4()
{
    // sanity check
    if(dist_ds < 0){
        ROS_ERROR("Invalid distance to docking station: %.2f", dist_ds);
        l4 = 1;
        return;
    }
    if(dist_job < 0){
        ROS_ERROR("Invalid distance to closest job: %.2f", dist_job);
        l4 = 0;
        return;
    }
    if(dist_job == 0 && dist_ds == 0){
        //ROS_ERROR("Invalid distances to closest job and docking station. Both are zero!");
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
    //TODO

    // create an auction message and fill it with data
    adhoc_communication::EmAuction auction_msg;
    auction_msg.auction = auction_id;
    auction_msg.robot = robot_id;
    auction_msg.docking_station = docking_station;
    auction_msg.bid = llh;

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

void docking::cb_position_robots(const adhoc_communication::MmListOfPoints::ConstPtr& msg)
{
    mutex_position_robots.lock();

    // get positions of all robots
    position_robots.positions.clear();
    for(int i = 0; i < msg.get()->positions.size(); ++i){
        position_robots.positions.push_back(msg.get()->positions.at(i));
    }

    // update number of robots
    robots = position_robots.positions.size();

    // update number of active robots
    robots_active = robots; //TODO

    mutex_position_robots.unlock();
}

void docking::cb_position_frontiers(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{

}

void docking::cb_position_docking_stations(const adhoc_communication::MmListOfPoints::ConstPtr& msg)
{

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
