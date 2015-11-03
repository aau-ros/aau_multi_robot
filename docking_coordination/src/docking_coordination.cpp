#include <ros/ros.h>
#include <docking_coordination.h>

using namespace std;

docking_coordination::docking_coordination()
{
    // read weights for the likelihood values from parameter file
    nh.param("w1", w1, 0.25);
    nh.param("w2", w2, 0.25);
    nh.param("w3", w3, 0.25);
    nh.param("w4", w4, 0.25);

    // initialize private variables
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
    update_llh();

    // subscribe to battery management topics
    sub_battery = nh.subscribe("battery_state", 1, &docking_coordination::battery_callback, this);
    sub_charging = nh.subscribe("charging", 1, &docking_coordination::charging_callback, this);
}

void docking_coordination::update_llh()
{
    update_l1();
    update_l2();
    update_l3();
    update_l4();

    llh = w1*l1 + w2*l2 + w3*l3 + w4*l4;
}

void docking_coordination::update_l1()
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

void docking_coordination::update_l2()
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

    ROS_ERROR("run time: %.2f, charge time: %.2f", time_run, time_charge);

    // compute l2
    l2 = time_charge / (time_charge + time_run);
}

void docking_coordination::update_l3()
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

void docking_coordination::update_l4()
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

void docking_coordination::battery_callback(const battery_mgmt::Battery::ConstPtr& msg)
{
    time_run = msg->remaining_time;
}

void docking_coordination::charging_callback(const battery_mgmt::Charge::ConstPtr &msg)
{
    time_charge = msg->remaining_time;
}