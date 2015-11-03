#ifndef DOCKING_COORDINATION_H
#define DOCKING_COORDINATION_H

#include "battery_mgmt/Battery.h"
#include "battery_mgmt/Charge.h"

class docking_coordination
{
public:
    /**
     * Constructor
     */
    docking_coordination();

    /**
     * Update the likelihood value.
     */
    void update_llh();

private:
    /**
     * The node handle.
     */
    ros::NodeHandle nh;

    /**
     * Subscribers for the required topics.
     */
    ros::Subscriber sub_battery, sub_charging;

    /**
     * Total number of robots and number of active robots.
     */
    int robots, robots_active;

    /**
     * Total number of docking stations and number of free docking stations.
     */
    int ds, ds_vacant;

    /**
     * Time needed to fully charge the battery and time left until battery depletion.
     */
    double time_charge, time_run;

    /**
     * Total number of currently available jobs (e.g. frontiers for exploration) and number of jobs in close proximity (e.g. in local costmap).
     */
    int jobs, jobs_close;

    /**
     * Distance to docking station with highest likelihood and distance to closest job.
     */
    double dist_ds, dist_job;

    /**
     * The likelihood value for going recharging. This value is used in the auctions.
     */
    double llh, l1, l2, l3, l4;

    /**
     * The weights for the weighted sum of the likelihood values l1,...,l4.
     */
    double w1, w2, w3, w4;

    /**
     * Update the likelihood value l1.
     */
    void update_l1();

    /**
     * Update the likelihood value l2.
     */
    void update_l2();

    /**
     * Update the likelihood value l3.
     */
    void update_l3();

    /**
     * Update the likelihood value l4.
     */
    void update_l4();

    /**
     * Battery state of charge (SOC) callback.
     */
    void battery_callback(const battery_mgmt::Battery::ConstPtr& msg);

    /**
     * Battery charging callback.
     */
    void charging_callback(const battery_mgmt::Charge::ConstPtr &msg);
};

#endif  /* DOCKING_COORDINATION_H */