#ifndef DOCKING_H
#define DOCKING_H

#include <energy_mgmt/battery_state.h>

class docking
{
public:
    /**
     * Constructor.
     */
    docking();

    /**
     * Update the likelihood value.
     * @param energy_mgmt::battery_state battery_state: The current state of the battery.
     */
    void update_llh(energy_mgmt::battery_state battery_state);

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
     * Likelihood values for going recharging. The llh value is used in the auctions.
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
};

#endif  /* DOCKING_H */