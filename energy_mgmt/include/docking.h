#ifndef DOCKING_H
#define DOCKING_H

#include <navfn/navfn_ros.h>
#include <boost/thread/mutex.hpp>
#include <adhoc_communication/MmListOfPoints.h>
#include <adhoc_communication/ExpFrontier.h>
#include <energy_mgmt/battery_state.h>

using namespace std;

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
     * Service client for sending an auction.
     */
    ros::ServiceClient sc_send_auction;

    /**
     * Subscribers for the required topics.
     */
    ros::Subscriber sub_robot_positions, sub_frontiers_positions, sub_docking_positions, sub_auction;

    /**
     * Callbacks for the subscribed topics.
     */
    void cb_position_robots(const adhoc_communication::MmListOfPoints::ConstPtr& msg);
    void cb_position_frontiers(const adhoc_communication::ExpFrontier::ConstPtr& msg);
    void cb_position_docking_stations(const adhoc_communication::MmListOfPoints::ConstPtr& msg);
    void cb_auction(const adhoc_communication::EmAuction::ConstPtr& msg);

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
     * Start or respond to an auction for a docking station.
     * @param int docking_station: The docking station that this robot wants to charge at.
     * @param int id: The ID of the auction. If it is left to default (i.e. 0), then a new auction is started. Otherwise this robot participates at the given auction.
     * @return bool: Success of auction.
     */
    bool auction(int docking_station, int id=0);

    /**
     * Send an auction to a multicast group.
     * @param string multicast_group: The multicast group to send the auction to.
     * @param adhoc_communication::EmAuction auction: The auction that will be sent.
     * @param string topic: The topic name which the auction will be published in.
     * @return bool: Success of transmission.
     */
    bool docking::auction_send_multicast(string multicast_group, adhoc_communication::EmAuction auction, string topic);

    /**
     * Navigation function object for calculating paths.
     */
    navfn::NavfnROS nav;

    /**
     * Name and id of the robot.
     */
    string robot_name, robot_prefix;
    int robot_id;

    /**
     * Positions of robots.
     */
    adhoc_communication::MmListOfPoints position_robots;

    /**
     * Positions of frontiers.
     */
    adhoc_communication::MmListOfPoints position_frontiers;

    /**
     * Positions of docking stations.
     */
    adhoc_communication::MmListOfPoints position_docking_stations;

    /**
     * Mutexes for locking lists.
     */
    boost::mutex mutex_position_robots;

    /**
     * ID of the last auction.
     */
    int auction_id;

    /**
     * Total number of robots and number of active robots (i.e. robots that are neither charging nor idle).
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
};

#endif  /* DOCKING_H */
