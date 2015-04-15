#ifndef RENDEZVOUS_H
#define RENDEZVOUS_H

#include "ros/ros.h"
#include <tf/tf.h>
#include <explorer/switchExplRend.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Rendezvous
{
public:
	Rendezvous();
	int iAm;
    tf::Stamped<tf::Pose> robotPose;
    bool move_robot(double x, double y);
    void relayRobot();
    std::string robot_prefix;
    int waitForResult;

private:
	double home_x, home_y;
	ros::ServiceClient expl_client;
    ros::NodeHandle *nh;
};

#endif RENDEZVOUS_H

// param robot_prefix
