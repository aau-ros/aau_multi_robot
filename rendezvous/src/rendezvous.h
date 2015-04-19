#ifndef RENDEZVOUS_H
#define RENDEZVOUS_H

#include "ros/ros.h"
#include <tf/tf.h>
#include <explorer/switchExplRend.h>
#include <explorer/getPosition.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Rendezvous
{
public:
	Rendezvous();
    ros::NodeHandle *nh;
	int iAm;
    tf::Stamped<tf::Pose> robotPose;
    bool move_robot(double x, double y);
    void relayRobot();
    std::string robot_prefix;
    int waitForResult;
    costmap_2d::Costmap2DROS* costmap2d_local;
    costmap_2d::Costmap2DROS* costmap2d_global;
    costmap_2d::Costmap2DROS* costmap2d_local_size;
    std::string move_base_frame;
    //tf::TransformListener& tf;


private:
	double home_x, home_y;
	ros::ServiceClient expl_client;
    ros::ServiceClient position_client;
};

#endif RENDEZVOUS_H

// param robot_prefix
