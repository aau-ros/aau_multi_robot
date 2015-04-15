#include "rendezvous.h"

Rendezvous::Rendezvous()
{
    nh = new ros::NodeHandle("~");

    // parameter
    nh->param<std::string>("robot_prefix", robot_prefix, "");
    nh->param<int>("waitForResult", waitForResult, 3);

    // create client for explorer service
    std::string service = robot_prefix + std::string("/explorer/switchExplRend");
    expl_client = nh->serviceClient<explorer::switchExplRend>(service.c_str());

    // get and set home point
    this->home_x = robotPose.getOrigin().getX();
    this->home_y = robotPose.getOrigin().getY();

}

void Rendezvous::relayRobot()
{
    // wait till robots are moving
    ros::Duration(30).sleep();

    // call service to stop explorer
    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;

    if(this->expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service");
    }

    //remember actual point as rendezvous
    if(!costmap2d_local->getRobotPose(robotPose)){
        ROS_ERROR("Failed to get RobotPose");
    }
    double rend_x = this->robotPose.getOrigin().getX();
    double rend_y = this->robotPose.getOrigin().getY();


    // continue moving between home and rendezvous
    while(true)
    {
        if(this->move_robot(this->home_x, this->home_y) == false)
        {
            ROS_DEBUG("Failed to move robot to home position");
        }

        ros::Duration(5).sleep();

        if(this->move_robot(rend_x, rend_y) == false)
        {
            ROS_DEBUG("Failed to move robot to rendezvous");
        }

    }
}

// method from explorer
bool Rendezvous::move_robot(double position_x, double position_y)
{

    /*
     * Move the robot with the help of an action client. Goal positions are
     * transmitted to the robot and feedback is given about the actual
     * driving state of the robot.
     */
    /*if (!costmap2d_local->getRobotPose(robotPose))
    {
        ROS_ERROR("Failed to get RobotPose");
    }*/

    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(10.0)));

    move_base_msgs::MoveBaseGoal goal_msgs;

    //goal_msgs.target_pose.header.frame_id = move_base_frame;
    goal_msgs.target_pose.pose.position.x = position_x;
    goal_msgs.target_pose.pose.position.y = position_y;
    goal_msgs.target_pose.pose.position.z = 0;
    goal_msgs.target_pose.pose.orientation.x = 0;
    goal_msgs.target_pose.pose.orientation.y = 0;
    goal_msgs.target_pose.pose.orientation.z = 0;
    goal_msgs.target_pose.pose.orientation.w = 1;

    ac.sendGoal(goal_msgs);

    ac.waitForResult(ros::Duration(waitForResult)); //waitForResult ... paramter
    while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
    {
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Not longer PENDING");

    while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
    {
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Not longer ACTIVE");

    while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_INFO("ABORTED");
            return false;
        }
    }
    ROS_INFO("TARGET REACHED");
    return true;
}

