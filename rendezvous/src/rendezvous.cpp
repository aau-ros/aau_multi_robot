#include "rendezvous.h"

Rendezvous::Rendezvous()
{
    nh = new ros::NodeHandle("~");

    // parameter
    nh->param<std::string>("robot_prefix", robot_prefix, "");
    nh->param<int>("waitForResult", waitForResult, 3);
    nh->param<std::string>("move_base_frame",move_base_frame,"map");

    this->rendezvousPoints = new std::vector<RendezvousPoint>;

    // create client for explorer service
    std::string service = robot_prefix + std::string("/explorer/switchExplRend");
    expl_client = nh->serviceClient<explorer::switchExplRend>(service.c_str());

    // create client for explorer service
    service = robot_prefix + std::string("/explorer/getPosition");
    position_client = nh->serviceClient<explorer::getPosition>(service.c_str());

    ros::Duration(20).sleep();

    // get and set home point
    explorer::getPosition srv;
    srv.request.name = "home";
    if((this->position_client.call(srv)) == false)
    {
        ROS_ERROR("Failed to call explorer service 'getPosition'");
    }
    else
    {
        this->home_x = srv.response.x;
        this->home_y = srv.response.y;
    }

    // add home point as first element in vector of rendezvous points
    addRendezvous(home_x, home_y);
}

void Rendezvous::exploreRobot()
{
    // explore till timeout
    //nh->createTimer(ros::Duration(60), &Rendezvous::callbackMoveToRendezvous, this);

    ros::Duration(60).sleep();

    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = true;

    if(expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
    }

    callbackMoveToRendezvous();
}

void Rendezvous::callbackMoveToRendezvous()
{
    //call service to get current robot position from explorer
    double rend_x;
    double rend_y;
    explorer::getPosition srv;
    srv.request.name = "";
    if(this->position_client.call(srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'getPosition'");
    }
    else
    {
        rend_x = srv.response.x;
        rend_y = srv.response.y;
    }

    // add position to vector of rendezvousPoints
    addRendezvous(rend_x, rend_y);

    // move to last not visited rendezvous
    for(int i = 0; i<rendezvousPoints->size(); i++){
        if(rendezvousPoints->at(i).visited == false){
            currentRendezvous = rendezvousPoints->at(i);
        }
    }

    move_robot(currentRendezvous.x, currentRendezvous.y);

    // wait there
    ros::Duration(60).sleep();

    // (if mission is not finished) explore till timeout
    exploreRobot();
}

void Rendezvous::addRendezvous(double new_x, double new_y)
{
    RendezvousPoint rend;
    rend.x = new_x;
    rend.y = new_y;
    rend.visited = false;
    rendezvousPoints->push_back(rend);
}

void Rendezvous::relayRobot()
{
    // wait till robots are moving
    ros::Duration(60).sleep();

    // call service to stop explorer
    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;

    if(this->expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'");
    }

    //call service to get current robot position from explorer
    double rend_x;
    double rend_y;
    explorer::getPosition srv;
    srv.request.name = "";
    if(this->position_client.call(srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'getPosition'");
    }
    else
    {
        rend_x = srv.response.x;
        rend_y = srv.response.y;
    }

    // continue moving between home and rendezvous
    while(true)
    {
        if(this->move_robot(home_x, home_y) == false)
        {
            ROS_ERROR("Failed to move robot to home position");
        }

        ros::Duration(5).sleep();

        if(this->move_robot(rend_x, rend_y) == false)
        {
            ROS_ERROR("Failed to move robot to rendezvous");
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

    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(10.0)));

    move_base_msgs::MoveBaseGoal goal_msgs;

    goal_msgs.target_pose.header.frame_id = move_base_frame;
    goal_msgs.target_pose.pose.position.x = position_x;
    goal_msgs.target_pose.pose.position.y = position_y;
    goal_msgs.target_pose.pose.position.z = 0;
    goal_msgs.target_pose.pose.orientation.x = 0;
    goal_msgs.target_pose.pose.orientation.y = 0;
    goal_msgs.target_pose.pose.orientation.z = 0;
    goal_msgs.target_pose.pose.orientation.w = 1;

    ac.sendGoal(goal_msgs);

    ac.waitForResult(ros::Duration(waitForResult));
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

