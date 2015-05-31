#include "rendezvous.h"

// Konstructor
Rendezvous::Rendezvous()
{
    nh = new ros::NodeHandle("~");

    // parameter
    nh->param<std::string>("robot_prefix", robot_prefix, "");
    nh->param<int>("waitForResult", waitForResult, 3);
    nh->param<std::string>("move_base_frame",move_base_frame,"map");

    // TODO: parameters for all waiting times / durations needed for rendezvous

    this->rendezvousPoints = new std::vector<RendezvousPoint>;

    // create clients for explorer services
    std::string service = robot_prefix + std::string("/explorer/switchExplRend");
    expl_client = nh->serviceClient<explorer::switchExplRend>(service.c_str());

    service = robot_prefix + std::string("/explorer/getPosition");
    position_client = nh->serviceClient<explorer::getPosition>(service.c_str());

    service = robot_prefix + std::string("/explorer/MissionFinished");
    mission_client = nh->serviceClient<explorer::MissionFinished>(service.c_str());

    // create clients for ad_hoc_communication services
    service = robot_prefix + std::string("/adhoc_communication/send_string");
    hallo_client = nh->serviceClient<adhoc_communication::SendString>(service.c_str());

    //service = robot_prefix + std::string("/adhoc_communication/SendRendezvous");
    //rendezvous_client = nh->serviceClient<adhoc_communication::SendRendezvous>(service.c_str());

    // wait till information about home is available
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

    // add home point as first element to vector of rendezvous points
    addRendezvous(home_x, home_y);
}

void Rendezvous::exploreRobot(){
    // start explorer
    // count number of rendezvous
    // set exploration time accordingly
    // explore for exploration time
    // timeout -> callbackfunction, return to rendezvous


    numberMeetings++;

    // explore till timeout
    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = true;
    if(expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
    }

    double explorationTime = (80.0 + (10*numberMeetings));

    ROS_DEBUG("explore %f sec!", explorationTime);

    ros::Duration(explorationTime).sleep();

    callbackMoveToRendezvous();

}

void Rendezvous::callbackMoveToRendezvous() {
    // stop explorer
    // remember point as next rendezvous
    // return to current rendezvous
    // call hallo() to check if relay in communication range
    // rendezvous() with relay    OR   wait at rendezvous till timeout
    // if mission is not finished explore further

    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;
    if(expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
    }

    // get current robot position
    double rend_x;
    double rend_y;
    explorer::getPosition srv;
    srv.request.name = "currentPos";
    if(this->position_client.call(srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'getPosition'");
    }
    else
    {
        rend_x = srv.response.x;
        rend_y = srv.response.y;
    }

    // add current position to rendezvous points vector
    addRendezvous(rend_x, rend_y);

    // determine last not visited rendezvous and move there
    for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
        i != rendezvousPoints->end(); ++i)
    {
        if(i->visited == false){
            currentRendezvous = *i;
            ROS_DEBUG("timeout -> move to x = %f , y = %f", currentRendezvous.x, currentRendezvous.y);
            break;
        }
    }

    visualize_rendezvous(currentRendezvous.x, currentRendezvous.y);
    if(move_robot(currentRendezvous.x, currentRendezvous.y)){
        ROS_DEBUG("explorer at Rendezvous, waiting for relay...");
        visualize_visited_rendezvous(currentRendezvous.x, currentRendezvous.y);
    }
    else
    {
        ROS_DEBUG("EXPLORER failed to move to rendezvous!");
        visualize_unreachable_rendezvous(currentRendezvous.x, currentRendezvous.y);

        // get current robot position
        double pos_x, pos_y;
        explorer::getPosition srv;
        srv.request.name = "currentPos";
        if(this->position_client.call(srv) == false)
        {
            ROS_ERROR("Failed to call explorer service 'getPosition'");
        }
        else
        {
            pos_x = srv.response.x;
            pos_y = srv.response.y;
        }
        ROS_DEBUG("explorer at ( %f / % f)", pos_x, pos_y);

        // stay here, maybe near enough to be able to communicatio with relay anyway

    }

    // TODO (in any case):
    // - bool teamInRange = hallo()
    // - if teamInRange -> rendezvous()
    // - else: rendezvous failed -> recovery strategy

    // mark rendezvous as visited
    // case 1: robot reached rendezvous -> marking ok
    // case 2: robot failed to reach rendezvous -> probably rendezvous not reachable, robot should not try again ..
    for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
        i != rendezvousPoints->end(); ++i)
    {
        if(i->x == currentRendezvous.x && i->y == currentRendezvous.y){
            i->visited = true;
        }
    }

    explorer::MissionFinished mission_srv;
    if(this->mission_client.call(mission_srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'missionFinished'");
    }
    else if(mission_srv.response.finished == false)
    {
        //ROS_DEBUG("EXPLORER will explore further because mission is not finished.");
        this->exploreRobot();
    }
    else
    {
        ROS_DEBUG("EXPLORER :  mission finished !!!!");
        //ROS_DEBUG("will start explorer again, should return home");
        srv_msg.request.explore = true;
        if(expl_client.call(srv_msg) == false)
        {
            ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
        }
    }


}

void Rendezvous::relayRobot(){
    // stop explorer
    // wait at home till explorer returns for the first time
    // while (mission is not finished)
        // wait at home for ? sec
        // move to rendezvous
        // call hallo() to check if explorer in range
        // rendezvous() with explorer   OR  wait at rendezvous till timeout
        // move back home

    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;

    if(this->expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'");
    } else {
        //ROS_DEBUG("RELAY stopped explorer!");
    }

    // TODO:
    // -    wait at home till explorer returns
    // -    agree next rendezvous with explorer
    // -    store it in rendezvous vector

    explorer::MissionFinished mission_srv;
    if(this->mission_client.call(mission_srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'missionFinished'");
    } else {
        missionFinished = mission_srv.response.finished;
    }

    while(missionFinished == false)
    {
        // determine last not visited rendezvous
        for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
            i != rendezvousPoints->end(); ++i)
        {
            if(i->visited == false)
            {
                currentRendezvous = *i;
                ROS_DEBUG("RELAY : move to x = %f , y = %f", currentRendezvous.x, currentRendezvous.y);
                break;
            }
        }

        visualize_rendezvous(currentRendezvous.x, currentRendezvous.y);

        // move to current rendezvous
        if(move_robot(currentRendezvous.x, currentRendezvous.y) == true)
        {
            visualize_visited_rendezvous(currentRendezvous.x, currentRendezvous.y);

            ROS_DEBUG("relay at rendezvous, waiting for explorer...");

        }
        else
        {
            ROS_INFO("Failed to move relay to rendezvous!");
            visualize_unreachable_rendezvous(currentRendezvous.x, currentRendezvous.y);

            // get current robot position
            double pos_x;
            double pos_y;
            explorer::getPosition srv;
            srv.request.name = "currentPos";
            if(this->position_client.call(srv) == false)
            {
                ROS_ERROR("Failed to call explorer service 'getPosition'");
            }
            else
            {
                pos_x = srv.response.x;
                pos_y = srv.response.y;
            }
            ROS_DEBUG("relay at ( %f / %f ) now", pos_x, pos_y);

            // stay here, maybe near enough to communicate
            // TODO: better solution
        }

        // mark rendezvous as visited (in any case, relay should not move there again)
        for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
            i != rendezvousPoints->end(); ++i)
        {
            if(i->x == currentRendezvous.x && i->y == currentRendezvous.y){
                i->visited = true;
            }
        }

        // TODO ... in any case for a specific time, if not really at rendezvous, maybe in communication range anyway
        // -   call bool teamInRange = hallo()
        // -   if teamInRange ... call rendezvous()
        // -   else rendezvous not successful -> recovery strategy!?

        // after rendezvous or timeout: move home again
        if(move_robot(home_x, home_y))
        {
            ROS_DEBUG("relay at home, it will wait here for 20 sec.");
            ros::Duration(20.0).sleep();
            // TODO: make parameters for all waiting / exploring durations
        }
        else
        {
            ROS_ERROR("Failed to move robot home!");
            // TODO: recovery strategy
        }

        if(this->mission_client.call(mission_srv) == false)
        {
            ROS_ERROR("Failed to call explorer service 'missionFinished'");
        } else {
            missionFinished = mission_srv.response.finished;
        }
    }

    ROS_DEBUG("RELAY :  mission finished !!!!");

}

void Rendezvous::callback_hallo(const adhoc_communication::RecvString msg)
{
    ROS_DEBUG("received %s message from: %s", msg.data.c_str(), msg.src_robot.c_str());
    teamMemberInRange = true;
//    ROS_DEBUG("compare %s with %s", msg.src_robot.c_str(), myBuddy_prefix.c_str());

//    std::string temp = msg.src_robot.c_str();
//    if(temp.compare(1,7, myBuddy_prefix.c_str()) == 0){
//        ROS_DEBUG("teamMemberInRange = %d", teamMemberInRange);
//        teamMemberInRange = true;
//        ROS_DEBUG("equal; teamMemberInRange = %d", teamMemberInRange);

//    }
//    else
//    {
//        ROS_DEBUG("not equal");
//    }
}

bool Rendezvous::hallo()
{
    // broadcast a 'hallo' message on topic 'checkCommunicationRange' to check if any other robot is in communication range
    // returns (true) if there was an answer from 'myBuddy' robot in maxWaitTime seconds
    //         (false) if 'myBuddy' didn't answer in maxWaitTime seconds

    // TODO: maxWaitTime: parameter!!

    double maxWaitTime = 10;
    double start = ros::Time::now().toSec();
    timeoutAtRendezvous = false;
    teamMemberInRange = false;      // private variable, changed in callback_hallo()

    adhoc_communication::SendString hallo_msg;
    hallo_msg.request.dst_robot = "";                         // broadcast
    hallo_msg.request.data = "hallo";
    hallo_msg.request.topic = myBuddy_prefix + "/rendezvous/checkCommunicationRange";

    ros::Rate r(10);                                     // rate = 5 hz
    while(ros::ok() && timeoutAtRendezvous == false)
    {
        if(hallo_client.call(hallo_msg) == true)
        {
            //ROS_DEBUG("Successfully called 'sendString' service to broadcast 'hallo'!");
            // TODO: check response.status to know if transmission successfull
        }
        else
        {
            ROS_ERROR("Failed to call 'sendString' service of ad_hoc_communication node!");
        }

        // TODO callback only useful if in short time

        // get callback
        ros::spinOnce();
        if(teamMemberInRange == true)
        {
            ROS_DEBUG("second robot is in communication range!");
            return true;
        }
        else
        {
            //ROS_DEBUG("second robot is not in communication range!");
        }

        if((ros::Time::now().toSec() - start) == maxWaitTime)
        {
            ROS_DEBUG("tried to reach my other team members for %f sec. But failed!", maxWaitTime);
            timeoutAtRendezvous = true;
        }

        r.sleep();
    }

    return false;
}

void Rendezvous::test_hallo()
{
    double explorationTime = 30.0;

    ros::Duration(explorationTime).sleep();

    // stop explorer
    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;

    if(this->expl_client.call(srv_msg) == false)
    {
        ROS_DEBUG("In Rendezvous::commute()");
        ROS_ERROR("Failed to call explorer service 'switchExplRend'");
    } else {
        //ROS_DEBUG("stopped explorer!");
    }

     //get current robot position (:= rendezvous)
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
        ROS_DEBUG("robot stopped at point x = %f \t y = %f", rend_x, rend_y);
    }

     //continue moving between home and rendezvous
    ROS_DEBUG("Continue moving between home and rendezvous!, try if in communication range of other robot at home");
    while(true)
    {
        visualize_rendezvous(home_x, home_y);
        if(this->move_robot(home_x, home_y) == false)
        {
            ROS_ERROR("Failed to move robot to home position");
            srv_msg.request.explore = true;

            if(this->expl_client.call(srv_msg) == false)
            {
                ROS_ERROR("Failed to call explorer service 'switchExplRend'");
            } else {
                ROS_DEBUG("start explorer again");
            }
            break;
        }
        else
        {
            ROS_DEBUG("robot at home, try to check if buddy in range!");
            this->hallo();
        }

        visualize_rendezvous(rend_x, rend_y);
        if(this->move_robot(rend_x, rend_y) == false)
        {
            ROS_ERROR("Failed to move robot to rendezvous");
            srv_msg.request.explore = true;

            if(this->expl_client.call(srv_msg) == false)
            {
                ROS_ERROR("Failed to call explorer service 'switchExplRend'");
            } else {
                ROS_DEBUG("start explorer again.");
            }
            break;
        }
        else
        {
            ROS_DEBUG("robot at rendezvous!");
            ros::Duration(5.0).sleep();
        }

    }

}

void Rendezvous::rendezvous(){

    // new RendezvousMsg [x,y,z,utility, time, ownID, destinationID, status = request / ack, finished] == ack RendezvousMsg

    //if(iAm == EXPLORER)
    //{
        // send last entry in rendezvousPoint vector as next rendezvous to relay
        // wait upon ack
        // overwrite/delete/don't change last point in vector according to relays answer
    //}
    //else if(iAm == RELAY)
    //{
        // get possible next rendezvous from msg from explorer
        // check if new rendezvous (little changed) is reachable
        // yes ... send ack
        // no ... send other possible point
        // add new rendezvous (little changed) to own list of points
    //}

    // wait some time, till all information about maps are exchanged for sure
    ros::Duration(30.0).sleep();

    // TODO:
    // - parameter for sleeping time
    // - solution if any point is not reachable for relay
}


void Rendezvous::visualize_rendezvous(double x, double y)
{
    rendezvousPoint.header.seq = rendezvous_point_message++;
    rendezvousPoint.header.stamp = ros::Time::now();
    rendezvousPoint.header.frame_id = move_base_frame;
    rendezvousPoint.point.x = x;
    rendezvousPoint.point.y = y;

    ros::NodeHandle nh("rendezvous");
    pub_rendezvous = nh.advertise < geometry_msgs::PointStamped > ("rendezvous", 100, true);
    pub_rendezvous.publish < geometry_msgs::PointStamped > (rendezvousPoint);
}

void Rendezvous::visualize_visited_rendezvous(double x, double y)
{
    visited_rendezvousPoint.header.seq = visited_rendezvous_point_message++;
    visited_rendezvousPoint.header.stamp = ros::Time::now();
    visited_rendezvousPoint.header.frame_id = move_base_frame;
    visited_rendezvousPoint.point.x = x;
    visited_rendezvousPoint.point.y = y;

    ros::NodeHandle nh("visited_rendezvous");
    pub_visited_rendezvous = nh.advertise < geometry_msgs::PointStamped > ("visited_rendezvous", 100, true);
    pub_visited_rendezvous.publish < geometry_msgs::PointStamped > (visited_rendezvousPoint);
}

void Rendezvous::visualize_unreachable_rendezvous(double x, double y)
{
    unreachable_rendezvousPoint.header.seq = unreachable_rendezvous_point_message++;
    unreachable_rendezvousPoint.header.stamp = ros::Time::now();
    unreachable_rendezvousPoint.header.frame_id = move_base_frame;
    unreachable_rendezvousPoint.point.x = x;
    unreachable_rendezvousPoint.point.y = y;

    ros::NodeHandle nh("unreachable_rendezvous");
    pub_unreachable_rendezvous = nh.advertise < geometry_msgs::PointStamped > ("unreachable_rendezvous", 100, true);
    pub_unreachable_rendezvous.publish < geometry_msgs::PointStamped > (unreachable_rendezvousPoint);
}

void Rendezvous::addRendezvous(double new_x, double new_y)
{
    RendezvousPoint rend;
    rend.x = new_x;
    rend.y = new_y;
    rend.visited = false;
    rendezvousPoints->push_back(rend);
}

void Rendezvous::printRendezvousPoints()
{
    int n = 1;
    for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
        i != rendezvousPoints->end(); ++i)
    {
        ROS_DEBUG("%d - th Point: x = %f \t y = %f \t visited = %d \n", n, i->x, i->y, i->visited);
        n++;
    }

}

// not used yet
bool Rendezvous::reachable(double x, double y)
{
    ROS_DEBUG("Determine if goal ( %f / %f) is reachable", x, y);
    MoveBaseClient testGoal("move_base", true);
    while(!testGoal.waitForServer(ros::Duration(10.0)));
    move_base_msgs::MoveBaseGoal goal_msg;

    goal_msg.target_pose.header.frame_id = move_base_frame;
    goal_msg.target_pose.pose.position.x = x;
    goal_msg.target_pose.pose.position.y = y;
    goal_msg.target_pose.pose.position.z = 0;
    goal_msg.target_pose.pose.orientation.x = 0;
    goal_msg.target_pose.pose.orientation.y = 0;
    goal_msg.target_pose.pose.orientation.z = 0;
    goal_msg.target_pose.pose.orientation.w = 1;

    testGoal.sendGoal(goal_msg);

    testGoal.waitForResult(ros::Duration(waitForResult));

    while (testGoal.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_DEBUG(" current state: %s", testGoal.getState().toString().c_str());
        if (testGoal.getState() == actionlib::SimpleClientGoalState::ABORTED)
        {

            return false;
        }
    }
    return true;
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
        //ROS_DEBUG(" current state: %s", ac.getState().toString().c_str());
        ros::Duration(0.5).sleep();
    }

    while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
    {
        //ROS_DEBUG(" current state: %s", ac.getState().toString().c_str());
        ros::Duration(0.5).sleep();

    }

    while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        //ROS_DEBUG(" current state: %s", ac.getState().toString().c_str());
        if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_DEBUG(" current state: %s", ac.getState().toString().c_str());
            return false;
        }
    }
    return true;
}

// Test function - stop explorer node and start it again
void Rendezvous::stopStartExplorer()
{
    // stop explorer after 'explorationTime' sec,
    // wait for 'sleepTime' sec and start explorer again
    // till total space is explored


    int explorationTime = 80;
    int sleepTime = 10;

    explorer::MissionFinished mission_srv;
    if(this->mission_client.call(mission_srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'missionFinished'");
    } else {
        missionFinished = mission_srv.response.finished;
    }

    double pos_x;
    double pos_y;
    explorer::getPosition srv;
    srv.request.name = "currentPos";

    while(missionFinished == false)
    {
        //ROS_DEBUG("mission not completed, robot will explore for %d sec!", explorationTime);
        ros::Duration(explorationTime).sleep();

        //ROS_DEBUG("%s  : stop explorer!", robot_prefix.c_str());
        explorer::switchExplRend srv_msg;
        srv_msg.request.explore = false;

        if(expl_client.call(srv_msg) == false)
        {
            ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
        } else {

            if(this->position_client.call(srv) == false)
            {
                ROS_ERROR("Failed to call explorer service 'getPosition'");
            }
            else
            {
                pos_x = srv.response.x;
                pos_y = srv.response.y;
            }

            ROS_DEBUG("%s stopped at point x = %f \t y = %f ", robot_prefix.c_str(), pos_x, pos_y);
        }

        //ROS_DEBUG("%s : sleep for %d sec!", robot_prefix.c_str(), sleepTime);
        ros::Duration(sleepTime).sleep();

        ROS_DEBUG("%s  : explore again!", robot_prefix.c_str());
        srv_msg.request.explore = true;
        if(expl_client.call(srv_msg) == false)
        {
            ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
        }

        if(this->mission_client.call(mission_srv) == false)
        {
            ROS_ERROR("Failed to call explorer service 'missionFinished'");
        } else {
            missionFinished = mission_srv.response.finished;
        }
    }

     ROS_DEBUG("%s : mission finished!", robot_prefix.c_str());
}

// stop robot after exporationTime seconds and let it commute between point where it stopped and home
void Rendezvous::commute(){

    double explorationTime = 80.0;

    ros::Duration(explorationTime).sleep();

    // stop explorer
    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;

    if(this->expl_client.call(srv_msg) == false)
    {
        ROS_DEBUG("In Rendezvous::commute()");
        ROS_ERROR("Failed to call explorer service 'switchExplRend'");
    } else {
        //ROS_DEBUG("stopped explorer!");
    }

     //get current robot position (:= rendezvous)
    double rend_x;
    double rend_y;
    explorer::getPosition srv;
    srv.request.name = "";
    if(this->position_client.call(srv) == false)
    {
        ROS_DEBUG("In Rendezvous::commute()");
        ROS_ERROR("Failed to call explorer service 'getPosition'");
    }
    else
    {
        rend_x = srv.response.x;
        rend_y = srv.response.y;
        ROS_DEBUG("robot stopped at point x = %f \t y = %f", rend_x, rend_y);
    }

     //continue moving between home and rendezvous
    ROS_DEBUG("Continue moving between home and rendezvous!");
    while(true)
    {
        visualize_rendezvous(home_x, home_y);
        if(this->move_robot(home_x, home_y) == false)
        {
            ROS_ERROR("Failed to move robot to home position");
            srv_msg.request.explore = true;

            if(this->expl_client.call(srv_msg) == false)
            {
                ROS_ERROR("Failed to call explorer service 'switchExplRend'");
            } else {
                ROS_DEBUG("start explorer again");
            }
            break;
        }
        else
        {
            ROS_DEBUG("robot at home!");
            ros::Duration(5.0).sleep();
        }

        visualize_rendezvous(rend_x, rend_y);
        if(this->move_robot(rend_x, rend_y) == false)
        {
            ROS_ERROR("Failed to move robot to rendezvous");
            srv_msg.request.explore = true;

            if(this->expl_client.call(srv_msg) == false)
            {
                ROS_ERROR("Failed to call explorer service 'switchExplRend'");
            } else {
                ROS_DEBUG("start explorer again.");
            }
            break;
        }
        else
        {
            ROS_DEBUG("robot at rendezvous!");
            ros::Duration(5.0).sleep();
        }

    }
}

// explorer first version, works with --enable-comm
void Rendezvous::exploreRobot_unlimited()
{
    // count number of rendezvous and
    // increase exploration time
    // starts with 1, so first exploration is 90 sec

    numberMeetings++;

    // explore till timeout
    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = true;
    if(expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
    }

    double explorationTime = (80.0 + (10*numberMeetings));

    ROS_DEBUG("explore %f sec!", explorationTime);

    ros::Duration(explorationTime).sleep();

    callbackMoveToRendezvous_unlimited();
}

// explorer first version, works with --enable-comm
void Rendezvous::callbackMoveToRendezvous_unlimited()
{
    //ROS_DEBUG("timeout -> move back to rendezvous");

    // stop explorer
    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;
    if(expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
    }

    // get current robot position
    double rend_x;
    double rend_y;
    explorer::getPosition srv;
    srv.request.name = "currentPos";
    if(this->position_client.call(srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'getPosition'");
    }
    else
    {
        rend_x = srv.response.x;
        rend_y = srv.response.y;
    }

    // add current position to rendezvous points vector
    addRendezvous(rend_x, rend_y);

    // publish current point as next rendezvous
    rendezvous::RendezvousPoint msg;
    msg.x = rend_x;
    msg.y = rend_y;
    pub.publish(msg);
    // ROS_DEBUG("EXPLORER published new Rendezvous Point: x = %f, y = %f", msg.x, msg.y);

    visualize_rendezvous(rend_x, rend_y);

    // determine last not visited rendezvous, move there and wait for 25 sec at rendezvous
    for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
        i != rendezvousPoints->end(); ++i)
    {
        if(i->visited == false){
            currentRendezvous = *i;
            ROS_DEBUG("timeout -> move to x = %f , y = %f", currentRendezvous.x, currentRendezvous.y);
            break;
        }
    }

    if(currentRendezvous.x == home_x && currentRendezvous.y == home_y)
    {
        ROS_DEBUG("timeout, but rendezvous is home, explorer will not move there!");
    }
    else
    {
        visualize_rendezvous(currentRendezvous.x, currentRendezvous.y);
        if(move_robot(currentRendezvous.x, currentRendezvous.y)){
            ROS_DEBUG("EXPLORER at Rendezvous, wait here for 5 sec!");
            ros::Duration(5).sleep();
            visualize_visited_rendezvous(currentRendezvous.x, currentRendezvous.y);
        }
        else
        {
            ROS_DEBUG("EXPLORER failed to move to rendezvous!");
            visualize_unreachable_rendezvous(currentRendezvous.x, currentRendezvous.y);
        }
     }

    // mark rendezvous as visited
    // case 1: robot reached rendezvous -> marking ok
    // case 2: robot failed to reach rendezvous -> probably rendezvous not reachable, robot should not try again .. TODO
    // case 3: rendezvous == home -> marking ok, bc explore robot should not return home till mission is finished
    for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
        i != rendezvousPoints->end(); ++i)
    {
        if(i->x == currentRendezvous.x && i->y == currentRendezvous.y){
            i->visited = true;
        }
    }

    ROS_DEBUG("--------------- EXPLORER: Vector of rendezvous points --------------------- ");
    printRendezvousPoints();
    ROS_DEBUG("--------------------------------------------------------------------------------------");

    // (if mission is not finished) explore again till next timeout

    explorer::MissionFinished mission_srv;
    if(this->mission_client.call(mission_srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'missionFinished'");
    }
    else if(mission_srv.response.finished == false)
    {
        //ROS_DEBUG("EXPLORER will explore further because mission is not finished.");
        this->exploreRobot_unlimited();
    }
    else                                            // if mission is finished, start explorer node again
    {
        ROS_DEBUG("EXPLORER :  mission finished !!!!");
        ROS_DEBUG("will start explorer again, should return home");
        srv_msg.request.explore = true;
        if(expl_client.call(srv_msg) == false)
        {
            ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
        }
    }


}

// relay first version, works with --enable-comm
void Rendezvous::relayRobot_unlimited()
{
    // stop explorer
    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;

    if(this->expl_client.call(srv_msg) == false)
    {
        ROS_DEBUG("In Rendezvous::relayRobot()");
        ROS_ERROR("Failed to call explorer service 'switchExplRend'");
    } else {
        //ROS_DEBUG("RELAY stopped explorer!");
    }

    explorer::MissionFinished mission_srv;
    if(this->mission_client.call(mission_srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'missionFinished'");
    } else {
        missionFinished = mission_srv.response.finished;
    }

    while(missionFinished == false)
    {
        // if new rendezvous available, get it
        ros::spinOnce();

        // move to last not visited rendezvous or stay if there isn't any new rendezvous
        for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
            i != rendezvousPoints->end(); ++i)
        {
            if(i->visited == false)
            {
                currentRendezvous = *i;
                ROS_DEBUG("RELAY : move to x = %f , y = %f", currentRendezvous.x, currentRendezvous.y);
                break;
            }
            else
            {
                // no new rendezvous available -> stay
                ros::Duration(10).sleep();
            }
        }

        visualize_rendezvous(currentRendezvous.x, currentRendezvous.y);
        if(move_robot(currentRendezvous.x, currentRendezvous.y) == true)
        {
            // mark rendezvous as visited
            for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
                i != rendezvousPoints->end(); ++i)
            {
                if(i->x == currentRendezvous.x && i->y == currentRendezvous.y){
                    i->visited = true;
                }
            }

            visualize_visited_rendezvous(currentRendezvous.x, currentRendezvous.y);

            ROS_DEBUG("RELAY at rendezvous; return home in 10 sec.");
            ros::Duration(10).sleep();

            // TODO: wait here for EXPLORE robot

            visualize_rendezvous(home_x, home_y);
            if(move_robot(home_x, home_y))
            {
                 //ROS_DEBUG("RELAY at home, it will wait here for 20 sec.");

                 if(this->mission_client.call(mission_srv) == false)
                 {
                     ROS_ERROR("Failed to call explorer service 'missionFinished'");
                 } else {
                     missionFinished = mission_srv.response.finished;
                 }


            }

            ros::Duration(20).sleep();

        }
        else
        {
            ROS_ERROR("Failed to move robot to rendezvous!");

            visualize_unreachable_rendezvous(currentRendezvous.x, currentRendezvous.y);

            // mark rendezvous as visited (relay should not try to reach this rendezvous again, bc maybe its not reachable)
            for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
                i != rendezvousPoints->end(); ++i)
            {
                if(i->x == currentRendezvous.x && i->y == currentRendezvous.y){
                    i->visited = true;
                }
            }

            ros::Duration(20).sleep();
        }

        if(this->mission_client.call(mission_srv) == false)
        {
            ROS_ERROR("Failed to call explorer service 'missionFinished'");
        } else {
            missionFinished = mission_srv.response.finished;
        }
    }

    ROS_DEBUG("RELAY :  mission finished !!!!");
}

// function of relay; will be called when its explorer publishes their next rendezvous, works with --enable-comm
void Rendezvous::new_Rendezvous_available(const rendezvous::RendezvousPointConstPtr& msg)
{
    ROS_DEBUG("RELAY received Point x = %f, y = %f", msg->x, msg->y);

    // first change point a little to avoid crush
    // check if manipulated point is reachable
    // then add it to own vector of rendezvous

//    bool  pointOK = false;
//    double delta = 0.0001;
//    int i = 0;

//    while(pointOK == false && i <= 6)
//    {
//        i++;
//        pointOK = reachable((msg->x)+delta, (msg->y)+delta);
//        ROS_DEBUG("Point ( %f / %f ) is not reachable for relay.", (msg->x)+delta, (msg->y)+delta);
//        delta = delta*i;
//    }

//    if(pointOK == false){
//        ROS_WARN("Given rendezvous R(%f / %f) is not reachable for RELAY!", msg->x, msg->y);
//    } else {
//        addRendezvous((msg->x)+delta, (msg->y)+delta);
//    }

    double delta = 0.0001;
    addRendezvous((msg->x)+delta, (msg->y)+delta);

    ROS_DEBUG("RELAY: Vector of rendezvous points");
    printRendezvousPoints();
}
