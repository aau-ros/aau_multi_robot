#include "rendezvous.h"

Rendezvous::Rendezvous()
{
    nh = new ros::NodeHandle("~");
    rendezvousPoints = new std::vector<RendezvousPoint>;

    // parameters
    nh->param<std::string>("robot_prefix", robot_prefix, "");
    nh->param<int>("waitForResult", waitForResult, 3);
    nh->param<std::string>("move_base_frame",move_base_frame,"map");
    nh->param<double>("maxWaitTime", maxWaitTime, 40.0);                // how long robots wait at rendezvous for others
    nh->param<double>("explorationTime", explorationTime, 50.0);        // how long explorer explores before returning to rendezvous
    nh->param<double>("rendezvousTime", rendezvousTime, 10.0);          // how long robots stay at rendezvous to exchange information

    nh->setParam("maxWaitTime", 60.0);
    nh->setParam("explorationTime", 100.0);
    nh->setParam("rendezvousTime", 15.0);

    nh->getParam("maxWaitTime", maxWaitTime);
    nh->getParam("explorationTime", explorationTime);
    nh->getParam("rendezvousTime", rendezvousTime);

    // service clients
    std::string service = robot_prefix + std::string("/explorer/switchExplRend");
    expl_client = nh->serviceClient<explorer::switchExplRend>(service.c_str());

    service = robot_prefix + std::string("/explorer/getPosition");
    position_client = nh->serviceClient<explorer::getPosition>(service.c_str());

    service = robot_prefix + std::string("/explorer/MissionFinished");
    mission_client = nh->serviceClient<explorer::MissionFinished>(service.c_str());

    service = robot_prefix + std::string("/adhoc_communication/send_string");
    hallo_client = nh->serviceClient<adhoc_communication::SendString>(service.c_str());

    //service = robot_prefix + std::string("/adhoc_communication/send_rendezvous");
    //rendezvous_client = nh->serviceClient<adhoc_communication::SendRendezvous>(service.c_str());

    service = robot_prefix + std::string("/adhoc_communication/send_point");
    rendezvous_client = nh->serviceClient<adhoc_communication::SendMmPoint>(service.c_str());

    service = robot_prefix + std::string("/move_base/make_plan");
    plan_client = nh->serviceClient<nav_msgs::GetPlan>(service.c_str());

    // Subscriber
    //topic = robot_prefix + std::string("/map");
    //plan_sub = nh->subscribe(topic, 1, &Rendezvous::callback_map, this);

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

    // home is first rendezvous point
    addRendezvous(home_x, home_y);
    // visualize it to have a history length > 1 for rviz
    visualize_rendezvous(home_x, home_y);

    rendezvous_state = A;
    num_successfulRend = 0;
    num_failedRend = 0;
    num_planB = 0;

    /*
     *  CREATE LOG PATH
     * Following code enables to write the output to a file
     * which is localized at the log_path
     */
     initLogPath();
     csv_file = log_path + std::string("periodical.log");
     log_file = log_path + std::string("rendezvous.log");

     // start taking time
     time_start = ros::Time::now();

}

void Rendezvous::exploreRobot(){

    explorer::switchExplRend srv_msg;
    explorer::MissionFinished mission_srv;
    if(this->mission_client.call(mission_srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'missionFinished'");
    } else
    {
         missionFinished = mission_srv.response.finished;
    }

    while(missionFinished == false)
    {
        rendezvousSuccessful = false;

        // explore for exploration_time seconds
        srv_msg.request.explore = true;
        if(expl_client.call(srv_msg) == false)
        {
            ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
        }

        ros::Duration(15).sleep();

        if(expl_client.call(srv_msg) == false)
        {
            ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
        }

        ROS_DEBUG("explore %f sec!", explorationTime);

        ros::Duration(explorationTime).sleep();

        //stop explorer
        explorer::switchExplRend srv_msg;
        srv_msg.request.explore = false;
        if(expl_client.call(srv_msg) == false)
        {
            ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
        }

        ros::Duration(15).sleep();   // sleep to allow explorer to set all paramters

        // in case robot was moving during first service call..
        srv_msg.request.explore = false;
        if(this->expl_client.call(srv_msg) == false)
        {
            ROS_ERROR("Failed to call explorer service 'switchExplRend'");
        }

        ros::Duration(15).sleep();   // sleep to allow explorer to set all paramters


        // get current robot position
        explorer::getPosition srv;
        srv.request.name = "currentPos";
        if(this->position_client.call(srv) == false)
        {
            ROS_ERROR("Failed to call explorer service 'getPosition'");
        }
        else
        {
            possibleRendezvous.x = srv.response.x;
            possibleRendezvous.y = srv.response.y;
        }

        //ros::Duration(5).sleep();

        // determine last not visited rendezvous -> in rendezvousPoints vector are only reachable and agreed upon points!
        //rendezvousVector_mutex.lock();
        for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
            i != rendezvousPoints->end(); ++i)
        {
            if(i->visited == false){
                nextRendezvous = *i;
                ROS_DEBUG("timeout -> move to x = %f , y = %f", nextRendezvous.x, nextRendezvous.y);
                break;
            }
        }
        //rendezvousVector_mutex.unlock();

        ros::Duration(5).sleep();

        //ROS_DEBUG("explorer: list of rendezvous points after timeout:");
        //printRendezvousPoints();

        // move robot to current rendezvous
        visualize_rendezvous(nextRendezvous.x, nextRendezvous.y);

        if(move_robot(nextRendezvous.x, nextRendezvous.y)){
            ROS_DEBUG("explorer at Rendezvous, waiting %f sec for relay...", maxWaitTime);
            visualize_visited_rendezvous(nextRendezvous.x, nextRendezvous.y);

            // check if other team member is in communication range
            // yes ... start to rendezvous
            // no ... rendezvous failed -> plan B

            teamMemberInRange = false;
            teamMemberInRange = hallo(maxWaitTime);
            ros::Duration(5).sleep();

            if(teamMemberInRange == true)
            {
                ROS_DEBUG("other robot is in communication range, start to rendezvous!");
                rendezvousSuccessful = rendezvous();
            }
            else
            {
                ROS_WARN("Rendezvous failed!!!");
                ROS_DEBUG("... because of timeout %f.", maxWaitTime);
                rendezvousSuccessful = false;
            }
        }
        else
        {
            ROS_WARN("Rendezvous failed!!!");
            ROS_DEBUG("... because robot couldn't move there.");
            rendezvousSuccessful = false;
            visualize_unreachable_rendezvous(nextRendezvous.x, nextRendezvous.y);
        }

        ros::Duration(4).sleep();

        if(rendezvousSuccessful == false)
        {
            num_failedRend++;
            ROS_DEBUG("Going to call plan B!");
            planB();
        }
        else
        {
            num_successfulRend++;
            ROS_DEBUG("Rendezvous successful!");
            ROS_DEBUG("explorer: list of rendezvous points after rendezvous:");
            printRendezvousPoints();
        }

        explorer::MissionFinished mission_srv;
        if(this->mission_client.call(mission_srv) == false)
        {
            ROS_ERROR("Failed to call explorer service 'missionFinished'");
        } else
        {
             missionFinished = mission_srv.response.finished;
        }
    }

    ROS_DEBUG("explorer:  mission finished !!!!");
    //ROS_DEBUG("will start explorer again, should return home");
//    srv_msg.request.explore = true;
//    if(expl_client.call(srv_msg) == false)
//    {
//        ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
//    }

    move_robot(home_x, home_y);
    indicateSimulationEnd();
}

void Rendezvous::relayRobot(){

    ROS_DEBUG("relay explores %f sec! First rendezvous will be at home!", explorationTime);

    ros::Duration(explorationTime).sleep();

    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;
    if(this->expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'");
    }
    ros::Duration(15).sleep();   // sleep to allow explorer to set all paramters

    // in case robot was moving during first service call..
    srv_msg.request.explore = false;
    if(this->expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'");
    }
    ros::Duration(15).sleep();   // sleep to allow explorer to set all paramters

    explorer::MissionFinished mission_srv;
    if(this->mission_client.call(mission_srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'missionFinished'");
    } else {
        missionFinished = mission_srv.response.finished;
    }

    possibleRendezvous.x = home_x;
    possibleRendezvous.y = home_y;

    while(missionFinished == false)
    {
        rendezvousSuccessful = false;

        // determine last not visited rendezvous
        //rendezvousVector_mutex.lock();
        for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
            i != rendezvousPoints->end(); ++i)
        {
            if(i->visited == false)
            {
                nextRendezvous = *i;
                ROS_DEBUG("relay: move to x = %f , y = %f", nextRendezvous.x, nextRendezvous.y);
                break;
            }
        }
        //rendezvousVector_mutex.unlock();

        visualize_rendezvous(nextRendezvous.x, nextRendezvous.y);

        teamMemberInRange = false;

        // move to current rendezvous
        if(move_robot(nextRendezvous.x, nextRendezvous.y) == true)
        {
            visualize_visited_rendezvous(nextRendezvous.x, nextRendezvous.y);
            ROS_DEBUG("relay at rendezvous, waiting for explorer...");

            teamMemberInRange = hallo(maxWaitTime*2);       // relay waits 2 times maxWaitTime -> rendezvous should not fail bc of wait time
            ros::Duration(5).sleep();
        }
        else
        {
            ROS_WARN("Rendezvous failed!!");
            ROS_INFO("... because robot failed to move to rendezvous!!");
            rendezvousSuccessful = false;

//            // get current robot position
//            explorer::getPosition srv;
//            srv.request.name = "currentPos";
//            if(this->position_client.call(srv) == false)
//            {
//                ROS_ERROR("Failed to call explorer service 'getPosition'");
//            }
//            else
//            {
//                if((srv.response.x == home_x) && (srv.response.y == home_y))
//                {
//                    ROS_WARN("Rendezvous failed!!");
//                    ROS_DEBUG("... because failed to move robot to rendezvous");
//                    rendezvousSuccessful = false;
//                }
//                else
//                {
//                    ROS_DEBUG("stay here and try to rendezvous, maybe in communication range anyway");
//                    visualize_unreachable_rendezvous(nextRendezvous.x, nextRendezvous.y);
//                    teamMemberInRange = hallo(maxWaitTime*2);       // relay waits 2 times maxWaitTime -> rendezvous should not fail bc of wait time
//                    ros::Duration(5).sleep();
//                }
//            }
        }

        if(teamMemberInRange == true)
        {
            ROS_DEBUG("other robot is in communication range, start to rendezvous!");
            rendezvousSuccessful = rendezvous();
        }
        else
        {
            ROS_WARN("Rendezvous failed!!");
            ROS_DEBUG("... because other robot didnt come for %f sec", 2*maxWaitTime);
            rendezvousSuccessful = false;
        }

        if(rendezvousSuccessful == true){
            num_successfulRend++;
            ROS_DEBUG("relay: list of rendezvous points after rendezvous:");
            printRendezvousPoints();

            double start = ros::Time::now().toSec();

            //move home again
            if(move_robot(home_x, home_y))
            {
                //ROS_DEBUG("relay at home, it will wait here for %f sec.", explorationTime);
                double home = ros::Time::now().toSec();
                ros::Duration((explorationTime - (home-start)) + rendezvousTime).sleep();
            }
            else
            {
                ROS_WARN("Failed to move robot home!");
                ROS_DEBUG("relay will move to next rendezvous and hopefully manage to move home next time");
            }
        }
        else
        {
            num_failedRend++;
            //ROS_WARN("Rendezvous failed!!");
            ROS_DEBUG("going to call plan B!");
            planB();
            // next move to backup rendezvous not home, bc it would take to long
            ros::Duration(explorationTime-rendezvousTime).sleep();
        }

        if(this->mission_client.call(mission_srv) == false)
        {
            ROS_ERROR("Failed to call explorer service 'missionFinished'");
        } else {
            missionFinished = mission_srv.response.finished;
        }
    }

    ROS_DEBUG("relay :  mission finished !!!!");
    move_robot(home_x, home_y);
    indicateSimulationEnd();

}

void Rendezvous::base_station()
{
    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;

    if(this->expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'");
    }

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
    }
}

void Rendezvous::callback_hallo(const adhoc_communication::RecvString msg)
{
    ROS_DEBUG("received %s message from: %s", msg.data.c_str(), msg.src_robot.c_str());

    std::string temp2 = msg.src_robot.c_str();
    int n = temp2.size();
    std::string temp = myBuddy_prefix.substr(1,n);

    if(strcmp(temp.c_str(), temp2.c_str()) == 0)
    {
        teamMemberInRange = true;
    }
}

bool Rendezvous::hallo(double max_wait_time)
{

    /* broadcast a 'hallo' message on topic '/rendezvous/checkCommunicationRange' to check if any other robot is in communication range
       returns (true) if there were 2 hallo from 'myBuddy' robot in maxWaitTime seconds
               (false) if 'myBuddy' didn't answer in maxWaitTime seconds */

    double start = ros::Time::now().toSec();
    int hallo_received = 0;
    timeoutAtRendezvous = false;

    adhoc_communication::SendString hallo_msg;
    hallo_msg.request.dst_robot = "";
    hallo_msg.request.data = "hallo";
    hallo_msg.request.topic = "/rendezvous/checkCommunicationRange";

    ros::Rate r(0.5);
    while(ros::ok())
    {
        if(hallo_client.call(hallo_msg) == true)
        {
            //ROS_DEBUG("Successfully called 'sendString' service to broadcast 'hallo'!, response status is: %s", hallo_msg.response.status);
        }
        else
        {
            ROS_ERROR("Failed to call 'sendString' service of ad_hoc_communication node!");
        }

        teamMemberInRange = false;  // private variable, changed in callback_hallo()

        // get callback
        ros::spinOnce();
        if(teamMemberInRange == true){
            hallo_received++;
            //ROS_DEBUG("received %d - th hallo", hallo_received);
        }
        else
        {
            //ROS_DEBUG("second robot is not in communication range!");
        }

        if(hallo_received == 2){
            //ROS_DEBUG("second robot is in communication range!");
            return true;
        }

        if((ros::Time::now().toSec() - start) == max_wait_time)
        {
            //ROS_DEBUG("tried to reach my other team members for %f sec. But failed!", maxWaitTime);
            timeoutAtRendezvous = true;
            return false;
        }

        r.sleep();
    }

    return false;
}

void Rendezvous::callback_rendezvous(const adhoc_communication::MmPoint msg)
{
    //ROS_DEBUG("received msg from %s ", msg.src_robot.c_str());

    std::string src_id = "/" + msg.src_robot;

    if(src_id == myBuddy_prefix && rendezvous_state == A && iAm == RELAY)
    {
        ROS_DEBUG("received possible rendezvous (%f / %f) from %s", msg.x, msg.y, msg.src_robot.c_str());

        possibleRendezvous.x = msg.x;
        possibleRendezvous.y = msg.y;

        rendezvous_state = D;
    }
    else if(src_id == myBuddy_prefix && rendezvous_state == A && iAm == RELAY &&  msg.x == 0 && msg.y == 0)
    {
        ROS_DEBUG("received FIN message from %s", msg.src_robot.c_str());
        missionFinished = true;
    }
    else if(src_id == myBuddy_prefix && rendezvous_state == A && iAm == EXPLORER)
    {
        ROS_DEBUG("received ACK for rendezvous (%f / %f) from %s", msg.x, msg.y, msg.src_robot.c_str());
        rendezvous_state = C;
    }
    else if(src_id == myBuddy_prefix && rendezvous_state == A && iAm == EXPLORER && msg.x == 0 && msg.y == 0)
    {
        ROS_DEBUG("suggested rendezvous (%f / %f) was not reachable for %s", msg.x, msg.y, msg.src_robot.c_str());
        rendezvous_state = E;
    }
    else
    {
        ROS_DEBUG("in callback_rendezvous(): received message but dont know how to handle");
        rendezvous_state = E;
    }
}

bool Rendezvous::rendezvous()
{
    visualize_visited_rendezvous(nextRendezvous.x, nextRendezvous.y);

    // mark rendezvous as visited
    // case 1: robot reached rendezvous -> mark ok
    // case 2: rendezvous failed for some reason -> do not try again ..
    //rendezvousVector_mutex.lock();
    for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
        i != rendezvousPoints->end(); ++i)
    {
        if(i->x == nextRendezvous.x && i->y == nextRendezvous.y){
            i->visited = true;
        }
    }
    //rendezvousVector_mutex.unlock();

    // first sleep for rendezvousTime to exchange map information
    ros::Duration(rendezvousTime).sleep();

    if(iAm == EXPLORER)
    {
         rendezvous_state = B;
    }
    else if(iAm == RELAY)
    {
        rendezvous_state = A;
    }

    /*
      state A : in communication range at rendezvous, waiting for messages
      state B : point in possibleRendezvous is reachable, send it to other robot as suggestion
      state C : received acknowledgement for suggested point. set nextRendezvous and return true
      state D : received possible next rendezvous, if it is reachable send acknowledgement, otherwise send other point as fail message
      state E : rendezvous failed, return false

      initial state relay = A
      initial state explorer = B
    */

    //ROS_DEBUG("In rendezvous(): i am %s, initial state is: %s", iAm, (char)rendezvous_state);

    adhoc_communication::MmPoint point_msg;
    adhoc_communication::SendMmPoint rzv_msg;

    if(iAm == EXPLORER && missionFinished == true){
        ROS_DEBUG("Exploration is finished, tell the relay and return home!");

        point_msg.x = 0;
        point_msg.y = 0;

        rzv_msg.request.dst_robot = "";
        rzv_msg.request.point = point_msg;
        rzv_msg.request.topic = "/rendezvous/rendezvousAgreement";

        possibleRendezvous.x = home_x;
        possibleRendezvous.y = home_y;
        rendezvous_state = C;
    }

    while(true){

        if(rendezvous_state == A)
        {
            //ROS_DEBUG("STATE = A: In communication range at rendezvous, waiting for messages!");
            ros::Duration(3).sleep();
            ros::spinOnce();

            if(missionFinished == true)
            {
                possibleRendezvous.x = home_x;
                possibleRendezvous.y = home_y;
                rendezvous_state = C;
            }
        }
        else if(rendezvous_state == B)
        {
            // send REQ for rendezvous to other robot
            ROS_DEBUG("STATE = B: send possible point (%f / %f) to %s", possibleRendezvous.x, possibleRendezvous.y, myBuddy_prefix.c_str());

            point_msg.src_robot = robot_prefix;
            point_msg.x = possibleRendezvous.x;
            point_msg.y = possibleRendezvous.y;

            rzv_msg.request.dst_robot = "";
            rzv_msg.request.point = point_msg;
            rzv_msg.request.topic = "/rendezvous/rendezvousAgreement";

            if(rendezvous_client.call(rzv_msg) == true)
            {
                //ROS_DEBUG("Successfully called 'send_point' service");
            }
            else
            {
                ROS_ERROR("Failed to call 'send_point' service of ad_hoc_communication node!");
            }
            rendezvous_state = A;
        }
        else if(rendezvous_state == C)
        {
            // next rendezvous is fixed or mission is finished -> exchange maps and return
            addRendezvous(possibleRendezvous.x, possibleRendezvous.y);

            ROS_DEBUG("STATE = C: next rendezvous is (%f / %f)!", possibleRendezvous.x, possibleRendezvous.y);

            visualize_rendezvous(possibleRendezvous.x, possibleRendezvous.y);
            return true;
        }
        else if(rendezvous_state == D)
        {
            // received REQ for rendezvous from other robot -> check if possibleRendezvous reachable

           //if(reachable(possibleRendezvous.x, possibleRendezvous.y))
           // {
                //ROS_DEBUG("STATE = D: possible rendezvous is reachable -> send ACK!");
                ROS_DEBUG("STATE = D: without reachable() -> send ACK!");

                point_msg.x = possibleRendezvous.x;
                point_msg.y = possibleRendezvous.y;

                rendezvous_state = C;
           // }
//            else
//            {
//                ROS_DEBUG("STATE = D: possible rendezvous is not reachable!");
//                point_msg.x = home_x;
//                point_msg.y = home_y;
//                rendezvous_state = E;
//            }

            point_msg.src_robot = robot_prefix;

            rzv_msg.request.dst_robot = "";
            rzv_msg.request.point = point_msg;
            rzv_msg.request.topic = "/rendezvous/rendezvousAgreement";

            if(rendezvous_client.call(rzv_msg) == true)
            {
                //ROS_DEBUG("Successfully called 'send_point' service");
            }
            else
            {
                ROS_ERROR("Failed to call 'send_point' service of ad_hoc_communication node!");
                rendezvous_state = E;
            }
        }
        else
        {
            //ROS_ERROR("rendezvous failed!");
            return false;
        }

    }
}

bool Rendezvous::reachable(double x, double y)
{
    //ROS_DEBUG("in reachable()");

    ROS_DEBUG("Determine if goal ( %f / %f) is reachable", x, y);

    double delta = 0.00001;       // how much goal point is changed to avoid crushes
    double tolerance = 0.00001;

    geometry_msgs::PoseStamped startPosition, goalPoint;

    if(iAm == RELAY)            // relay will start from home point to next rendezvous
    {
        startPosition.pose.position.x = home_x;
        startPosition.pose.position.y = home_y;
    }
    else if(iAm == EXPLORER)   // for explorer just take last visited rendezvous from rendezvous vector
    {
        //rendezvousVector_mutex.lock();
        startPosition.pose.position.x = (rendezvousPoints->back()).x;
        startPosition.pose.position.y = (rendezvousPoints->back()).y;
        //rendezvousVector_mutex.unlock();
    }

    startPosition.header.frame_id = move_base_frame;

    // to avoid crush, change goal a little bit
    goalPoint.pose.position.x = x + delta;
    goalPoint.pose.position.y = y + delta;
    goalPoint.header.frame_id = move_base_frame;

    ROS_DEBUG("start position is (%f / %f), goal is (%f / %f) ",
              startPosition.pose.position.x, startPosition.pose.position.y,
              goalPoint.pose.position.x, goalPoint.pose.position.y);

    nav_msgs::GetPlan srv_msg;
    srv_msg.request.goal = goalPoint;
    srv_msg.request.start = startPosition;


    if(plan_client.call(srv_msg))
    {
        geometry_msgs::PoseStamped new_goal = srv_msg.response.plan.poses.back();
        ROS_DEBUG("goal (%f / %f) is reachable with %f tolerance!", new_goal.pose.position.x, new_goal.pose.position.y, tolerance);
        possibleRendezvous.x = new_goal.pose.position.x;
        possibleRendezvous.y = new_goal.pose.position.y;

        return true;
    }
    else
    {
        ROS_WARN("Service call 'move_base/make_plan failed!!");
        ROS_DEBUG("goal is not reachable!");
        return false;
    }
}

void Rendezvous::planB()
{    //ROS_DEBUG("in planB");

    num_planB++;
    // delete last rendezvous
    //rendezvousVector_mutex.lock();
    rendezvousPoints->pop_back();
    //rendezvousVector_mutex.unlock();

    // next rendezvous should be last successful one -> modify 'visited' of last rendezvous
    //rendezvousVector_mutex.lock();
    RendezvousPoint temp = rendezvousPoints->back();
    temp.visited = false;
    rendezvousPoints->push_back(temp);
    //rendezvousVector_mutex.unlock();

    ROS_DEBUG("list of rendezvous points after executing plan B");
    printRendezvousPoints();
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
    //rendezvousVector_mutex.lock();
    rendezvousPoints->push_back(rend);
    //rendezvousVector_mutex.unlock();
}

void Rendezvous::printRendezvousPoints()
{
    int n = 1;
    //rendezvousVector_mutex.lock();
    for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
        i != rendezvousPoints->end(); ++i)
    {
        ROS_DEBUG("%d - th Point: x = %f \t y = %f \t visited = %d \n", n, i->x, i->y, i->visited);
        n++;
    }
    //rendezvousVector_mutex.unlock();

}

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

void Rendezvous::indicateSimulationEnd()
{
    /// FIXME: remove this stuff once ported to multicast
    std::stringstream robot_number;
    robot_number << robot_prefix;

    std::string prefix = "/robot_";
    std::string status_directory = "/simulation_status";
    std::string robo_name = prefix.append(robot_number.str());
    std::string file_suffix(".finishedRendezvous");

    std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
    std::string status_path = ros_package_path + status_directory;
    std::string status_file = status_path + robo_name + file_suffix;

    /// TODO: check whether directory exists
    boost::filesystem::path boost_status_path(status_path.c_str());
    if(!boost::filesystem::exists(boost_status_path))
        if(!boost::filesystem::create_directories(boost_status_path))
            ROS_ERROR("Cannot create directory %s.", status_path.c_str());
    std::ofstream outfile(status_file.c_str());
    outfile.close();
    ROS_INFO("Creating file %s to indicate end of exploration.", status_file.c_str());

}

void Rendezvous::initLogPath()
{
   /*
    *  CREATE LOG PATH
    * Following code enables to write the output to a file
    * which is localized at the log_path
    */

    nh->param<std::string>("log_path",log_path,"");

    std::string robo_name = robot_prefix;

    log_path = log_path.append("/rendezvous");
    log_path = log_path.append(robo_name);
    log_path = log_path.append("/");
    ROS_INFO("Logging files to %s", log_path.c_str());

    boost::filesystem::path boost_log_path(log_path.c_str());
    if(!boost::filesystem::exists(boost_log_path))
        if(!boost::filesystem::create_directories(boost_log_path))
            ROS_ERROR("Cannot create directory %s.", log_path.c_str());

}

void Rendezvous::log_info()
{
    fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs_csv << "#time, x_coordinate, y_coordinate, distance_to_home" << std::endl;
    fs_csv.close();

    while(ros::ok() && missionFinished != true)
    {
        ros::Duration time = ros::Time::now() - time_start;
        double exploration_time = time.toSec();
        double x, y, distance_to_home;

        // get current robot position
        explorer::getPosition srv;
        srv.request.name = "currentPos";
        if(this->position_client.call(srv) == false)
        {
            ROS_ERROR("Failed to call explorer service 'getPosition'");
        }
        else
        {
            x = srv.response.x;
            y = srv.response.y;
        }

        distance_to_home = sqrt((x*x)+(y*y));

        fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        fs_csv << exploration_time << ","
               << x << ","
               << y << ","
               << distance_to_home
               << std::endl;
        fs_csv.close();

        save_info();

        ros::Duration(10.0).sleep();
    }
}

void Rendezvous::save_info()
{
    ros::Duration ros_time = ros::Time::now() - time_start;
    double exploration_time = ros_time.toSec();

    std::string tmp_log = log_file;
    fs.open(tmp_log.c_str(), std::fstream::in | std::fstream::trunc | std::fstream::out);

    time_t raw_time;
    struct tm* timeinfo;
    time (&raw_time);
    timeinfo = localtime (&raw_time);

    fs << "[Rendezvous based exploration]" << std::endl;
    fs << "time_file_written    			= " << asctime(timeinfo); // << std::endl;
    fs << "start_time           			= " << time_start << std::endl;
    fs << "end_time             			= " << ros::Time::now() << std::endl;
    fs << "duration   			        = " << exploration_time << std::endl;

    if(iAm == RELAY){
        fs << "role                             = " << "RELAY" << std::endl;
    } else if(iAm == EXPLORER){
        fs << "role                             = " << "EXPLORER" << std::endl;
    } else if(iAm == BASE){
        fs << "role                             = " << "BASE" << std::endl;
    }
    fs << "ID of team member                = " << myBuddy_prefix << std::endl;
    fs << "exploration time                 = " << explorationTime << std::endl;
    fs << "max wait time at rendezvous      = " << maxWaitTime << std::endl;
    fs << "rendezvous time                  = " << rendezvousTime << std::endl;
    fs << "# successful rendezvous          = " << num_successfulRend << std::endl;
    fs << "# failed rendezvous              = " << num_failedRend << std::endl;
    fs << "# executing plan B               = " << num_planB << std::endl;

    int n = 1;
    for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
        i != rendezvousPoints->end(); ++i)
    {
        fs << n << "-th rendezvous point: ( " << i->x << " / " << i->y << " )" << std::endl;
        n++;
    }

    fs.close();
}
