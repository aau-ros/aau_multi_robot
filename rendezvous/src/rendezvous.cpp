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

    nh->setParam("maxWaitTime", 40.0);
    nh->setParam("explorationTime", 60.0);
    nh->setParam("rendezvousTime", 20.0);

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

    rendezvous_state = A;

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
        numberMeetings++;

        // explore for exploration_time seconds
        srv_msg.request.explore = true;
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

        // determine last not visited rendezvous -> in rendezvousPoints vector are only reachable and agreed upon points!
        for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
            i != rendezvousPoints->end(); ++i)
        {
            if(i->visited == false){
                nextRendezvous = *i;
                ROS_DEBUG("timeout -> move to x = %f , y = %f", nextRendezvous.x, nextRendezvous.y);
                break;
            }
        }

        ROS_DEBUG("explorer: list of rendezvous points after timeout:");
        printRendezvousPoints();

        // move robot to current rendezvous
        visualize_rendezvous(nextRendezvous.x, nextRendezvous.y);
        if(move_robot(nextRendezvous.x, nextRendezvous.y)){
            ROS_DEBUG("explorer at Rendezvous, waiting %f sec for relay...", maxWaitTime);
            visualize_visited_rendezvous(nextRendezvous.x, nextRendezvous.y);
        }
        else
        {
            ROS_WARN("failed to move to rendezvous!");
            visualize_unreachable_rendezvous(nextRendezvous.x, nextRendezvous.y);

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

        // check if other team member is in communication range
        // yes ... start to rendezvous
        // no ... rendezvous failed -> plan B

        teamMemberInRange = hallo(maxWaitTime);
        ros::Duration(2).sleep();

        if(teamMemberInRange == true)
        {
            //ROS_DEBUG("other robot is in communication range, start to rendezvous!");
            rendezvousSuccessful = rendezvous();
        }
        else
        {
            ROS_DEBUG("Tried to reach other robot for %f seconds. But rendezvous failed.", maxWaitTime);
            // TODO: strategy for rendezvous failed
        }

        if(rendezvousSuccessful == false)
        {
            // to avoid travelling home every time if rendezvous fails for explorer
            addRendezvous(possibleRendezvous.x, possibleRendezvous.y);
            //nextRendezvous.x = possibleRendezvous.x;
            //nextRendezvous.y = possibleRendezvous.y;
        } else {
            ROS_DEBUG("Rendezvous successful!");
        }

        // mark rendezvous as visited
        // case 1: robot reached rendezvous -> marking ok
        // case 2: rendezvous failed for some reason -> do not try again ..
        for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
            i != rendezvousPoints->end(); ++i)
        {
            if(i->x == nextRendezvous.x && i->y == nextRendezvous.y){
                i->visited = true;
            }
        }

        ROS_DEBUG("explorer: list of rendezvous points after rendezvous:");
        printRendezvousPoints();

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
    srv_msg.request.explore = true;
    if(expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'!");
    }

}

void Rendezvous::relayRobot(){

    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;

    if(this->expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'");
    } else {
        //ROS_DEBUG("relay stopped explorer!");
    }

    // TODO: move robot home, in case it stopped not at home but at his first frontier?

    //sleep at least for explorationTime of explorer, because first rendezvous will be at home
    ros::Duration(explorationTime + 20.0).sleep();

    explorer::MissionFinished mission_srv;
    if(this->mission_client.call(mission_srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'missionFinished'");
    } else {
        missionFinished = mission_srv.response.finished;
    }

    while(missionFinished == false)
    {
        rendezvousSuccessful = false;

        // determine last not visited rendezvous
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

        visualize_rendezvous(nextRendezvous.x, nextRendezvous.y);

        // move to current rendezvous
        if(move_robot(nextRendezvous.x, nextRendezvous.y) == true)
        {
            visualize_visited_rendezvous(nextRendezvous.x, nextRendezvous.y);
            ROS_DEBUG("relay at rendezvous, waiting for explorer...");

        }
        else
        {
            ROS_INFO("Failed to move relay to rendezvous!");
            visualize_unreachable_rendezvous(nextRendezvous.x, nextRendezvous.y);
        }

        // mark rendezvous as visited (in any case, relay should not move there again)
        for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
            i != rendezvousPoints->end(); ++i)
        {
            if(i->x == nextRendezvous.x && i->y == nextRendezvous.y){
                i->visited = true;
            }
        }

        // check if other team member is in communication range
        // yes ... start to rendezvous
        // no ... rendezvous failed -> plan B

        teamMemberInRange = hallo(maxWaitTime*2);       // relay waits 2 times maxWaitTime -> rendezvous should not fail bc of wait time
        ros::Duration(2).sleep();

        if(teamMemberInRange == true)
        {
            ROS_DEBUG("other robot is in communication range, start to rendezvous!");
            rendezvousSuccessful = rendezvous();
        }
        else
        {
            ROS_WARN("Rendezvous failed!!");
            //ROS_DEBUG("Tried to reach other robot for %f seconds. But we never met.", maxWaitTime);
            // TODO: strategy for rendezvous failed
        }

        ROS_DEBUG("relay: list of rendezvous points after rendezvous:");
        printRendezvousPoints();

        // after rendezvous or timeout: move home again
        if(move_robot(home_x, home_y))
        {
            ROS_DEBUG("relay at home, it will wait here for %f sec.", rendezvousTime + 30.0);
            ros::Duration(rendezvousTime + 30.0).sleep();
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

    ROS_DEBUG("relay :  mission finished !!!!");

}

void Rendezvous::base_station()
{
    double maxWaitTime = 10.0;

    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;

    if(this->expl_client.call(srv_msg) == false)
    {
        ROS_ERROR("Failed to call explorer service 'switchExplRend'");
    } else {
        //ROS_DEBUG("stopped explorer!");
    }

    bool inCommunicationRange;

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

    while(true)
    {
        inCommunicationRange = hallo(maxWaitTime);
        if(inCommunicationRange)
        {
            ROS_DEBUG("relay is in communication range!");
            ROS_DEBUG("exchange information");
            ros::Duration(30.0).sleep();
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
    //ROS_DEBUG("in callback_rendezvous()");

    //ROS_DEBUG("received msg from %s ", msg.src_robot.c_str());

    std::string src_id = "/" + msg.src_robot;

    /*if(iAm == RELAY && msg.status == "FIN" && msg.src_robot == myBuddy_prefix)
    {
        ROS_DEBUG("received FIN from %s -> return home!", msg.src_robot.c_str());
        rendezvous_state = C;
    }
    else*/
    //if(rendezvous_state == B && msg.status == "ACK" && msg.src_robot == myBuddy_prefix)
        if(rendezvous_state == B && src_id == myBuddy_prefix && msg.x == possibleRendezvous.x && msg.y == possibleRendezvous.y)
    {
        ROS_DEBUG("received ACK for rendezvous (%f / %f) from %s", msg.x, msg.y, msg.src_robot.c_str());
        rendezvous_state = C;
    }
    //else if(rendezvous_state == B && msg.status == "REQ" && msg.src_robot == myBuddy_prefix)
    else if(rendezvous_state == B && src_id == myBuddy_prefix)
    {
        ROS_DEBUG("received possible rendezvous (%f / %f) from %s", msg.x, msg.y, msg.src_robot.c_str());

        possibleRendezvous.x = msg.x;
        possibleRendezvous.y = msg.y;

        rendezvous_state = D;
    }
    //else if(rendezvous_state == A && msg.status == "REQ" && msg.src_robot == myBuddy_prefix)
    else if(rendezvous_state == A && src_id == myBuddy_prefix)
    {
        ROS_DEBUG("received possible rendezvous (%f / %f) from %s", msg.x, msg.y, msg.src_robot.c_str());

        possibleRendezvous.x = msg.x;
        possibleRendezvous.y = msg.y;

        rendezvous_state = D;
    }
    else
    {
        ROS_DEBUG("in callback_rendezvous(): received message but dont know how to handle");
    }
}

bool Rendezvous::rendezvous()
{
    ROS_DEBUG("in rendezvous()");

    /*
      state A : in communication range at rendezvous, waiting for messages (relay) or going to send first message (explorer)
      state B : point in possibleRendezvous is reachable, send it to other robot as suggestion
      state C : received acknowledgement for suggested point. set nextRendezvous and return true
      state D : received possible next rendezvous, if it is reachable send acknowledgement, otherwise send new suggestion)

      initial state = A
      */

    rendezvous_state = A;

    //ROS_DEBUG("In rendezvous(): i am %s, initial state is: %s", iAm, rendezvous_state);

    //adhoc_communication::RzvPoint point_msg;
    //adhoc_communication::SendRendezvous rzv_msg;

    adhoc_communication::MmPoint point_msg;
    adhoc_communication::SendMmPoint rzv_msg;

/*    if(iAm == EXPLORER && missionFinished == true){
        ROS_DEBUG("Exploration is finished, tell the relay and return home!");

        //point_msg.status = "FIN";
        point_msg.x = 0;
        point_msg.y = 0;
        //point_msg.z = 0;
        //point_msg.time = ros::Time.now().sec();

        rzv_msg.request.dst_robot = myBuddy_prefix;
        rzv_msg.request.msg = point_msg;
        rzv_msg.request.topic = "/rendezvous/rendezvousAgreement";

        rendezvous_state = C;

        //TODO: return home? wait for ACK of relay?
    } */

    while(true){

        if(rendezvous_state == A)
        {
            if(iAm == EXPLORER)
            {
                 ROS_DEBUG("STATE = A: Going to send possible rendezvous to relay");
                 rendezvous_state = B;
            }
            else if(iAm == RELAY)
            {
                ROS_DEBUG("STATE = A: In communication range at rendezvous, waiting for messages!");
                ros::Duration(1).sleep();
                ros::spinOnce();
            }
        }
        else if(rendezvous_state == B)
        {
            // send REQ for rendezvous to other robot

            ROS_DEBUG("STATE = B: send possible point (%f / %f) to %s", possibleRendezvous.x, possibleRendezvous.y, myBuddy_prefix.c_str());

            point_msg.src_robot = robot_prefix;
            //point_msg.status = "REQ";
            point_msg.x = possibleRendezvous.x;
            point_msg.y = possibleRendezvous.y;
            //point_msg.z = 0;
            //point_msg.time = (ros::Time.now() + explorationTime);

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

            ros::Duration(5).sleep();
            ros::spinOnce();
        }
        else if(rendezvous_state == C)
        {
            // next rendezvous is fixed or mission is finished -> exchange maps and return
            addRendezvous(possibleRendezvous.x, possibleRendezvous.y);

            ROS_DEBUG("STATE = C: next rendezvous is (%f / %f)!", possibleRendezvous.x, possibleRendezvous.y);

            ros::Duration(rendezvousTime).sleep();
            return true;
        }
        else if(rendezvous_state == D)
        {
            // received REQ for rendezvous from other robot -> check if possibleRendezvous reachable

            if(reachable(possibleRendezvous.x, possibleRendezvous.y))
            {
                ROS_DEBUG("STATE = D: possible rendezvous is reachable -> send ACK!");
                //point_msg.status = "ACK";

                point_msg.src_robot = robot_prefix;

                point_msg.x = possibleRendezvous.x;
                point_msg.y = possibleRendezvous.y;
                //point_msg.z = 0;
                //point_msg.time = (ros::Time.now() + explorationTime);

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

                rendezvous_state = C;
            }
            else
            {
                ROS_DEBUG("STATE = D: possible rendezvous is not reachable!");

                // determine another possible (reachable) rendezvous
                // + overwrite possibleRendezvous variable
                // -> done in reachable()

                // now: other possible rendezvous is last visited rendezvous.. TODO: better solution
                rendezvous_state = B;
            }

//            ROS_DEBUG("STATE = D: for now: do not check, only agree -> send ACK!");

//            point_msg.src_robot = robot_prefix;

//            point_msg.x = possibleRendezvous.x;
//            point_msg.y = possibleRendezvous.y;

//            rzv_msg.request.dst_robot = "";
//            rzv_msg.request.point = point_msg;
//            rzv_msg.request.topic = "/rendezvous/rendezvousAgreement";

//            if(rendezvous_client.call(rzv_msg) == true)
//            {
//                //ROS_DEBUG("Successfully called 'send_point' service");
//            }
//            else
//            {
//                ROS_ERROR("Failed to call 'send_point' service of ad_hoc_communication node!");
//            }
//            rendezvous_state = C;
        }
        else
        {
            ROS_ERROR("rendezvous failed!");
            return false;
        }

    }
}

bool Rendezvous::reachable(double x, double y)
{
    //ROS_DEBUG("in reachable()");

    ROS_DEBUG("Determine if goal ( %f / %f) is reachable", x, y);

    double delta = 0.003;       // how much goal point is changed to avoid crushes
    double tolerance = 0.005;

    geometry_msgs::PoseStamped startPosition, goalPoint;
    //std::vector<geometry_msgs::PoseStamped> global_plan;

    if(iAm == RELAY)            // relay will start from home point to next rendezvous
    {
        startPosition.pose.position.x = home_x;
        startPosition.pose.position.y = home_y;
    }
    else if(iAm == EXPLORER)   // for explorer just take last visited rendezvous from rendezvous vector
    {
        startPosition.pose.position.x = (rendezvousPoints->back()).x;
        startPosition.pose.position.y = (rendezvousPoints->back()).y;
    }

    // to avoid crush, change goal a little bit
    // TODO: first change point then makePlan() with tolerance -> theoretically same point could come out in the end?
    goalPoint.pose.position.x = x + delta;
    goalPoint.pose.position.y = y + delta;

    ROS_DEBUG("start position is (%f / %f), goal is (%f / %f) ",
              startPosition.pose.position.x, startPosition.pose.position.y,  goalPoint.pose.position.x, goalPoint.pose.position.y);

    nav_msgs::GetPlanRequest request;
    request.start = startPosition;
    request.goal = goalPoint;
    request.tolerance = tolerance;

    nav_msgs::GetPlan srv_msg;
    srv_msg.request = request;

    if(plan_client.call(srv_msg))
    {
        geometry_msgs::PoseStamped new_goal = srv_msg.response.plan.poses.back();
        ROS_DEBUG("goal (%f / %f) is reachable with %f toleranz!", tolerance, new_goal.pose.position.x, new_goal.pose.position.y);
        possibleRendezvous.x = new_goal.pose.position.x;
        possibleRendezvous.y = new_goal.pose.position.y;

        return true;
    }
    else
    {
        ROS_DEBUG("goal is not reachable!");

        // suggest last (successfull) rendezvous as plan B
        // TODO: better solution
        possibleRendezvous.x = (rendezvousPoints->back()).x;
        possibleRendezvous.y = (rendezvousPoints->back()).y;

        return false;
    }


//    if(nav.makePlan(startPosition, goalPoint, tolerance, global_plan))
//    {
//        geometry_msgs::PoseStamped new_goal = global_plan.back();
//        ROS_DEBUG("goal (%f / %f) is reachable with %f toleranz!", tolerance, new_goal.pose.position.x, new_goal.pose.position.y);
//        possibleRendezvous.x = new_goal.pose.position.x;
//        possibleRendezvous.y = new_goal.pose.position.y;

//        return true;
//    }
//    else
//    {
//        ROS_DEBUG("goal is not reachable!");

//        // suggest last (successfull) rendezvous as plan B
//        // TODO: better solution
//        possibleRendezvous.x = (rendezvousPoints->back()).x;
//        possibleRendezvous.y = (rendezvousPoints->back()).y;

//        return false;
//    }

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

// ------------------------------------------------------ TEST FUNCTIONS --------------------------------------------------------
// Test function - stop explorer node and start it again
void Rendezvous::stopStartExplorer()
{
    // stop explorer after 'explorationTime' sec,
    // wait for 'sleepTime' sec and start explorer again
    // till total space is explored


    int explorationTime = 80;
    int sleepTime = 25;

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
            nextRendezvous = *i;
            ROS_DEBUG("timeout -> move to x = %f , y = %f", nextRendezvous.x, nextRendezvous.y);
            break;
        }
    }

    if(nextRendezvous.x == home_x && nextRendezvous.y == home_y)
    {
        ROS_DEBUG("timeout, but rendezvous is home, explorer will not move there!");
    }
    else
    {
        visualize_rendezvous(nextRendezvous.x, nextRendezvous.y);
        if(move_robot(nextRendezvous.x, nextRendezvous.y)){
            ROS_DEBUG("EXPLORER at Rendezvous, wait here for 5 sec!");
            ros::Duration(5).sleep();
            visualize_visited_rendezvous(nextRendezvous.x, nextRendezvous.y);
        }
        else
        {
            ROS_DEBUG("EXPLORER failed to move to rendezvous!");
            visualize_unreachable_rendezvous(nextRendezvous.x, nextRendezvous.y);
        }
     }

    // mark rendezvous as visited
    // case 1: robot reached rendezvous -> marking ok
    // case 2: robot failed to reach rendezvous -> probably rendezvous not reachable, robot should not try again .. TODO
    // case 3: rendezvous == home -> marking ok, bc explore robot should not return home till mission is finished
    for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
        i != rendezvousPoints->end(); ++i)
    {
        if(i->x == nextRendezvous.x && i->y == nextRendezvous.y){
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
                nextRendezvous = *i;
                ROS_DEBUG("RELAY : move to x = %f , y = %f", nextRendezvous.x, nextRendezvous.y);
                break;
            }
            else
            {
                // no new rendezvous available -> stay
                ros::Duration(10).sleep();
            }
        }

        visualize_rendezvous(nextRendezvous.x, nextRendezvous.y);
        if(move_robot(nextRendezvous.x, nextRendezvous.y) == true)
        {
            // mark rendezvous as visited
            for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
                i != rendezvousPoints->end(); ++i)
            {
                if(i->x == nextRendezvous.x && i->y == nextRendezvous.y){
                    i->visited = true;
                }
            }

            visualize_visited_rendezvous(nextRendezvous.x, nextRendezvous.y);

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

            visualize_unreachable_rendezvous(nextRendezvous.x, nextRendezvous.y);

            // mark rendezvous as visited (relay should not try to reach this rendezvous again, bc maybe its not reachable)
            for(std::vector<RendezvousPoint>::iterator i = rendezvousPoints->begin();
                i != rendezvousPoints->end(); ++i)
            {
                if(i->x == nextRendezvous.x && i->y == nextRendezvous.y){
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

void Rendezvous::test_hallo()
{
    double explorationTime = 40.0;
    double exchangeTime = 20.0;         // how long they sleep at rendezvous to allow mapmerger to exchange maps
    double maxWaitTime = 10.0;          // how long they will wait at rendezvous trying to hear each other

    bool inCommunicationRange;

    ros::Duration(explorationTime).sleep();

    // stop explorer
    explorer::switchExplRend srv_msg;
    srv_msg.request.explore = false;

    if(this->expl_client.call(srv_msg) == false)
    {
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
            ROS_DEBUG("robot at home, try to check if buddy in range!");
            inCommunicationRange = hallo(maxWaitTime);
            if(inCommunicationRange == true)
            {
                ROS_DEBUG("second robot is in communication range! Stay here for %f seconds to exchange information.", exchangeTime);
                ros::Duration(exchangeTime).sleep();
            }
            else
            {
                ROS_DEBUG("tried to reach my other team members for %f sec. But failed!", maxWaitTime);
            }
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

void Rendezvous::test_relay_base_station()
{
    double explorationTime = 60.0;
    double exchangeTime = 40.0;         // how long they sleep at rendezvous to allow mapmerger to exchange maps
    double maxWaitTime = 10.0;          // how long they will wait at rendezvous trying to hear each other
    double sleepTime = 15.0;

    bool inCommunicationRange;

    ROS_DEBUG("explore for %f seconds", explorationTime);
    ros::Duration(explorationTime).sleep();

    explorer::MissionFinished mission_srv;
    if(this->mission_client.call(mission_srv) == false)
    {
        ROS_ERROR("Failed to call explorer service 'missionFinished'");
    } else {
        missionFinished = mission_srv.response.finished;
    }

    explorer::switchExplRend srv_msg;

    while(missionFinished == false)
    {
        // stop explorer
        srv_msg.request.explore = false;

        if(this->expl_client.call(srv_msg) == false)
        {
            ROS_ERROR("Failed to call explorer service 'switchExplRend'");
        } else {
            ROS_DEBUG("stopped explorer!");
        }

        ros::Duration(15).sleep();   // sleep to allow explorer to set all paramters

        // in case robot was moving during first service call..
        srv_msg.request.explore = false;
        if(this->expl_client.call(srv_msg) == false)
        {
            ROS_ERROR("Failed to call explorer service 'switchExplRend'");
        } else {
            //ROS_DEBUG("stopped explorer!");
        }

        ros::Duration(15).sleep();   // sleep to allow explorer to set all paramters

        // move home
        visualize_rendezvous(home_x, home_y);
        if(this->move_robot(home_x, home_y) == false)
        {
            ROS_ERROR("Failed to move robot to home position");
        }
        else
        {
            ROS_DEBUG("robot at home, sleep for %f sec!", sleepTime);
            ros::Duration(sleepTime).sleep();
//            inCommunicationRange = hallo(maxWaitTime);
//            if(inCommunicationRange == true)
//            {
//                ROS_DEBUG("base station is in communication range! Stay here for %f seconds to exchange information.", exchangeTime);
//                ros::Duration(exchangeTime).sleep();
//            }
//            else
//            {
//                ROS_DEBUG("tried to communicate with base station for %f sec. But failed!", maxWaitTime);
//            }
        }

        // start explorer
        srv_msg.request.explore = true;

        if(this->expl_client.call(srv_msg) == false)
        {
            ROS_ERROR("Failed to call explorer service 'switchExplRend'");
        } else {
            ROS_DEBUG("explore for %f sec now!", explorationTime);
        }

        ros::Duration(explorationTime).sleep();

        if(this->mission_client.call(mission_srv) == false)
        {
            ROS_ERROR("Failed to call explorer service 'missionFinished'");
        } else {
            missionFinished = mission_srv.response.finished;
        }
    }

}


