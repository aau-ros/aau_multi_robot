#include "auction_observer.h"

AuctionObserver::AuctionObserver() {
    auction_already_started = false;
    error_1 = false, error_2 = false, error_3 = false;
    robot_state = robot_state::INITIALIZING;
    loadParameters();
    ros::NodeHandle nh;
    sub_new_optimal_ds = nh.subscribe("explorer/new_optimal_ds", 10,
                                                         &AuctionObserver::newOptimalDsCallback, this);
                                                         
    set_robot_state_sc = nh.serviceClient<robot_state::SetRobotState>("robot_state/set_robot_state");
    get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");
}

void AuctionObserver::loadParameters() {
    ros::NodeHandle nh_tilde("~");
    if(!nh_tilde.getParam("auction_duration", auction_timeout)) {//TODO int or double?
        ROS_ERROR("invalid auction_timeout!!");
        auction_timeout = 3;
    }
    if(!nh_tilde.getParam("extra_auction_time", extra_auction_time)) { //TODO use getParam and the fact that it returns a bool!!! http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters#getParam.28.29
        extra_auction_time = 3;
        ROS_ERROR("Invalid extra_auction_time!");
    }
    if(!nh_tilde.getParam("reauctioning_timeout", reauctioning_timeout)) {
        ROS_ERROR("invalid reauctioning_timeout");
        reauctioning_timeout = 5;
    }
}

void AuctionObserver::sanityChecks() {
    robot_state = getRobotState();

    auction_manager->lock();   

    auction_t current_auction = auction_manager->getCurrentAuction();
    
    //TODO I could collect all this time_manager->simulationTimeNow().toSec() calls in a single call and save the time in a variable to simplify the tests
    if(current_auction.starting_time > 0 && current_auction.ending_time < 0 && (time_manager->simulationTimeNow().toSec() - current_auction.starting_time) > (auction_timeout + extra_auction_time) * 2) {
        ROS_FATAL("ROBOT SEEMS TO BE UNABLE TO COMPLETE AUCTION");
        error_1 = true;
    }

    if(current_auction.starting_time < 0 && time_manager->simulationTimeNow().toSec() > 600) {
        ROS_FATAL("ROBOT SEEMS TO BE UNABLE TO START AN AUCTION");
        error_2 = true;
    }

    if(robot_state == robot_state::IN_QUEUE && !auction_manager->isRobotParticipatingToAuction() && (time_manager->simulationTimeNow().toSec() - current_auction.starting_time) > (reauctioning_timeout + auction_timeout + extra_auction_time) * 2) {
        ROS_FATAL("ROBOT SEEMS STUCK IN QUEUE");
        error_3 = true;
    }

    auction_manager->unlock();
}

void AuctionObserver::actAccordingToRobotStateAndAuctionResult() { //TODO we should use a visitor //TODO this function is quite ugly...
    ROS_INFO("actAccordingToRobotStateAndAuctionResult");
    robot_state = getRobotState();

    auction_manager->lock();

    if(!auction_manager->isRobotParticipatingToAuction()) {
        ROS_INFO("acting accorgint to state");
        
        if(robot_state == choosing_next_action) {
            ROS_INFO("choosing_next_action state");
            if(auction_manager->isRobotWinnerOfMostRecentAuction())
                setRobotState(going_checking_vacancy);
            else
                setRobotState(exploring);

        }

        if(robot_state == exploring_for_graph_navigation)
            auction_manager->preventParticipationToAuctions();
        else
            auction_manager->allowParticipationToAuctions();

        if(robot_state == auctioning || robot_state == auctioning_2) {
            ROS_INFO("auctioning state");
            if(!auction_already_started) {
                auction_already_started = true;
                auction_manager->tryToAcquireDs();
            }
        }

        else
            auction_already_started = false;

        if(robot_state == in_queue) {
            ROS_INFO("in_queue state");
            if(auction_manager->isRobotWinnerOfMostRecentAuction())
                setRobotState(going_checking_vacancy);
            else {
                auction_t current_auction = auction_manager->getCurrentAuction();
                if(current_auction.ending_time > 0 && (time_manager->simulationTimeNow().toSec() - current_auction.ending_time) > reauctioning_timeout)
                    auction_manager->tryToAcquireDs();
            }
        }

        if(robot_state == charging)
            if(!auction_manager->isRobotWinnerOfMostRecentAuction())
                setRobotState(leaving_ds);

    }
    else
        ROS_INFO("robot is under auction");
        
    auction_manager->unlock();
}

unsigned int AuctionObserver::getRobotState() {
//    return robot_state_manager->getRobotState();
    robot_state::GetRobotState srv;
    while(!get_robot_state_sc.call(srv))
        ROS_INFO("call to get_robot_state failed");
    return srv.response.robot_state;
}

void AuctionObserver::setRobotState(unsigned int robot_state) {
    ROS_INFO("setting state");
    robot_state::SetRobotState srv;
    srv.request.robot_state = robot_state;
    while(!set_robot_state_sc.call(srv))
        ROS_INFO("call to get_robot_state failed");
//    robot_state_manager->setRobotState(robot_state);
}

void AuctionObserver::setAuctionManager(AuctionManagerInterface *auction_manager) {
    this->auction_manager = auction_manager;
}

void AuctionObserver::setTimeManager(TimeManagerInterface *time_manager) {
    this->time_manager = time_manager;
}

void AuctionObserver::setRobotStateManager(RobotStateManager2 *robot_state_manager) { //TODO set or inject?
    this->robot_state_manager = robot_state_manager;
}

void AuctionObserver::newOptimalDsCallback(const adhoc_communication::EmDockingStation::ConstPtr &msg) {
    auction_manager->setOptimalDs(msg.get()->id);
}
