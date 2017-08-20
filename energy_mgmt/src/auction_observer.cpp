#include "auction_observer.h"

AuctionObserver::AuctionObserver() {
    loadParameters();
    initializeVariables();
    createSubscribers();
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

void AuctionObserver::initializeVariables() {
    auction_already_started = false;
    error_1 = false, error_2 = false, error_3 = false;
    new_victory = false;
    last_auction_id = 0;
    robot_state = robot_state::INITIALIZING;
}

void AuctionObserver::createSubscribers() {
    ros::NodeHandle nh;
    sub_new_optimal_ds = nh.subscribe("explorer/new_optimal_ds", 10, &AuctionObserver::newOptimalDsCallback, this);
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

void AuctionObserver::actAccordingToRobotStateAndAuctionResult() { //TODO this function is quite ugly... we should use a visitor
    ROS_INFO("actAccordingToRobotStateAndAuctionResult");

    auction_manager->lock();
    analyzeAuctionResult();

    ROS_INFO("locking");
    robot_state_manager->lockRobotState();

    robot_state = getRobotState();

    if(!auction_manager->isRobotParticipatingToAuction()) {
        ROS_INFO("acting accorgint to state");
        
        if(robot_state == robot_state::CHOOSING_ACTION) {
            ROS_INFO("choosing_next_action state");
            if(new_victory)
                setRobotState(robot_state::GOING_CHECKING_VACANCY);
            else
                setRobotState(robot_state::COMPUTING_NEXT_GOAL);

        }

        if(robot_state == robot_state::exploring_for_graph_navigation)
            auction_manager->preventParticipationToAuctions();
        else
            auction_manager->allowParticipationToAuctions();

        if(robot_state == robot_state::AUCTIONING || robot_state == robot_state::auctioning_2) {
            ROS_INFO("auctioning state");
            if(!auction_already_started) {
                auction_already_started = true;
                auction_manager->tryToAcquireDs();
            }
        }

        else
            auction_already_started = false;

        if(robot_state == robot_state::IN_QUEUE) {
            ROS_INFO("in_queue state");
            if(auction_manager->isRobotWinnerOfMostRecentAuction())
                setRobotState(robot_state::GOING_CHECKING_VACANCY);
            else {
                auction_t current_auction = auction_manager->getCurrentAuction();
                if(current_auction.ending_time > 0 && (time_manager->simulationTimeNow().toSec() - current_auction.ending_time) > reauctioning_timeout)
                    auction_manager->tryToAcquireDs();
            }
        }

        if(robot_state == robot_state::CHARGING)
            if(!auction_manager->isRobotWinnerOfMostRecentAuction())
                setRobotState(robot_state::CHARGING_ABORTED);

    }
    else
        ROS_INFO("robot is under auction");

    ROS_INFO("unlocking");
    robot_state_manager->unlockRobotState();
   
    auction_manager->unlock();
}

unsigned int AuctionObserver::getRobotState() {
    return robot_state_manager->getRobotState();
}

void AuctionObserver::setRobotState(unsigned int robot_state) {
    robot_state_manager->setRobotState(robot_state);
}

void AuctionObserver::setAuctionManager(AuctionManagerInterface *auction_manager) {
    this->auction_manager = auction_manager;
}

void AuctionObserver::setTimeManager(TimeManagerInterface *time_manager) {
    this->time_manager = time_manager;
}

void AuctionObserver::setRobotStateManager(RobotStateManagerInterface *robot_state_manager) { //TODO set or inject?
    this->robot_state_manager = robot_state_manager;
}

void AuctionObserver::newOptimalDsCallback(const adhoc_communication::EmDockingStation::ConstPtr &msg) {
    optimal_ds_id = msg.get()->id;
    auction_manager->setOptimalDs(msg.get()->id);
}

void AuctionObserver::analyzeAuctionResult() {
    auction_t current_auction = auction_manager->getCurrentAuction();
    if(last_auction_id != current_auction.auction_id) {
        last_auction_id = current_auction.auction_id;
        if(optimal_ds_id != current_auction.docking_station_id)
            new_victory = false;
        else
            new_victory = auction_manager->isRobotWinnerOfMostRecentAuction();
    } else {
        new_victory = false;
    }
}
