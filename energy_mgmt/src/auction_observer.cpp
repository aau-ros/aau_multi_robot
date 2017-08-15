#include "auction_observer.h"

AuctionObserver::AuctionObserver() {
    auction_already_started = false;
    error_1 = false, error_2 = false, error_3 = false;
    robot_state = robot_state::INITIALIZING;
    loadParameters();
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
    robot_state = getRobotState();

    auction_manager->lock();

    if(!auction_manager->isRobotParticipatingToAuction()) {
        if(robot_state == robot_state::AUCTIONING) { //TODO(IMPORTANT) or auctioning_2
            if(!auction_already_started) {
                auction_already_started = true;
                auction_manager->tryToAcquireDs();
            }
        }

        else {
            auction_already_started = false;

            if(robot_state == robot_state::IN_QUEUE) {
                if(auction_manager->isRobotWinnerOfMostRecentAuction())
                    setRobotState(robot_state::GOING_CHECKING_VACANCY);
                else {
                    auction_t current_auction = auction_manager->getCurrentAuction();
                    if(current_auction.ending_time > 0 && (time_manager->simulationTimeNow().toSec() - current_auction.ending_time) > reauctioning_timeout)
                        auction_manager->tryToAcquireDs();
                }
            }

            else if(robot_state == robot_state::CHARGING)
                if(!auction_manager->isRobotWinnerOfMostRecentAuction())
                    setRobotState(robot_state::CHARGING_ABORTED);
        }
    }
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

void AuctionObserver::setRobotStateManager(RobotStateManager2 *robot_state_manager) { //TODO set or inject?
    this->robot_state_manager = robot_state_manager;
}
