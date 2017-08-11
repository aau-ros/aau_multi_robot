#include "auction_manager.h"

AuctionManager::AuctionManager(unsigned int robot_id) {
    initializeVariables(robot_id);
    createSubscribers();
    createServiceClients();
}

void AuctionManager::initializeVariables(unsigned int robot_id) {
    _b1 = false;
    _b2 = false;
    _b3 = false;
    _b4 = false;
    winner_of_auction = false;
    robot_cannot_participate_to_auctions = false;
    waiting_for_next_auction = false;
    this->robot_id = robot_id;
    auction_participation_state = IDLE;    
    time_last_participation = 0;
    local_auction_id = 0;
    _u2 = 0;

    ros::NodeHandle nh_tilde("~");
    int tmp;
    nh.param<int>("num_robots", tmp, -1); //TODO
    if(tmp < 0)
        ROS_FATAL("Invalid number of robots!");
    else
        num_robots = (unsigned int)tmp;
    if(!nh_tilde.getParam("auction_duration", auction_timeout)) {//TODO int or double?
        ROS_FATAL("invalid auction_timeout!!");
        auction_timeout = 3;
    }
    if(!nh_tilde.getParam("extra_auction_time", extra_auction_time)) { //TODO use getParam and the fact that it returns a bool!!! http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters#getParam.28.29
        extra_auction_time = 3;
        ROS_FATAL("Invalid extra_auction_time!");
    }
    if(!nh_tilde.getParam("reauctioning_timeout", reauctioning_timeout)) {
        ROS_ERROR("invalid reauctioning_timeout");
        reauctioning_timeout = 5;
    }
    nh_tilde.param<double>("sleep_time_between_two_participations", sleep_time_between_two_participations, 5); //s
}

void AuctionManager::createSubscribers() {
    std::string my_prefix = ""; //TODO(IMPORTANT)
    auction_reply_sub = nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/auction_reply", 1000, &AuctionManager::auctionReplyCallback, this);
    auction_result_sub = nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/auction_result", 1000, &AuctionManager::auctionResultCallback, this); //TODO do i need 'adhoc_communication' in the service name? and 'send_em_auction'?
    auction_starting_sub = nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/auction_starting", 1000, &AuctionManager::auctionStartingCallback, this);
}

void AuctionManager::createServiceClients() {
    std::string my_prefix = ""; //TODO(IMPORTANT)
    sc_send_auction = nh.serviceClient<adhoc_communication::SendEmAuction>(my_prefix + "adhoc_communication/send_em_auction");
}

void AuctionManager::setBidComputer(BidComputer *bid_computer) {
    this->bid_computer = bid_computer;
}

void AuctionManager::setTimeManager(TimeManagerInterface *time_manager) {
    this->time_manager = time_manager;
}

void AuctionManager::tryToAcquireDs() {
    ROS_ERROR("Starting new auction");
    auction_mutex.lock();

//    if(wait_for_ds >= 100)
//            return;

//        if (!optimal_ds_is_set() && need_to_charge)
//        {
//            waiting_to_discover_a_ds = true;
//            log_minor_error("The robot needs to recharge, but it doesn't know about any "
//                      "existing DS!");
//    //        compute_and_publish_path_on_ds_graph_to_home();
//            wait_for_ds++;
//            
//            if(ros::Time::now() - starting_time > ros::Duration(5*60))
//                log_major_error("robot seems unable to find DSs!!");
//            
//            if(wait_for_ds < 100) {
//                timer_restart_auction.stop(); //reduntant?
//                timer_restart_auction.setPeriod(ros::Duration(reauctioning_timeout), true);
//                timer_restart_auction.start();
//                
//                // Force explorer in queue
//                std_msgs::Empty msg;
//                pub_force_in_queue.publish(msg);
//                
//            }
//            else {
//                log_major_error("robot cannot recharge: stopping...");
//                std_msgs::Empty msg;
//                pub_finish.publish(msg);
//            }
//        }

    ROS_ERROR("Starting new auction");
    if(auction_participation_state == PARTICIPATING)
        ROS_INFO("The robot is already participating to an auction: let's wait for the other auction to terminate");
    else {
        ROS_ERROR("Starting new auction"); //TODO auction id
        auction_participation_state = MANAGING;
        bid_t bid = startNewAuction();
        scheduleAuctionTermination();
        sendBid(bid, "adhoc_communication/send_em_auction/new_auction");
    }

    auction_mutex.unlock();
}

bid_t AuctionManager::startNewAuction() {    
    bid_t bid;
    bid.auction_id = nextAuctionId();
    bid.robot_id = robot_id;
    bid.bid = bid_computer->getBid();
//    bid.ds_id = optimal_ds_id; //TODO(IMPORTANT)
    bid.ds_id = 20;
    bid.starting_time = time_manager->simulationTimeNow().toSec();
    auction_bids.push_back(bid);

    current_auction.auction_id = bid.auction_id;
    _u2 = current_auction.auction_id;
    ROS_ERROR("%u", _u2);

    return bid;
}

void AuctionManager::sendBid(bid_t bid, std::string topic) {
    ROS_INFO("Sending bid for auction %d to other robots", 1); //TODO
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = topic; //TODO(IMPORTANT)
//    srv.request.dst_robot = group_name; //TODO(IMPORTANT)
    srv.request.auction.auction = bid.auction_id;
    srv.request.auction.robot = bid.robot_id;
    srv.request.auction.docking_station = bid.ds_id;
    srv.request.auction.bid = bid.bid;
    srv.request.auction.starting_time = bid.starting_time;
    ROS_DEBUG("Calling service: %s", sc_send_auction.getService().c_str());
    sc_send_auction.call(srv); //TODO(IMPORTANT) check if call succeded
}

void AuctionManager::scheduleAuctionTermination() { //TODO(IMPORTANT) put also some safety procedure to avoid "un-termination"
    terminate_auction_timer = nh.createTimer(ros::Duration(auction_timeout), &AuctionManager::terminateAuctionCallback, this, true, true); //arguments: Duration _period, const TimerCallback &_callback, CallbackQueueInterface *_queue, bool oneshot=false, bool autostart=true (http://docs.ros.org/indigo/api/roscpp/html/structros_1_1TimerOptions.html)
}

void AuctionManager::terminateAuctionCallback(const ros::TimerEvent &event) {
    auction_mutex.lock();

    if(auction_participation_state != MANAGING)
        ROS_INFO("Auction %d has been aborted because a more recent auction has been started by another robot", 1); //TODO auction_id
    else {
        ROS_INFO("Auction %d has terminated: computing winner", 1); //TODO auction id
        unsigned int winner_id = computeAuctionWinner();
        winner_of_auction = isThisRobotTheWinner(winner_id);
//        timer_restart_auction.stop();

        bid_t bid;
        bid.robot_id = winner_id;
        sendAuctionResult(bid);

        auction_bids.clear();
    }

    auction_participation_state = IDLE;

    auction_mutex.unlock();
}

unsigned int AuctionManager::computeAuctionWinner() {
    bid_t winning_bid;
    winning_bid.robot_id = -1;
    winning_bid.bid = std::numeric_limits<float>::min();
    ROS_ERROR("!!!");

    _u1 = auction_bids.size();

    for (auto it = auction_bids.begin(); it != auction_bids.end(); it++)
    {
        ROS_DEBUG("robot_%d placed %.1f", it->robot_id, it->bid);
        if (it->bid > winning_bid.bid)
        {
            winning_bid.robot_id = it->robot_id;
            winning_bid.bid = it->bid;
        }
    }

    if(winning_bid.robot_id < 0) // this should not happen, since at least the robot that started the auction must have placed a bid
        ROS_ERROR("No winner for auction %d has been found", 1); //TODO auction_id //TODO raise exception?

    ROS_ERROR("The winner of auction %d is robot_%d", 1, winning_bid.robot_id); //TODO auction id
    return winning_bid.robot_id;
}

bool AuctionManager::isThisRobotTheWinner(unsigned int winner_id) {
    if (winner_id == robot_id)
    {
        ROS_INFO("Winner of the auction");  // TODO(minor) specify which auction
        return true;
    }
    else
    {
        ROS_INFO("Robot lost its own auction");
        return false;
    }
}

void AuctionManager::sendAuctionResult(bid_t bid) {
    ROS_INFO("Send results for auction %d to other robots", 1); //TODO
    sendBid(bid, "adhoc_communication/auction_result");
}

void AuctionManager::auctionReplyCallback(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    auction_mutex.lock();
    if(auction_participation_state != MANAGING)
        ROS_INFO("The robot received a bid, but it is not managing an auction: ignore it");
    
    else if (current_auction.auction_id != (unsigned int)msg.get()->auction)
        ROS_INFO("Received a bid that is not for the auction recently started by this robot: ignore it");
    
    else {
//          // probably this is unnecessary, but jsut to be safe...
//        for (auto it = auction_bids.begin(); it != auction_bids.end(); it++)
//            if (it->robot_id == (unsigned int)msg.get()->robot)
//            {
//                ROS_INFO("Received a bid that was already received before: ignore it");
//                return;
//            }

        ROS_DEBUG("Received bid (%f) from robot %d for currenct auction (%u)", msg.get()->bid, msg.get()->robot, current_auction.auction_id);
        bid_t bid;
        bid.robot_id = msg.get()->robot;
        bid.bid = msg.get()->bid;
        auction_bids.push_back(bid);
    }

    auction_mutex.unlock();
}

bool AuctionManager::isRobotParticipatingToAuction() {
    auction_mutex.lock();
    bool return_value = (auction_participation_state == PARTICIPATING || auction_participation_state == MANAGING);
    auction_mutex.unlock();
    return return_value;
}

bool AuctionManager::isRobotWinnerOfMostRecentAuction() {
    auction_mutex.lock();
    bool return_value = winner_of_auction;
    auction_mutex.unlock();
    return return_value;
}

unsigned int AuctionManager::nextAuctionId()
{
    ROS_DEBUG("Compute next auction ID");
    // Examples:
    //     23 * pow(10, (ceil(log10(15)))) + 11 = 23 * 10^2 + 11 = 2300 + 11 = 2311;
    //     23 * pow(10, (ceil(log10(10)))) +  9 = 23 * 10^1 +  9 =  230 +  9 =  239;
    
    local_auction_id++;
    _u5 = local_auction_id;
    _u4 = pow(10, (ceil(log10(num_robots))));

    ROS_ERROR("%d", (int) (local_auction_id * pow(10, (ceil(log10(num_robots)))) + robot_id));

    return local_auction_id * pow(10, (ceil(log10(num_robots)))) + robot_id;
}

void AuctionManager::auctionResultCallback(const adhoc_communication::EmAuction::ConstPtr &msg) {
    auction_mutex.lock();
    if(auction_participation_state == PARTICIPATING) {
        if ((unsigned int)msg.get()->robot == robot_id) //TODO all ids should be unsigned int
        {
            ROS_INFO("Winner of the auction started by another robot");
            winner_of_auction = true;           
        }
        else
        {
            ROS_INFO("Robot didn't win this auction started by another robot");
            winner_of_auction = false;
        }

        auction_participation_state = IDLE;
    }

    auction_mutex.unlock();
}

void AuctionManager::auctionStartingCallback(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    auction_mutex.lock();

    ROS_INFO("Received bid for a new auction (%d)", msg.get()->auction);
    _b1 = true;
    if(auction_participation_state == MANAGING) {
        ROS_INFO("This robot is currently managing an auction...");
        _b2 = true;
        if(current_auction.starting_time < msg.get()->starting_time) {
            ROS_INFO("... and the auction managed by the robot is older than the auction started by the other robot: the auction of this robot will be discarded");
            auction_participation_state = PARTICIPATING;
            _b3 = true;
        } else
            ROS_INFO("... and the auction managed by the robot is more recent than the auction started by the other robot: ignoring the auction started by the other robot");
    }
// TODO sanity checks on auction    
//    if(msg.get()->docking_station < 0 || (int)msg.get()->docking_station >= num_ds) {
//        log_major_error("Invalid id for an auction! Ignoring...");
//        ROS_ERROR("%d", msg.get()->docking_station);
//        return;
//    }
    
    else if(robot_cannot_participate_to_auctions) //TODO(IMPORTANT)
        ROS_INFO("robot is searching for a path on graph: ignore auction");  
     
    //TODO(IMPORTANT)
//    if (optimal_ds_is_set || (int)msg.get()->docking_station != optimal_ds_id)
//    {
//        /* Robot received a bid of an auction whose auctioned docking station is not
//         * the one the robot is interested in
//         * at the moment, so it won't participate to the auction */
//        ROS_INFO("Robot has no interested in participating to this auction");
//    }

    else {
        double bid_double = bid_computer->getBid();
        if (bid_double > msg.get()->bid)
        {    
            ROS_INFO("The robot can place an higher bid than the one received, so it is going to participate to the auction");
            auction_participation_state = PARTICIPATING;
            _b4 = true;
            terminate_auction_timer = nh.createTimer(ros::Duration(auction_timeout + extra_auction_time),                                              &AuctionManager::endAuctionParticipationCallback, this, true, true);

            bid_t bid;
            bid.robot_id = robot_id;
            bid.bid = bid_double;
            sendBid(bid, "adhoc_communication/send_em_auction/reply"); //TODO(IMPORTANT) check topic
        }
        else
            ROS_INFO("The robot has no chance to win, so it won't place a bid for this auction");
    }
    
    auction_mutex.unlock();
}

void AuctionManager::endAuctionParticipationCallback(const ros::TimerEvent &event) 
{
    //TODO(IMPORTANT) FIXME it seems not to work correctly sometimes...
    auction_mutex.lock();
    ROS_DEBUG("Force to consider auction concluded");
    auction_participation_state = IDLE;
    auction_mutex.unlock();
}

void AuctionManager::preventParticipationToAuctions() {
    auction_mutex.lock();
    robot_cannot_participate_to_auctions = true;
    auction_mutex.unlock();
}

void AuctionManager::allowParticipationToAuctions() {
    auction_mutex.lock();
    robot_cannot_participate_to_auctions = false;
    auction_mutex.unlock();
}

void AuctionManager::scheduleNextAuction() {
    auction_mutex.lock();
    timer_restart_auction = nh.createTimer(ros::Duration(reauctioning_timeout), &AuctionManager::restartAuctionCallback, this, true, true);
    waiting_for_next_auction = true; //TODO hmm... could we do better?
    auction_mutex.unlock();
}

void AuctionManager::isRobotWaitingForNextAuction() {
    return waiting_for_next_auction;
}

//TODO the fact that we have to unlock before calling tryToAcquireDs maybe means that we have a bad design...
void AuctionManager::restartAuctionCallback(const ros::TimerEvent &event) {
    auction_mutex.lock();
    ROS_INFO("Timeout for reauctioning");
    if(auction_participation_state != IDLE) {
        ROS_INFO("Robot is already participating to an auction");
        auction_mutex.unlock();
    } else if(winner_of_auction) {
        ROS_INFO("Robot has just won another auction, so let's avoid reauctioning");
        auction_mutex.unlock();
    }
    else {
        auction_mutex.unlock();
        tryToAcquireDs();
    }
    waiting_for_next_auction = false;
}

void AuctionManager::cancelScheduledAuction() {
    auction_mutex.lock();
    ROS_INFO("Timeout for reauctioning");
    timer_restart_auction.stop();
    auction_mutex.unlock();
}
