#include "auction_manager.h"

AuctionManager::AuctionManager() {
    initializeVariables();

//    if (msg.get()->state == in_queue)
//    {
//        ROS_DEBUG("Starting timer_restart_auction");
//        /* Schedule next auction (a robot goes in queue only if it has lost an
//         * auction started by itself? NOOOO!!! it could win is auction, then
//         * immediately lose a following one that was sovrapposing with the one
//         * started by it, so when both auctions are completed, the robot will
//         * seem to be lost only an auction started by another robot!!!*/
//        timer_restart_auction.stop(); //reduntant ?
//        timer_restart_auction.setPeriod(ros::Duration(reauctioning_timeout), true);
//        timer_restart_auction.start();
//        
//        //ROS_ERROR("Robot in queue!!!");
//    }

//    fs_info.open(info_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//    fs_info << "#robot_id,num_robots,ds_selection_policy,starting_absolute_x,"
//               "starting_absolute_y,w1,w2,w3,w4,auction_duration,reauctioning_timeout,extra_time" << std::endl;
//    fs_info << robot_id << "," << num_robots << "," << ds_selection_policy << "," << origin_absolute_x << ","
//            << origin_absolute_y << "," << w1 << "," << w2 << "," << w3 << "," << w4 << "," << auction_timeout << "," << reauctioning_timeout << "," << extra_time << std::endl;
//    fs_info.close();
}

void AuctionManager::initializeVariables() {
    auction_aborted = false;
    winner_of_auction = false;
    auction_participation_state = IDLE;
    ros::NodeHandle nh_tilde("~");
    nh_tilde.param<double>("auction_duration", auction_timeout, 3); //TODO(minor) int?
    nh_tilde.param<double>("extra_auction_time", extra_auction_time, 3); //TODO use getParam and the fact that it returns a bool!!! http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters#getParam.28.29
    nh_tilde.param<double>("reauctioning_timeout", reauctioning_timeout, 10); //s
}

void AuctionManager::createSubscribers() {
    std::string my_prefix = ""; //TODO
    auction_reply_sub = nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/auction_reply", 1000, &AuctionManager::auctionReplyCallback, this);
    auction_result_sub = nh.subscribe(my_prefix + "adhoc_communication/auction_result", 1000, &AuctionManager::auctionResultCallback, this); //TODO do i need 'adhoc_communication' in the service name?
    auction_starting_sub = nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/auction_starting", 1000, &AuctionManager::auctionStartingCallback, this);
}

void AuctionManager::createServiceClients() {
    std::string my_prefix = ""; //TODO
    sc_send_auction = nh.serviceClient<adhoc_communication::SendEmAuction>(my_prefix + "adhoc_communication/send_em_auction");
}

void AuctionManager::tryToAcquireDs() {
    auction_mutex.lock();

//    if(wait_for_ds >= 100)
//            return;

//        if (!optimal_ds_is_set() && need_to_charge)
//        {
//            waiting_to_discover_a_ds = true;
//            log_minor_error("The robot needs to recharge, but it doesn't know about any "
//                      "existing DS!");  // TODO(minor) improve...
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

    if(auction_participation_state == PARTICIPATING)
        ROS_INFO("The robot is already participating to an auction: let's wait for the other auction to terminate");
    else {
        ROS_INFO("Starting new auction"); //TODO auction id
        auction_participation_state = MANAGING;
        bid_t bid = startNewAuction();
        scheduleAuctionTermination();
        sendBid(bid, "adhoc_communication/send_em_auction/new_auction");
    }
}

bid_t AuctionManager::startNewAuction() {    
    bid_t bid;
    bid.auction_id = 1; //TODO
    bid.robot_id = robot_id;
    bid.bid = bc.getBid();
    bid.ds_id = optimal_ds_id;
    auction_bids.push_back(bid);
    return bid;
}

void AuctionManager::sendBid(bid_t bid, std::string topic) {
    ROS_INFO("Sendind bid for auction %d to other robots", 1); //TODO
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = topic; //TODO
//    srv.request.dst_robot = group_name;
//    srv.request.auction.auction = next_auction_id();
//    srv.request.auction.robot = robot_id;
//    srv.request.auction.docking_station = get_optimal_ds_id();
//    srv.request.auction.bid = get_llh();
    ROS_DEBUG("Calling service: %s", sc_send_auction.getService().c_str());
    sc_send_auction.call(srv); //TODO check if call succeded
}

void AuctionManager::scheduleAuctionTermination() { //TODO put also some safety procedure to avoid "un-termination"
    terminate_auction_timer = nh.createTimer(ros::Duration(auction_timeout), &AuctionManager::terminateAuctionCallback, this, true, true); //arguments: Duration _period, const TimerCallback &_callback, CallbackQueueInterface *_queue, bool oneshot=false, bool autostart=true (http://docs.ros.org/indigo/api/roscpp/html/structros_1_1TimerOptions.html)
}

void AuctionManager::terminateAuctionCallback(const ros::TimerEvent &event) {
    auction_mutex.lock();

    //TODO check if auction has been aborted meanwhile
    if(auction_aborted) {
        ROS_INFO("Auction %d has been aborted because a more recent auction has been started by another robot", 1); //TODO auction_id //TODO use timestamps
        auction_aborted = false;
    }
    else {
        ROS_INFO("Auction %d has terminated: computing winner", 1); //TODO auction id
        unsigned int winner_id = computeAuctionWinner();
        
        winner_of_auction = isThisRobotTheWinner(winner_id);

        timer_restart_auction.stop();  // TODO(minor) i'm not sure that this follows the idea in the paper... jsut put a check in the timer callback...
        bid_t bid;
        bid.robot_id = winner_id;
        sendAuctionResult(bid);

        auction_bids.clear();
    }

    auction_participation_state = IDLE;

    auction_mutex.unlock();
}

unsigned int AuctionManager::computeAuctionWinner() {
    unsigned int winner_id = -1;
    float winner_bid = std::numeric_limits<float>::min();

    for (auto it = auction_bids.begin(); it != auction_bids.end(); it++)
    {
        ROS_DEBUG("robot_%d placed %f", it->robot_id, it->bid);
        if (it->bid > winner_bid)
        {
            winner_id = it->robot_id;
            winner_bid = it->bid;
        }
    }

    if(winner_id < 0) // this should not happen, since at least the robot that started the auction must have placed a bid
        ROS_ERROR("No winner for auction %d has been found", 1); //TODO auction_id //TODO raise exception?

    ROS_DEBUG("The winner of auction %d is robot_%d", 1, winner_id); //TODO auction id
    return winner_id;
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

//void docking::start_periodic_auction() {
//    ROS_INFO("Periodic re-auctioning");
//    
//    //start auction only if no other one for teh same ds is on going: this is to avoid an "infinite loop" of auctions"
////    if (participating_to_auction == 0)  // Notice that it is still possible that
//                                        // two robots start an auction at the same
//                                        // time...
//        ROS_INFO("calling start_new_auction()");                          
//        start_new_auction();
////    else
////    {
////        started_own_auction = true;  // otherwise a robot could not start the auction
////                                     // because the following if is true, then win
////                                     // another robot auction and stop the time, then
////                                     // lost another and not be reset in queue... //TODO(minor) not very clean...
////        ROS_INFO("Robot is already participating to an auction: let's wait "
////                  "instead of starting another one...");
////        timer_restart_auction.stop(); //reduntant?
////        timer_restart_auction.setPeriod(ros::Duration(reauctioning_timeout), true);
////        timer_restart_auction.start(); //TODO would be better to start the timer only when the robot losts the auction...
////    }
//}

//void docking::timer_callback_schedure_auction_restarting(const ros::TimerEvent &event)
//{
//    ROS_INFO("Periodic auction timeout");
//    start_periodic_auction();
//}

void AuctionManager::auctionReplyCallback(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    auction_mutex.lock();

    if(auction_participation_state != MANAGING)
        ROS_INFO("Received a bid that has arrived too late, since the associated auction has already finished: ignore it");
    
    else if (current_auction.auction_id != (unsigned int)msg.get()->auction)
        ROS_INFO("Received a bid that is not for the auction recently started by this robot: ignore it");
    
    else {

        // TODO(minor) probably this is unnecessary, but jsut to be safe...
        for (auto it = auction_bids.begin(); it != auction_bids.end(); it++)
            if (it->robot_id == (unsigned int)msg.get()->robot)
            {
                ROS_INFO("Received a bid that was already received before: ignore it");
                return;
            }
            
        ROS_DEBUG("Received bid (%f) from robot %d for currenct auction (%u)", msg.get()->bid, msg.get()->robot, current_auction.auction_id);
        bid_t bid;
        bid.robot_id = msg.get()->robot;
        bid.bid = msg.get()->bid;
        auction_bids.push_back(bid);

    }

    auction_mutex.unlock();
}

bool AuctionManager::isRobotParticipatingToAuction() {
    return (auction_participation_state == PARTICIPATING || auction_participation_state == MANAGING);
}

bool AuctionManager::isRobotWinnerOfMostRecentAuction() {
    return winner_of_auction;
}

//int docking::next_auction_id()
//{
//    ROS_DEBUG("Compute next auction ID");
//    
//    /* Increase local auction ID, and then return the global one */
//    local_auction_id++;
//    auction_id = local_auction_id * pow(10, (ceil(log10(num_robots)))) + robot_id;
//    return auction_id;
//    
//    /*
//    Examples:
//    23 * pow(10, (ceil(log10(15)))) + 11 = 23 * 10^2 + 11 = 2300 + 11 = 2311;
//    23 * pow(10, (ceil(log10(10)))) +  9 = 23 * 10^1 +  9 =  230 +  9 =  239;
//    */
//}

void AuctionManager::auctionResultCallback(const adhoc_communication::EmAuction::ConstPtr &msg) {
    auction_mutex.lock();
     
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

    auction_mutex.unlock();
}

void AuctionManager::auctionStartingCallback(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    auction_mutex.lock();

    ROS_INFO("Received bid for a new auction (%d)", msg.get()->auction);

    if(auction_participation_state == MANAGING) {
        ROS_INFO("This robot is currently managing an auction...");
        if(current_auction.starting_time < msg.get()->starting_time) {
            ROS_INFO("... and the auction managed by the robot is older than the auction started by the other robot: the auction of this robot will be discarded");
            auction_participation_state = PARTICIPATING;
        } else
            ROS_INFO("... and the auction managed by the robot is more recent than the auction started by the other robot: ignoring the auction started by the other robot");
    }
// TODO sanity checks on auction    
//    if(msg.get()->docking_station < 0 || (int)msg.get()->docking_station >= num_ds) {
//        log_major_error("Invalid id for an auction! Ignoring...");
//        ROS_ERROR("%d", msg.get()->docking_station);
//        return;
//    }
    
    else if(robot_cannot_participate_to_auctions) //TODO
        ROS_INFO("robot is searching for a path on graph: ignore auction"); //this is done because the search of a path on the DS graph can take a lot of time (at least with the version that is in used at the time of writing these lines of code), so a robot could basically "stall" a DS, since it could gain access to it, but since it's making computations it cannot used it (and it is not safe to stop the computations, since if we stop them every time that a robot wins an auction, it won't be able to find a path on the graph since it usually takes some time and so it is very likely that the robot will win an auction at some point)
    
    else {
//        // TODO(minor) should do this ckeck also when the robot receive the result of an auction
//        bool already_known_ds = false;
//        for(std::vector<ds_t>::iterator it = discovered_ds.begin(); it != discovered_ds.end(); it++) //TODO(minor) create list discovered_ds + ds
//            if((int)msg.get()->docking_station == (*it).id) {
//                already_known_ds = true;
//                break;
//            }
//        for(std::vector<ds_t>::iterator it = ds.begin(); it != ds.end(); it++)
//            if((int)msg.get()->docking_station == (*it).id) {
//                already_known_ds = true;
//                break;
//            }
//            
//        if(!already_known_ds) {
//            adhoc_communication::SendEmDockingStation srv_msg;
//            srv_msg.request.topic = "adhoc_communication/resend_ds_list";
//            srv_msg.request.dst_robot = group_name; //TODO(minor) use a string everywhere...
//            sc_send_docking_station.call(srv_msg);
//            return;
//        }
    }  
     
//    if (optimal_ds_is_set || (int)msg.get()->docking_station != optimal_ds_id)
//    {
//        /* Robot received a bid of an auction whose auctioned docking station is not
//         * the one the robot is interested in
//         * at the moment, so it won't participate to the auction */
//        ROS_INFO("Robot has no interested in participating to this auction");
//    }
//    else
    {
        double bid_double = bc.getBid();
        if (bid_double > msg.get()->bid)
        {    
            ROS_INFO("The robot can place an higher bid than the one received, so it is going to participate to the auction");

            terminate_auction_timer = nh.createTimer(ros::Duration(auction_timeout + extra_auction_time),                                              &AuctionManager::end_auction_participation_timer_callback, this, true, true);

            bid_t bid;
            bid.robot_id = robot_id;
            bid.bid = bid_double;
            sendBid(bid, "adhoc_communication/send_em_auction/reply"); //TODO check topic
        }
        else
            ROS_INFO("The robot has no chance to win, so it won't place a bid for this auction");
    }
    
    auction_mutex.unlock();
}

// TODO(minor) what if instead the auction result are received after this timer? what if instead it is received a lot early?
void AuctionManager::end_auction_participation_timer_callback(const ros::TimerEvent &event) 
{
    //FIXME it seems not to work correctly sometimes...
    ROS_DEBUG("Force to consider auction concluded");
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
