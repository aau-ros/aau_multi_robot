#include "auction_manager.h"

AuctionManager::AuctionManager() {

}

void AuctionManager::startNewAuction() {
    auction_mutex.lock();
    participating_to_auction = true;
    scheduleAuctionTermination();

//    if(wait_for_ds >= 100)
//            return;
//        
//    mutex_auction.lock();
    
//    if(robot_is_auctioning) {
//        log_minor_error("robot_is_auctioning is true, but shoud be false!!");
////        timer_finish_auction.stop();
////        auction_winner = false;
////        robot_is_auctioning = false;
////        expired_own_auction = true;
////        managing_auction = false;
////        discard_auction = true;
//    }
//    else {
//        discard_auction = false;

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

//        else {
//            // do not use this code
//        //    if (participating_to_auction > 0) {
//        //        ROS_INFO("robot is already participating to some auctions... do not start new one");
//        //        started_own_auction = true; //otherwise a robot won't go in queue!!!!
//        //        return;
//        //    }

//            ROS_INFO("Starting new auction");
//            
//            //optimal_ds_mutex.lock();
//            
//            id_auctioned_ds = get_optimal_ds_id();
//            
//            /* Keep track of robot bid */
//            auction_bid_t bid;
//            bid.robot_id = robot_id;
//        //    if(counter == 0) {
//        //        bid.bid = 100;
//        //        counter++;
//        //    }
//        //    else
//            bid.bid = get_llh();
//            auction_bids.push_back(bid);

//            /* The robot is starting an auction */
//            managing_auction = true;  // TODO(minor) reduntant w.r.t started_own_auction???
//            started_own_auction = true;
//            //update_state_required = true;
//        //    participating_to_auction++; //must be done after get_llh(), or the llh won't be computed correctly //TODO(minor) very bad in this way...

//            robot_is_auctioning = true;
//            start_own_auction_time = ros::Time::now();
//            

//            //optimal_ds_mutex.unlock();

//            /* Start auction timer to be notified of auction conclusion */
//        //    timer_finish_auction.stop();
//        //    timer_finish_auction.setPeriod(ros::Duration(auction_timeout), true);
//        //    timer_finish_auction.start();

//            /* Send broadcast message to inform all robots of the new auction */
//            adhoc_communication::SendEmAuction srv;
//            srv.request.topic = "adhoc_communication/send_em_auction/new_auction";
//            srv.request.dst_robot = group_name;
//            srv.request.auction.auction = next_auction_id();
//            srv.request.auction.robot = robot_id;
//            srv.request.auction.docking_station = get_optimal_ds_id();
//            srv.request.auction.bid = get_llh();
//            ROS_DEBUG("Calling service: %s", sc_send_auction.getService().c_str());
//            sc_send_auction.call(srv);
//        }
//    }
//    
//    mutex_auction.unlock();
//    
//    ROS_INFO("Auction started");

    auction_mutex.unlock();
}

void AuctionManager::scheduleAuctionTermination() { //TODO put also some safety procedure to avoid "un-termination"
    terminate_auction_timer = nh.createTimer(ros::Duration(auction_timeout), &AuctionManager::terminateAuctionCallback, this, true, true); //arguments: Duration _period, const TimerCallback &_callback, CallbackQueueInterface *_queue, bool oneshot=false, bool autostart=true (http://docs.ros.org/indigo/api/roscpp/html/structros_1_1TimerOptions.html)
}

void AuctionManager::terminateAuctionCallback(const ros::TimerEvent &event) {
    computeAuctionWinner();
}

void AuctionManager::computeAuctionWinner() {

}

bool AuctionManager::isRobotParticipatingToAuction() {
    return participating_to_auction;
}

//void AuctionManager::preventAuctionP
