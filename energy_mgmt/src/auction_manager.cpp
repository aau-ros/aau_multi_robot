#include "auction_manager.h"

AuctionManager::AuctionManager() {
    auction_aborted = false;
    winner_of_auction = false;
    participating_to_auction = false;
    ros::NodeHandle nh_tilde("~");
    nh_tilde.param<double>("auction_duration", auction_timeout, 3); //TODO(minor) int?
    nh_tilde.param<double>("extra_auction_time", extra_auction_time, 3); //TODO use getParam and the fact that it returns a bool!!! http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters#getParam.28.29
    nh_tilde.param<double>("reauctioning_timeout", reauctioning_timeout, 10); //s

    bc.getBid();

//    timer_finish_auction = nh.createTimer(ros::Duration(auction_timeout), &docking::timerCallback, this, true, false);
//    timer_restart_auction =
//        nh.createTimer(ros::Duration(reauctioning_timeout), &docking::timer_callback_schedure_auction_restarting, this, true, false);

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

void AuctionManager::createSubscribers() {
//    std::string my_prefix = "";
//    sub_auction_winner_adhoc =
//        nh.subscribe(my_prefix + "adhoc_communication/auction_winner", 1000, &docking::cb_auction_result, this);
//    sub_auction_reply =
//        nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/reply", 1000, &docking::cb_auction_reply, this);
//    sub_auction_starting = nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/new_auction", 1000,
//                                        &docking::cb_new_auction, this);
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
    auction_mutex.lock();

    //TODO check if auction has been aborted meanwhile
    if(auction_aborted) {
        ROS_INFO("Auction %d has been aborted because a more recent auction has been started by another robot", 1); //TODO auction_id //TODO use timestamps
        auction_aborted = false;
    }
    else {
        ROS_INFO("Auction %d has terminated: computing winner", 1); //TODO auction id
        unsigned int winner_id = computeAuctionWinner();
        auction_bids.clear();
        winner_of_auction = isThisRobotTheWinner(winner_id);

            timer_restart_auction.stop();  // TODO(minor) i'm not sure that this follows the
                                           // idea in the paper... jsut put a
                                           // check in the timer callback...
        sendAuctionResult();
    }

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

//    /* Computation completed */
//    
//    ROS_INFO("Auction completed");
////    participating_to_auction--;

//    if(!robot_is_auctioning)
//        log_minor_error("robot_is_auctioning is false but should be true!");
//                
//    robot_is_auctioning = false;
//    expired_own_auction = true;
}

void AuctionManager::sendAuctionResult() {
//    ROS_DEBUG("Send auction results to other robots");
//    adhoc_communication::SendEmAuction srv_msg;
//    srv_msg.request.topic = "adhoc_communication/auction_winner";
//    srv_msg.request.dst_robot = group_name;
//    srv_msg.request.auction.auction = auction_id;
//    srv_msg.request.auction.robot = winner;
//    srv_msg.request.auction.docking_station = get_optimal_ds_id();
//    srv_msg.request.auction.bid = get_llh(); //TODO wrong, although unused (?) because maybe meanwhile

//    // ROS_ERROR("\n\t\e[1;34m%s\e[0m", sc_send_auction.getService().c_str());
//    sc_send_auction.call(srv_msg);
}

bool AuctionManager::isRobotParticipatingToAuction() {
    return participating_to_auction;
}

bool AuctionManager::isRobotWinnerOfMostRecentAuction() {
    return winner_of_auction;
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

//void docking::start_new_auction()
//{
//    auction_manager.startNewAuction();
//}

//void docking::cb_auction_result(const adhoc_communication::EmAuction::ConstPtr &msg)
//{
//    mutex_auction.lock();
//    if (!optimal_ds_is_set())
//    {
//        ROS_INFO("The robot does not know about any existing DS!");  // TODO(minor) it
//                                                                      // means that
//                                                                      // it missed
//                                                                      // some
//                                                                      // messages!!
//        mutex_auction.unlock();
//        return;
//    }

//    // TODO(minor) the robot must check for its participation to the auction!!!

//    // ROS_INFO("Received result of auction ... //TODO(minor) complete!!

//    /* Check if the robot is interested in the docking station that was object of
//     * the auction whose result has been just
//     * received */
//     
//    bool participation = false;
//    for(unsigned int i=0; i < auctions.size(); i++)
//        if(msg.get()->auction == (unsigned int)auctions.at(i).auction_id) {
//            participation = true;
//            break;
//        }
//     
//    if ((int)msg.get()->docking_station == get_optimal_ds_id())  // TODO check if the robot already knows this DS! //TODO what if the robot changes best_ds between the start of this auction and the 
//    {
//        ROS_INFO("Received result of an auction to which the robot participated");  // TODO(minor)
//                                                                                    // acutally
//                                                                                    // maybe
//                                                                                    // it
//                                                                                    // didn't participate because its
//                                                                                    // bid was lost OR because only now its best_ds is the one of the auction...

//        /* Since the robot received the result of an auction to which it took part,
//         * explorer node must be informed of a
//         * possible change in the robot state */
//        update_state_required = true; //TODO maybe it would be better when it receive the bid?

//        /* Check if the robot is the winner of the auction */
//        if (robot_id == (int)msg.get()->robot) //TODO all ids should be unsigned int
//        {
//            /* The robot won the auction */
//            ROS_INFO("Winner of the auction started by another robot");
//            auction_winner = true;
//            lost_other_robot_auction = false;  // TODO(minor) redundant?

//            // TODO(minor) and what if best_ds is updated just a moment before starting the
//            // auction??? it shoul be ok because (in the sense that it couldn't be the winner of this auction, since it didn't participate
//            // the if above would be false
//            //timer_restart_auction.stop();  // TODO(minor)  i'm not sure that this follows the
//                                           // idea in the paper... but probably
//                                           // this is needed otherwise when i have many pendning auction and with a
//                                           // timeout enough high, i could have an inifite loop of restarting
//                                           // auctions... or I could but a control in the timer_callback!!!
////            id_next_optimal_ds = (int)msg.get()->docking_station;


//        }
//        else
//        {
//            /* The robot has lost an auction started by another robot (because the
//             * robot that starts an auction does not
//             * receive the result of that auction with this callback */
//            // TODO(minor) should check if the robto took part to the auction
//            ROS_INFO("Robot didn't win this auction started by another robot");
//            auction_winner = false;
//            lost_other_robot_auction = true;
//            
//        }
//        
////        if(auctions.begin() + index_auction >= auctions.size())
////            log_major_error("auctions.begin() + index_auction >= auctions.size()");
//            
////      participating_to_auction--;            
////        auctions.erase(auctions.begin() + index_auction);

//    }
//    else
//        if(participation)
//            log_major_error("participation is true, but optimal_ds_id != auctioned_id!!!"); //actually this can easily happen
//        else
//            ROS_DEBUG("Received result of an auction the robot was not interested in: "
//                  "ignoring");
//    mutex_auction.unlock();

//}

//void docking::timer_callback_schedure_auction_restarting(const ros::TimerEvent &event)
//{
//    ROS_INFO("Periodic auction timeout");
//    start_periodic_auction();
//}

//void docking::update_robot_state()  // TODO(minor) simplify
//{
//    ROS_INFO("Updating robot state...");
//    
//    mutex_auction.lock();
//    
//    if(robot_is_auctioning) {
//        ROS_INFO("robot is auctioning");
//        if(ros::Time::now() - start_own_auction_time > ros::Duration(auction_timeout))
//        {
//            ROS_INFO("auction concluded!");
//            conclude_auction();
//        }
//        else {
//            ROS_INFO("auction still ongoing...");
//            ROS_INFO("ros::Time::now(): %f", ros::Time::now().toSec());
//            ROS_INFO("start_own_auction_time: %f", start_own_auction_time.toSec());
//        }
//    } else 
//        ROS_INFO("robot has not started its own auction");
//    
//    // sanity check
//    if(!waiting_to_discover_a_ds && robot_state == in_queue && (ros::Time::now() - changed_state_time > ros::Duration(2*60))) {
//        log_major_error("robot stucked in queue!!!!");
//        //timer_finish_auction.stop();
//        auction_winner = false;
//        robot_is_auctioning = false;
//        expired_own_auction = true;
//        managing_auction = false;
//        discard_auction = true;
//        std_msgs::Empty msg;
//        pub_lost_own_auction.publish(msg);  
//        ROS_INFO("pub_lost_own_auction");
//    }
//    
//    // check expired auctions
//    for(auto it = auctions.begin(); it != auctions.end(); )
//        if(ros::Time::now().toSec() - it->starting_time > (float)(auction_timeout + extra_time)) {
//            ROS_INFO("erasing auction");
//            //participating_to_auction--;
//            auctions.erase(it);
//            it = auctions.begin(); //TODO horrible way to restart... check iterator invalidation, etc... or invert scanning order
//        }
//        else {
//            it++;
//            ROS_DEBUG("%f",ros::Time::now().toSec() - it->starting_time);
//        }
//            
//    /*
//     * Check if:
//     * - there are no more pending auctions: this is to avoid to communicate
//     *contradicting information to the explorer
//     *   node about the next state of the robot, so it is better to wait that all
//     *the auctions have been finished and
//     *   then make a "global analisys" of the sistuation;
//     * - an update of the state is required, i.e., that at least one auction was
//     *recently performed and that the robot
//     *   took part to it (i.e., that it placed a bid).
//     *
//     * Notice that it may happen that the energy_mgmt node thinks that an update
//     *of the state is required, but maybe it
//     * is not necessary: it is the explorer node that has to make some ckecks and,
//     *only if necessary, update the state;
//     * the energy_mgmt node just informs the explorer node that something has
//     *recently happened.
//     */
//    if (expired_own_auction || (update_state_required && auctions.size() == 0)) //TODO we could use auctions.size() and delete participatin_to-auction
//    {
//        /* An update of the robot state is required and it can be performed now */
//        ROS_INFO("Sending information to explorer node about the result of recent "
//                 "auctions");

//        /* Create the empty message to be sent */
//        std_msgs::Empty msg;

//        /* If the robot is not the winner of the most recent auction, notify
//         * explorer.
//         * Notice that for sure it took part to at least one auction, or
//         * 'update_state_required' would be false and we
//         * wouldn't be in this then-branch */
//        if (!auction_winner)
//        {
//            if (started_own_auction) {  // TODO(minor) should be better to use a variqable that
//                                      // keep track of teh fact that the robot started
//                                      // its own auction, since if !auction_winner is
//                                      // true, it is already enough to know that the
//                                      // robot needs to recharge (i.e., lost its own
//                                      // auction)...
//                                      /* Notify explorer node about the lost of an auction started by the
//                                       * robot itself */
//                pub_lost_own_auction.publish(msg);  
//                ROS_INFO("pub_lost_own_auction");
//            }

//            /* If the robot has lost an auction that was not started by it, notify
//             * explorer (because if the robot was
//             * recharging, it has to leave the docking station) */
//            else {
//                pub_lost_other_robot_auction.publish(msg);
//                ROS_INFO("pub_lost_other_robot_auction");
//            }
//        }

//        /* Robot is the winner of at least one auction, notify explorer */
//        else
//        {
//            going_to_ds = true; //TODO(minor) very bad way to solve the problem of a robot that has jsut won anotehr robot auction, but the explorer node is still not aware of this fact, and so will communicate to docking that the robot is now in "auctioning" state ...
//            
//            /* If the robot is already approaching a DS to recharge, if it is already
//             *charging, etc., ignore the fact
//             *that meanwhile it has won another auction.
//             *
//             * This check (i.e., the if condition) is necessary because, while moving
//             *toward the DS, the robot could
//             *have changed its currently optimal DS and it could have also taken part
//             *to an auction for this new DS and
//             *won it; without the check the explorer node would be notified for a
//             *change of target DS even if the robot
//             *is still approaching the old one, which could cause problems when the
//             *robot state changes from
//             *checking_vacancy to going_charging, since it would move to the new DS
//             *without doing again the vacancy
//             *check (this is because the move_robot() function in explorer node uses
//             *the coordinates of the currently
//             *set target DS when the robot wants to reach a DS). */
//            if (robot_state != charging || 
//                robot_state != going_charging || 
//                robot_state != going_checking_vacancy ||
//                robot_state != checking_vacancy)
//            {
//                //safety check
//                if(!optimal_ds_is_set())
//                    log_major_error("THIS SHOULD NOT HAPPEN!");
//                    
//                /* Notify explorer node about the new target DS.
//                 * Notice that it is important that target_ds is updated only here,
//                 * because otherwise there could be problem when the robot communicates
//                 * to the other robot that the DS that is currently targettting is now
//                 * vacant/occupied. */
//                
////                bool found_ds = false;
//////                if(id_next_optimal_ds < 0 || id_next_optimal_ds >= num_ds) {
//////                    log_major_error("Invalid id_next_optimal_ds");
//////                    ROS_DEBUG("id_next_target_ds: %d", id_optimal_target_ds);
//////                }
//////                boost::shared_lock< boost::shared_mutex > lock(ds_mutex);
////                for(unsigned int i=0; i < ds.size(); i++)
////                    if(ds[i].id == id_next_optimal_ds && id_next_optimal_ds != get_optimal_ds_id()) {
//////                        best_ds = &ds[i];
//////                        set_target_ds_given_index(i); //necessary because we need to set the DS with ID 'id_next_target_ds' as the target DS, since it could be different from the current target DS //TODO should we do the same also with the ID of the optimal DS?
////                        set_optimal_ds_given_index(i);
////                        found_ds = true;
////                        break;
////                    }
////                if(!found_ds)
////                    ROS_FATAL("this should not happen!!");
//                    
////                geometry_msgs::PointStamped msg1;
////                msg1.point.x = get_target_ds_x();
////                msg1.point.y = get_target_ds_y();
////                pub_new_target_ds.publish(msg1);

//                /* Notify explorer node about the victory */
//                pub_won_auction.publish(msg);  // TODO(minor) it is important that this is after the other pub!!!! can we do better? //TODO probably is reduntant to publish also info about the target ds
//                ROS_INFO("pub_won_auction");
////                ROS_DEBUG("target_ds: %d", get_target_ds_id());

//            }
//            else
//                ROS_INFO("The robot has already won an auction: ignore the result of "
//                          "the most recent auction");
//        }

//        /* Reset all the variables that are used to keep information about the
//         * auctions results (i.e., about the next
//         * robot state) */
//        update_state_required = false;
//        auction_winner = false;
//        started_own_auction = false;
//        expired_own_auction = false;
//        //timers.clear();  // TODO(minor) inefficient!!
//    }
//    else
//    {
//        /* Do nothing, just print some debug text */
//        //if (participating_to_auction > 0 && !update_state_required)
//        if (auctions.size() == 0 && !update_state_required)
//            ROS_DEBUG("There are still pending auctions (%lu), and moreover no update is "
//                      "necessary for the moment", (long unsigned int) auctions.size());
//        else if (!update_state_required)
//            ROS_DEBUG("No state update required");
//        else if (auctions.size() > 0)
//            ROS_DEBUG("There are still pending auctions (%lu), cannot update robot state", (long unsigned int) auctions.size());
//        else {
//            log_major_error("the number of pending auctions is negative!!!"); //TODO can't happen now...
//        }
//    }
//    
//    mutex_auction.unlock();  
//    
//    ROS_INFO("ending update_robot_state()");
//}

//void AuctionManager::cb_auction_reply(const adhoc_communication::EmAuction::ConstPtr &msg)
//{
//    if (!managing_auction)
//    {
//        ROS_INFO("Received a bid that has arrived too late, since the associated auction has already finished: ignore it");
//        return;
//    }
//    
//    if (auction_id != (int)msg.get()->auction)
//    {
//        ROS_INFO("Received a bid that is not for the auction recently started by this robot: ignore it");
//        return;
//    }

//    // TODO(minor) probably this is unnecessary, but jsut to be safe...
//    for (std::vector<auction_bid_t>::iterator it = auction_bids.begin(); it != auction_bids.end(); it++)
//        if ((*it).robot_id == (int)msg.get()->robot)
//        {
//            ROS_INFO("Received a bid that was already received before: ignore it");
//            return;
//        }
//        
//    ROS_DEBUG("Received bid (%f) from robot %d for currenct auction (%d)", msg.get()->bid, msg.get()->robot, auction_id);
//    
//    auction_bid_t bid;
//    bid.robot_id = msg.get()->robot;
//    bid.bid = msg.get()->bid;
//    auction_bids.push_back(bid);
//}

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

//TODO stop auction of a bid from another robot with a "hierarchically higher" id for the same target DS is received
//void docking::cb_new_auction(const adhoc_communication::EmAuction::ConstPtr &msg)
//{
//    ROS_INFO("Received bid for a new auction (%d)", msg.get()->auction);
//    
//    if(msg.get()->docking_station < 0 || (int)msg.get()->docking_station >= num_ds) {
//        log_major_error("Invalid id for an auction! Ignoring...");
//        ROS_ERROR("%d", msg.get()->docking_station);
//        return;
//    }
//    
//    if(robot->state == exploring_for_graph_navigation) {
//        ROS_INFO("robot is searching for a path on graph: ignore auction"); //this is done because the search of a path on the DS graph can take a lot of time (at least with the version that is in used at the time of writing these lines of code), so a robot could basically "stall" a DS, since it could gain access to it, but since it's making computations it cannot used it (and it is not safe to stop the computations, since if we stop them every time that a robot wins an auction, it won't be able to find a path on the graph since it usually takes some time and so it is very likely that the robot will win an auction at some point)
//        return;
//    }
//    
//    // TODO(minor) should do this ckeck also when the robot receive the result of an auction
//    bool already_known_ds = false;
//    for(std::vector<ds_t>::iterator it = discovered_ds.begin(); it != discovered_ds.end(); it++) //TODO(minor) create list discovered_ds + ds
//        if((int)msg.get()->docking_station == (*it).id) {
//            already_known_ds = true;
//            break;
//        }
//    for(std::vector<ds_t>::iterator it = ds.begin(); it != ds.end(); it++)
//        if((int)msg.get()->docking_station == (*it).id) {
//            already_known_ds = true;
//            break;
//        }
//        
//    if(!already_known_ds) {
//        adhoc_communication::SendEmDockingStation srv_msg;
//        srv_msg.request.topic = "adhoc_communication/resend_ds_list";
//        srv_msg.request.dst_robot = group_name; //TODO(minor) use a string everywhere...
//        sc_send_docking_station.call(srv_msg);
//        return;
//    }
//        
//    

//    /*
//    // set auction id
//    if(id > auction_id) // it is a new action from another robot, respond
//        auction_id = id;
//    else if(id > 0){    // it is an old auction from another robot, ignore
//        ROS_ERROR("Bad auction ID, it should be greater than %d!", auction_id);
//        return false;
//    }
//    else                // it is a new auction by this robot
//        ; //++auction_id;
//    */

//    /* Check if the robot has some interested in participating to this auction,
//     * i.e., if the auctioned DS is the one
//     * currently targetted by the robot */
//    



//    //optimal_ds_mutex.lock();  //no need for this
//    mutex_auction.lock();    




//     
//    if (!optimal_ds_is_set() || (int)msg.get()->docking_station != get_optimal_ds_id())
//    {
//        /* Robot received a bid of an auction whose auctioned docking station is not
//         * the one the robot is interested in
//         * at the moment, so it won't participate to the auction */
//        ROS_INFO("Robot has no interested in participating to this auction");
//    }
//    else
//    {
//        if (get_llh() > msg.get()->bid)
//        {
//        
//            
//        
//            /* The robot is interested in participating to the auction */
//            ROS_INFO("The robot can place an higher bid than the one received, so it is going to participate to the auction");

//            /* Start timer to force the robot to consider the auction concluded after
//             * some time, even in case it does not
//             * receive the result of the auction.
//             * To keep the timer active, we must store it on the heap; the timer is
//             * just pushed back in vector 'timers',
//             * which is cleared when no auction is pending (and so no timer is
//             * active). A better management should be
//             * possible, for instance using insert(), at(), etc., but in previous
//             * tries the node always crashed when using
//             * these functions... */
////            ros::Timer timer = nh.createTimer(ros::Duration(auction_timeout + extra_time),
////                                              &docking::end_auction_participation_timer_callback, this, true, false);
//            //timer.start();
//            //timers.push_back(timer);
//              
//            auction_t new_auction;
//            new_auction.starting_time = (double)ros::Time::now().toSec();
//            new_auction.auction_id = msg.get()->auction;
////            participating_to_auction++;
//            auctions.push_back(new_auction);           

//            adhoc_communication::SendEmAuction srv;
//            srv.request.dst_robot = group_name;
//            srv.request.topic = "adhoc_communication/send_em_auction/reply";
//            srv.request.auction.auction = msg.get()->auction;
//            srv.request.auction.robot = robot_id;
//            srv.request.auction.docking_station = get_optimal_ds_id();
//            srv.request.auction.bid = get_llh();
//            ROS_DEBUG("Calling service: %s", sc_send_auction.getService().c_str());
//            sc_send_auction.call(srv);
//        }
//        else
//        {
//            ROS_INFO("The robot has no chance to win, so it won't place a bid for "
//                     "this auction");
//        }
//    }
//    
//    mutex_auction.unlock();
//    //optimal_ds_mutex.unlock();    
//    
//}

// TODO(minor) what if instead the
                                                                                      // auction result are received
                                                                                      // after this timer? what if
                                                                                      // instead it is received a lot
                                                                                      // early?
//void docking::end_auction_participation_timer_callback(const ros::TimerEvent &event) 
                                                                                      
//{
    //FIXME it seems not to work correctly sometimes...
    //ROS_DEBUG("Force to consider auction concluded");
    //participating_to_auction--;
//}

//void AuctionManager::preventAuctionP
