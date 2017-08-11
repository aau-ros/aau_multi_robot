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

//    else {
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
//    }
