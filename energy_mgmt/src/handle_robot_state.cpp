void docking::handle_robot_state()
{
    // TODO(minor) better update...
        
    if(robot_state == robot_state::CHARGING && (next_robot_state != robot_state::LEAVING_DS && next_robot_state != finished)) {
        log_major_error("invalid state after robot_state::CHARGING!!!");
        ROS_INFO("current state: robot_state::CHARGING");   
        ROS_INFO("next state: %d", next_robot_state);
    }
    
    if(has_to_free_optimal_ds && next_robot_state == robot_state::LEAVING_DS) {
//        set_optimal_ds_vacant(true);
        free_ds(id_ds_to_be_freed);
        has_to_free_optimal_ds = false;
    }
        
    if (next_robot_state != robot_state::GOING_CHECKING_VACANCY) //TODO(minor) very bad... maybe in if(... == robot_state::CHECKING_VACANCY) would be better...
        going_to_ds = false;
        
//    if (has_to_free_optimal_ds && next_robot_state != fully_charged && next_robot_state != robot_state::LEAVING_DS) //TODO maybe since we put the DS as occupied only when we start charging, we could put it as free when we leave it already (put are we sure that this doens't cause problems somewhere else?)... although the robot_state::LEAVING_DS state is so short that it makes almost no different
//     {
//            has_to_free_optimal_ds = false;
//            set_optimal_ds_vacant(true);
//    }

    if (next_robot_state == robot_state::IN_QUEUE)
    {
        ;
    }
    else if (next_robot_state == robot_state::GOING_CHARGING)
    {
        ;  // ROS_ERROR("\n\t\e[1;34m Robo t going charging!!!\e[0m");
    }
    else if (next_robot_state == robot_state::CHARGING)
    {
        id_ds_to_be_freed = get_optimal_ds_id();
        has_to_free_optimal_ds = true;
        set_optimal_ds_vacant(false); // we could thing of doing it ealrly... but this would mean that the other robots will think that a DS is occupied even if it is not, which means that maybe one of them could have a high value of the llh and could get that given DS, but instead the robot will give up and it will try with another DS (this with the vacant stragety), so it could be disadvantaging...
    }
    else if (next_robot_state == robot_state::GOING_CHECKING_VACANCY)
    {
        ;  // ROS_ERROR("\n\t\e[1;34m going checking vacancy!!!\e[0m");
    }
    else if (next_robot_state == robot_state::CHECKING_VACANCY)
    {
//        if(get_target_ds_id() < 0 || get_target_ds_id() >= num_ds) {
//            log_major_error("sending invalid DS id 5!!!");
//            ROS_ERROR("%d",  get_target_ds_id());  
//        }
    
        ;  // ROS_ERROR("\n\t\e[1;34m checking vacancy!!!\e[0m");
        adhoc_communication::SendEmDockingStation srv_msg;
        srv_msg.request.topic = "adhoc_communication/check_vacancy";
        srv_msg.request.dst_robot = group_name;
//        srv_msg.request.docking_station.id = get_target_ds_id();  // target_ds, not best_ds!!!!!
        srv_msg.request.docking_station.id = get_optimal_ds_id();  // target_ds, not best_ds!!!!!
        srv_msg.request.docking_station.header.message_id = getAndUpdateMessageIdForTopic("adhoc_communication/check_vacancy");
        srv_msg.request.docking_station.header.sender_robot = robot_id;
        sc_send_docking_station.call(srv_msg);
    }
    else if (next_robot_state == robot_state::AUCTIONING)
    {
        ROS_INFO("Robot needs to recharge");
        if(!going_to_ds) //TODO(minor) very bad check... to be sure that only if the robot has not just won
                                  // another auction it will start its own (since maybe explorer is still not aware of this and so will communicate "robot_state::AUCTIONING" state...); do we have other similar problems?
            ROS_ERROR("calling start_new_auction()");
//            start_new_auction(); 

        if(!optimal_ds_is_set())
            log_major_error("!optimal_ds_is_set");
                                  // TODO(minor) only if the robot has not been just interrupted from recharging
    }
    else if (next_robot_state == auctioning_2) {
        if(ds_selection_policy == 2)
		{
			if(finished_bool) {
                ROS_ERROR("No more frontiers..."); //TODO(minor) probably this checks are reduntant with the ones of explorer
                std_msgs::Empty msg;
                pub_finish.publish(msg);
            } else {
		        ROS_INFO("Robot needs to recharge");
		        if(!going_to_ds) //TODO(minor) very bad check... to be sure that only if the robot has not just won
		                                  // another auction it will start its own (since maybe explorer is still not aware of this and so will communicate "robot_state::AUCTIONING" state...); do we have other similar problems?
		        {
		            ros::Duration(1).sleep();
		            ROS_ERROR("calling start_new_auction()");
//		            start_new_auction();
		        }
			}
	    } else if(graph_navigation_allowed) {
//	        moving_along_path = true;
//	        compute_and_publish_path_on_ds_graph();
//            if(finished_bool) {
//                std_msgs::Empty msg;
//                pub_finish.publish(msg);
//            } else {
//                ROS_INFO("Robot needs to recharge");
//                if(!going_to_ds) //TODO(minor) very bad check... to be sure that only if the robot has not just won
//                                          // another auction it will start its own (since maybe explorer is still not aware of this and so will communicate "robot_state::AUCTIONING" state...); do we have other similar problems?
//                {
//                    ros::Duration(10).sleep();
//                    start_new_auction();
//                }
//            }
        } else {
            ROS_ERROR("DS graph cannot be navigated with this strategy...");
            ROS_INFO("DS graph cannot be navigated with this strategy...");
            std_msgs::Empty msg;
            pub_finish.publish(msg);
        }   
    }
    else if (next_robot_state == auctioning_3) {
        if(graph_navigation_allowed) {
	        compute_and_publish_path_on_ds_graph_to_home();
            if(finished_bool) {
                ROS_ERROR("No more frontiers..."); //TODO(minor) probably this checks are reduntant with the ones of explorer
                std_msgs::Empty msg;
                pub_finish.publish(msg);
            } else {
                ROS_INFO("Robot needs to recharge");
                if(!going_to_ds) //TODO(minor) very bad check... to be sure that only if the robot has not just won
                                          // another auction it will start its own (since maybe explorer is still not aware of this and so will communicate "robot_state::AUCTIONING" state...); do we have other similar problems?
                {
                    ros::Duration(10).sleep();
                    ROS_ERROR("calling start_new_auction()");
//                    start_new_auction();
                }
            }
        } else {
            ROS_ERROR("DS graph cannot be navigated with this strategy...");
            ROS_INFO("DS graph cannot be navigated with this strategy...");
            std_msgs::Empty msg;
            pub_finish.publish(msg);
        }   
    }
    else if (next_robot_state == robot_state::LEAVING_DS)
    {
        going_to_ds = false;
        //free_ds(id_ds_to_be_freed); //it is better to release the DS when the robot has exited the fully_charged or robot_state::LEAVING_DS state, but sometimes (at the moment for unknown reasones) this takes a while, even if the robot has already phisically released the DS...
    }
    else if (next_robot_state == robot_state::MOVING_TO_FRONTIER || next_robot_state == robot_state::GOING_IN_QUEUE)
        ;
    else if(next_robot_state == robot_state::COMPUTING_NEXT_GOAL)
    {
        ;
    }
    else if (next_robot_state == finished)
    {
        finalize();
    }
    else if (next_robot_state == stuck)
    {
        finalize();
    }
    else if (next_robot_state == stopped)
    {
        ;
    }
    else if (next_robot_state == exploring_for_graph_navigation)
    {
        ;
    }
    else
    {
        ROS_FATAL("\n\t\e[1;34m none of the above!!!\e[0m");
        return;
    }
        
    robot_state = next_robot_state;
    robot->state = robot_state;
    
    changed_state_time = ros::Time::now();

    send_robot();

}

void docking::get_robot_state() {
    robot_state::GetRobotState msg;
    while(!get_robot_state_sc.call(msg))
        ROS_ERROR("get state failed");
    next_robot_state = static_cast<robot_state::robot_state_t>(msg.response.robot_state);
}
