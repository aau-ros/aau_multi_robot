void docking::handle_robot_state()
{    
    if((next_robot_state == robot_state::CHARING_COMPLETED || next_robot_state == robot_state::CHARING_ABORTED) && robot_state != next_robot_state) {
        free_ds(id_ds_to_be_freed);
        has_to_free_optimal_ds = false;
    }    

    if (next_robot_state == robot_state::CHARGING)
    {
        id_ds_to_be_freed = get_optimal_ds_id();
        has_to_free_optimal_ds = true;
        set_optimal_ds_vacant(false); // we could thing of doing it ealrly... but this would mean that the other robots will think that a DS is occupied even if it is not, which means that maybe one of them could have a high value of the llh and could get that given DS, but instead the robot will give up and it will try with another DS (this with the vacant stragety), so it could be disadvantaging...
    }

    else if (next_robot_state == robot_state::CHECKING_VACANCY)
    {
        adhoc_communication::SendEmDockingStation srv_msg;
        srv_msg.request.topic = "adhoc_communication/check_vacancy";
        srv_msg.request.dst_robot = group_name;
        srv_msg.request.docking_station.id = get_optimal_ds_id();
        srv_msg.request.docking_station.header.message_id = getAndUpdateMessageIdForTopic("adhoc_communication/check_vacancy");
        srv_msg.request.docking_station.header.sender_robot = robot_id;
        sc_send_docking_station.call(srv_msg);
    }

    else if (next_robot_state == robot_state::AUCTIONING)
    {
        if(!optimal_ds_is_set())
            log_major_error("!optimal_ds_is_set");
    }

    else if (next_robot_state == auctioning_3) {
        compute_and_publish_path_on_ds_graph_to_home();
        if(finished_bool) {
            ROS_ERROR("No more frontiers..."); //TODO(minor) probably this checks are reduntant with the ones of explorer
            std_msgs::Empty msg;
            pub_finish.publish(msg);
        } 
    }
        
    robot_state = next_robot_state;
    robot->state = robot_state;

    send_robot();

}

void docking::get_robot_state() {
    robot_state::GetRobotState msg;
    while(!get_robot_state_sc.call(msg))
        ROS_ERROR("get state failed");
    next_robot_state = static_cast<robot_state::robot_state_t>(msg.response.robot_state);
}
