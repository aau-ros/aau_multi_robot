void docking::send_ds() {
    ROS_INFO("send_ds");
    adhoc_communication::SendEmDockingStation srv_msg;
    srv_msg.request.topic = "docking_stations";
    srv_msg.request.dst_robot = group_name;
    
    ds_mutex.lock();
    
//    boost::shared_lock< boost::shared_mutex > lock(ds_mutex);
    for(unsigned int i=0; i < ds.size(); i++) {
        double x, y;
        rel_to_abs(ds.at(i).x, ds.at(i).y, &x, &y);
        srv_msg.request.docking_station.x = x;  // it is necessary to fill also this fields because when a Ds is
                                                // received, robots perform checks on the coordinates
        srv_msg.request.docking_station.y = y;
        srv_msg.request.docking_station.id = ds.at(i).id;
        srv_msg.request.docking_station.vacant = ds.at(i).vacant;
        srv_msg.request.docking_station.header.timestamp = ds.at(i).timestamp; //TODO should timestamp be part of ds_t or not ???
        srv_msg.request.docking_station.header.sender_robot = robot_id;
        srv_msg.request.docking_station.header.message_id = getAndUpdateMessageIdForTopic("docking_stations");
        sc_send_docking_station.call(srv_msg);
    }
    
    ds_mutex.unlock();
}

unsigned int docking::getAndUpdateMessageIdForTopic(std::string topic) {
    mutex_message.lock();
    auto search = topic_ids.find(topic);
    if(search == topic_ids.end()) {
        topic_ids.insert({topic, 1});
        search = topic_ids.find(topic);
    }
    unsigned int return_value = search->second * pow(10, (ceil(log10(num_robots)))) + robot_id;
    search->second++;
    mutex_message.unlock();
    return return_value;
}

bool docking::checkAndUpdateReceivedMessageId(std::string topic, unsigned int message_id, unsigned int sender_robot_id) {
    bool return_value = false;
    auto search = received_topic_ids.find(topic);
    unsigned int local_id = (unsigned int)(floor(message_id / pow(10, (ceil(log10(num_robots))))));
    if(search == received_topic_ids.end()) {
//        ROS_FATAL("invalid topic"); //TODO insert automatically? but in this case we cannot avoid errors due to wrong topics
        received_topic_ids.insert({topic, {}});
        search = received_topic_ids.find(topic);
        ROS_INFO("inserted topic %s", topic.c_str());
    }
    if(search != received_topic_ids.end())
    {
        auto search2 = search->second.find(sender_robot_id);
        if(search2 == search->second.end()) {
//            ROS_FATAL("invalid robot");
            search->second.insert({sender_robot_id, local_id - 1});
            search2 = search->second.find(sender_robot_id);
            ROS_INFO("inserted robot %u for topic %s", sender_robot_id, topic.c_str());
        }
        if(search2 != search->second.end()) 
        {
            unsigned int expected_id = search2->second + 1;
            return_value = (local_id == expected_id);
            search2->second = local_id;
            if(!return_value) {
                ROS_DEBUG("message_id: %u", message_id);
                ROS_DEBUG("local_id: %u", local_id);
                ROS_DEBUG("expected_id: %u", expected_id);
            }
        }            
    }

    return return_value;
}

void docking::send_optimal_ds() {
    adhoc_communication::EmDockingStation msg_optimal;
    msg_optimal.id = get_optimal_ds_id();
    msg_optimal.x = get_optimal_ds_x();
    msg_optimal.y = get_optimal_ds_y();
    pub_new_optimal_ds.publish(msg_optimal);
}

void docking::send_robot()
{    
    ROS_INFO("send_robot");
    
    adhoc_communication::SendEmRobot robot_msg;
    robot_msg.request.dst_robot = group_name;
    robot_msg.request.topic = "robots";
    robot_msg.request.robot.id = robot_id;
    robot_msg.request.robot.x = robot->x;
    robot_msg.request.robot.y = robot->y;
    robot_msg.request.robot.home_world_x = robot->home_world_x;
    robot_msg.request.robot.home_world_y = robot->home_world_y;
    robot_msg.request.robot.state = robot->state;
    robot_msg.request.robot.header.message_id = getAndUpdateMessageIdForTopic("robots");
    robot_msg.request.robot.header.sender_robot = robot_id;
    robot_msg.request.robot.header.timestamp = ros::Time::now().toSec();
    
    if (optimal_ds_is_set())
        robot_msg.request.robot.selected_ds = get_optimal_ds_id();
    else
        robot_msg.request.robot.selected_ds = -10;
    sc_send_robot.call(robot_msg);
    
    adhoc_communication::EmRobot robot_msg_2;
    robot_msg_2.home_world_x = robot->home_world_x;
    robot_msg_2.home_world_y = robot->home_world_y;
    pub_this_robot.publish(robot_msg_2);

    ros::Duration time = ros::Time::now() - time_start;
    fs2_csv.open(csv_file_2.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs2_csv << ros::Time::now().toSec() << "," << ros::WallTime::now().toSec() << "," << robot->x << "," << robot->y << std::endl;
    fs2_csv.close();
    
    ROS_INFO("robot sent");
}

