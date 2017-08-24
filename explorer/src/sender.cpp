

bool ExplorationPlanner::sendToMulticast(std::string multi_cast_group, adhoc_communication::ExpFrontier frontier_to_send, std::string topic)
{
//    ROS_INFO("SENDING frontier id: %ld    detected_by: %ld", frontier_to_send.id ,frontier_to_send.detected_by_robot);
    adhoc_communication::SendExpFrontier service_frontier; // create request of type any+

    std::stringstream robot_number;
    robot_number << robot_name;
    std::string prefix = "robot_";
    std::string robo_name = prefix.append(robot_number.str());

    if(robot_prefix_empty_param == true)
    {
        robo_name = robot_str;
//        robo_name = lookupRobotName(robot_name);
    }
    std::string destination_name = multi_cast_group + robo_name; //for multicast
//    std::string destination_name = robo_name; // unicast

    ROS_INFO("sending to multicast group '%s' on topic: '%s'",destination_name.c_str(), topic.c_str());
    service_frontier.request.dst_robot = destination_name;
    service_frontier.request.frontier = frontier_to_send;
    service_frontier.request.topic = topic;

    if (ssendFrontier.call(service_frontier))
    {
            ROS_DEBUG("Successfully called service  sendToMulticast");

            if(service_frontier.response.status)
            {
                    ROS_DEBUG("adhoc comm returned successful transmission");
                    return true;
            }
            else
            {
                ROS_DEBUG("Failed to send to multicast group %s!",destination_name.c_str());
                return false;
            }
    }
    else
    {
     ROS_WARN("Failed to call service sendToMulticast [%s]",ssendFrontier.getService().c_str());
     return false;
    }
}

bool ExplorationPlanner::sendToMulticastAuction(std::string multi_cast_group, adhoc_communication::ExpAuction auction_to_send, std::string topic)
{
    adhoc_communication::SendExpAuction service_auction; // create request of type any+

    std::stringstream robot_number;
    robot_number << robot_name;
    std::string prefix = "robot_";
    std::string robo_name = prefix.append(robot_number.str());

    if(robot_prefix_empty_param == true)
    {
//        robo_name = lookupRobotName(robot_name);
        robo_name = robot_str;
    }

    std::string destination_name = multi_cast_group + robo_name; //for multicast
//    std::string destination_name = robo_name; // unicast

    ROS_INFO("sending auction to multicast group '%s' on topic '%s'",destination_name.c_str(), topic.c_str());
    service_auction.request.dst_robot = destination_name;
    service_auction.request.auction = auction_to_send;
    service_auction.request.topic = topic;

    if (ssendAuction.call(service_auction))
    {
            ROS_DEBUG("Successfully called service sendToMulticast");

            if(service_auction.response.status)
            {
                    ROS_DEBUG("Auction was multicasted successfully.");
                    return true;
            }
            else
            {
                ROS_WARN("Failed to send auction to mutlicast group %s!",destination_name.c_str());
                return false;
            }
    }
    else
    {
     ROS_WARN("Failed to call service sendToMulticastAuction [%s]",ssendAuction.getService().c_str());
     return false;
    }
}

bool ExplorationPlanner::compute_and_publish_ds_path(double max_available_distance, int *result) {
    //just compute the goal ds for the moment //TODO complete
    ROS_DEBUG("Searching path on DS graph; %u frontiers exist", frontiers.size());
    double min_dist = std::numeric_limits<int>::max();
    ds_t *min_ds = NULL;
    int retry = 0;
    
    // "safety" initialization (if the caller gets -1 as a value, something went wrong)
    *result = -1;
    
    // search for the closest DS with EOs
//    while (min_ds == NULL && retry < 5) {
//        for (unsigned int i = 0; i < ds_list.size(); i++)
//        {
//            for (unsigned int j = 0; j < frontiers.size(); j++)
//            {
//                // check if the DS has EOs
//                double dist = distance(ds_list.at(i).x, ds_list.at(i).y, frontiers.at(j).x_coordinate, frontiers.at(j).y_coordinate);
//                if (dist < 0) {
//                    ROS_WARN("Distance computation failed");
//                    continue;
//                }
//                if (dist * 2 < max_available_distance)
//                {
//                    // the DS has EOS: check if it is the closest DS with EOs
//                    double dist2 = distance_from_robot(ds_list.at(i).x, ds_list.at(i).y);
//                    if (dist2 < 0) {
//                        ROS_WARN("Distance computation failed");
//                        continue;
//                    }
//                    if (dist2 < min_dist)
//                    {
//                        min_dist = dist2;
//                        min_ds = &ds_list.at(i);
//                    }
//                    
//                    break;
//                }
//            }
//        }
//        retry++;
//        if(min_ds == NULL)
//            ros::Duration(3).sleep();
//    }

    min_ds = min_ds_for_path_traversal;
    
    if (min_ds == NULL) {
        ROS_INFO("min_ds == NULL");
        *result = 1;
        return false;
        // this could happen if distance() always fails... //TODO(IMPORTANT) what happen if I return and the explorer node needs to reach a frontier?
    }

    // compute closest DS
    min_dist = std::numeric_limits<int>::max();
    ds_t *closest_ds = NULL;
    retry = 0;
    while(closest_ds == NULL && retry < 10) {
        for (unsigned int i = 0; i < ds_list.size(); i++)
        {
            double dist = distance_from_robot(ds_list.at(i).x, ds_list.at(i).y);
            if (dist < 0) {
                ROS_WARN("Distance computation failed");
                continue;
            }

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_ds = &ds_list.at(i);
            }
        }
        retry++;
        if(closest_ds == NULL)
            ros::Duration(3).sleep();
    }
    if(closest_ds == NULL)  {
        ROS_INFO("closest_ds == NULL");
        *result = 2;
        return false;
    }
    
    if(closest_ds->id == min_ds->id) {
        ROS_INFO("closest_ds->id == min_ds->id");
        *result = 3;
//        return false; //DO NOT RETURN!!! we can still find a path...
    }

    adhoc_communication::EmDockingStation msg;
    msg.id = min_ds->id;
    ROS_DEBUG("publishing path...");
    publish_goal_ds_for_path_navigation.publish(msg);
    
    ROS_DEBUG("Finished compute_and_publish_ds_path()");
    *result = 0;
    return true;
    
}

bool ExplorationPlanner::my_negotiate()
{
    winner_of_auction = true;
    
#ifndef QUICK_SELECTION
     
    frontiers_under_auction.push_back(*my_selected_frontier);

    ROS_INFO("Publish fronter for negotiation");
    adhoc_communication::ExpFrontier negotiation_list;
    adhoc_communication::ExpFrontierElement negotiation_element;
    //negotiation_element.detected_by_robot_str = 
    negotiation_element.detected_by_robot = robot_name;
//    negotiation_element.detected_by_robot = my_selected_frontier->detected_by_robot;
    negotiation_element.x_coordinate = my_selected_frontier->x_coordinate;
    negotiation_element.y_coordinate = my_selected_frontier->y_coordinate;
    negotiation_element.id = my_selected_frontier->id;
        
    negotiation_list.frontier_element.push_back(negotiation_element);

    my_sendToMulticast("mc_", negotiation_list, "send_frontier_for_coordinated_exploration");

#endif

    first_negotiation_run = false;
}

bool ExplorationPlanner::my_sendToMulticast(std::string multi_cast_group, adhoc_communication::ExpFrontier frontier_list, std::string topic)
{    
    //ROS_ERROR("SENDING frontier id: %ld", frontier_list.frontier_element[0].id);
    adhoc_communication::SendExpFrontier service_frontier; // create request of type any+

    std::stringstream robot_number;
    robot_number << robot_name;
    std::string prefix = "robot_";
    std::string robo_name = prefix.append(robot_number.str());

    if(robot_prefix_empty_param == true)
    {
        robo_name = robot_str;
    }
    std::string destination_name = multi_cast_group + robo_name; //for multicast
    ROS_INFO("sending to multicast group '%s' on topic: '%s'",destination_name.c_str(), topic.c_str());
    service_frontier.request.dst_robot = destination_name;
    service_frontier.request.frontier = frontier_list;
    service_frontier.request.topic = topic;

    if (ssendFrontier.call(service_frontier))
    {
            ROS_DEBUG("Successfully called service  sendToMulticast");

            if(service_frontier.response.status)
            {
                    ROS_DEBUG("adhoc comm returned successful transmission");
                    return true;
            }
            else
            {
                ROS_DEBUG("Failed to send to multicast group %s!",destination_name.c_str());
                return false;
            }
    }
    else
    {
     ROS_ERROR("Failed to call service sendToMulticast [%s]",ssendFrontier_2.getService().c_str());
     return false;
    }
}

void ExplorationPlanner::my_negotiationCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{
    ROS_DEBUG("Received frontier (%.2f, %.2f) by robot %d", msg.get()->frontier_element[0].x_coordinate,  msg.get()->frontier_element[0].y_coordinate, msg.get()->frontier_element[0].detected_by_robot);

    double robot_x, robot_y;
    
    //ROS_INFO("Transform frontier coordinates");
        
    std::stringstream robot_number;
    robot_number << msg.get()->frontier_element[0].detected_by_robot;
    std::string robo_name, prefix = "robot_";
    robo_name = prefix.append(robot_number.str());

    //std::string service_topic = robo_name2.append("/map_merger/transformPoint"); // FIXME for real scenario!!! robot_might not be used here
    std::string service_topic = "map_merger/transformPoint";

//              ROS_INFO("Robo name: %s   Service to subscribe to: %s", robo_name.c_str(), service_topic.c_str());
    
    client = nh_transform.serviceClient<map_merger::TransformPoint>(service_topic);
    ros::service::waitForService(service_topic, ros::Duration(3).sleep());
    if(!ros::service::exists(service_topic, true)) {
        ROS_FATAL("service to translate coordinates does not exist!!!");
        return;
    }

    service_message.request.point.x = msg.get()->frontier_element[0].x_coordinate;
    service_message.request.point.y = msg.get()->frontier_element[0].y_coordinate;
    service_message.request.point.src_robot = robo_name;   
    //ROS_DEBUG("Robot name:  %s", service_message.request.point.src_robot.c_str());
    
    //ROS_ERROR("Old x: %f   y: %f", msg.get()->frontier_element[0].x_coordinate, msg.get()->frontier_element[0].y_coordinate);

    ROS_INFO("calling map_merger service to translate coordinates");
    if(client.call(service_message))
        ; //ROS_ERROR("New x: %.1f   y: %.1f", service_message.response.point.x, service_message.response.point.y);
    else {
        ROS_ERROR("FAILED call to translate coordinates for frontier negotiation!");
        return;   
    }
    
    ROS_DEBUG("Old x: %f   y: %f", msg.get()->frontier_element[0].x_coordinate, msg.get()->frontier_element[0].y_coordinate);
    ROS_DEBUG("New x: %f   y: %f", service_message.response.point.x, service_message.response.point.y);

    //acquire_mutex(&store_frontier_mutex, __FUNCTION__); //TODO maybe we need a mutex, but it causes deadlocks...
//    int index = -1;
//    for(int i=0; i<frontiers.size(); i++) //TODO inefficient (and the robot could be unable to send the frontier in time...)
//        if( fabs(frontiers.at(i).x_coordinate - service_message.response.point.x) < 1.0 && fabs(frontiers.at(i).y_coordinate - service_message.response.point.y) < 1.0 ) { //TODO correct?
//            index = i;
//            break;
//        }
//        
//    if(index < 0) {
////        ROS_INFO("robot doesn't know the auctioned frontier: ignoring it");
//        ROS_INFO("robot does NOT know the auctioned frontier");
////        return;
//    } else {
//        ROS_INFO("robot knows the auctioned frontier");
//    }
    
    frontier_t new_frontier;
    new_frontier.x_coordinate = service_message.response.point.x;
    new_frontier.y_coordinate = service_message.response.point.y;
    
    if(!my_check_efficiency_of_goal(available_distance_for_reply, &new_frontier)) {
        ROS_INFO("this frontier is currently not reachable by the robot: do not reply");
        return;
    }

    ROS_INFO("Replying to frontier auction");
//    double cost = frontier_cost(&frontiers.at(index));
    double cost = frontier_cost(&new_frontier);
    //release_mutex(&store_frontier_mutex, __FUNCTION__);
   
    //ROS_ERROR("%d + %d + %d + %f", d_g, d_gb, d_gbe, theta);
    
    adhoc_communication::ExpFrontier negotiation_list;
    adhoc_communication::ExpFrontierElement negotiation_element;
    //negotiation_element.detected_by_robot = my_selected_frontier->detected_by_robot;
    negotiation_element.x_coordinate = msg.get()->frontier_element[0].x_coordinate;
    negotiation_element.y_coordinate = msg.get()->frontier_element[0].y_coordinate;
    //negotiation_element.id = my_selected_frontier->id;
    negotiation_element.bid = cost;
    negotiation_list.frontier_element.push_back(negotiation_element);
    
    my_sendToMulticast("mc_", negotiation_list, "reply_to_frontier");
    
}
