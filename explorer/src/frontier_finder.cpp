 void frontiers()
    {
        while(!explorer_ready)
            ros::Duration(10).sleep();
    
        ros::Duration(10).sleep();
        ROS_INFO("Can start periodical recomputation and publishing of frontiers"); //TODO probably now the findFrontiers, etc., in explorer are useless... but we have to be sure that when we search for a frontier we have an updated map
        
        while (ros::ok())
        {
            ros::Rate(0.1).sleep();
            
//            if(robot_state == robot_state::IN_QUEUE) //idle mode
//                continue;

            print_mutex_info("frontiers()", "acquiring");
            costmap_mutex.lock();
            print_mutex_info("frontiers()", "lock");
            
            exploration->transformToOwnCoordinates_frontiers();
            exploration->transformToOwnCoordinates_visited_frontiers();
            
            exploration->initialize_planner("exploration planner", costmap2d_local, costmap2d_global, NULL);

            exploration->findFrontiers();
            exploration->clearVisitedFrontiers();
            exploration->clearUnreachableFrontiers();
            exploration->clearSeenFrontiers(costmap2d_global);
            
            //ROS_INFO("publish_frontier_list and visualize them in rviz");
            exploration->publish_frontier_list();
//            exploration->publish_visited_frontier_list();  // TODO(minor) this doesn0t work really well since it publish
                                                           // only the frontier visited by this robot...

            /* Publish frontier points for rviz */
            exploration->visualize_Frontiers();
            //exploration->visualize_Clusters();
            //visualize

            costmap_mutex.unlock();
            print_mutex_info("frontiers()", "unlock");
            
            frontiers_found = true;

        }
    }


void ExplorationPlanner::initialize_planner(std::string name,
		costmap_2d::Costmap2DROS *costmap, costmap_2d::Costmap2DROS *costmap_global, DistanceComputerInterface *distance_computer) {

    ROS_INFO("Initializing the planner");

	//copy the pointed costmap to be available in ExplorationPlanner

	this->costmap_ros_ = costmap;
        this->costmap_global_ros_ = costmap_global;

        if(initialized_planner == false)
        {
                nav.initialize("navigation_path", costmap_global_ros_);
                initialized_planner = true;
//                this->distance_computer = distance_computer;
        }
    int dim = costmap_global_ros_->getCostmap()->getSizeInCellsY() * costmap_global_ros_->getCostmap()->getSizeInCellsY();
    float dim_meters = costmap_global_ros_->getCostmap()->getSizeInMetersY() * costmap_global_ros_->getCostmap()->getSizeInMetersY();
    //ROS_ERROR("%d, %.0f, %.0f", dim, dim_meters, (float) dim * 0.05 * 0.05 );
	//Occupancy_grid_array is updated here
	this->setupMapData();

	last_mode_ = FRONTIER_EXPLORE;
	this->initialized_ = true;
	
	//ROS_ERROR("Initialized");

	/*
	 * reset all counter variables, used to count the number of according blocks
	 * within the occupancy grid.
	 */
	unknown = 0, free = 0, lethal = 0, inflated = 0;

}

bool ExplorationPlanner::transformToOwnCoordinates_frontiers()
{
    ROS_INFO("Transform frontier coordinates");

    //store_frontier_mutex.lock();
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    
    //ROS_ERROR("Transform frontier coordinates - lock acquired");

    for(int i = 0; i < frontiers.size(); i++)
    {
        bool same_robot = false;
        if(robot_prefix_empty_param == true)
        {
            if(frontiers.at(i).detected_by_robot_str.compare(robot_str) == 0)
                same_robot = true;
//                ROS_ERROR("Same Robot Detected");
        }else
        {
            if(frontiers.at(i).detected_by_robot == robot_name)
                same_robot = true;
        }
        if(same_robot == false)
        {
//            ROS_ERROR("Same robot is false");
            bool transform_flag = false;
            for(int j=0; j < transformedPointsFromOtherRobot_frontiers.size(); j++)
            {
                if(robot_prefix_empty_param == 0)
                {
                    if(transformedPointsFromOtherRobot_frontiers.at(j).id == frontiers.at(i).id && frontiers.at(i).detected_by_robot_str.compare(transformedPointsFromOtherRobot_frontiers.at(j).robot_str)== 0)
                    {
                        transform_flag = true;
                        break;
                    }
                }else
                {
                    if(transformedPointsFromOtherRobot_frontiers.at(j).id == frontiers.at(i).id)
                    {
                        transform_flag = true;
                        break;
                    }
                }
            }

            if(transform_flag != true)
            {
                std::string robo_name, robo_name2;

                if(robot_prefix_empty_param == false)
                {
                    std::stringstream robot_number;
                    robot_number << frontiers.at(i).detected_by_robot;

                    std::string prefix = "robot_";
                    robo_name = prefix.append(robot_number.str());
                    ROS_DEBUG("Robots name is        %s", robo_name.c_str());

                    std::stringstream robot_number2;
                    robot_number2 << robot_name;

                    std::string prefix2 = "/robot_";
                    robo_name2 = prefix2.append(robot_number2.str());
                }
                else
                {
//                    ROS_ERROR("Get Robot Name ... ");
//                    robo_name = lookupRobotName(frontiers.at(i).detected_by_robot);
//                    robo_name2 = lookupRobotName(robot_name);
                    robo_name = frontiers.at(i).detected_by_robot_str;
                    robo_name2 = robot_str;
                    ROS_DEBUG("Robot: %s   transforms from robot: %s", robo_name2.c_str(), robo_name.c_str());
                }



                std::string service_topic = robo_name2.append("/map_merger/transformPoint"); // FIXME for real scenario!!! robot_might not be used here

//              ROS_INFO("Robo name: %s   Service to subscribe to: %s", robo_name.c_str(), service_topic.c_str());

                client = nh_transform.serviceClient<map_merger::TransformPoint>(service_topic);
                
                //F
                ros::service::waitForService(service_topic, ros::Duration(3).sleep());
                if(!ros::service::exists(service_topic, true))
                    return false;

                service_message.request.point.x = frontiers.at(i).x_coordinate;
                service_message.request.point.y = frontiers.at(i).y_coordinate;
                service_message.request.point.src_robot = robo_name;

                ROS_DEBUG("Robot name:  %s", service_message.request.point.src_robot.c_str());
                ROS_DEBUG("Old x: %f   y: %f", frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);

                //ROS_ERROR("calling client");
                if(client.call(service_message))
                {
                    frontiers.at(i).x_coordinate = service_message.response.point.x;
                    frontiers.at(i).y_coordinate = service_message.response.point.y;

                    transform_point_t transform_point;
                    transform_point.id = frontiers.at(i).id;

                    if(robot_prefix_empty_param == true)
                    {
                        transform_point.robot_str = frontiers.at(i).detected_by_robot_str;
                    }
                    transformedPointsFromOtherRobot_frontiers.push_back(transform_point);
                    ROS_DEBUG("New x: %.1f   y: %.1f",service_message.response.point.x, service_message.response.point.y);
                }
                else
                    ; //ROS_ERROR("FAILED!");
            }
        }
    }
    release_mutex(&store_frontier_mutex, __FUNCTION__);
    ROS_INFO("Transform frontier coordinates DONE");
    return true;
}

bool ExplorationPlanner::transformToOwnCoordinates_visited_frontiers()
{
    ROS_INFO("Transform Visited Frontier Coordinates");

    for(int i = 0; i < visited_frontiers.size(); i++)
    {
        bool same_robot = false;
        if(robot_prefix_empty_param == true)
        {
            if(visited_frontiers.at(i).detected_by_robot_str.compare(robot_str) == 0)
                same_robot = true;
//                ROS_ERROR("Same Robot Detected");
        }else
        {
            if(visited_frontiers.at(i).detected_by_robot == robot_name)
                same_robot = true;
        }
        if(same_robot == false)
        {
            bool transform_flag = false;
            for(int j=0; j < transformedPointsFromOtherRobot_visited_frontiers.size(); j++)
            {
                if(robot_prefix_empty_param == 0)
                {
                    if(transformedPointsFromOtherRobot_visited_frontiers.at(j).id == visited_frontiers.at(i).id && visited_frontiers.at(i).detected_by_robot_str.compare(transformedPointsFromOtherRobot_visited_frontiers.at(j).robot_str)== 0)
                    {
                        transform_flag = true;
                        break;
                    }
                }else
                {
                    if(transformedPointsFromOtherRobot_visited_frontiers.at(j).id == visited_frontiers.at(i).id)
                    {
                        transform_flag = true;
                        break;
                    }
                }
            }

            if(transform_flag != true)
            {

                std::string robo_name, robo_name2;

                if(robot_prefix_empty_param == false)
                {
                    std::stringstream robot_number;
                    robot_number << visited_frontiers.at(i).detected_by_robot;

                    std::string prefix = "robot_";
                    robo_name = prefix.append(robot_number.str());
                    ROS_DEBUG("Robots name is        %s", robo_name.c_str());

                    std::stringstream robot_number2;
                    robot_number2 << robot_name;

                    std::string prefix2 = "/robot_";
                    robo_name2 = prefix2.append(robot_number2.str());
                }
                else
                {
//                    robo_name = lookupRobotName(visited_frontiers.at(i).detected_by_robot);
//                    robo_name2 = lookupRobotName(robot_name);

                    robo_name = visited_frontiers.at(i).detected_by_robot_str;
                    robo_name2 = robot_str;
                     ROS_DEBUG("Robot: %s   transforms from robot: %s", robo_name2.c_str(), robo_name.c_str());
                }


                std::string service_topic = robo_name2.append("/map_merger/transformPoint"); // FIXME for real scenario!!! robot_ might not be used here

                client = nh_transform.serviceClient<map_merger::TransformPoint>(service_topic);
                
                //F
                ros::service::waitForService(service_topic, ros::Duration(3).sleep());
                if(!ros::service::exists(service_topic, true))
                    return false;

                service_message.request.point.x = visited_frontiers.at(i).x_coordinate;
                service_message.request.point.y = visited_frontiers.at(i).y_coordinate;
                service_message.request.point.src_robot = robo_name;

                ROS_DEBUG("Old visited x: %f   y: %f", visited_frontiers.at(i).x_coordinate, visited_frontiers.at(i).y_coordinate);

                ROS_ERROR("calling");
                if(client.call(service_message))
                {
                    visited_frontiers.at(i).x_coordinate = service_message.response.point.x;
                    visited_frontiers.at(i).y_coordinate = service_message.response.point.y;

                    transform_point_t transform_point;
                    transform_point.id = visited_frontiers.at(i).id;

                    if(robot_prefix_empty_param == true)
                    {
                        transform_point.robot_str = visited_frontiers.at(i).detected_by_robot_str;
                    }
                    transformedPointsFromOtherRobot_visited_frontiers.push_back(transform_point);
                    ROS_DEBUG("New visited x: %.1f   y: %.1f",service_message.response.point.x, service_message.response.point.y);
                }
                else
                    ; //ROS_ERROR("FAILED!!!!");
            }
        }
    }
    ROS_INFO("Transform visited frontier coordinates DONE");
    return true;
}

void ExplorationPlanner::printFrontiers()
{
    for(int i = 0; i < frontiers.size(); i++)
    {
        if(robot_prefix_empty_param == true)
        {
            ROS_INFO("Frontier %d:   x: %f   y: %f   robot: %s", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot_str.c_str());
        }else
        {
            ROS_INFO("Frontier %d:   x: %f   y: %f", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);
        }
    }
}

bool ExplorationPlanner::storeFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id)
{
    frontier_t new_frontier;

    if(robot_prefix_empty_param == true)
    {
        ROS_DEBUG("Storing Frontier ID: %d   Robot: %s", id, detected_by_robot_str.c_str());
        if(id != -1)
        {
            new_frontier.id = id;
        }else
        {
            new_frontier.id = frontier_id_count++;
        }
        new_frontier.detected_by_robot_str = detected_by_robot_str;
        new_frontier.x_coordinate = x;
        new_frontier.y_coordinate = y;
        
        //F
        new_frontier.cluster_id = -1;

        //store_frontier_mutex.lock();
        acquire_mutex(&store_frontier_mutex, __FUNCTION__);
        discovered_new_frontier = true;
        frontiers.push_back(new_frontier);
        release_mutex(&store_frontier_mutex, __FUNCTION__);
    }else
    {
        if(detected_by_robot != robot_name)
        {
            new_frontier.id = id;
        }
        else
        {
           new_frontier.id = (robot_name * 10000) + frontier_id_count++;
        }

        new_frontier.detected_by_robot = detected_by_robot;
        new_frontier.x_coordinate = x;
        new_frontier.y_coordinate = y;
        
        //F
        new_frontier.cluster_id = -1;

        //store_frontier_mutex.lock();
        acquire_mutex(&store_frontier_mutex, __FUNCTION__);
        discovered_new_frontier = true;
        frontiers.push_back(new_frontier);
        release_mutex(&store_frontier_mutex, __FUNCTION__);
    }

    return true;
}

bool ExplorationPlanner::removeStoredFrontier(int id, std::string detected_by_robot_str)
{
    for(int i= 0; i < frontiers.size(); i++)
    {
        if(robot_prefix_empty_param == true)
        {
            ROS_DEBUG("Removing frontier with id '%d' detected by robot '%s'", frontiers.at(i).id, frontiers.at(i).detected_by_robot_str.c_str());
            if(frontiers.at(i).id == id && frontiers.at(i).detected_by_robot_str.compare(detected_by_robot_str) == 0)
            {
                ROS_DEBUG("Removing Frontier ID: %d  at position: %d  of Robot: %s", frontiers.at(i).id, i, frontiers.at(i).detected_by_robot_str.c_str());
                
                //store_frontier_mutex.lock();
                acquire_mutex(&store_frontier_mutex, __FUNCTION__);
                acquire_mutex(&mutex_erase_frontier, __FUNCTION__);
                frontiers.erase(frontiers.begin()+i);
//                if(i > 0)
//                {
//                    i --;
//                }
                erased = true;
                release_mutex(&store_frontier_mutex, __FUNCTION__);
                release_mutex(&mutex_erase_frontier, __FUNCTION__);
                
                //break; //FIXME ... only a test
            }
        }
        else
        {
            if(frontiers.at(i).id == id)
            {
            
                //store_frontier_mutex.lock();
                acquire_mutex(&store_frontier_mutex, __FUNCTION__);
                acquire_mutex(&mutex_erase_frontier, __FUNCTION__);
                erased = true;
                frontiers.erase(frontiers.begin()+i);
                if(i > 0)
                {
                    i --;
                }
                release_mutex(&store_frontier_mutex, __FUNCTION__);
                release_mutex(&mutex_erase_frontier, __FUNCTION__);
                break;
            }
        }
    }
    return true;
}

bool ExplorationPlanner::storeVisitedFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id)
{

    frontier_t visited_frontier;


    if(robot_prefix_empty_param == true)
    {
        ROS_DEBUG("Storing Visited Frontier ID: %d   Robot: %s", visited_frontier_id_count, detected_by_robot_str.c_str());
        if(id != -1)
        {
            visited_frontier.id = id;
        }else
        {
            visited_frontier.id = visited_frontier_id_count++;
        }

        visited_frontier.detected_by_robot_str = detected_by_robot_str;
        visited_frontier.x_coordinate = x;
        visited_frontier.y_coordinate = y;

        store_visited_mutex.lock();
        visited_frontiers.push_back(visited_frontier);
        store_visited_mutex.unlock();

    }else
    {
         if(detected_by_robot != robot_name)
        {
            visited_frontier.id = id;
        }
        else
        {
           visited_frontier.id = (robot_name * 10000) + visited_frontier_id_count++;
        }

        //ROS_ERROR("STORING %f, %f", x, y);
        visited_frontier.detected_by_robot = detected_by_robot;
        visited_frontier.x_coordinate = x;
        visited_frontier.y_coordinate = y;

        store_visited_mutex.lock();
        visited_frontiers.push_back(visited_frontier);
        store_visited_mutex.unlock();
    }

    bool break_flag = false;
    for(int i = 0; i < clusters.size(); i++)
    {
        for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
        {
            if(clusters.at(i).cluster_element.at(j).id == id)
            {
                ROS_DEBUG("Set cluster unreachable count to 0");
                clusters.at(i).unreachable_frontier_count = 0;
                break_flag = true;
                break;
            }
        }
        if(break_flag == true)
        {
            break;
        }
    }

    return true;
}

bool ExplorationPlanner::removeVisitedFrontier(int id, std::string detected_by_robot_str)
{
    for(int i= 0; i< visited_frontiers.size(); i++)
    {
        if(robot_prefix_empty_param == true)
        {

            if(visited_frontiers.at(i).id == id && visited_frontiers.at(i).detected_by_robot_str.compare(detected_by_robot_str) == 0)
            {
                ROS_INFO("Removing Visited Frontier ID: %d  at position: %d  of Robot: %s", visited_frontiers.at(i).id, i, visited_frontiers.at(i).detected_by_robot_str.c_str());
                store_visited_mutex.lock();
                visited_frontiers.erase(visited_frontiers.begin()+i);
//                if(i > 0)
//                {
//                    i --;
//                }
                store_visited_mutex.unlock();
                break;
            }
        }else
        {
            if(visited_frontiers.at(i).id == id)
            {
                store_visited_mutex.lock();
                visited_frontiers.erase(visited_frontiers.begin()+i);
                if(i > 0)
                {
                    i --;
                }
                store_visited_mutex.unlock();
                break;
            }
        }
    }
    return true;
}

bool ExplorationPlanner::storeUnreachableFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id)
{
    frontier_t unreachable_frontier;

    if(robot_prefix_empty_param == true)
    {
        ROS_DEBUG("Storing Unreachable Frontier ID: %d   Robot: %s", unreachable_frontier_id_count, detected_by_robot_str.c_str());

        if(id != -1)
        {
            unreachable_frontier.id = id;
        }else
        {
            unreachable_frontier.id = unreachable_frontier_id_count++;
        }

        unreachable_frontier.detected_by_robot_str = detected_by_robot_str;
        unreachable_frontier.x_coordinate = x;
        unreachable_frontier.y_coordinate = y;

        unreachable_frontiers.push_back(unreachable_frontier);
        //my_unreachable_frontiers.push_back(unreachable_frontier);

    }else
    {
        if(detected_by_robot != robot_name)
        {
            unreachable_frontier.id = id;
        }
        else
        {
           unreachable_frontier.id = (robot_name * 10000) + unreachable_frontier_id_count++;
        }

        unreachable_frontier.detected_by_robot = detected_by_robot;
        unreachable_frontier.x_coordinate = x;
        unreachable_frontier.y_coordinate = y;

        //frontiers.push_back(unreachable_frontier);
        unreachable_frontiers.push_back(unreachable_frontier); //F
        //my_unreachable_frontiers.push_back(unreachable_frontier);

    }

    bool break_flag = false;
    for(int i = 0; i < clusters.size(); i++)
    {
        for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
        {
            if(clusters.at(i).cluster_element.at(j).id == id)
            {
                ROS_WARN("Increasing cluster unreachable count");
                clusters.at(i).unreachable_frontier_count++;
                break_flag = true;
                break;
            }
        }
        if(break_flag == true)
        {
            break;
        }
    }

    return true;

}

void ExplorationPlanner::frontierCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{
    // TODO: Allow only frontiers that are not too close to walls
    //ROS_ERROR("frontierCallback, %u", msg.get()->frontier_element.size());

    adhoc_communication::ExpFrontierElement frontier_element;
    for(unsigned int i = 0; i < msg.get()->frontier_element.size(); i++)
    {
        frontier_element = msg.get()->frontier_element.at(i);
        bool result = true;
        for (unsigned int j = 0; j < frontiers.size(); j++)
        {
            if(robot_prefix_empty_param == true)
            {
                //ROS_ERROR("FrontierCallback ... ");
                if(frontiers.at(j).detected_by_robot_str.compare(frontier_element.detected_by_robot_str) == 0 && frontier_element.id == frontiers.at(j).id)
                {
                    ROS_ERROR("Same Detected ...");
                    result = false;
                    break;
                }
            }
            else
            {
                if(frontier_element.detected_by_robot == robot_name)
                {
                    //ROS_ERROR("This frontier was detected by this robot: ignoring");
                    result = false;
                    break;
                }
                else if (frontier_element.id == frontiers.at(j).id)
                {
                    //ROS_ERROR("This frontier has been already received: ignoring");
                    result = false;
                    break;
                }
            }
        }
        if (result == true)
        {

            if(robot_prefix_empty_param == true)
            {
                //ROS_ERROR("Received New Frontier with ID: %ld  Robot: %s", frontier_element.id, frontier_element.detected_by_robot_str.c_str());
                storeFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, frontier_element.detected_by_robot_str, frontier_element.id);
            }
            else
            {
                //ROS_ERROR("Received New Frontier of Robot %ld with ID %ld", frontier_element.detected_by_robot, frontier_element.id);
                if(frontier_element.detected_by_robot != robot_name)
                {
                    storeFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, "", frontier_element.id);
                }
            }
        }
    }

}

void ExplorationPlanner::visited_frontierCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{
    adhoc_communication::ExpFrontierElement frontier_element;
    for(unsigned int i = 0; i < msg.get()->frontier_element.size(); i++)
    {
        frontier_element = msg.get()->frontier_element.at(i);

        bool result = true;
        for (unsigned int j = 0; j < visited_frontiers.size(); j++)
        {
            if(frontier_element.detected_by_robot == robot_name)
            {
                result = false;
                break;
            }
            else
            {
                if (frontier_element.id == visited_frontiers.at(j).id)
                {
                    result = false;
                    break;
                }
            }
        }
        if (result == true)
        {
            ROS_DEBUG("Received New Visited Frontier of Robot %d with ID %d", frontier_element.detected_by_robot, frontier_element.id);
            if(robot_prefix_empty_param == true)
            {
                ROS_DEBUG("Storing Visited Frontier ID: %d  Robot: %s", frontier_element.id, frontier_element.detected_by_robot_str.c_str());
                storeVisitedFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, frontier_element.detected_by_robot_str, frontier_element.id);
            }else
            {
                if(frontier_element.detected_by_robot != robot_name)
                {
                    storeVisitedFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, "", frontier_element.id);
                }
            }
        }
    }
}

bool ExplorationPlanner::publish_frontier_list()
{
//    ROS_INFO("PUBLISHING FRONTIER LIST");

    publish_subscribe_mutex.lock();

    adhoc_communication::ExpFrontier frontier_msg;
    //ROS_ERROR("%u", frontiers.size());
    for(unsigned int i = 0; i<frontiers.size(); i++)
    {
        /* Send only frontiers detected by this robot (to avoid creating very huge message, which would require a lot of time to be completely analyzed) */
        if(frontiers.at(i).detected_by_robot != robot_name)
            continue;
            
        adhoc_communication::ExpFrontierElement frontier_element;

        frontier_element.id = frontiers.at(i).id;
        frontier_element.detected_by_robot = frontiers.at(i).detected_by_robot;
        frontier_element.detected_by_robot_str = frontiers.at(i).detected_by_robot_str;
        frontier_element.robot_home_position_x = frontiers.at(i).robot_home_x;
        frontier_element.robot_home_position_y = frontiers.at(i).robot_home_y;
        frontier_element.x_coordinate = frontiers.at(i).x_coordinate;
        frontier_element.y_coordinate = frontiers.at(i).y_coordinate;

        frontier_msg.frontier_element.push_back(frontier_element);
    }

    pub_frontiers.publish(frontier_msg);
    //sendToMulticast("mc_",frontier_msg, "frontiers");

    publish_subscribe_mutex.unlock();
}

bool ExplorationPlanner::publish_visited_frontier_list()
{
//    ROS_INFO("PUBLISHING VISITED FRONTIER LIST");

    publish_subscribe_mutex.lock();

    adhoc_communication::ExpFrontier visited_frontier_msg;

    for(unsigned int i = 0; i<visited_frontiers.size(); i++)
    {
        adhoc_communication::ExpFrontierElement visited_frontier_element;
        visited_frontier_element.id = visited_frontiers.at(i).id;
        visited_frontier_element.detected_by_robot = visited_frontiers.at(i).detected_by_robot;
        visited_frontier_element.detected_by_robot_str = visited_frontiers.at(i).detected_by_robot_str;
        visited_frontier_element.robot_home_position_x = visited_frontiers.at(i).robot_home_x;
        visited_frontier_element.robot_home_position_y = visited_frontiers.at(i).robot_home_y;
        visited_frontier_element.x_coordinate = visited_frontiers.at(i).x_coordinate;
        visited_frontier_element.y_coordinate = visited_frontiers.at(i).y_coordinate;
        
        //ROS_ERROR("PUBLISHING %f, %f", visited_frontier_element.x_coordinate, visited_frontier_element.y_coordinate);

        visited_frontier_msg.frontier_element.push_back(visited_frontier_element);
    }

    pub_visited_frontiers.publish(visited_frontier_msg);
    //sendToMulticast("mc_",visited_frontier_msg, "visited_frontiers");

    publish_subscribe_mutex.unlock();
}

void ExplorationPlanner::clearVisitedFrontiers()
{
//    ROS_INFO("Clear Visited");

    /* visited_frontier.at(0) is the Home point. Do not compare
     * with this point. Otherwise this algorithm deletes it, a new
     * frontier close to the home point is found which is then again
     * deleted since it is to close to the home point. This would not
     * have any impact on the exploration, but in simulation mode
     * (simulation = true) the frontier_ID is steadily up counted.
     * This is not necessary!
     */
    std::vector<int> goals_to_clear;

    for (unsigned int i = 1; i < visited_frontiers.size(); i++)
    {
        for (unsigned int j = 0; j < frontiers.size(); j++)
	{
            double diff_x = visited_frontiers.at(i).x_coordinate - frontiers.at(j).x_coordinate;
            double diff_y = visited_frontiers.at(i).y_coordinate - frontiers.at(j).y_coordinate;

            if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE) {
                if(robot_prefix_empty_param == true)
                {
                    removeStoredFrontier(frontiers.at(j).id, frontiers.at(j).detected_by_robot_str);
                    if(j > 0)
                    {
                        j--;
                    }
                }else
                {
                    removeStoredFrontier(frontiers.at(j).id, "");
                    if(j > 0)
                    {
                        j--;
                    }
//                    goals_to_clear.push_back(frontiers.at(j).id);
                }
                break;
            }
        }
    }

//    for(int i= 0; i< goals_to_clear.size(); i++)
//    {
//        removeStoredFrontier(goals_to_clear.at(i), goals_to_clear.);
//    }
}

void ExplorationPlanner::clearUnreachableFrontiers()
{
//    ROS_INFO("Clear UNREACHABLE");

    /* visited_frontier.at(0) is the Home point. Do not compare
     * with this point. Otherwise this algorithm deletes it, a new
     * frontier close to the home point is found which is then again
     * deleted since it is to close to the home point. This would not
     * have any impact on the exploration, but in simulation mode
     * (simulation = true) the frontier_ID is steadily up counted.
     * This is not necessary!
     */
    std::vector<int> goals_to_clear;

    for (unsigned int i = 1; i < unreachable_frontiers.size(); i++)
    {
        for (unsigned int j = 0; j < frontiers.size(); j++)
	{
            double diff_x = unreachable_frontiers.at(i).x_coordinate - frontiers.at(j).x_coordinate;
            double diff_y = unreachable_frontiers.at(i).y_coordinate - frontiers.at(j).y_coordinate;

            if (fabs(diff_x) <= 0.2 && fabs(diff_y) <= 0.2) {
//                goals_to_clear.push_back(frontiers.at(j).id);
                if(robot_prefix_empty_param == true)
                {
                    removeStoredFrontier(frontiers.at(j).id, frontiers.at(j).detected_by_robot_str);
                    if(j > 0)
                    {
                        j--;
                    }
                }else
                {
                    removeStoredFrontier(frontiers.at(j).id, "");
                    if(j > 0)
                    {
                        j--;
                    }
    //                    goals_to_clear.push_back(frontiers.at(j).id);
                }
                break;
            }
        }
    }

//    for(int i= 0; i< goals_to_clear.size(); i++)
//    {
//        removeStoredFrontier(goals_to_clear.at(i));
//    }
}

void ExplorationPlanner::clearSeenFrontiers(costmap_2d::Costmap2DROS *global_costmap)
{
//    ROS_INFO("Clear Seen");
    unsigned int mx, my, point;
    std::vector<int> neighbours, goals_to_clear;

//    this->costmap_global_ros_ = global_costmap;
//    costmap_global_ros_->getCostmapCopy(costmap_global_);
//    costmap_global_ros_->getCostmap();

//    ROS_INFO("Map origin  x: %f    y: %f", global_costmap->getCostmap()->getOriginX(), global_costmap->getCostmap()->getOriginY());
//    ROS_INFO("Map size    x: %d    y: %d", global_costmap->getCostmap()->getSizeInCellsX(), global_costmap->getCostmap()->getSizeInCellsY());
    if(frontiers.size() > 1)
    {
        for(unsigned int i = 0; i < frontiers.size(); i++)
        {
            unsigned int new_mx, new_my;
            bool unknown_found = false;
            bool obstacle_found = false;
            bool freespace_found = false;

            mx = 0;
            my = 0;
            
            //ROS_ERROR("%.2f", costmap_ros_->getCostmap()->getResolution());
           // ROS_INFO("frontier x: %f   y: %f", frontiers.at(i).x_coordinate,frontiers.at(i).y_coordinate);
            if(!global_costmap->getCostmap()->worldToMap(frontiers.at(i).x_coordinate,frontiers.at(i).y_coordinate,mx,my))
            {
                ROS_ERROR("Cannot convert coordinates successfully.");
                continue;
            }
//            ROS_INFO("Map coordinates mx: %d  my: %d",mx,my);

            neighbours = getMapNeighbours(mx, my, 6);

//            ROS_INFO("Neighbours: %u", neighbours.size());
            for (unsigned int j = 0; j < neighbours.size()/2; j++)
            {


//                ROS_INFO("Get Neighbour %d and %d",j*2, j*2+1);
                new_mx = neighbours.at(j*2);
                new_my = neighbours.at(j*2+1);
//                ROS_INFO("got access");


//                ROS_INFO("Calculating at position x: %d    y: %d", new_mx, new_my);

                //F
                unsigned char cost;
                //cost = global_costmap->getCostmap()->getCost(new_mx, new_my);
                cost = getCost(global_costmap, new_mx, new_my);
                
                
//                ROS_INFO("x position: %d       y position: %d", new_mx, new_my);
//                ROS_INFO("Got Cost");
                if(cost == costmap_2d::NO_INFORMATION)
                {
                    unknown_found = true;
                }
                else if(cost == costmap_2d::FREE_SPACE)
                {
                    freespace_found = true;
                }
                else if(cost == costmap_2d::LETHAL_OBSTACLE)
                {
                    obstacle_found = true;
                }
//                ROS_INFO("Done");
            }

            if(unknown_found == false || obstacle_found == true || freespace_found == false)
            {

//                goals_to_clear.push_back(frontiers.at(i).id);
                if(robot_prefix_empty_param == true)
                {
                    removeStoredFrontier(frontiers.at(i).id, frontiers.at(i).detected_by_robot_str);
                    if(i > 0)
                    {
                        i--;
                    }
                }else
                {
                    removeStoredFrontier(frontiers.at(i).id, "");
                    if(i > 0)
                    {
                        i--;
                    }
                }
                seen_frontier_list.push_back(frontiers.at(i));

            }
        }
    }
    
    ROS_DEBUG("frontiers.size() after cleaning: %u", frontiers.size());
}

/**
 * Find a backoff point in the neighborhood of (x,y) that is surrounded by only free space cells.
 * The backoff point must be clear of obstacles up to a distance of INNER_DISTANCE.
 * The neighborhood of (x,y) consists of 40 points surrounding (x,y).
 */
bool ExplorationPlanner::smartGoalBackoff(double x, double y, costmap_2d::Costmap2DROS *global_costmap, std::vector<double> *new_goal)
{
    unsigned int mx, my, new_mx, new_my, inner_mx, inner_my;
    double wx, wy;
    std::vector<int> neighbours, inner_neighbours;

    if(!global_costmap->getCostmap()->worldToMap(x,y,mx,my))
    {
        ROS_ERROR("Cannot convert coordinates successfully.");
        //ROS_ERROR("%f, %f", x, y);
        //ROS_ERROR("%f, %f", global_costmap->getCostmap()->getSizeInMetersX(), global_costmap->getCostmap()->getSizeInMetersY());
        return false;
    }
    //ROS_DEBUG("Map coordinates mx: %d  my: %d",mx,my);
    //ROS_ERROR("%f, %f", global_costmap->getCostmap()->getSizeInMetersX(), global_costmap->getCostmap()->getSizeInMetersY());


    neighbours = getMapNeighbours(mx, my, 40);
    //ROS_DEBUG("Got neighbours");
    for (unsigned int j = 0; j< neighbours.size()/2; j++)
    {
        // ROS_DEBUG("Get neighbours %d and %d",j*2,j*2+1); // bad place for debug output
        new_mx = neighbours.at(j*2);
        new_my = neighbours.at(j*2+1);

        //F
        unsigned char cost;
        //cost = global_costmap->getCostmap()->getCost(new_mx, new_my);
        cost = getCost(global_costmap, new_mx, new_my);

        if( cost == costmap_2d::FREE_SPACE)
        {
            bool back_off_goal_found = true;

            inner_neighbours = getMapNeighbours(new_mx, new_my, INNER_DISTANCE);
            for (unsigned int i = 0; i< inner_neighbours.size()/2; i++)
            {
                // ROS_DEBUG("Get inner neighbours %d and %d",i*2,i*2+1); // bad place for debug output!
                inner_mx = inner_neighbours.at(i*2);
                inner_my = inner_neighbours.at(i*2+1);

                //F
                unsigned char inner_cost;
                //inner_cost = global_costmap->getCostmap()->getCost(inner_mx, inner_my);
                inner_cost = getCost(global_costmap, inner_mx, inner_my);
                
                if( inner_cost != costmap_2d::FREE_SPACE)
                {
                    back_off_goal_found = false;
                    break;
                }
            }

            if(back_off_goal_found == true)
            {
                global_costmap->getCostmap()->mapToWorld(new_mx, new_my, wx, wy);
                new_goal->push_back(wx);
                new_goal->push_back(wy);
                return true;
            }
        }
    }
    return false;
}

std::vector<int> ExplorationPlanner::getMapNeighbours(unsigned int point_x, unsigned int point_y, int distance)
{
    std::vector<int> neighbours;

    for(int i = 0; i< distance; i++)
    {
        neighbours.push_back(point_x+i+1);
        neighbours.push_back(point_y);

        neighbours.push_back(point_x-i-1);
        neighbours.push_back(point_y);

        neighbours.push_back(point_x);
        neighbours.push_back(point_y+i+1);

        neighbours.push_back(point_x);
        neighbours.push_back(point_y-i-1);
    }
    return neighbours;
}

/*
 * searches the occupancy grid for frontier cells and merges them into one target point per frontier.
 * The returned frontiers are in world coordinates.
 */
void ExplorationPlanner::findFrontiers()
{

    ROS_INFO("Find Frontiers");
    allFrontiers.clear();
    int select_frontier = 1;
    std::vector<double> final_goal,start_points;

    /*
     * check for all cells in the occupancy grid whether
     * or not they are frontier cells. If a possible frontier is found, true is
     * returned
     */
    int new_frontier_point = 0;
    for (unsigned int i = 0; i < num_map_cells_; i++)
    {
        int new_frontier_point = isFrontier_2(i);
        if (new_frontier_point != 0)
        {
            /*
             * If isFrontier() returns true, the point which is checked to be a frontier
             * is indeed a frontier.
             */

            /*
             * Push back adds data x to a vector.
             * If a frontier was found, the position of the frontier is stored
             * in the allFrontiers vector.
             */
            allFrontiers.push_back(new_frontier_point);
        }
    }
    ROS_INFO("Found %u frontier cells which are transformed into frontiers points. Starting transformation...", allFrontiers.size());

    /*
     * Iterate over all frontiers. The frontiers stored in allFrontiers are
     * already REAL frontiers and can be approached by the robot to get new
     * informations of the environment.
     * To limit the amount of frontiers and only pick those which are valuable to
     * drive there. The rest of the frontiers are stored in a goal buffer which contain all
     * frontiers within the map. Additionally check whether or not a newly found
     * frontier has already been added to the list. If it is already in the list, do
     * not make a new entry with the coordinates of the frontier!
     */
     
     //F
    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
    
    // If there are no free cells in the map, get rid of all the previously stored frontiers, since for sure they are no more valid frontiers (or there should be some free cells in the map)
    if(allFrontiers.size() == 0)
        frontiers.clear();
    
    for (unsigned int i = 0; i < allFrontiers.size(); ++i)
    {
        geometry_msgs::PoseStamped finalFrontier;
        double wx, wy, wx2, wy2, wx3, wy3;
        unsigned int mx, my, mx2, my2, mx3, my3;
        bool result;

//        acquire_mutex(&costmap_mutex, __FUNCTION__);
        costmap_ros_->getCostmap()->indexToCells(allFrontiers.at(i), mx, my);
        costmap_ros_->getCostmap()->mapToWorld(mx, my, wx, wy);
//        release_mutex(&costmap_mutex, __FUNCTION__);

        //ROS_INFO("index: %d   map_x: %d   map_y: %d   world_x: %f   world_y: %f", allFrontiers.at(i), mx, my, wx, wy);

        /*
         * Check neighboring frontiers and only remember them if they are further
         * then a distance of MAX_GOAL_RANGE away from each other, discard otherwise.
         */
        if(select_frontier == 1)
        {
            result = true;
            
            for (unsigned int j = 0; j < frontiers.size(); j++)
            {
                if (fabs(wx - frontiers.at(j).x_coordinate) <= MAX_GOAL_RANGE && fabs(wy - frontiers.at(j).y_coordinate) <= MAX_GOAL_RANGE)
                {
                    result = false;
                    break;
                }
            }
            
            //F
//            if(result)
//                for (unsigned int j = 0; j < visited_frontiers.size(); j++)
//                {
//                    if (fabs(wx - visited_frontiers.at(j).x_coordinate) <= MAX_GOAL_RANGE && fabs(wy - visited_frontiers.at(j).y_coordinate) <= MAX_GOAL_RANGE)
//                    {
//                        result = false;
//                        break;
//                    }
//                }
            if(result)
                for (unsigned int j = 0; j < unreachable_frontiers.size(); j++)
                {
                    if (fabs(wx - unreachable_frontiers.at(j).x_coordinate) <= MAX_GOAL_RANGE && fabs(wy - unreachable_frontiers.at(j).y_coordinate) <= MAX_GOAL_RANGE)
                    {
                        result = false;
                        break;
                    }
                }
            //end_F
            
            if (result == true)
            {
                storeFrontier_without_locking(wx,wy,robot_name,robot_str,-1);
            }  
            
        }
        else if(select_frontier == 2)
        {
            std::vector<int> neighbour_index;

            for (unsigned int j = 0; j < allFrontiers.size(); ++j)
            {
            
//                acquire_mutex(&costmap_mutex, __FUNCTION__);
                costmap_ros_->getCostmap()->indexToCells(allFrontiers[j], mx2, my2);
                costmap_ros_->getCostmap()->mapToWorld(mx2, my2, wx2, wy2);
//                release_mutex(&costmap_mutex, __FUNCTION__);

                if (fabs(wx - wx2) <= MINIMAL_FRONTIER_RANGE && fabs(wy - wy2) <= MINIMAL_FRONTIER_RANGE && fabs(wx - wx2) != 0 && fabs(wy - wy2) != 0)
                {
                    neighbour_index.push_back(allFrontiers[j]);
                }
            }


            for (unsigned int n = 0; n < neighbour_index.size(); ++n)
            {
//                acquire_mutex(&costmap_mutex, __FUNCTION__);
                costmap_ros_->getCostmap()->indexToCells(neighbour_index[n], mx2, my2);
                costmap_ros_->getCostmap()->mapToWorld(mx2, my2, wx2, wy2);
//                release_mutex(&costmap_mutex, __FUNCTION__);

                while(true)
                {
                    bool end_point_found = true;
                    for (unsigned int k = 0; k < allFrontiers.size(); ++k)
                    {
//                        acquire_mutex(&costmap_mutex, __FUNCTION__);
                        costmap_ros_->getCostmap()->indexToCells(allFrontiers[k], mx3, my3);
                        costmap_ros_->getCostmap()->mapToWorld(mx3, my3, wx3, wy3);
//                        release_mutex(&costmap_mutex, __FUNCTION__);

                        if (fabs(wx2 - wx3) <= MINIMAL_FRONTIER_RANGE && fabs(wy2 - wy3) <= MINIMAL_FRONTIER_RANGE && wx2 != wx3 && wy2 != wy3 && wx != wx3 && wy != wy3)
                        {
                            wx  = wx2;
                            wy  = wy2;
                            wx2 = wx3;
                            wy2 = wy3;
                            end_point_found = false;
                        }
                    }
                    if(end_point_found == true)
                    {
                        start_points.push_back(wx2);
                        start_points.push_back(wy2);

                        break;
                    }
                }
            }
            goal_buffer_x.push_back(start_points.at(0)+(start_points.at(2)-start_points.at(0))/2);
            goal_buffer_y.push_back(start_points.at(1)+(start_points.at(3)-start_points.at(1))/2);
        }
    }
  
    if(!received_scan && allFrontiers.size() == 0) {
        ROS_ERROR("Apparently no laser scan has been received yet: retying later...");
        retrying_searching_frontiers++;
    }
    else {
        retrying_searching_frontiers = 0;
        received_scan = true;   
    }
    
    //F
    release_mutex(&store_frontier_mutex, __FUNCTION__);
    ROS_INFO("Size of all frontiers in the list: %u", frontiers.size());
}

void ExplorationPlanner::setupMapData() {

	if ((this->map_width_ != costmap_ros_->getCostmap()->getSizeInCellsX())
			|| (this->map_height_ != costmap_ros_->getCostmap()->getSizeInCellsY())) {
		this->deleteMapData();

		map_width_ = costmap_ros_->getCostmap()->getSizeInCellsX();
		map_height_ = costmap_ros_->getCostmap()->getSizeInCellsY();
        num_map_cells_ = map_width_ * map_height_;

		ROS_INFO("Costmap size in cells: width:%d   height: %d   map_cells:%d",map_width_,map_height_,num_map_cells_);

		// initialize exploration_trans_array_, obstacle_trans_array_, goalMap and frontier_map_array_
//		exploration_trans_array_ = new unsigned int[num_map_cells_];
//		obstacle_trans_array_ = new unsigned int[num_map_cells_];
//		is_goal_array_ = new bool[num_map_cells_];
//		frontier_map_array_ = new int[num_map_cells_];
//		clearFrontiers();
//		resetMaps();
	}


	occupancy_grid_array_ = costmap_ros_->getCostmap()->getCharMap();

	for (unsigned int i = 0; i < num_map_cells_; i++) {

		countCostMapBlocks(i);

//	  if(occupancy_grid_array_[i] == costmap_2d::NO_INFORMATION || occupancy_grid_array_[i] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || occupancy_grid_array_[i] == costmap_2d::LETHAL_OBSTACLE || occupancy_grid_array_[i] == costmap_2d::FREE_SPACE)
//	  {
//	  }else
//	  {
//		  ROS_DEBUG("%d",(int)occupancy_grid_array_[i]);
//		  //occupancy_grid_array_[i] = '0'; // write 0 to the grid!
//	  }
	}
	ROS_INFO("--------------------- Iterate through Costmap --------------------");
	ROS_INFO("Free: %d  Inflated: %d  Lethal: %d  unknown: %d rest: %d", free,
			inflated, lethal, unknown,
			num_map_cells_ - (free + inflated + lethal + unknown));
	ROS_INFO("------------------------------------------------------------------");
        free_space = free;
}

int ExplorationPlanner::isFrontier(int point) {


	if (isFree(point)) {

		/*
		 * The point is either a obstacle or a point with not enough
		 * information about
		 * Therefore, check if the point is surrounded by other NO_INFORMATION
		 * points. Then further space to explore is found ---> frontier
		 */
		//int Neighbours = 0;
		//int points[((int)pow(8,Neighbours+1))+8]; // NEIGHBOURS points, each containing 8 adjacent points
		int no_inf_count = 0;
//		int inscribed_count = 0;
		int adjacent_points[16];
		/*
		 * Now take one point and lookup all adjacent points (surrounding points).
		 * The variable adjacentPoints contains all neighboring point (up, right, left ...)
		 */

		/*
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[0]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[1]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[2]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[3]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[4]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[5]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[6]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[7]);
		 */

		//histogram[(int)occupancy_grid_array_[point]]++;

		if ((int) occupancy_grid_array_[point] == costmap_2d::FREE_SPACE)//<= threshold_free)
		{
			getAdjacentPoints(point, adjacent_points);
			for (int i = 0; i < 16; i++) // length of adjacent_points array
			{
                if (adjacent_points[i] < 0)
                {
                    continue;
                }
				if (occupancy_grid_array_[adjacent_points[i]] == costmap_2d::NO_INFORMATION) {
					no_inf_count++;
//                                        ROS_DEBUG("No information found!");
				}
                                else if (occupancy_grid_array_[adjacent_points[i]] == costmap_2d::LETHAL_OBSTACLE) {
					/*
					 * Do not break here ... In some scenarios it may happen that unknown and free space
					 * form a border, but if just a small corridor is available there, it would not be
					 * detected as frontier, since even one neighboring block is an inflated block.
					 * Therefore do nothing, if even one unknown block is a neighbor of an free space
					 * block, then it is a frontier!
					 */

					//inscribed_count++;
//                                        ROS_DEBUG("Obstacle found");
					return(0);
				}
			}

			/*
			 * Count the number of lethal obstacle (unknown space also included
			 * here). If an inflated obstacle was detected ... the point is not
			 * a possible frontier and should no longer be considered.
			 * Otherwise, when all surronding blocks --> 64+7 are unknown just the
			 * one in the middle is free space, then we found a frontier!!
			 * On every found frontier, true is returned
			 */
			if (no_inf_count > 0)
			{
				/*
				 * Above a adjacent Point is taken and compared with NO_INFORMATION.
				 * If the point contains no information, the next surrounding point from that point
				 * is taken and again compared with NO_INFORMATION. If there are at least
				 * two points with no information surrounding a no information point, then return true!
				 * (at least two points with no information means that there is sufficient space
				 * with not enough information about)
				 */

                            //return(backoff(point));
                           return(point);
			}
		}
	}
	return(0);
}

int ExplorationPlanner::isFrontier_2(int point) {


	if (isFree(point)) {

		/*
		 * The point is either a obstacle or a point with not enough
		 * information about
		 * Therefore, check if the point is surrounded by other NO_INFORMATION
		 * points. Then further space to explore is found ---> frontier
		 */
		//int Neighbours = 0;
		//int points[((int)pow(8,Neighbours+1))+8]; // NEIGHBOURS points, each containing 8 adjacent points
		int no_inf_count = 0;
//		int inscribed_count = 0;
		int adjacent_points[16];
		/*
		 * Now take one point and lookup all adjacent points (surrounding points).
		 * The variable adjacentPoints contains all neighboring point (up, right, left ...)
		 */

		/*
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[0]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[1]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[2]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[3]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[4]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[5]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[6]);
		 ROS_DEBUG("Adjacent Point: %d",adjacentPoints[7]);
		 */

		//histogram[(int)occupancy_grid_array_[point]]++;

		if ((int) occupancy_grid_array_[point] == costmap_2d::FREE_SPACE)//<= threshold_free)
		{
			getAdjacentPoints(point, adjacent_points);
			for (int i = 0; i < 16; i++) // length of adjacent_points array
			{
                if (adjacent_points[i] < 0)
                {
                    continue;
                }
				if (occupancy_grid_array_[adjacent_points[i]] == costmap_2d::NO_INFORMATION) {
					no_inf_count++;
//                                        ROS_DEBUG("No information found!");
				}
                                else if (occupancy_grid_array_[adjacent_points[i]] == costmap_2d::LETHAL_OBSTACLE) {
					/*
					 * Do not break here ... In some scenarios it may happen that unknown and free space
					 * form a border, but if just a small corridor is available there, it would not be
					 * detected as frontier, since even one neighboring block is an inflated block.
					 * Therefore do nothing, if even one unknown block is a neighbor of an free space
					 * block, then it is a frontier!
					 */

					//inscribed_count++;
//                                        ROS_DEBUG("Obstacle found");
					return(0);
				}
			}

			/*
			 * Count the number of lethal obstacle (unknown space also included
			 * here). If an inflated obstacle was detected ... the point is not
			 * a possible frontier and should no longer be considered.
			 * Otherwise, when all surronding blocks --> 64+7 are unknown just the
			 * one in the middle is free space, then we found a frontier!!
			 * On every found frontier, true is returned
			 */
			if (no_inf_count > 0)
			{
				/*
				 * Above a adjacent Point is taken and compared with NO_INFORMATION.
				 * If the point contains no information, the next surrounding point from that point
				 * is taken and again compared with NO_INFORMATION. If there are at least
				 * two points with no information surrounding a no information point, then return true!
				 * (at least two points with no information means that there is sufficient space
				 * with not enough information about)
				 */

                            //return(backoff(point));
                           return(point);
			}
		}
	}
	return(0);
}

inline void ExplorationPlanner::getAdjacentPoints(int point, int points[]) {

	/*
	 * Get all surrounding neighbors and the neighbors of those.
	 */

	/*
	 * 		ooo ooo
	 *
	 */

	/*
	 points[0] = left(point);
	 points[1] = right(point);
	 points[2] = left(points[0]);
	 points[3] = right(points[1]);
	 points[4] = left(points[2]);
	 points[5] = right(points[3]);
	 points[6] = left(points[4]);
	 points[7] = right(points[5]);
	 */

	/*
	 *            o
	 *          o   o
	 *            o
	 *
	 */

        /*
	points[0] = left(point);
	points[1] = up(point);
	points[2] = right(point);
	points[3] = down(point);
        */

	/*
	 *          o o o
	 *          o   o
	 *          o o o
	 *
	 */


	 points[0] = left(point);
	 points[1] = up(point);
	 points[2] = right(point);
	 points[3] = down(point);
	 points[4] = upleft(point);
	 points[5] = upright(point);
	 points[6] = downright(point);
	 points[7] = downleft(point);

	/*
	 *        *   *   *
	 *          o o o
	 *        * o   o *
	 *          o o o
	 *        *   *   *
	 */

	 points[8] =  left(points[0]);
	 points[9] =  up(points[1]);
	 points[10] = right(points[2]);
	 points[11] = down(points[3]);
	 points[12] = upleft(points[4]);
	 points[13] = upright(points[5]);
	 points[14] = downright(points[6]);
	 points[15] = downleft(points[7]);


	/*
	 *        * + * + *
	 *        + o o o +
	 *        * o   o *
	 *        + o o o +
	 *        * + * + *
	 */
	/*
	 points[16] = up(points[4]);
	 points[17] = up(points[5]);
	 points[18] = right(points[5]);
	 points[19] = right(points[6]);
	 points[20] = down(points[6]);
	 points[21] = down(points[7]);
	 points[22] = left(points[7]);
	 points[23] = left(points[4]);
	 */

	/*
	 *      #     #     #
	 *        *   *   *
	 *          o o o
	 *      # * o   o * #
	 *          o o o
	 *        *   *   *
	 *      #     #     #
	 */
	/*
	 points[16] = left(points[8]);
	 points[17] = up(points[9]);
	 points[18] = right(points[10]);
	 points[19] = down(points[11]);
	 points[20] = upleft(points[12]);
	 points[21] = upright(points[13]);
	 points[22] = downright(points[14]);
	 points[23] = downleft(points[15]);
	 */
}

bool ExplorationPlanner::countCostMapBlocks(int point) {

       if (occupancy_grid_array_[point] == costmap_2d::NO_INFORMATION) {
//		ROS_DEBUG("[isFree] NO_INFORMATION");
		unknown++;
		return true;
	} else if ((int) occupancy_grid_array_[point] == costmap_2d::FREE_SPACE) {
		free++;
//		ROS_DEBUG("[isFree] FREE SPACE FOUND");
		return true;
	}

	else if ((int) occupancy_grid_array_[point] == costmap_2d::LETHAL_OBSTACLE) {
		lethal++;
//		ROS_DEBUG("[isFree] LETHAL OBSTACLE FOUND");
		return true;
	} else if ((int) occupancy_grid_array_[point] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
		inflated++;
//		ROS_DEBUG("[isFree] INSCRIBED INFLATED OBSTACLE FOUND");
		return true;
	} else
        {
		int undefined = (unsigned char) occupancy_grid_array_[point];
		// An obstacle was found. This is no free space, so a FALSE is returned
//		ROS_DEBUG("undefined value: %d", undefined);
		return true;
	}
	return false;
}

bool ExplorationPlanner::isFree(int point) {
	if (isValid(point)) {

		return true;
	} else {
		ROS_ERROR("Point is not valid! Reason: negative number ");
	}
	return false;
}

inline bool ExplorationPlanner::isValid(int point) {
	return (point >= 0);
}

void ExplorationPlanner::clearFrontiers() {
	std::fill_n(frontier_map_array_, num_map_cells_, 0);
}

inline int ExplorationPlanner::left(int point) {
	// only go left if no index error and if current point is not already on the left boundary
	if ((point % map_width_ != 0)) {
		return point - 1;
	}
	return -1;
}

inline int ExplorationPlanner::upleft(int point) {
	if ((point % map_width_ != 0) && (point >= (int) map_width_)) {
		return point - map_width_ - 1;
	}
	return -1;

}

inline int ExplorationPlanner::up(int point) {
	if (point >= (int) map_width_) {
		return point - map_width_;
	}
	return -1;
}

inline int ExplorationPlanner::upright(int point) {
	if ((point >= (int) map_width_) && ((point + 1) % (int) map_width_ != 0)) {
		return point - map_width_ + 1;
	}
	return -1;
}

inline int ExplorationPlanner::right(int point) {
	if ((point + 1) % map_width_ != 0) {
		return point + 1;
	}
	return -1;

}

inline int ExplorationPlanner::downright(int point) {
	if (((point + 1) % map_width_ != 0)
			&& ((point / map_width_) < (map_width_ - 1))) {
		return point + map_width_ + 1;
	}
	return -1;

}

inline int ExplorationPlanner::down(int point) {
	if ((point / map_width_) < (map_width_ - 1)) {
		return point + map_width_;
	}
	return -1;
}

inline int ExplorationPlanner::downleft(int point) {
	if (((point / map_width_) < (map_width_ - 1))
			&& (point % map_width_ != 0)) {
		return point + map_width_ - 1;
	}
	return -1;
}

unsigned char ExplorationPlanner::getCost(costmap_2d::Costmap2DROS *costmap, unsigned int cell_x, unsigned int cell_y) {
    if(cell_x >= costmap->getCostmap()->getSizeInCellsX() || cell_y >= costmap->getCostmap()->getSizeInCellsY() ) {
        //ROS_ERROR("Try to get the cost of a cell outside the costmap: returning LETHAL_OBSTACLE...");
        return costmap_2d::LETHAL_OBSTACLE;
    }
    return costmap->getCostmap()->getCost(cell_x, cell_y);
}
