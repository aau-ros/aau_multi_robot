

void docking::update_ds_graph() {

    //ds_mutex.lock();
    mutex_ds_graph.lock();
    for (unsigned int i = 0; i < ds.size(); i++) {
        for (unsigned int j = i; j < ds.size(); j++) {
            //safety checks   
            if( (unsigned int)ds[i].id >= ds_graph.size() || (unsigned int)ds[j].id >= (ds_graph[ds[i].id]).size() || ds[i].id < 0 || ds[j].id < 0)
                log_major_error("SIZE ERROR!!! WILL CAUSE SEGMENTATION FAULT!!!");          
            if (i == j)
                ds_graph[ds[i].id][ds[j].id] = 0; //TODO(minor) maybe redundant...
            else
            {
                double dist = -1;
                dist = distance(ds.at(i).x, ds.at(i).y, ds.at(j).x, ds.at(j).y);
                if (dist < 0)
                {
                    ROS_ERROR("Cannot compute DS graph at the moment: computation of distance between ds%d and ds%d failed; retring later", ds.at(i).id, ds.at(j).id);
                    recompute_graph = true;
                }
                //ROS_ERROR("%f", dist);
                //ROS_ERROR("%f", conservative_maximum_distance_one_way());
                if(conservative_maximum_distance_one_way() <= 0){
                    ROS_ERROR("Invalid value from conservative_maximum_distance_one_way() ...");
                    recompute_graph = true;
                }
                if (dist < conservative_maximum_distance_one_way())
                {
                    //update distance only if it is better than the one already stored (which can happen if a new shorter path between two ds is found)
//                    if (ds_graph[ds[i].id][ds[j].id] == -1 || dist < ds_graph[ds[i].id][ds[j].id])
                    {
                        ds_graph[ds[i].id][ds[j].id] = dist;
                        ds_graph[ds[j].id][ds[i].id] = dist;
                    }
                }
                else
                {
                    ds_graph[ds[i].id][ds[j].id] = 0;
                    ds_graph[ds[j].id][ds[i].id] = 0;
                }
            }
        }
    }
    recompute_graph = false;
    mutex_ds_graph.unlock();
    
    if(!test_mode) {
        graph_fs.open(graph_file.c_str(), std::ofstream::out | std::ofstream::trunc);
        
        graph_fs << "#sort according to DS ID" << std::endl;
        for(int i=0; i < num_ds; i++) {
            graph_fs << std::setw(5);
            for(int j=0; j < num_ds; j++)
                graph_fs << (int)ds_graph[i][j] << "   ";
            graph_fs << std::endl;
        }
        
        graph_fs << std::endl;
        
        graph_fs << "#sort according to discovery order" << std::endl;
        for(unsigned int i=0; i < ds.size(); i++) {
            graph_fs << std::setw(5);
            for(unsigned int j=0; j < ds.size(); j++)
                graph_fs << (int)ds_graph[ds[i].id][ds[j].id] << "   ";
            graph_fs << std::endl;
        }
        
        graph_fs.close();
    }
    //ds_mutex.unlock();
    
}

void docking::timer_callback_recompute_ds_graph(const ros::TimerEvent &event) {
    ROS_INFO("Periodic recomputation of DS graph");
    update_ds_graph();
    //compute_MST();
}

void docking::compute_and_publish_path_on_ds_graph() {

    ROS_INFO("computing path on DS graph");

    jobs_mutex.lock();
    std::vector<adhoc_communication::ExpFrontierElement> jobs_local_list;
    jobs_local_list = jobs; //TODO we should do the same also when computing the parameters... although they are updated by a callback so it should be ok, but just to be safe...
    jobs_mutex.unlock();
    
//    boost::shared_lock< boost::shared_mutex > lock(ds_mutex);

    ROS_DEBUG("%lu", (long unsigned int)jobs.size());
    double min_dist = numeric_limits<int>::max();
    ds_t *min_ds = NULL;
    int retry = 0;
//    while (min_ds == NULL && retry < 5) { //OMG no!!! in this way we will never select the closest ds with eos, but just the first ds found with eos!!!
        for (unsigned int i = 0; i < ds.size(); i++)
        {
            for (unsigned int j = 0; j < jobs_local_list.size(); j++)
            {
                double dist = distance(ds.at(i).x, ds.at(i).y, jobs_local_list.at(j).x_coordinate, jobs_local_list.at(j).y_coordinate);
                if (dist < 0) {
                    ROS_ERROR("Distance computation failed");
                    continue;
                }

                if (dist < conservative_maximum_distance_with_return())
                {
                    double dist2 = distance_from_robot(ds.at(i).x, ds.at(i).y);
                    if (dist2 < 0) {
                        ROS_ERROR("Distance computation failed");
                        continue;
                    }

                    if (dist2 < min_dist)
                    {
                        min_dist = dist2;
                        min_ds = &ds.at(i);
                    }
                    
                    break;
                }
            }
        }
        retry++;
        if(min_ds == NULL)
            ros::Duration(3).sleep();
//    }
    
    
    if (min_ds == NULL) {
        std_msgs::Empty msg;
        pub_finish.publish(msg);
        log_major_error("No DS with EOs was found"); //this shouldn't happen because auctioning_2 should be entered only if explorer thinks that there are EOs
        // this could happen if distance() always fails... //TODO(IMPORTANT) what happen if I return and the explorer node needs to reach a frontier?
    }
    
//    boost::shared_lock< boost::shared_mutex > lock2(ds_mutex);

    // compute closest DS
    min_dist = numeric_limits<int>::max();
    ds_t *closest_ds = NULL;
    retry = 0;
    while(closest_ds == NULL && retry < 10) {
        for (unsigned int i = 0; i < ds.size(); i++)
        {
            double dist = distance_from_robot(ds.at(i).x, ds.at(i).y);
            if (dist < 0) {
                ROS_ERROR("Distance computation failed");
                continue;
            }

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_ds = &ds.at(i);
            }
        }
        retry++;
        if(closest_ds == NULL)
            ros::Duration(3).sleep();
    }
    
    if(closest_ds == NULL)  {
        std_msgs::Empty msg;
        pub_finish.publish(msg);
        log_major_error("impossible...");
        return;
    }

    path.clear();
    index_of_ds_in_path = 0;
    
    bool ds_found_with_mst = false;
    if(closest_ds->id == min_ds->id) {
        log_minor_error("closest_ds->id == min_ds->id, this should not happen...");
        ds_found_with_mst = true;
        path_navigation_tries++;
        path.push_back(closest_ds->id);
    }
    else {
         ds_found_with_mst = find_path_2(closest_ds->id, min_ds->id, path);
        path_navigation_tries = 0;    
    }
    
    if(path_navigation_tries > 4) {
        log_major_error("Too many times closest_ds->id == min_ds->id in a row");
        ROS_INFO("finished_bool = true");
//        finished_bool = true;
        finalize();
        return;
    }

    if (ds_found_with_mst)
    {
        ROS_INFO("Found path on DS graph to reach a DS with EOs");
        adhoc_communication::MmListOfPoints msg_path;  // TODO(minor)
                                                       // maybe I can
                                                       // pass directly
                                                       // msg_path to
                                                       // find_path...
        for (unsigned int i = 0; i < path.size(); i++)
            for (unsigned int j = 0; j < ds.size(); j++)
                if (ds[j].id == path[i])
                {
                    adhoc_communication::MmPoint point;
                    point.x = ds[j].x, point.y = ds[j].y;
                    msg_path.positions.push_back(point);
                    ROS_INFO("%d: ds%d (%.1f, %.1f)", i, ds[j].id, ds[j].x, ds[j].y);
                }

        pub_moving_along_path.publish(msg_path);
        
        for (unsigned int j = 0; j < ds.size(); j++)
            if (path[0] == ds[j].id)
            {
                //TODO(minor) it should be ok... but maybe it would be better to differenciate an "intermediate target DS" from "target DS": moreover, are we sure that we cannot compute the next optimal DS when moving_along_path is true?
                set_optimal_ds_given_index(j);
//                set_target_ds_given_index(j);
//                ROS_INFO("target_ds: %d", get_target_ds_id());
                break;
            }
    }
    else {
        log_major_error("No path found on DS graph: terminating exploration"); // this should not happen, since it means that either the DSs are not well placed (i.e., they cannot cover the whole environment) or that the battery life is not enough to move between neighboring DSs even when fully charged");
        std_msgs::Empty msg;
        pub_finish.publish(msg);
    }
}

void docking::simple_compute_and_publish_path_on_ds_graph() {

    ROS_INFO("simple computing path on DS graph");

//    boost::shared_lock< boost::shared_mutex > lock(ds_mutex);

    // compute closest DS
    double min_dist = numeric_limits<int>::max();
    ds_t *closest_ds = NULL;
    int retry = 0;
    while(closest_ds == NULL && retry < 10) {
        for (unsigned int i = 0; i < ds.size(); i++)
        {
            double dist = distance_from_robot(ds.at(i).x, ds.at(i).y);
            if (dist < 0) {
                ROS_ERROR("Distance computation failed");
                continue;
            }

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_ds = &ds.at(i);
            }
        }
        retry++;
        if(closest_ds == NULL)
            ros::Duration(3).sleep();
    }
    
    if(closest_ds == NULL)  {
        std_msgs::Empty msg;
        pub_finish.publish(msg);
        log_major_error("impossible...");
        return;
    }

    path.clear();
    index_of_ds_in_path = 0;
    
    mutex_ds_graph.lock();
    bool ds_found_with_mst = false;
    if(closest_ds->id == goal_ds_path_id) {
        log_minor_error("closest_ds->id == goal_ds_path_id, this should not happen...");
        ds_found_with_mst = true;
        path_navigation_tries++;
        path.push_back(closest_ds->id);
    }
    else {
         ds_found_with_mst = find_path_2(closest_ds->id, goal_ds_path_id, path);
        path_navigation_tries = 0;    
    }
    mutex_ds_graph.unlock();

    if (ds_found_with_mst)
    {
        ROS_INFO("Found path on DS graph to reach a DS with EOs");
        adhoc_communication::MmListOfPoints msg_path;  // TODO(minor)
                                                       // maybe I can
                                                       // pass directly
                                                       // msg_path to
                                                       // find_path...
        for (unsigned int i = 0; i < path.size(); i++)
            for (unsigned int j = 0; j < ds.size(); j++)
                if (ds[j].id == path[i])
                {
                    adhoc_communication::MmPoint point;
                    point.x = ds[j].x, point.y = ds[j].y;
                    msg_path.positions.push_back(point);
                    ROS_INFO("%d: ds%d (%.1f, %.1f)", i, ds[j].id, ds[j].x, ds[j].y);
                }

        pub_moving_along_path.publish(msg_path);
        
        for (unsigned int j = 0; j < ds.size(); j++)
            if (path[0] == ds[j].id)
            {
                //TODO(minor) it should be ok... but maybe it would be better to differenciate an "intermediate target DS" from "target DS": moreover, are we sure that we cannot compute the next optimal DS when moving_along_path is true?
                set_optimal_ds_given_index(j);
//                set_target_ds_given_index(j);
//                ROS_INFO("target_ds: %d", get_target_ds_id());
                break;
            }
    }
    else {
//        log_major_error("No path found on DS graph: terminating exploration"); // this should not happen, since it means that either the DSs are not well placed (i.e., they cannot cover the whole environment) or that the battery life is not enough to move between neighboring DSs even when fully charged");
//        std_msgs::Empty msg;
//        pub_finish.publish(msg);
        ;
    }
}

void docking::goal_ds_for_path_navigation_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg) {
    ROS_INFO("received goal ds for path navigation");
    moving_along_path = true;
    goal_ds_path_id = msg.get()->id;
    simple_compute_and_publish_path_on_ds_graph();
    if(finished_bool) {
        std_msgs::Empty msg;
        pub_finish.publish(msg);
    } else {
        ROS_INFO("Robot needs to recharge");
        if(!going_to_ds) //TODO(minor) very bad check... to be sure that only if the robot has not just won
                                  // another auction it will start its own (since maybe explorer is still not aware of this and so will communicate "robot_state::AUCTIONING" state...); do we have other similar problems?
        {
            ROS_ERROR("calling start_new_auction()");
//            start_new_auction();
        }
        else {
            ROS_INFO("going_to_ds is true, so we cannot start another auction");
        }
    }
}

void docking::compute_and_publish_path_on_ds_graph_to_home() {
double min_dist = numeric_limits<int>::max();
    ds_t *min_ds = NULL;
    int retry = 0;
    
//    boost::shared_lock< boost::shared_mutex > lock(ds_mutex);
    
//    while (min_ds == NULL && retry < 5) {
        for (unsigned int i = 0; i < ds.size(); i++)
        {
                double dist = distance(ds.at(i).x, ds.at(i).y, 0, 0); //TODO use robot_home_x
                if (dist < 0)
                    continue;

                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_ds = &ds.at(i);
                }
                    
        }
        retry++;
//    }
    
    if (min_ds == NULL) {
        std_msgs::Empty msg;
        pub_finish.publish(msg);
        log_major_error("impossible, min_ds == NULL for going home...");
        // this could happen if distance() always fails... //TODO(IMPORTANT) what happen if I return and the explorer node needs to reach a frontier?
        return;
    }

    // compute closest DS
    min_dist = numeric_limits<int>::max();
    ds_t *closest_ds = NULL;
    retry = 0;
    while(closest_ds == NULL && retry < 10) {
        for (unsigned int i = 0; i < ds.size(); i++)
        {
            double dist = distance_from_robot(ds.at(i).x, ds.at(i).y);
            if (dist < 0)
                continue;

            if (dist < min_dist)
            {
                min_dist = dist;
                closest_ds = &ds.at(i);
            }
        }
      
        retry++;
    }
    
    if(closest_ds == NULL)  {
        std_msgs::Empty msg;
        pub_finish.publish(msg);
        log_major_error("impossible, closest_ds == NULL for going home...");
        return;
    }

    path.clear();
    index_of_ds_in_path = 0;
    bool ds_found_with_mst = find_path_2(closest_ds->id, min_ds->id, path);
    
    if (ds_found_with_mst)
    {

        adhoc_communication::MmListOfPoints msg_path;  // TODO(minor)
                                                       // maybe I can
                                                       // pass directly
                                                       // msg_path to
                                                       // find_path...
        for (unsigned int i = 0; i < path.size(); i++)
            for (unsigned int j = 0; j < ds.size(); j++)
                if (ds[j].id == path[i])
                {
                    adhoc_communication::MmPoint point;
                    point.x = ds[j].x, point.y = ds[j].y;
                    msg_path.positions.push_back(point);
                }

        pub_moving_along_path.publish(msg_path);
        
        for (unsigned int j = 0; j < ds.size(); j++)
            if (path[0] == ds[j].id)
            {
                //TODO(minor) it should be ok... but maybe it would be better to differenciate an "intermediate target DS" from "target DS": moreover, are we sure that we cannot compute the next optimal DS when moving_along_path is true?
                set_optimal_ds_given_index(j);
//                set_target_ds_given_index(j);
//                ROS_INFO("target_ds: %d", get_target_ds_id());
                break;
            }
    }
    else {
        log_major_error("no path found to reach home!!!");
        ROS_INFO("finished_bool = true");
        finalize();        
//        finished_bool = true;
    }
}

void docking::next_ds_callback(const std_msgs::Empty &msg)
{
//    boost::shared_lock< boost::shared_mutex > lock(ds_mutex);
    if(path.size() == 0)
        log_major_error("path.size() == 0");
    
    if(index_of_ds_in_path < 0) {
        log_major_error("index_of_ds_in_path");
        return;
    }
    
    if ((unsigned int)index_of_ds_in_path < path.size() - 1)
    {
        ROS_INFO("Select next DS on the path in the DS graph to reach the final DS with EOs");
        index_of_ds_in_path++;
        ROS_DEBUG("%d", index_of_ds_in_path);
        for (unsigned int i = 0; i < ds.size(); i++)
            if (path[index_of_ds_in_path] == ds[i].id)
            {   
                ROS_INFO("Next DS on path: ds%d", ds[i].id);
//                set_target_ds_given_index(i);  // TODO(minor) probably ok...
                set_optimal_ds_given_index(i);    // TODO(minor) VERY BAD!!!!
                break;
            }
    }
    else {
        if(moving_along_path) {
            ROS_INFO("We have reached the final DS and charged at it");
            moving_along_path = false;
        }
        else {
            log_major_error("we are not moving along path!!!");
        }
    }
}
