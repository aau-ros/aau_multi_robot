void docking::cb_battery(const explorer::battery_state::ConstPtr &msg)
{
    //ROS_DEBUG("Received battery state");

    /* Store new battery state */
    battery.soc = msg.get()->soc;
    battery.remaining_time_charge = msg.get()->remaining_time_charge;
    battery.remaining_time_run = msg.get()->remaining_time_run;
    battery.remaining_distance = msg.get()->remaining_distance;

    next_remaining_distance = battery.remaining_distance;
    
    ROS_DEBUG("SOC: %d%%; rem. time: %.1f; rem. distance: %.1f", (int) (battery.soc * 100.0), battery.remaining_time_run, battery.remaining_distance);
    
    //TODO(minor) very bad way to be sure to set maximum_travelling_distance...
    if(maximum_travelling_distance < msg.get()->remaining_distance)
        maximum_travelling_distance = msg.get()->remaining_distance;
}

void docking::ds_with_EOs_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg) {
    ROS_INFO("received info on EOs for ds%d", msg.get()->id);
    for(unsigned int i=0; i<ds.size(); i++)
        if(ds.at(i).id == msg.get()->id) {
            ds.at(i).has_EOs = msg.get()->has_EOs;
            break;
        }
}

void docking::cb_docking_stations(const adhoc_communication::EmDockingStation::ConstPtr &msg)
{
    if(!checkAndUpdateReceivedMessageId("docking_stations", msg.get()->header.message_id, msg.get()->header.sender_robot)) {
        log_major_error("invalid message id!");
        return;   
    }
    
    ds_mutex.lock();

    ROS_INFO("received ds%d", msg.get()->id);

    /* Check if DS is in list already */
    bool new_ds = true;
    for (unsigned int i = 0; i < ds.size(); i++)
    {
        if (ds[i].id == msg.get()->id)
        {
            // coordinates don't match //TODO(minor) do it...
            double x, y;
            
            abs_to_rel(msg.get()->x, msg.get()->y, &x, &y);
            
            if (ds[i].x != x || ds[i].y != y) {
                if(!already_printed_matching_error) {
                    ROS_ERROR("Coordinates of docking station %d from robot %u do not match: (%.2f, "
                              "%.2f) != (%.2f, %.2f)",
                              ds[i].id, (unsigned int)msg.get()->header.sender_robot, ds[i].x, ds[i].y, x, y);
                    ROS_ERROR("original coord.s: (%.2f, %.2f)", msg.get()->x, msg.get()->y);
                    log_major_error("Coordinates of docking station do not match");
                    already_printed_matching_error = true;
                }
                break;
            }

            /* DS is already in list, update its state; notice that, due to message loss, it could be possible that the robot has received a new state (vacant or occupied) for the DS that it is no different from the one already stored by the robot */

            new_ds = false;

            
            if(ds.at(i).timestamp > msg.get()->header.timestamp)
                continue;

            if (ds[i].vacant != msg.get()->vacant)
            {
                ds[i].vacant = msg.get()->vacant;
                ds.at(i).timestamp = msg.get()->header.timestamp;
                ROS_INFO("ds%d is now %s", msg.get()->id,
                          (msg.get()->vacant ? "vacant" : "occupied"));
                          
                // if the ds is now vacant and it's robot's target ds and the robot is in queue, the robot can start already start a new auction
                //TODO but in this way we start an auction when the DS has become vacant because the robot that was previously recharging at that DS was forced to leave because it has just lost an auction, which means that we have already a winner for the DS...
                
                //TODO but if instead the robot is going in queue, it won't restart the auction immediately... we should check when transictioning from robot_state::GOING_IN_QUEUE to in_queue if the DS is still occupied
                    
            }
            else
                ROS_DEBUG("State of ds%d is unchanged (%s)", msg.get()->id,
                          (msg.get()->vacant ? "vacant" : "occupied"));
            
            break;
        }
    }
    

    /* If the DS is new, add it */
    if (new_ds)
    {
        // check that DS is not already in the list of discovered (but possibly not reachable DSs)
        bool already_discovered = false;
         for (unsigned int i = 0; i < discovered_ds.size() && !already_discovered; i++)
            if (discovered_ds[i].id == msg.get()->id) {
                ROS_INFO("The received DS (%d) has been already discovered, even if at the moment the robot does not know how to reach it", msg.get()->id);
                already_discovered = true;
            }
            
        if(!already_discovered) {
            //TODO(minor) use a function...
            ds_t s;
            s.id = msg.get()->id;
            
            abs_to_rel(msg.get()->x, msg.get()->y, &s.x, &s.y);
            
            s.vacant = msg.get()->vacant;
            s.timestamp = msg.get()->header.timestamp;
            discovered_ds.push_back(s); //discovered, but not reachable, since i'm not sure if it is reachable for this robot...
            ROS_INFO("New docking station received: ds%d (%f, %f)", s.id, s.x, s.y);

            /* Remove DS from the vector of undiscovered DSs */
            for (auto it = undiscovered_ds.begin(); it != undiscovered_ds.end(); it++)
                if ((*it).id == s.id)
                {
                    undiscovered_ds.erase(it);
                    break;
                }
        }
    }
    
    ds_mutex.unlock();

}

void docking::cb_jobs(const adhoc_communication::ExpFrontier::ConstPtr &msg)
{
    /* NB. Here we use a surely inefficient approach: every time we clear the frontier vector and we store the whole set of unvisited frontiers from skratch. A better approach would be of course to just remove the visited frontiers and add the new discovered ones: notice however that a frontier could have been visited by another robot, so it is not easy to implement a service in explorer that returns the frontiers that have been visited since the last call of this service */ //TODO(minor) do it...

    /* Clear previous information about frontiers */
    //jobs.clear();

    /* Store new information about frontiers */
    /*
    for (int i = 0; i < msg.get()->frontier_element.size(); ++i)
    {
        adhoc_communication::ExpFrontierElement frontier_element = msg.get()->frontier_element.at(i);
        job_t job;
        job.id = frontier_element.id;
        job.x = frontier_element.x_coordinate;
        job.y = frontier_element.y_coordinate;
        jobs.push_back(job);
    }
    */
    
    jobs_mutex.lock();
    jobs = msg.get()->frontier_element;
    jobs_mutex.unlock();
    
    if(jobs.size() > 0)
        no_jobs_received_yet = false;

}

void docking::cb_robots(const adhoc_communication::EmRobot::ConstPtr &msg)
{
    ROS_DEBUG("docking: Received information from robot %d", msg.get()->id);
    
    robot_mutex.lock();

    /* Log information */ //TODO(minor) better log file...
    ros::Duration time = ros::Time::now() - time_start;
    fs3_csv.open(csv_file_3.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs3_csv << ros::Time::now().toSec() << "," << ros::WallTime::now().toSec() << "," 
        << msg.get()->id << std::endl;
    fs3_csv.close();

    /* Check if robot is in list already */
    bool new_robot = true;
    for (unsigned int i = 0; i < robots.size(); ++i)
    {
        if (robots[i].id == msg.get()->id)
        {
            new_robot = false;

            robots[i].state = static_cast<robot_state::robot_state_t>(msg.get()->state);
            robots[i].x = msg.get()->x;
            robots[i].y = msg.get()->y;
            robots[i].selected_ds = msg.get()->selected_ds;
                
            break;
        }
    }

    /* If it is a new robot, add it */
    if (new_robot)
    {
        /* Store robot information */    
        robot_t new_robot;
        new_robot.id = msg.get()->id;
        new_robot.state = static_cast<robot_state::robot_state_t>(msg.get()->state);
        new_robot.x = msg.get()->x;
        new_robot.y = msg.get()->y;
        new_robot.selected_ds = msg.get()->selected_ds;
        robots.push_back(new_robot);

        /* Recompute number of robots */
        int count = 0;
        for (unsigned int i = 0; i < robots.size(); i++)
            count++;
        num_robots = count;  // TODO(minor) also works for real exp?
    }
    
    robot_mutex.unlock();
    
}
