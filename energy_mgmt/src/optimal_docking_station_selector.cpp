#include "optimal_docking_station_selector.h"

OptimalDockingStationSelector::OptimalDockingStationSelector() {
    ROS_INFO("Creating instance of OptimalDockingStationSelector class");
    loadParameters();
    ROS_INFO("Instance successfully created");
}

void OptimalDockingStationSelector::loadParameters() {
    ros::NodeHandle nh_tilde("~");
    int tmp;
    if(!nh_tilde.getParam("num_robots", tmp))
        ROS_FATAL("PARAM NOT FOUND");
    num_robots = (unsigned int)tmp_num_robots;

    if(!nh_tilde.param<int>("ds_selection_policy", tmp))
        ROS_FATAL("PARAM NOT FOUND");
    docking_station_selection_policy = (unsigned int)tmp;
}

void OptimalDockingStationSelector::setDockingStationManager() {

}

void docking::compute_optimal_ds() //TODO(minor) best waw to handle errors in distance computation??
//TODO(minor) maybe there are more efficient for some steps; for instance, i coudl keep track somewhere of the DS with EOs and avoid recomputing them two times in the same call of this funcion...
{   
    ROS_INFO("Compute optimal DS");
    
//    boost::shared_lock< boost::shared_mutex > lock(ds_mutex);

    /* Compute optimal DS only if at least one DS is reachable (just for efficiency and debugging) */

    if (ds.size() > 0 && can_update_ds() && !moving_along_path && !going_to_ds) //TODO but in these way we are not updating the optimal_ds less frequently... and moreover it affects also explorer...
    {

        // copy content (notice that if jobs is modified later, the other vector is not affected: http://www.cplusplus.com/reference/vector/vector/operator=/)
        jobs_mutex.lock();
        std::vector<adhoc_communication::ExpFrontierElement> jobs_local_list;
        jobs_local_list = jobs; //TODO we should do the same also when computing the parameters... although they are updated by a callback so it should be ok, but just to be safe...
        jobs_mutex.unlock();

        /* Store currently optimal DS (for debugging ans safety checks)
         *
         * NB: even if the robot position does not change during the execution of
         * this function, the distance between the robot and a fixed point computed
         * using distance_from_robot() could change, because it calls a function in
         * explorer node that uses a costmap to compute the distance, and since the
         * costmap could be updated during the execution of compute_optimal_ds() (or,
         * more simply, the algorithm used to compute the distance is not 100%
         * precise), the distance could change: this means that we need to check at
         * the end if the optimal DS that we have computed is really a
         * different DS from the previous one; this can happens for instance because
         * the other robots are moving. Of course the differences between two
         * different calls of distance_from_robot with the same point as argument
         * will be very little, but they are enough to make us think that we have
         * found a DS that is better than the one already selected, even if the new
         * DS is exactly the old one. Another problems is that we could select a DS
         * that is not really the closest at the end of the execution of
         * compute_optimal_ds() because the distances could have changed, but we
         * accept this. */

        if(old_optimal_ds_id > 10000) //can happen sometimes... why? segmentation fault?
            log_major_error("WHAT?????????????????????????");

        // TODO(minor) functions
        /* "Closest" policy */
        if (ds_selection_policy == 0) // TODO(minor) switch-case
            compute_closest_ds();

        /* "Vacant" policy */
        else if (ds_selection_policy == 1)
        {
            /* Check if there are vacant DSs that are currently reachable. If there are, check also which one is the closest to the robot */
            bool found_reachable_vacant_ds = false, found_vacant_ds = false;
            double min_dist = numeric_limits<int>::max();
            for (std::vector<ds_t>::iterator it = ds.begin(); it != ds.end(); it++)
            {
                if ((*it).vacant)
                {
                    ROS_DEBUG("ds%d is vacant", it->id);
                    found_vacant_ds = true;
                    
                    double dist = distance_from_robot((*it).x, (*it).y);
                    if (dist < 0)
                        continue;
                    if(dist < conservative_remaining_distance_one_way() ) {
                        /* We have just found a vacant DS that is reachable (possibly the one already selected as optimal DS before calling
                         * this function).
                         *
                         * Notice that is is important to consider also the already selected optimal DS when we loop on all
                         *the DSs, since we have to update variable 'found_vacant_ds' to avoid falling back to "closest"
                         *policy, i.e., we cannot use a check like best_ds->id != (*it).id here */
                        found_reachable_vacant_ds = true;

                        /* Check if that DS is also the closest one */
                        double dist = distance_from_robot((*it).x, (*it).y);
                        if (dist < 0) {
                            ROS_ERROR("Distance computation failed: skipping this DS in the computation of the optimal DS..."); //TODO(minor) place everywhere, and write it better... //TODO maybe we should instead skip the computation of the next optimal DS in these cases...
                            continue;
                        }
                        if (dist < min_dist)
                        {
                            min_dist = dist;
                            next_optimal_ds_id = it->id;
                        }
                    }
                } else
                    ROS_DEBUG("ds%d is occupied", it->id);
            }

            /* If no DS is vacant at the moment, use "closest" policy to update the
             * optimal DS */
            if (!found_reachable_vacant_ds)
            {
                if(found_vacant_ds)
                    ROS_DEBUG("All the vacant DSs are currently unreachable: fall back to 'closest' policy");
                else
                    ROS_DEBUG("No vacant DS found: fall back to 'closest' policy");
                compute_closest_ds();
            }
            else
                ROS_DEBUG("'vacant' policy found an optimal DS");
        }

        /* "Opportune" policy */
        else if (ds_selection_policy == 2)
        {
            if (!moving_along_path) //TODO reduntant
            {
                /* Check if there are reachable DSs (i.e., DSs that the robot can reach with the remaining battery life) with EOs */
                double min_dist = numeric_limits<int>::max();
                bool found_reachable_ds_with_eo = false, found_ds_with_eo = false;
                for (unsigned int i = 0; i < ds.size(); i++) {
//                    for (unsigned int j = 0; j < jobs_local_list.size(); j++)
//                    {
//                        double dist = distance(ds.at(i).x, ds.at(i).y, jobs_local_list.at(j).x_coordinate, jobs_local_list.at(j).y_coordinate);
//                        if (dist < 0)
//                            continue;
//                        if (dist < conservative_maximum_distance_with_return())
//                        {
//                            /* We have found a DS with EOs */
//                            found_ds_with_eo = true;

//                            //TODO(minor) maybe it will be more efficient to invert the checks? but am I sure taht it works considering the code later?
//                            /* Check if that DS is also directly reachable (i.e., without
//                             * recharging at intermediate DSs) */
//                            double dist2 = distance_from_robot(ds.at(i).x, ds.at(i).y);
//                            if (dist2 < 0)
//                                continue;
//                            if (dist2 < conservative_remaining_distance_one_way())
//                            {
//                                /* We have found a DS that is directly reachable and with EOs */
//                                found_reachable_ds_with_eo = true;

//                                /* Check if it also the closest reachable DS with EOs */
//                                if (dist2 < min_dist)  // TODO(minor) maybe another heuristics would be better...
//                                {
//                                    /* Update optimal DS */
//                                    min_dist = dist2;
//                                    next_optimal_ds_id = ds.at(i).id;
//                                }
//                            }
//                        }
                        if(ds.at(i).has_EOs) {
                            found_ds_with_eo = true;

                            //TODO(minor) maybe it will be more efficient to invert the checks? but am I sure taht it works considering the code later?
                            /* Check if that DS is also directly reachable (i.e., without
                             * recharging at intermediate DSs) */
                            double dist2 = distance_from_robot(ds.at(i).x, ds.at(i).y);
                            if (dist2 < 0)
                                continue;
                            if (dist2 < conservative_remaining_distance_one_way())
                            {
                                /* We have found a DS that is directly reachable and with EOs */
                                found_reachable_ds_with_eo = true;

                                /* Check if it also the closest reachable DS with EOs */
                                if (dist2 < min_dist)  // TODO(minor) maybe another heuristics would be better...
                                {
                                    /* Update optimal DS */
                                    min_dist = dist2;
                                    next_optimal_ds_id = ds.at(i).id;
                                }
                            }
                        }
                    }

                /* If there are no reachable DSs with EOs, check if there are DSs with
                 * EOs: if there are, compute a path on the graph of the DSs to reach
                 * one of these DSs, otherwise jsut use "closest" policy */
                if (!found_reachable_ds_with_eo)
                {
                    if (found_ds_with_eo)
                    {
                        /* Compute a path formed by DSs that must be used for recharging, to
                         * reach a DS with EOs */
                        bool ds_found_with_mst = false;

                        
                        //if (!OPP_ONLY_TWO_DS)  // TODO(minor)
                        {
                            //TODO(minor) is it better to reacharge at each intermediate DS as soon as one Ds is found, or just when it is strictly necessary???
                            // compute closest DS with EOs // TODO(minor) are we sure that this is
                            // what the paper asks?
                            double min_dist = numeric_limits<int>::max();
                            ds_t *min_ds = NULL;
                            for (unsigned int i = 0; i < ds.size(); i++)
                            {
                                for (unsigned int j = 0; j < jobs_local_list.size(); j++)
                                {
                                    double dist = distance(ds.at(i).x, ds.at(i).y, jobs_local_list.at(j).x_coordinate, jobs_local_list.at(j).y_coordinate);
                                    if (dist < 0)
                                        continue;

                                    if (dist < conservative_maximum_distance_with_return())
                                    {
                                        double dist2 = distance_from_robot(ds.at(i).x, ds.at(i).y);
                                        if (dist2 < 0)
                                            continue;

                                        if (dist2 < min_dist)
                                        {
                                            min_dist = dist2;
                                            min_ds = &ds.at(i);
                                        }
                                        
                                        break;
                                    }
                                }
                            }
                            if (min_ds == NULL) {
                                ;  // this could happen if distance() always fails... //TODO(IMPORTANT) what happen if I return and the explorer node needs to reach a frontier?
                            } else {

                                // compute closest DS
                                min_dist = numeric_limits<int>::max();
                                ds_t *closest_ds = NULL;
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

                                path.clear();
                                index_of_ds_in_path = 0;
                                if(closest_ds == NULL)
                                    ROS_ERROR("NO CLOSEST DS HAS BEEN FOUND!!!");
                                ds_found_with_mst = find_path_2(closest_ds->id, min_ds->id, path);

                                if (ds_found_with_mst)
                                {
    //                                int closest_ds_id;

                                    moving_along_path = true;

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
                                            next_optimal_ds_id = ds.at(j).id;
    //                                        set_target_ds_given_index(j);
    //                                        ROS_INFO("target_ds: %d", get_target_ds_id());
                                        }
                                }
                                else
                                    // closest policy //TODO(minor) when could this happen?
                                    compute_closest_ds();
                            }
                        }

                        /*
                        else  // only two ds... not really implemented...
                        {
                            if (!moving_along_path)
                            {
                                double first_step_x, first_step_y, second_step_x, second_step_y;
                                for (int i = 0; i < ds.size(); i++)
                                {
                                    bool existing_eo;
                                    for (int j = 0; j < jobs_local_list.size(); j++)
                                    {
                                        double dist = distance(ds.at(i).x, ds.at(i).y, jobs_local_list.at(j).x, jobs_local_list.at(j).y);
                                        if (dist < 0)
                                            continue;
                                        if (dist < battery.remaining_distance / 2)
                                        {
                                            existing_eo = true;
                                            for (int k = 0; k < ds.size(); k++)
                                            {
                                                dist = distance(ds.at(i).x, ds.at(i).y, ds.at(k).x, ds.at(k).y);
                                                if (dist < 0)
                                                    continue;
                                                if (k != i && dist < battery.remaining_distance / 2)
                                                {
                                                    double dist2 = distance_from_robot(ds.at(k).x, ds.at(k).y);
                                                    if (dist2 < 0)
                                                        continue;
                                                    if (dist2 < battery.remaining_distance / 2)
                                                    {
                                                        moving_along_path = true;
                                                        ds_found_with_mst = true;
                                                        first_step_x = ds.at(k).x;
                                                        first_step_y = ds.at(k).y;
                                                        second_step_x = ds.at(i).x;
                                                        second_step_y = ds.at(i).y;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                if (ds_found_with_mst)
                                {
                                    moving_along_path = true;

                                    adhoc_communication::MmListOfPoints msg_path;
                                    msg_path.positions[0].x = first_step_x;
                                    msg_path.positions[0].y = first_step_y;
                                    msg_path.positions[1].x = second_step_x;
                                    msg_path.positions[1].y = second_step_y;

                                    pub_moving_along_path.publish(msg_path);
                                }
                            }
                        }
                        */
                    }
                    else 
						if(no_jobs_received_yet)
	                        compute_closest_ds();  // TODO(minor) although probably the robot will think
	                                               // that the exploration is over given how
	                                               // explorer works for the moment...
						else {
						    ROS_INFO("finished_bool = true");
//                        	finished_bool = true;
                            finalize();
                        }
                }
            }
            else
                ROS_INFO("Already moving along path...");
        }

        /* "Current" policy */
        else if (ds_selection_policy == 3)
        {
            /* If no optimal DS has been selected yet, use "closest" policy, otherwise use "current" policy */ //TODO(minor) assumption
            if (!optimal_ds_is_set()) {
                ROS_DEBUG("No optimal docking station selected yet: fall back to 'closest' policy");
                compute_closest_ds();
            }
            else
            {
                /* If the currently optimal DS has still EOs, keep using it, otherwise use
                 * "closest" policy */
//                bool existing_eo = false;
//                for (unsigned int i = 0; i < jobs_local_list.size(); i++)
//                {
//                    double dist = distance(get_optimal_ds_x(), get_optimal_ds_y(), jobs_local_list.at(i).x_coordinate, jobs_local_list.at(i).y_coordinate);
//                    if (dist < 0) {
//                        //ROS_ERROR("Computation of DS-frontier distance failed: ignore this frontier (i.e., do not consider it an EO)");
//                        continue;
//                    }
//                    if (dist < conservative_maximum_distance_with_return())
//                    {
//                        existing_eo = true;
//                        break;
//                    }
//                }
                bool existing_eo;
                for(unsigned int i=0; i < ds.size(); i++)
                    if(ds.at(i).id == get_optimal_ds_id()) {
                        existing_eo = ds.at(i).has_EOs;
                        break;                    
                    }
                if (!existing_eo) {
                    ROS_DEBUG("Current optimal DS has no more EOs: use 'closest' policy to compute new optimal DS");
                    compute_closest_ds();
                }
            }
        }

        /* "Flocking" policy */
        else if (ds_selection_policy == 4) //TODO(minor) comments and possibly use functions...
        {   
            /* Compute DS with minimal cost */
            double min_cost = numeric_limits<int>::max();
            for (unsigned int d = 0; d < ds.size(); d++)
            {
                /* n_r */
                int count = 0;
                for (unsigned int i = 0; i < robots.size(); i++)
                    if (optimal_ds_is_set() &&
                        robots.at(i).selected_ds == get_optimal_ds_id())
                        count++;
                double n_r = (double)count / (double)num_robots;

                /* d_s */
                int sum_x = 0, sum_y = 0;
                for (unsigned int i = 0; i < robots.size(); i++)
                {
                    sum_x += robots.at(i).x;
                    sum_y += robots.at(i).y;
                }
                double flock_x = (double)sum_x / (double)num_robots;
                double flock_y = (double)sum_y / (double)num_robots;
                double max_distance = numeric_limits<int>::min();
                for(unsigned int h=0; h < ds.size(); h++) {
                    double dist = distance(ds[d].x, ds[d].y, ds[h].x, ds[h].y);
                    if(dist < 0)
                        continue; //TODO(minor) hmm...
                    if(dist < conservative_maximum_distance_one_way())
                        if(dist > max_distance)
                            max_distance = dist;
                    
                }
                if(max_distance == numeric_limits<int>::min())
                    continue; //TODO(minor) hmm
                double d_s = distance(ds.at(d).x, ds.at(d).y, flock_x, flock_y) / max_distance;
                if (d_s < 0)
                    continue;

                /* theta_s */
                double swarm_direction_x = 0, swarm_direction_y = 0;
                for (unsigned int i = 0; i < robots.size(); i++)
                {
                    double robot_i_optimal_ds_x = -1, robot_i_optimal_ds_y = -1;

                    for (unsigned int k = 0; k < ds.size(); k++)
                        if (robots.at(i).selected_ds == ds.at(k).id)
                        {
                            robot_i_optimal_ds_x = ds.at(k).x;
                            robot_i_optimal_ds_y = ds.at(k).y;
                        }
                        
                    if(robot_i_optimal_ds_x < 0 || robot_i_optimal_ds_y < 0)
                        ROS_ERROR("Invalid index(es)!");

                    swarm_direction_x += robot_i_optimal_ds_x - robots.at(i).x;
                    swarm_direction_y += robot_i_optimal_ds_y - robots.at(i).y;
                }
                double rho = atan2(swarm_direction_y, swarm_direction_x) * 180 / PI; //degree; e.g., with atan2(1,1), rho is 45.00
                                                                              //To compute the value, the function takes into account the sign of both arguments in order to determine the quadrant.
                double alpha = atan2((ds.at(d).y - robot->y), (ds.at(d).x - robot->x)) * 180 / PI;
                double theta_s = fabs(alpha - rho) / (double)180;

                /* d_f */
                double d_f = numeric_limits<int>::max();
                for (unsigned int i = 0; i < jobs_local_list.size(); i++)
                {
                    double dist = distance(ds[d].x, ds[d].y, jobs_local_list[i].x_coordinate, jobs_local_list[i].y_coordinate);
                    if (dist < 0)
                        continue;
                    if (dist < d_f)
                        d_f = dist;
                }

                /* resulting cost (as a sum of all the previously computed parameters) */
                double cost = n_r + d_s + theta_s + d_f;
                if (cost < min_cost)
                {
                    // Check that the candidate new optimal DS is reachable
                    double dist = distance_from_robot(ds.at(d).x, ds.at(d).y);
                    if (dist < 0 || dist >= conservative_maximum_distance_with_return())
                        continue;
                    else {
                        // The candidate new optimal DS can be set as new optimal DS (until we don't find a better one)
                        min_cost = cost;
                        next_optimal_ds_id = ds.at(d).id;
                    }                    
                }
            }
        }

//        bool changed = false;
        /* If a new optimal DS has been found, parameter l4 of the charging likelihood function must be updated. Notice that the other robots will be informed about this when the send_robot_information() function is called */
//        if (!optimal_ds_is_set())
//            ROS_DEBUG("No optimal DS has been selected yet");      
        if (old_optimal_ds_id != next_optimal_ds_id)
        {
            ROS_INFO("calling lockState()");
            robot_state_manager->lockRobotState();
            if(can_update_ds() || waiting_to_discover_a_ds)
            {
//                optimal_ds_mutex.lock();                

                waiting_to_discover_a_ds = false;
                finished_bool = false; //TODO(minor) find better place...
//                changed = true;
                set_optimal_ds(next_optimal_ds_id);
                
                if(get_optimal_ds_id() < 0 || get_optimal_ds_id() >= num_ds) { //can happen sometimes... buffer overflow somewhere?
                    log_major_error("OH NO!!!!!!!!!!!!");
                    ROS_INFO("%d", get_optimal_ds_id());
                }

                old_optimal_ds_id = get_optimal_ds_id(); //TODO reduntant now, we could use get_optimal_ds_id also in the if...
                
                /* Notify explorer about the optimal DS change */
                adhoc_communication::EmDockingStation msg_optimal;
                msg_optimal.id = get_optimal_ds_id();
                msg_optimal.x = get_optimal_ds_x();
                msg_optimal.y = get_optimal_ds_y();
                pub_new_optimal_ds.publish(msg_optimal);

//                optimal_ds_mutex.unlock(); 
            }
            ROS_INFO("calling unlockState()");
            robot_state_manager->unlockRobotState();
        }
        else
            ROS_INFO("Optimal DS unchanged");
            
//        if(get_optimal_ds_id() != get_target_ds_id())  {
//            if(robot_state != robot_state::GOING_IN_QUEUE && robot_state != robot_state::GOING_CHECKING_VACANCY && robot_state != robot_state::CHECKING_VACANCY && robot_state != robot_state::GOING_CHARGING && robot_state != robot_state::CHARGING && robot_state != in_queue) //TODO exclude also in_queue???
//            {   
//                changed = true;
//                ROS_INFO("Update target DS, and inform explorer");
//                set_target_ds(get_optimal_ds_id());

//                // If we inform explorer here, we are sure that it will know the target DS even if it loses an auction, so it knows where to go in_queue
//                geometry_msgs::PointStamped msg1;
//                msg1.point.x = get_target_ds_x();
//                msg1.point.y = get_target_ds_y();
//                pub_new_target_ds.publish(msg1);
//                
//            }
//            else
//                ROS_INFO("Target DS cannot be updated at the moment");
//        }
//        
//        if(changed) {

//        }
            
    }
    else {
        if (ds.size() <= 0)
            ROS_DEBUG("No DS"); 
        else if(going_to_ds) 
            ROS_DEBUG("Robot is going to recharge: optimal DS cannot be updated at the moment"); 
        else if(moving_along_path)
            ROS_DEBUG("robot is moving along path, cannot compute optimal DS");
        else
            ROS_ERROR("This should be impossibile...");
    }
                  
    ROS_INFO("end compute_optimal_ds()");          
}

void docking::log_optimal_ds() {
    ROS_INFO("logging");
//    optimal_ds_mutex.lock();
    /* Keep track of the optimal and target DSs in log file */
//    if(old_optimal_ds_id_for_log != get_optimal_ds_id() || old_target_ds_id_for_log != get_target_ds_id() ) {
    if(old_optimal_ds_id_for_log != get_optimal_ds_id() ) {
        ros::Duration time = ros::Time::now() - time_start;
        fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//        ROS_DEBUG("%d", target_ds_id);
        fs_csv << ros::Time::now().toSec() << "," << ros::WallTime::now().toSec() << ","
            << get_optimal_ds_id() << std::endl;
        fs_csv.close();
        old_optimal_ds_id_for_log = get_optimal_ds_id();
//        old_target_ds_id_for_log = get_target_ds_id();
    }
//    optimal_ds_mutex.unlock();
}

void docking::cb_robots(const adhoc_communication::EmRobot::ConstPtr &msg)
{
    ROS_DEBUG("docking: Received information from robot %d", msg.get()->id);
    
    //ROS_ERROR("(%.1f, %.1f)", msg.get()->x, msg.get()->y);
    if (DEBUG) //TODO(minor) move away...
    {
        debug_timers[msg.get()->id].stop();
        debug_timers[msg.get()->id].setPeriod(ros::Duration(20), true);
        debug_timers[msg.get()->id].start();
        return;
    }
    
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
            /* The robot is not new, update its information */

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


void docking::cb_docking_stations(const adhoc_communication::EmDockingStation::ConstPtr &msg)
{

    if(!checkAndUpdateReceivedMessageId("docking_stations", msg.get()->header.message_id, msg.get()->header.sender_robot)) {
        log_major_error("invalid message id!");
        return;   
    }

    ROS_INFO("received ds%d", msg.get()->id);
//    boost::shared_lock< boost::shared_mutex > lock(ds_mutex);
    ds_mutex.lock();
    
    // Safety check on the received DS
    if(msg.get()->id < 0 || msg.get()->id > num_ds) {
        log_major_error("Invalid DS id");
        ROS_ERROR("%d", msg.get()->id);
        return;
    }

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
            
            if(s.id < 0 || s.id >= num_ds)
                log_major_error("invalid ds id 3!!!");
            
            abs_to_rel(msg.get()->x, msg.get()->y, &s.x, &s.y);
            
            s.vacant = msg.get()->vacant;
            s.timestamp = msg.get()->header.timestamp;
            discovered_ds.push_back(s); //discovered, but not reachable, since i'm not sure if it is reachable for this robot...
            ROS_INFO("New docking station received: ds%d (%f, %f)", s.id, s.x, s.y);

            /* Remove DS from the vector of undiscovered DSs */
            for (std::vector<ds_t>::iterator it = undiscovered_ds.begin(); it != undiscovered_ds.end(); it++)
                if ((*it).id == s.id)
                {
                    undiscovered_ds.erase(it);
                    break;
                }
        }
    }
    
    ds_mutex.unlock();

}

void docking::compute_closest_ds()
{
//    ROS_DEBUG("Using 'closest' strategy to compute optimal DS");
    double min_dist = numeric_limits<int>::max();
    for (std::vector<ds_t>::iterator it = ds.begin(); it != ds.end(); it++)
    {
        double dist = distance_from_robot((*it).x, (*it).y, false);
        //ROS_ERROR("ds%d: %f, %f", it->id, dist, distance_from_robot((*it).x, (*it).y, true));
        if (dist < 0)
            continue; //TODO(minor) sure?
        if (dist < min_dist)
        {
            min_dist = dist;
            next_optimal_ds_id = it->id;
        }
    }
}

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

bool docking::can_update_ds() {
    return robot_state != robot_state::CHOOSING_ACTION && robot_state != robot_state::AUCTIONING && robot_state != auctioning_2 && robot_state != robot_state::GOING_CHECKING_VACANCY && robot_state != robot_state::CHECKING_VACANCY && robot_state != robot_state::CHARGING && robot_state != robot_state::GOING_IN_QUEUE && robot_state != robot_state::IN_QUEUE;
}

void docking::ds_with_EOs_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg) {
    ROS_INFO("received info on EOs for ds%d", msg.get()->id);
    for(unsigned int i=0; i<ds.size(); i++)
        if(ds.at(i).id == msg.get()->id) {
            ds.at(i).has_EOs = msg.get()->has_EOs;
            break;
        }
}

double docking::distance_from_robot(double goal_x, double goal_y, bool euclidean)
{
    return distance(robot->x, robot->y, goal_x, goal_y, euclidean);
}

double docking::distance(double start_x, double start_y, double goal_x, double goal_y, bool euclidean)
{
    /* Use euclidean distance if required by the caller */
    if (euclidean)
    {
        double dx = (goal_x - start_x);
        double dy = (goal_y - start_y);
        return sqrt(dx * dx + dy * dy);
    }

    /* Otherwise, use actual distance: ask explorer node to compute it (using its costmap) */    
    explorer::Distance srv_msg;
    srv_msg.request.x1 = start_x;
    srv_msg.request.y1 = start_y;
    srv_msg.request.x2 = goal_x;
    srv_msg.request.y2 = goal_y;
    
    //ros::service::waitForService("explorer/distance");   
    for(int i = 0; i < 10 && !sc_distance; i++) {
        ROS_FATAL("NO MORE CONNECTION!");
        ros::Duration(1).sleep();
        //sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance", true);
    }
    for (int i = 0; i < 10; i++)
        if (sc_distance.call(srv_msg) && srv_msg.response.distance >= 0) {
            return srv_msg.response.distance;
        } else
            ros::Duration(1).sleep();
    
    /* If the service is not working at the moment, return invalid value */ //TODO(minor) raise exception?
    ROS_ERROR("Unable to compute distance at the moment...");
    return -1;
}



void docking::abs_to_rel(double absolute_x, double absolute_y, double *relative_x, double *relative_y)
{
    // Use these if the /map frame origin coincides with the robot starting position in Stage (which should be the case)
    *relative_x = absolute_x - origin_absolute_x;
    *relative_y = absolute_y - origin_absolute_y;

    // Use these if the /map frame origin coincides with Stage origin
//    *relative_x = absolute_x;
//    *relative_y = absolute_y;
}

//DONE++
void docking::rel_to_abs(double relative_x, double relative_y, double *absolute_x, double *absolute_y)
{
    // Use these if the /map frame origin coincides with the robot starting position in Stage (which should be the case)
    *absolute_x = relative_x + origin_absolute_x;
    *absolute_y = relative_y + origin_absolute_y;
    
    // Use these if the /map frame origin coincides with Stage origin
//    *absolute_x = relative_x;
//    *absolute_y = relative_y;
}
