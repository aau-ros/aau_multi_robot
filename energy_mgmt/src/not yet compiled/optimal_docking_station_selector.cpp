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

void docking::computeOptimalDs() //TODO(minor) best waw to handle errors in distance computation??
//TODO(minor) maybe there are more efficient for some steps; for instance, i coudl keep track somewhere of the DS with EOs and avoid recomputing them two times in the same call of this funcion...
{   
    ROS_INFO("Compute optimal DS");
    {
        jobs_mutex.lock();
        std::vector<adhoc_communication::ExpFrontierElement> jobs_local_list;
        jobs_local_list = jobs; //TODO we should do the same also when computing the parameters... although they are updated by a callback so it should be ok, but just to be safe...
        jobs_mutex.unlock();

        if (ds_selection_policy == 0) // TODO(minor) switch-case
            closestPolicy()();

        else if (ds_selection_policy == 1)
            vacantPolicy();

        else if (ds_selection_policy == 2)
            opportuneOptimalDs();

        else if (ds_selection_policy == 3)
            currentOptimalDs();

        /* "Flocking" policy */
        else if (ds_selection_policy == 4)
            flockingOptimalDs();
     
        if (optimal_docking_station.id != next_optimal_ds_id)
        {
            ROS_INFO("Changing optimal DS");
            old_optimal_ds_id = next_optimal_ds_id;
            docking_station_manager.set_optimal_ds(next_optimal_ds_id);
            
            /* Notify explorer about the optimal DS change */ //TODO use service to be sure
            adhoc_communication::EmDockingStation msg_optimal;
            msg_optimal.id = get_optimal_ds_id();
            msg_optimal.x = get_optimal_ds_x();
            msg_optimal.y = get_optimal_ds_y();
            pub_new_optimal_ds.publish(msg_optimal);
        }
        else
            ROS_INFO("Optimal DS unchanged");
                  
    ROS_INFO("end compute_optimal_ds()");          
}

void docking::closestPolicy()
{
    double min_dist = numeric_limits<int>::max();
    for (auto it = ds.begin(); it != ds.end(); it++)
    {
        double dist = distance_from_robot(it->x, it->y, false);
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

void OptimalDockingStationSelector::setDockingStationManager() {

}

void vacantPolicy() {
    /* Check if there are vacant DSs that are currently reachable. If there are, check also which one is the closest to the robot */
    bool found_reachable_vacant_ds = false, found_vacant_ds = false;
    double min_dist = numeric_limits<int>::max();
    for (auto it = ds.begin(); it != ds.end(); it++)
    {
        if (it->vacant)
        {
            ROS_DEBUG("ds%d is currently vacant", it->id);
            found_vacant_ds = true;
            
            double dist = distance_from_robot((*it).x, (*it).y);
            if (dist < 0)
                continue;
            if(dist < conservative_remaining_distance_one_way() ) {
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
            ROS_DEBUG("ds%d is currently occupied", it->id);
    }

    /* If no DS is vacant at the moment, use "closest" policy to update the
     * optimal DS */
    if (found_reachable_vacant_ds)
        ROS_DEBUG("'vacant' policy found an optimal DS");
    else
    {
        if(found_vacant_ds)
            ROS_DEBUG("All the vacant DSs are currently unreachable: fall back to 'closest' policy");
        else
            ROS_DEBUG("No DS is currently vacant: fall back to 'closest' policy");
        closestPolicy()();
    }
        
}

void opprtuneOptimalDs() {
    if (!moving_along_path) //TODO reduntant
    {
        /* Check if there are reachable DSs (i.e., DSs that the robot can reach with the remaining battery life) with EOs */
        double min_dist = numeric_limits<int>::max();
        bool found_reachable_ds_with_eo = false, found_ds_with_eo = false;
        for (unsigned int i = 0; i < ds.size(); i++) {
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
                            closestPolicy()();
                    }
                }
                
            }
            else 
			    if(no_jobs_received_yet)
                    closestPolicy()();  // TODO(minor) although probably the robot will think
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

void currentOptimalDs()
{
    /* If no optimal DS has been selected yet, use "closest" policy, otherwise use "current" policy */ //TODO(minor) assumption
    if (!optimal_ds_is_set()) {
        ROS_DEBUG("No optimal docking station selected yet: fall back to 'closest' policy");
        closestPolicy()();
    }
    else
    {
        bool existing_eo;
        for(unsigned int i=0; i < ds.size(); i++)
            if(ds.at(i).id == get_optimal_ds_id()) {
                existing_eo = ds.at(i).has_EOs;
                break;                    
            }
        if (existing_eo)
            ROS_DEBUG("Current optimal DS has still EOs: keep using it");
        else {
            ROS_DEBUG("Current optimal DS has no more EOs: use 'closest' policy to compute new optimal DS");
            closestPolicy()();
        }
    }
}

void flockingOptimalDs() {
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
