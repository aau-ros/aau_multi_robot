void selectFrontier() {
    if(moving_along_path) {
        ROS_INFO("moving along DS path");
        if(ds_path_counter < ds_path_size - 1)
        {
            ros::spinOnce();
            ROS_INFO("Trying to reach next DS");
            ROS_INFO("ds_path_counter: %d; ds_path_size: %d", ds_path_counter, ds_path_size);
            double next_ds_x = complex_path[ds_path_counter+1].x;
            double next_ds_y = complex_path[ds_path_counter+1].y;
            double dist = -1;
            ROS_INFO("Compute distance");
            for(int i=0; i<5 && dist < 0; i++) {
                dist = exploration->distance_from_robot(next_ds_x, next_ds_y); //TODO(minor) very bad way to check... -> parameter...
                if(dist<0)
                    ros::Duration(1).sleep();
            }
            ROS_INFO("Distance computed");
            
            if(dist < 0) {
                log_major_error("unable to compute distance to reach next DS!!!");
                if(retry_recharging_current_ds < 3) {
                    ROS_INFO("unable to compute distance");
                    ROS_ERROR("reauction for current one");
                    ROS_INFO("reauction for current one");
                    counter++;
                    move_robot_away(counter);
                    update_robot_state_2(robot_state::AUCTIONING);
                    retry_recharging_current_ds++;
                }
                else
                    log_major_error("finished because cannot reach next ds in path");
                    update_robot_state_2(stopped);
                    finalize_exploration();
            }

            else if( (full_battery && dist > conservative_maximum_available_distance) || (!full_battery && dist > available_distance) ) {
                if(full_battery)
                {
                    if(fabs(dist - conservative_maximum_available_distance) > 7.0)
                        log_major_error("MAJOR ERROR WITH DS GRAPH");
                    else
                        log_minor_error("minor error with ds graph");
                    ROS_DEBUG("distance to next DS: %.2f", dist);
                    ROS_DEBUG("maximum_traveling_distance: %.2f", conservative_maximum_available_distance);
                    
                    // we force to move to next ds
                    retry_recharging_current_ds = 0;
                    ds_path_counter++;
                    optimal_ds_x = complex_path[ds_path_counter].x; 
                    optimal_ds_y = complex_path[ds_path_counter].y; 
    //                                    double world_x, world_y;
    //                                    map_to_world(optimal_ds_x, optimal_ds_y, &world_x, &world_y); //TODO to be implemented
    //                                    ROS_INFO("Going to next DS in path, which is at (%f, %f)", world_x, world_y);
                    ROS_INFO("Going to next DS in path, which is at (%f, %f)", optimal_ds_x, optimal_ds_y);
                    std_msgs::Empty msg;
                    pub_next_ds.publish(msg);
                    update_robot_state_2(robot_state::GOING_CHECKING_VACANCY); //TODO(minor) maybe it should start an auction before, but in that case we must check that it is not too close to the last optimal_ds (in fact optimal_ds is the next one)
                    
                }
                else {
                    ROS_INFO("Cannot reach next DS on the path: reauction for current one");
                    counter++;
                    move_robot_away(counter);
                    update_robot_state_2(robot_state::AUCTIONING);
                }
            } else {
                retry_recharging_current_ds = 0;
                ds_path_counter++;
                optimal_ds_x = complex_path[ds_path_counter].x; 
                optimal_ds_y = complex_path[ds_path_counter].y; 
    //                                    double world_x, world_y;
    //                                    map_to_world(optimal_ds_x, optimal_ds_y, &world_x, &world_y); //TODO to be implemented
    //                                    ROS_INFO("Going to next DS in path, which is at (%f, %f)", world_x, world_y);
                ROS_INFO("Going to next DS in path, which is at (%f, %f)", optimal_ds_x, optimal_ds_y);
                std_msgs::Empty msg;
                pub_next_ds.publish(msg);
                update_robot_state_2(robot_state::GOING_CHECKING_VACANCY); //TODO(minor) maybe it should start an auction before, but in that case we must check that it is not too close to the last optimal_ds (in fact optimal_ds is the next one)
            }
            
            continue;
            
        }
        else 
        {
            ROS_INFO("Finished path traversal");
            moving_along_path = false;
            std_msgs::Empty msg;
            pub_next_ds.publish(msg);
            
            if(going_home) {
                //move_home_if_possible();
                move_home();
            } else
                continue;
            
        }
    }

    // TODO(minor) do those sorting works correclty?
    /* Sort frontiers, firstly from nearest to farthest and then by
     * efficiency */
    ROS_INFO("SORTING FRONTIERS...");

    /* Look for a frontier as goal */
    ROS_INFO("DETERMINE GOAL...");

    fs_exp_se_log.open(exploration_start_end_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs_exp_se_log << ros::Time::now() - time << ": " << "Compute goal" << std::endl;
    fs_exp_se_log.close();

    ros::spinOnce(); // to update available_distance

    if(robot_state != robot_state::CHARGING_COMPLETED)
        available_distance = next_available_distance;    

        print_mutex_info("explore()", "acquiring");
        costmap_mutex.lock();
        print_mutex_info("explore()", "lock");
        
        if(exploration->updateRobotPose()) {
            exploration->updateOptimalDs();
            goal_determined = exploration->my_determine_goal_staying_alive(1, 2, conservative_available_distance(available_distance), &final_goal, count, &robot_str, -1, battery_charge > 50, w1, w2, w3, w4);
        }
        else {
            log_major_error("robot pose not updated");
            goal_determined = false;
        }

    ROS_INFO("GOAL DETERMINED: %s; counter: %d", (goal_determined ? "yes" : "no"), count);

    fs_exp_se_log.open(exploration_start_end_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs_exp_se_log << ros::Time::now() - time << ": " << "Finished" << std::endl;
    fs_exp_se_log.close();

    fs_computation_time.open(computation_time_log.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs_computation_time << exploration->frontier_selected << "," << exploration->number_of_frontiers << "," << exploration->sort_time << "," << exploration->selection_time << std::endl;
    fs_computation_time.close();

    ros::Duration d = ros::Time::now() - time_2;
    if(d > ros::Duration(5 * 60)) {
        log_minor_error("very slow...");
    }

    /* Check if the robot has found a reachable frontier */
    if (goal_determined == true)
    {
        //this can be true if we are at the end of the DS path (i.e., we have reached the last DS and we have 
    //                            if(moving_along_path) {
    //                                ROS_INFO("Finished path traversal");
    //                                moving_along_path = false;
    //                                std_msgs::Empty msg;
    //                                pub_next_ds.publish(msg);
    //                            }

        /* The robot has found a reachable frontier: it can move toward it */                            
        update_robot_state_2(robot_state::MOVING_TO_FRONTIER);
        retries3 = 0;
        retries5 = 0;
        
        // TODO(minor) ...
        if (exit_countdown != EXIT_COUNTDOWN)
        {
            exit_countdown = EXIT_COUNTDOWN;
            ROS_ERROR("To be safe, resetting exit countdown at starting "
                      "value; something must have gone wrong however, "
                      "because this shoulw never happen...");
        }
        
        ROS_DEBUG("reset retries counter");
        retries = 0;
    }

    else
    {

    //                            if(exploration->recomputeGoal() && retries < 7) { //TODO(IMPORTANT)
    //                                ROS_ERROR("Goal not found due to some computation failure, trying to recompute goal...");
    //                                ROS_INFO("Goal not found due to some computation failure, trying to recompute goal...");
    ////                                ROS_ERROR("Goal not found due to some computation failure, start auction...");
    ////                                ROS_INFO("Goal not found due to some computation failure, start auction...");
    //                                retries++;
    //                                ROS_DEBUG("%d", retries);
    //                                
    //                                ros::Duration(3).sleep();
    //                                costmap_mutex.unlock();
    //                                print_mutex_info("explore()", "unlock");
    ////                                update_robot_state_2(robot_state::AUCTIONING);
    //                                continue;                             
    //                            }
        ROS_INFO("goal not determined: selecting next action...");
        
        if(!optima_ds_set) {
            ROS_ERROR("no optimal ds: forcing to go in queue..."); //done by energy_mgmt;
            update_robot_state_2(robot_state::AUCTIONING);
        }
        else {
            
    //                            if(moving_along_path) {
    //                                ROS_INFO("moving_along_path: reauctiong for current DS (the last of the path)");
    //                                counter++;
    //                                move_robot_away(counter);
    //                                update_robot_state_2(robot_state::AUCTIONING);
    //                                std_msgs::Empty msg;
    //                                pub_next_ds.publish(msg);
    //                            }
            
            if(retries >= 4)
                log_major_error("too many retries, this shouldn't happend");
            
            //if the robot is not fully charged, recharge it, since the checks to detect if there is a reachable frontier can be very computational expensive
    //                            if(robot_state != robot_state::CHARGING_COMPLETED) {
    //                                update_robot_state_2(robot_state::AUCTIONING);
    //                                costmap_mutex.unlock();
    //                                print_mutex_info("explore()", "unlock");
    //                                continue;
    //                            } 
            
            //TODO we could think aabout moving this part in docking.cpp.. but then we have to be sure that explorer won't continue to think that there are reachable frontiers in another part of the enviroment while energy_mgmt will publish the path for another
    //                            if(retries2 < 4 && retries3 < 5 && retries5 < 10) {
            if(retries2 < 1 && retries3 < 5 && retries5 < 10) {
    //                                retries5++;
                bool error = false;
                ros::spinOnce(); //to udpate available_distance
                if( !exploration->existFrontiers() ) {
                    ROS_INFO("No more frontiers: moving home...");
                    move_home_if_possible();
                
                //TODO use 0.99 as coefficient?
                } 
                else
                {
                    if(exploration->discovered_new_frontier)
                        retries6 = 0;
                    else
                        retries6++;
                    ROS_INFO("retries6: %d", retries6);
                    if(retries6 >= 3) {
                        log_major_error("tried too many times to navigate graph: retries6 >= 3");
                        move_home_if_possible();
                    }
                    else
                    {
                        exploration->discovered_new_frontier = false;
                        exploration->updateOptimalDs();
                        if( exploration->existFrontiersReachableWithFullBattery(conservative_maximum_available_distance, &error) ) {
                            ROS_INFO("There are still frontiers that can be reached from the current DS: start auction for this DS...");
                            counter++;
                            move_robot_away(counter);
                            update_robot_state_2(robot_state::AUCTIONING);
                            retries4 = 0;
                        }
                        else {
                        
                            update_robot_state_2(exploring_for_graph_navigation);
                            ros::Duration(1).sleep();
                        
                            if( ds_graph_navigation_allowed && exploration->existReachableFrontiersWithDsGraphNavigation(0.999*conservative_maximum_available_distance, &error) ) {
                                ROS_INFO("There are frontiers that can be reached from other DSs: start moving along DS graph...");
                                
                                int result = -1;
                                exploration->compute_and_publish_ds_path(conservative_maximum_available_distance, &result);
                                if(result == 0) //TODO very very orrible idea, using result...
                                {
                                    ROS_INFO("path successfully found");
                                    
                                    counter++;
                                    move_robot_away(counter);
                                    update_robot_state_2(auctioning_2);
                                    retries2 = 0;
                                    retries4 = 0;
                                }
                                else {
                                    retries2++;
                                    update_robot_state_2(auctioning_2);
                                    if(result == 1)
                                        log_major_error("No DS with EOs was found");
                                    else if(result == 2)
                                        log_major_error("impossible, no closest ds found...");
                                    else if(result == 3) {
                                        log_minor_error("closest_ds->id == min_ds->id, this should not happen...");
                                        counter++;
                                        move_robot_away(counter);
                                        update_robot_state_2(auctioning_2);
                                        retries3++;
                                    }
                                    else
                                        log_major_error("invalid result value");
                                }
                            }
                            else {
                                ROS_DEBUG("errors: %s", (error ? "yes" : "no") );
                                if(error) {
                                    ROS_ERROR("Failure in checking if reachable frontiers still exists: retrying...");
                                    ROS_INFO("Failure in checking if reachable frontiers still exists: retrying...");
                                    ros::Duration(3).sleep();
                                    retries2++;
                                    costmap_mutex.unlock();
                                    print_mutex_info("explore()", "unlock");
                                    update_robot_state_2(robot_state::CHOOSING_ACTION);
                                    continue;
                                }
                                else {
                                    retries4++;
                                    if(retries4 < 3) {
                                        ROS_INFO("retrying to search if one of the remaining frontiers is reachable");
                                        costmap_mutex.unlock();
                                        print_mutex_info("explore()", "unlock");
                                        update_robot_state_2(robot_state::CHOOSING_ACTION);
                                        continue;
                                    }
                                    else
                                    {
                                        log_minor_error("There are still unvisited frontiers, but the robot cannot reach them even with full battery: exploration can be concluded; robot will go home...");
                                        move_home_if_possible();
                                    }
                                }
                            }
                        }
                     }
                 }
            }
            else {
                if(retries2 >= 4)
                    log_major_error("tried too many times to navigate graph: retries2 >= 4");
                else if(retries3 >=5)
                    log_major_error("tried too many times to navigate graph: retries3 >= 5");
                else
                    log_major_error("tried too many times to navigate graph: retries5 >= 10");
                move_home_if_possible();
            }
        }
    }

    costmap_mutex.unlock();
    print_mutex_info("explore()", "unlock");
}


 /*
//                    * Use mutex to lock the critical section (access to the costmap)
//                    * since rosspin tries to update the costmap continuously
//                    */
//                    ROS_DEBUG("COSTMAP STUFF");
//                    print_mutex_info("explore()", "acquiring");
//                    costmap_mutex.lock();
//                    ROS_DEBUG("COSTMAP STUFF, lock aquired");
//                    print_mutex_info("explore()", "lock");

//                    exploration->transformToOwnCoordinates_frontiers();
//                    exploration->transformToOwnCoordinates_visited_frontiers();

//                    exploration->initialize_planner("exploration planner", costmap2d_local, costmap2d_global, NULL);
//                        
//                    exploration->transformToOwnCoordinates_frontiers();
//                    exploration->transformToOwnCoordinates_visited_frontiers();
//                      
//                    exploration->findFrontiers();
//                    exploration->clearVisitedFrontiers();
//                    exploration->clearUnreachableFrontiers();
//                    exploration->clearSeenFrontiers(costmap2d_global);

//                    if(skip_findFrontiers)
//                        exploration->clearVisitedFrontiers();
//                    exploration->clearUnreachableFrontiers(); //should remove frontiers that are marked as unreachable from 'frontiers' vector
//                    if(skip_findFrontiers)
//                        exploration->clearSeenFrontiers(costmap2d_global);

//                    costmap_mutex.unlock();
//                    print_mutex_info("explore()", "unlock");
//                    ROS_DEBUG("COSTMAP STUFF, lock released");





