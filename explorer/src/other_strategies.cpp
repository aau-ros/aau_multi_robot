                /*********** EXPLORATION STRATEGIES ************
                * 0 ... Navigate to nearest frontier TRAVEL PATH
                * 1 ... Navigate using auctioning with cluster selection using NEAREST
                *       selection (Kuhn-Munkres)
                * 2 ... Navigate to furthest frontier
                * 3 ... Navigate to nearest frontier EUCLIDEAN DISTANCE
                * 4 ... Navigate to random frontier
                * 5 ... Cluster frontiers, then navigate to nearest cluster using
                *       EUCLIDEAN DISTANCE (with and without negotiation)
                * 6 ... Cluster frontiers, then navigate to random cluster (with and
                *       without negotiation)
                *
                * ENERGY AWARE STRATEGIES:
                * 7 ... Navigate to frontier that satisfies energy efficient cost
                *       function with staying-alive path planning
                * 8 ... Navigate to leftmost frontier (Mei et al. 2006) with
                *       staying-alive path planning
                * 9 ... Just like strategy 0 but with staying-alive path planning
                */

                /*************** SORTING METHODS ***************
                * Choose which strategy to take.
                * 1 ... Sort the buffer from furthest to nearest frontier
                * 2 ... Sort the buffer from nearest to furthest frontier, normalized to
                *       the robots actual position (EUCLIDEAN DISTANCE)
                * 3 ... Sort the last 10 entries to shortest TRAVEL PATH
                * 4 ... Sort all cluster elements from nearest to furthest (EUCLIDEAN
                *       DISTANCE)
                * 5 ... (missing description)
                * 6 ... (missing description)
                * 7 ... Sort frontiers in sensor range clock wise (starting from left of
                *       robot) and sort the remaining frontiers from nearest to furthest
                *       (EUCLIDEAN DISTANCE)
                */

                /* Compute next frontier to be explored according to the selected
                 * exploration strategy */

                if (frontier_selection == 0)
                {
                    exploration->sort(2);
                    exploration->sort(3);

                    while (true)
                    {
                        goal_determined = exploration->determine_goal(2, &final_goal, count, 0, &robot_str);
                        ROS_DEBUG("Goal_determined: %d   counter: %d", goal_determined, count);
                        if (goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            negotiation = true;
                            if (negotiation == true)
                            {
                                break;
                            }
                            count++;
                        }
                    }
                }
                else if (frontier_selection == 1)
                {
                    costmap_mutex.lock();
                    if (cluster_initialize_flag == true)
                    {
                        exploration->clearVisitedAndSeenFrontiersFromClusters();
                    }
                    else
                    {
                        /*
                        * This is only necessary for the first run
                        */

                        exploration->sort(2);
                        cluster_initialize_flag = true;
                    }

                    exploration->clusterFrontiers();
                    exploration->sort(4);
                    exploration->sort(5);

                    costmap_mutex.unlock();

                    exploration->visualizeClustersConsole();

                    while (true)
                    {
                        final_goal.clear();
                        robot_str.clear();
                        goal_determined = exploration->determine_goal(5, &final_goal, 0, cluster_element, &robot_str);

                        ROS_ERROR("Cluster element: %d", cluster_element);

                        int cluster_vector_position = -1;

                        if (cluster_element != -1)
                        {
                            if (exploration->clusters.size() > 0)
                            {
                                for (unsigned int i = 0; i < exploration->clusters.size(); i++)
                                {
                                    if (exploration->clusters.at(i).id == cluster_element)
                                    {
                                        if (exploration->clusters.at(i).cluster_element.size() > 0)
                                        {
                                            cluster_vector_position = i;
                                        }
                                        break;
                                    }
                                }
                            }
                        }
                        ROS_ERROR("Cluster vector position: %d", cluster_vector_position);

                        if (cluster_vector_position >= 0)
                        {
                            if (exploration->clusters.at(cluster_vector_position).unreachable_frontier_count >=
                                number_unreachable_frontiers_for_cluster)
                            {
                                goal_determined = false;
                                ROS_ERROR("Cluster inoperateable");
                            }
                            else
                            {
                                ROS_ERROR("Cluster operateable");
                            }
                        }

                        if (goal_determined == false)
                        {
                            ROS_INFO("No goal was determined, cluster is empty. Bid for "
                                     "another one");

                            final_goal.clear();
                            robot_str.clear();
                            clusters_available_in_pool.clear();

                            bool auctioning =
                                exploration->auctioning(&final_goal, &clusters_available_in_pool, &robot_str);
                            if (auctioning == true)
                            {
                                goal_determined = true;
                                cluster_element = final_goal.at(4);
                                counter_waiting_for_clusters = 0;
                                break;
                            }
                            else
                            {
                                if (exploration->clusters.size() > 0)  // clusters_available_in_pool.size() > 0)
                                {
                                    ROS_INFO("No cluster was selected but other robots are "
                                             "operating ... waiting for new clusters");
                                    counter_waiting_for_clusters++;
                                    break;
                                }
                                else
                                {
                                    /*
                                    * If NO goals are selected at all, iterate over the global
                                    * map to find some goals.
                                    */
                                    ROS_ERROR("No goals are available at all");
                                    cluster_element = -1;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            ROS_INFO("Still some goals in current cluster, finish this job first");
                            break;
                        }
                    }
                }
                else if (frontier_selection == 2)
                {
                    exploration->sort(1);

                    /*
                    * Choose which strategy to take.
                    * 1 ... least to first goal in list
                    * 2 ... first to last goal in list
                    *
                    * Determine_goal() takes world coordinates stored in
                    * the frontiers list(which contain every found frontier with
                    *sufficient
                    * spacing in between) and returns the most attractive one, based on
                    *the
                    * above defined strategies.
                    */
                    while (true)
                    {
                        goal_determined = exploration->determine_goal(1, &final_goal, count, 0, &robot_str);
                        if (goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            negotiation = true;
                            if (negotiation == true)
                            {
                                break;
                            }
                            count++;
                        }
                    }
                }
                else if (frontier_selection == 3)
                {
                    exploration->sort(2);

                    while (true)
                    {
                        goal_determined = exploration->determine_goal(2, &final_goal, count, 0, &robot_str);
                        if (goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            negotiation = true;
                            if (negotiation == true)
                            {
                                break;
                            }
                            count++;
                        }
                    }
                }
                else if (frontier_selection == 4)
                {
                    while (true)
                    {
                        goal_determined = exploration->determine_goal(3, &final_goal, count, 0, &robot_str);
                        ROS_DEBUG("Goal_determined: %d   counter: %d", goal_determined, count);
                        if (goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            negotiation = true;
                            if (negotiation == true)
                            {
                                break;
                            }
                            count++;
                        }
                    }
                }
                else if (frontier_selection == 5)
                {
                    if (cluster_initialize_flag == true)
                    {
                        exploration->clearVisitedAndSeenFrontiersFromClusters();
                    }
                    else
                    {
                        /*
                        * This is necessary for the first run
                        */
                        if (robot_id == 0)
                            ros::Duration(10).sleep();

                        exploration->sort(2);
                    }

                    exploration->clusterFrontiers();
                    exploration->sort(4);
                    exploration->sort(5);

                    clusters_available_in_pool.clear();
                    while (true)
                    {
                        final_goal.clear();
                        goal_determined =
                            exploration->determine_goal(4, &final_goal, count, cluster_element, &robot_str);

                        if (goal_determined == false)
                        {
                            ROS_INFO("Another cluster is not available, no cluster determined");
                            break;
                        }
                        else
                        {
                            if (cluster_initialize_flag == false)
                            {
                                /*
                                *  todo ... just to reduce the selection of the same
                                * clusters since no auctioning is implemented jet.
                                */
                                if (robot_id == 1)
                                {
                                    ros::Duration(5).sleep();
                                }

                                cluster_initialize_flag = true;
                            }

                            /*
                            * If negotiation is not needed, simply uncomment
                            * and set the negotiation to TRUE.
                            */
                            negotiation =
                                exploration->negotiate_Frontier(final_goal.at(0), final_goal.at(1), final_goal.at(2),
                                                                final_goal.at(3), final_goal.at(4));
                            if (negotiation == true)
                            {
                                ROS_DEBUG("Negotiation was successful");
                                cluster_element = final_goal.at(4);
                                counter_waiting_for_clusters = 0;
                                break;
                            }
                            else
                            {
                                cluster_element = final_goal.at(4);
                                counter_waiting_for_clusters++;
                                ROS_ERROR("Negotiation was not successful, try next cluster");
                            }
                            count++;
                            /*
                            * In order to make one robot wait until the other finds
                            * another cluster to operate in, one have to fill the
                            * clusters_available_in_pool
                            * vector if count is increased at least once which determines
                            * that at least one cluster
                            */
                            clusters_available_in_pool.push_back(1);
                        }
                    }
                }
                else if (frontier_selection == 6)
                {
                    if (cluster_initialize_flag == true)
                    {
                        exploration->clearVisitedAndSeenFrontiersFromClusters();
                    }

                    exploration->clusterFrontiers();
                    exploration->sort(6);
                    exploration->visualizeClustersConsole();

                    if (cluster_initialize_flag == false)
                    {
                        ros::Duration(2).sleep();
                    }
                    cluster_initialize_flag = true;

                    while (true)
                    {
                        goal_determined =
                            exploration->determine_goal(4, &final_goal, count, cluster_element, &robot_str);

                        if (goal_determined == false)
                        {
                            break;
                        }
                        else
                        {
                            /*
                            * If negotiation is not needed, simply uncomment
                            * and set the negotiation to TRUE.
                            */
                            negotiation =
                                exploration->negotiate_Frontier(final_goal.at(0), final_goal.at(1), final_goal.at(2),
                                                                final_goal.at(3), final_goal.at(4));

                            if (negotiation == true)
                            {
                                cluster_element = final_goal.at(4);
                                break;
                            }
                            else
                            {
                                cluster_element = int(exploration->clusters.size() * rand() / (RAND_MAX));
                                ROS_INFO("Random cluster_element: %d  from available %lu clusters", cluster_element,
                                         (long unsigned int)exploration->clusters.size());
                            }
                            count++;
                        }
                    }
                }

else if (frontier_selection == 8)
                {
                    // sort frontiers clock wise starting from left
                    exploration->sort(7);

                    // look for a frontier as goal
                    ROS_INFO("DETERMINE GOAL...");
                    goal_determined = exploration->determine_goal_staying_alive(1, 2, available_distance, &final_goal,
                                                                                count, &robot_str, -1);
                    ROS_INFO("Goal_determined: %d   counter: %d", goal_determined, count);

                    // found a frontier, go there
                    if (goal_determined == true)
                    {
                        robot_state = robot_state::COMPUTING_NEXT_GOAL;
                        exit_countdown = EXIT_COUNTDOWN;
                        charge_countdown = EXIT_COUNTDOWN;
                    }

                    // robot cannot reach any frontier, even if fully charged
                    // simulation is over
                    else if (recharging == false || robot_state == robot_state::CHARGING_COMPLETED)
                    {
                        exit_countdown--;
                        ROS_ERROR("Shutdown in: %d", exit_countdown);
                        if (exit_countdown <= 0)
                            finalize_exploration();
                        continue;
                    }

                    // robot cannot reach any frontier
                    // go charging
                    else
                    {
                        charge_countdown--;
                        if (charge_countdown <= 0)
                        {
                            ROS_INFO("Could not determine goal, need to recharge!");
                            robot_state = robot_state::GOING_CHARGING;
                        }
                        else
                            continue;
                    }
                }
                else if (frontier_selection == 9)
                {
                    // sort the frontiers from near to far
                    exploration->sort(2);
                    exploration->sort(3);

                    // look for a frontier as goal
                    goal_determined = exploration->determine_goal_staying_alive(1, 2, available_distance, &final_goal,
                                                                                count, &robot_str, -1);
                    ROS_DEBUG("Goal_determined: %d   counter: %d", goal_determined, count);

                    // found a frontier
                    // go there
                    if (goal_determined == true)
                    {
                        robot_state = robot_state::COMPUTING_NEXT_GOAL;
                        exit_countdown = EXIT_COUNTDOWN;
                        charge_countdown = EXIT_COUNTDOWN;
                    }

                    // robot cannot reach any frontier, even if fully charged
                    // simulation is over
                    else if (recharging == false || robot_state == robot_state::CHARGING_COMPLETED)
                    {
                        exit_countdown--;
                        ROS_ERROR("Shutdown in: %d", exit_countdown);
                        if (exit_countdown <= 0)
                            finalize_exploration();
                        continue;
                    }

                    // robot cannot reach any frontier
                    // go charging
                    else
                    {
                        charge_countdown--;
                        if (charge_countdown <= 0)
                        {
                            ROS_INFO("Could not determine goal, need to recharge!");
                            robot_state = robot_state::GOING_CHARGING;
                        }
                        else
                            continue;
                    }
                }
