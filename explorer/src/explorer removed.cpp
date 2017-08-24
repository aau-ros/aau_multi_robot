if (OPERATE_ON_GLOBAL_MAP == true)
        {
            costmap2d_local_size = new costmap_2d::Costmap2DROS("local_costmap", tf);
            costmap2d_local_size->pause();
            ROS_DEBUG("Starting Global costmap ...");
            costmap2d_global->start();
            costmap2d_local_size->start();

            costmap2d_local = costmap2d_global;
        }
        else
        {
            ROS_INFO("Creating local costmap ...");
            costmap2d_local = new costmap_2d::Costmap2DROS("local_costmap", tf);
            ROS_INFO("Local costmap created ... now performing costmap -> pause");
            costmap2d_local->pause();
            ROS_INFO("Pausing performed");
            ROS_INFO("Cost maps created");

            ROS_INFO("                                             ");

            ROS_INFO("Starting Global costmap ...");
            costmap2d_global->start();
            ROS_INFO("Starting Local costmap ... ");
            costmap2d_local->start();
            ROS_INFO("BOTH COSTMAPS STARTED AND RUNNING ...");
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


//        if (frontier_selection < 5 && frontier_selection != 1)
//        {
//            exploration->sort(2);

//            while (true)
//            {
//                exploration_flag = exploration->determine_goal(2, global_goal, counter, -1, robot_str);

//                if (exploration_flag == false)
//                {
//                    break;
//                }
//                else
//                {
//                    bool negotiation = true;
//                    negotiation = exploration->negotiate_Frontier(global_goal->at(0), global_goal->at(1),
//                                                                  global_goal->at(2), global_goal->at(3), -1);

//                    if (negotiation == true)
//                    {
//                        return true;
//                    }
//                    counter++;
//                }
//            }
//        }

//        else if (frontier_selection == 1 || frontier_selection == 6 || frontier_selection == 5)
//        {
//            costmap_mutex.lock();
//            exploration->clearVisitedAndSeenFrontiersFromClusters();

//            exploration->clusterFrontiers();

//            exploration->sort(4);
//            exploration->sort(5);

//            costmap_mutex.unlock();

//            cluster_element = -1;

//            while (true)
//            {
//                std::vector<double> goal_vector;
//                std::vector<std::string> robot_str_name;
//                std::vector<int> clusters_used_by_others;

//                goal_determined = exploration->determine_goal(5, &goal_vector, 0, cluster_element, robot_str);

//                if (goal_determined == false)
//                {
//                    ROS_INFO("No goal was determined, cluster is empty. Bid for another one");

//                    goal_vector.clear();
//                    bool auctioning = exploration->auctioning(&goal_vector, &clusters_used_by_others, &robot_str_name);
//                    if (auctioning == true)
//                    {
//                        goal_determined = true;
//                        cluster_element = goal_vector.at(4);

//                        global_goal->push_back(goal_vector.at(0));
//                        global_goal->push_back(goal_vector.at(1));
//                        global_goal->push_back(goal_vector.at(2));
//                        global_goal->push_back(goal_vector.at(3));
//                        global_goal->push_back(goal_vector.at(4));

//                        robot_str->push_back(robot_str_name.at(0));
//                        return true;
//                    }
//                    else
//                    {
//                        /*
//                        * If NO goals are selected at all, iterate over the global
//                        * map to find some goals.
//                        */
//                        ROS_ERROR("No goals are available at all");
//                        cluster_element = -1;
//                        break;
//                    }
//                }
//                else
//                {
//                    ROS_INFO("Still some goals in current cluster, finish this job first");
//                    break;
//                }
//            }

//            exploration->visualize_Cluster_Cells();
//        }

//        else
//        {
//            ROS_ERROR("Could not iterate over map, wrong strategy!");
//        }
