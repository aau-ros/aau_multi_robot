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



