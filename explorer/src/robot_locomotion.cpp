void move_home_if_possible() {
        ros::spinOnce();
        if(exploration->home_is_reachable(available_distance)) {
            bool completed_navigation = false;
            for (int i = 0; i < 5; i++)
            {
                if (completed_navigation == false)
                {
                    counter++;
                    going_home = true;
                    completed_navigation = move_robot(counter, home_point_x, home_point_y);
                    going_home = false;
                }
                else
                {
                    break;
                }
            }
            finalize_exploration();
        }
        else {
            going_home = true;
            counter++;
            move_robot_away(counter);
            update_robot_state_2(auctioning_3);
        }
    }
    
   void move_home() {
        ros::spinOnce();
        if(exploration->home_is_reachable(available_distance)) {
            bool completed_navigation = false;
            for (int i = 0; i < 5; i++)
            {
                if (completed_navigation == false)
                {
                    counter++;
                    going_home = true;
                    completed_navigation = move_robot(counter, home_point_x, home_point_y);
                    going_home = false;
                }
                else
                {
                    break;
                }
            }
            finalize_exploration();
        }
        else {
            log_major_error("robot has finished the exploration but cannot reach home!");
            finalize_exploration();
        }
    }


void move_robot_away_before_auctioning() {
    ROS_FATAL("MISSING");
//                double distance = -1;
//                int i = 0;
//                while(distance < 0 && i < 10) {
//                    exploration->distance_from_robot(optimal_ds_x, optimal_ds_y);
//                    i++;
//                    ros::Duration(2).sleep();
//                }
//                if(distance < 0)
//                    ROS_INFO("cannot comptue distance between robot and DS: leaving robot where it is");
//                else
//                    if (distance <
//                            min_distance_queue_ds)  // TODO could the DS change meanwhile???
//                            {
//                                ROS_INFO("ROBOT TOO CLOSE TO DS to start an auction: moving a little bit farther...");
//                                ROS_DEBUG("distance: %.2f; min_distance_queue_ds: %.2f", distance, min_distance_queue_ds);
//                                update_robot_state_2(moving_away_from_ds);
//                                fs_csv_state.open(csv_state_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//                                fs_csv_state << time << "," << "moving_away_from_ds" << std::endl; //TODO make real state
//                                fs_csv_state.close();
//                                move_robot_away();  // TODO(minor) move robot away also if in queue and too close...
//                                ROS_INFO("NOW it is ok...");
//                            }
                
}
