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


bool move_robot(int seq, double position_x, double position_y)
{
    ROS_INFO("Preparing to move toward goal (%.1f, %.1f)...", position_x, position_y);

    exploration->next_auction_position_x = position_x;
    exploration->next_auction_position_y = position_y;
    ros::Duration my_stuck_countdown = ros::Duration( (TIMEOUT_CHECK_1 - 2) * 60);
    ros::Duration my_fallback_countdown = ros::Duration(30);
    bool timer_started = false;
    ros::Time start_time_fallback;

    /* Move the robot with the help of an action client. Goal positions are transmitted to the robot and feedback is
     * given about the actual driving state of the robot. */
    //if (!costmap2d_local->getRobotPose(robotPose))
    //{
    //    ROS_ERROR("Failed to get RobotPose");  // TODO(minor) so what???
    //}

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(10.0)))
        ;

    move_base_msgs::MoveBaseGoal goal_msgs;

    goal_msgs.target_pose.header.seq = seq;                   // increase the sequence number
    goal_msgs.target_pose.header.frame_id = move_base_frame;  //"map";
    goal_msgs.target_pose.pose.position.x = position_x;
    goal_msgs.target_pose.pose.position.y = position_y;
    goal_msgs.target_pose.pose.position.z = 0;
    goal_msgs.target_pose.pose.orientation.x = 0;
    goal_msgs.target_pose.pose.orientation.y = 0;
    goal_msgs.target_pose.pose.orientation.z = 0;
    goal_msgs.target_pose.pose.orientation.w = 1;

    /* Get distance from goal */
    double remaining_distance = exploration->distance_from_robot(position_x, position_y);

    /* If the robot is moving toward a DS, check if it is already close to the DS: if it is, do not move it */
    if (remaining_distance > 0 && remaining_distance < queue_distance && (robot_state == robot_state::GOING_IN_QUEUE || robot_state == robot_state::GOING_CHECKING_VACANCY) )
    {
        //ROS_ERROR("\n\t\e[1;34mSTOP!! let's wait...\e[0m");
        //exploration->next_auction_position_x = robotPose.getOrigin().getX();
        //exploration->next_auction_position_y = robotPose.getOrigin().getY();
        return true;
    }

    /* Start moving */
    ROS_DEBUG("Setting goal...");
    ac.sendGoal(goal_msgs);

    /* Wait until the goal is set */
    ac.waitForResult(ros::Duration(waitForResult));  // TODO(minor) necessary?
    while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
    {
        ros::Duration(0.5).sleep();
    }
    ROS_DEBUG("Goal correctly set");
    ROS_DEBUG("Moving toward goal...");

    ros::Time time_before = ros::Time::now();
    prev_pose_x = pose_x;
    prev_pose_y = pose_y;
    while (robot_is_moving() && ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
    {
        // robot seems to be stuck
        if ( fabs(prev_pose_x - pose_x) < 0.1 && fabs(prev_pose_y - pose_y) < 0.1 && fabs(prev_pose_angle - pose_angle) < 0.1 )  // TODO(minor) ...
        {
            //stuck_countdown--; //TODO(minor)
            // if(stuck_countdown <= 5){

            // TODO(minor) if STUCK_COUNTDOWN is too low, even when the robot is
            // computing the frontier, it is believed to be stucked...
//                if (stuck_countdown <= 10)
//                {
//                    ROS_ERROR("Robot is not moving anymore, shutdown in: %d", stuck_countdown);
//                }

            if (my_stuck_countdown <= ros::Duration(0))
            {
                store_travelled_distance();

                ac.cancelGoal();
                exploration->next_auction_position_x = robotPose.getOrigin().getX();
                exploration->next_auction_position_y = robotPose.getOrigin().getY();
                approximate_success++;
            
                if( fabs(position_x - pose_x) < 1 && fabs(position_y - pose_y) < 1 ) {
                    ROS_ERROR("robot seems unable to received ACK from actionlib even if the goal have been reached");
                    ROS_INFO("robot seems unable to received ACK from actionlib even if the goal have been reached");
                    return true;
                } else
                    return false;
            }
            
            my_stuck_countdown -= ros::Time::now() - time_before;
            ROS_DEBUG("%.1f", my_stuck_countdown.toSec());
        }
        else
        {
            //ROS_ERROR("(%f, %f; %f) : (%f, %f; %f)", prev_pose_x, prev_pose_y, prev_pose_angle, pose_x, pose_y, pose_angle);
//                stuck_countdown = STUCK_COUNTDOWN;  // robot is moving again
            prev_pose_x = pose_x;
            prev_pose_y = pose_y;
            prev_pose_angle = pose_angle;
        }

        if(robot_state == robot_state::GOING_CHECKING_VACANCY || robot_state == robot_state::GOING_IN_QUEUE || robot_state == robot_state::GOING_CHARGING) {
            remaining_distance = exploration->distance_from_robot(position_x, position_y);

            /* Print remaining distance to be travelled to reach goal if the goal is a DS */
            if (robot_state == robot_state::GOING_CHECKING_VACANCY || robot_state == robot_state::GOING_IN_QUEUE)
                ROS_DEBUG("Remaining distance: %.3f\e[0m", remaining_distance);

            /* If the robot is approaching a DS to queue or to check if it is free, stop it when it is close enough to
             * the DS */
            if ((robot_state == robot_state::GOING_CHECKING_VACANCY || robot_state == robot_state::GOING_IN_QUEUE) && remaining_distance > 0 && remaining_distance < queue_distance)
            {
                ac.cancelGoal();
                exploration->next_auction_position_x = robotPose.getOrigin().getX();
                exploration->next_auction_position_y = robotPose.getOrigin().getY();
                
                store_travelled_distance();
                
                return true;
            }
            if( (robot_state == robot_state::GOING_CHARGING && remaining_distance < 3.0) || (robot_state == robot_state::GOING_IN_QUEUE && remaining_distance < 6.0) ) {
                if(!timer_started) {
                    timer_started = true;
                    start_time_fallback = ros::Time::now();
                }
                my_fallback_countdown -= ros::Time::now() - time_before;
                if(my_fallback_countdown <= ros::Duration(0)) {
                    ac.cancelGoal();
                    exploration->next_auction_position_x = robotPose.getOrigin().getX();
                    exploration->next_auction_position_y = robotPose.getOrigin().getY();
                    log_minor_error("consider that robot reached the DS");
                    
                    store_travelled_distance();
                    return true;
                }
            }
        }
        
        time_before = ros::Time::now();
        ros::Duration(1).sleep(); //TODO(minor)
    }

    while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if(!robot_is_moving()) {
            log_minor_error("robot was forced to stop from another thread");
            ac.cancelGoal();
            store_travelled_distance();
            exploration->next_auction_position_x = robotPose.getOrigin().getX();
            exploration->next_auction_position_y = robotPose.getOrigin().getY();
            return true;
        }
        
        if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            
            store_travelled_distance();
            exploration->next_auction_position_x = robotPose.getOrigin().getX();
            exploration->next_auction_position_y = robotPose.getOrigin().getY();
            
            if( (position_x - pose_x) * (position_x - pose_x) + (position_y - pose_y) * (position_y - pose_y) < 5*5 ) {
                  ROS_ERROR("Robot seems unable to closely reach the goal, but it is close enough to consider the goal reached... ");
                  if(moving_to_ds || going_home) 
                    log_minor_error("Robot didn't properly reach home/DS");
                    return true;
            }
            else  {
                ROS_INFO("ABORTED: goal not reached (robot is farther than 3 meters from goal)");
                return false;
            }                

        }
    }

    ROS_INFO("Goal reached");

    exploration->next_auction_position_x = robotPose.getOrigin().getX();
    exploration->next_auction_position_y = robotPose.getOrigin().getY();

    approximate_success = 0;
    store_travelled_distance();
    return true;
}
    
    bool move_robot_away(int seq)
    {
        ROS_INFO("Preparing to move away...");

        /* Move the robot with the help of an action client. Goal positions are transmitted to the robot and feedback is
         * given about the actual driving state of the robot. */
        //if (!costmap2d_local->getRobotPose(robotPose))
        //{
        //    ROS_ERROR("Failed to get RobotPose");  // TODO(minor) so what???
        //}
        
        double remaining_distance = exploration->distance_from_robot(optimal_ds_x, optimal_ds_y);
        
        if(remaining_distance > queue_distance / 2.0 || exploration->distance_from_robot(home_point_x, home_point_y) < 1 ) {
        
            ROS_INFO("alreayd away from DS"); // althought this could be false... in the sense that maybe the movement was simply aborted

            exploration->next_auction_position_x = robotPose.getOrigin().getX();
            exploration->next_auction_position_y = robotPose.getOrigin().getY();
//            store_travelled_distance();
            return true;
        }
       
        

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

        while (!ac.waitForServer(ros::Duration(10.0)))
            ;

        move_base_msgs::MoveBaseGoal goal_msgs;

        goal_msgs.target_pose.header.seq = seq;                   // increase the sequence number
        goal_msgs.target_pose.header.frame_id = move_base_frame;  //"map";
        goal_msgs.target_pose.pose.position.x = home_point_x;
        goal_msgs.target_pose.pose.position.y = home_point_y;
        goal_msgs.target_pose.pose.position.z = 0;
        goal_msgs.target_pose.pose.orientation.x = 0;
        goal_msgs.target_pose.pose.orientation.y = 0;
        goal_msgs.target_pose.pose.orientation.z = 0;
        goal_msgs.target_pose.pose.orientation.w = 1;

        /* Start moving */
        ROS_DEBUG("Setting goal...");
        ac.sendGoal(goal_msgs);

        /* Wait until the goal is set */
        ac.waitForResult(ros::Duration(waitForResult));  // TODO(minor) necessary?
        while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
        {
            ros::Duration(0.5).sleep();
        }
        ROS_DEBUG("Goal correctly set");
        ROS_DEBUG("Moving toward goal...");
        
        ros::Duration my_stuck_countdown = ros::Duration( (TIMEOUT_CHECK_1 - 2) * 60);
        ros::Time time_before = ros::Time::now();
        prev_pose_x = pose_x;
        prev_pose_y = pose_y;
        while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            // robot seems to be stuck
            if ( fabs(prev_pose_x - pose_x) < 0.1 && fabs(prev_pose_y - pose_y) < 0.1 && fabs(prev_pose_angle - pose_angle) < 0.1 )  // TODO(minor) ...
            {
                //stuck_countdown--; //TODO(minor)
                // if(stuck_countdown <= 5){

                // TODO(minor) if STUCK_COUNTDOWN is too low, even when the robot is
                // computing the frontier, it is believed to be stucked...
//                if (stuck_countdown <= 10)
//                {
//                    ROS_ERROR("Robot is not moving anymore, shutdown in: %d", stuck_countdown);
//                }

                if (my_stuck_countdown <= ros::Duration(0))
                {
//                    if( fabs(home_point_x - pose_x) < 1 && fabs(home_point_y - pose_y) < 1 ) {
//                        ROS_ERROR("robot seems unable to received ACK from actionlib even if the goal have been reached");
//                        ROS_INFO("robot seems unable to received ACK from actionlib even if the goal have been reached");
//                        ac.cancelGoal();
                        exploration->next_auction_position_x = robotPose.getOrigin().getX();
                        exploration->next_auction_position_y = robotPose.getOrigin().getY();
//                        approximate_success++;
                        store_travelled_distance();
                        return true;
//                    } else
//                        return false;
                }

                ros::Duration(1).sleep();
                
                my_stuck_countdown -= ros::Time::now() - time_before;
                time_before = ros::Time::now();

            }
            else
            {
                //ROS_ERROR("(%f, %f; %f) : (%f, %f; %f)", prev_pose_x, prev_pose_y, prev_pose_angle, pose_x, pose_y, pose_angle);
//                stuck_countdown = STUCK_COUNTDOWN;  // robot is moving again
                prev_pose_x = pose_x;
                prev_pose_y = pose_y;
                prev_pose_angle = pose_angle;
            }

            remaining_distance = exploration->distance_from_robot(optimal_ds_x, optimal_ds_y);

            /* Print remaining distance to be travelled to reach goal if the goal is a DS */
            if (remaining_distance > queue_distance / 2.0) {
//                ROS_DEBUG("Remaining distance: %.3f\e[0m", remaining_distance);

            /* If the robot is approaching a DS to queue or to check if it is free, stop it when it is close enough to
             * the DS */
                ac.cancelGoal();

                //ROS_ERROR("\n\t\e[1;34mOK!!!\e[0m");

                break;
            }

            // ros::Duration(0.5).sleep(); //TODO(minor)
        }

        ROS_INFO("DS left"); // althought this could be false... in the sense that maybe the movement was simply aborted

        exploration->next_auction_position_x = robotPose.getOrigin().getX();
        exploration->next_auction_position_y = robotPose.getOrigin().getY();
        store_travelled_distance();
        return true;
    }

    bool turn_robot(int seq)
    {
        double angle = 45;

        //if (!costmap2d_local->getRobotPose(robotPose))
        //{
        //    ROS_ERROR("Failed to get RobotPose");
        //}

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        while (!ac.waitForServer(ros::Duration(10.0)))
            ;

        move_base_msgs::MoveBaseGoal goal_msgs;

        goal_msgs.target_pose.header.seq = seq;  // increase the sequence number
        goal_msgs.target_pose.header.stamp = ros::Time::now();

        goal_msgs.target_pose.header.frame_id = move_base_frame;  //"map";
        goal_msgs.target_pose.pose.position.x = robotPose.getOrigin().getX();
        goal_msgs.target_pose.pose.position.y = robotPose.getOrigin().getY();
        goal_msgs.target_pose.pose.position.z = 0;
        goal_msgs.target_pose.pose.orientation.x = 0;  // sin(angle/2); // goals[0].pose.orientation.x;
        goal_msgs.target_pose.pose.orientation.y = 0;
        goal_msgs.target_pose.pose.orientation.z = sin(angle / 2);
        goal_msgs.target_pose.pose.orientation.w = cos(angle / 2);

        ac.sendGoal(goal_msgs);
        ac.waitForResult(ros::Duration(waitForResult));

        while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
        {
            ros::Duration(0.5).sleep();
        }
        ROS_INFO("Not longer PENDING");

        while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            ros::Duration(0.5).sleep();
        }
        ROS_INFO("Not longer ACTIVE");

        while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
            {
                ROS_INFO("ABORTED");
                return false;
            }
        }
        ROS_INFO("ROTATION ACCOMBLISHED");
        return true;
    }
