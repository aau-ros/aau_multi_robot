#include "distance_computer.h"

DistanceComputer::DistanceComputer(costmap_2d::Costmap2DROS *costmap) {
    this->costmap = costmap;
}

ros::Time DistanceComputer::simulationTimeNow() {
    return ros::Time::now();
}

double DistanceComputer::computeDistance(double start_x, double start_y, double target_x, double target_y) {
    return 0.0;
}

    bool distance_from_robot_callback(explorer::DistanceFromRobot::Request &req,
                                      explorer::DistanceFromRobot::Response &res)
    {
        res.distance = exploration->distance_from_robot(req.x, req.y);
        if(res.distance >= 0)
            return true;
        return false;
    }
    
    bool distance(explorer::Distance::Request &req, explorer::Distance::Response &res)
    {
        res.distance = exploration->distance(req.x1, req.y1, req.x2, req.y2);
        if(res.distance >= 0)
            return true;
        return false;
    }

/**
 * Compute the length of the trajectory from the robots current position to a given target
 * and store it in the global variable exploration_travel_path_global
 */
void ExplorationPlanner::trajectory_plan_store(double target_x, double target_y)
{
    double distance = trajectory_plan_meters(target_x, target_y);

    if(distance >= 0) {
        //F
        //exploration_travel_path_global += distance;
        //exploration_travel_path_global_meters += distance * costmap_ros_->getCostmap()->getResolution();
        exploration_travel_path_global_meters += distance;
    }
    else {
        ROS_ERROR("Failed to compute and store distance!");
        //ROS_ERROR("Failed to compute and store distance! Using euclidean distance as approximation..."); //TODO it could fail due to a failure of getRobotPose
        //exploration_travel_path_global_meters += euclidean_distance(target_x, target_y);
    }
}

double ExplorationPlanner::trajectory_plan_meters(double target_x, double target_y)
{
    if (!costmap_global_ros_->getRobotPose(robotPose))
    {
        ROS_ERROR("Failed to get RobotPose");
        return -1;
    }
    //ROS_ERROR("%f", costmap_global_ros_->getCostmap()->getResolution());
    double dist = trajectory_plan_meters(robotPose.getOrigin().getX(), robotPose.getOrigin().getY(), target_x, target_y);
    if(dist < 0) {
        ROS_WARN("failed distance");
    }
    return dist;
}

/**
 * Compute the length of the trajectory from a given start to a given target in meters
 */
double ExplorationPlanner::trajectory_plan_meters(double start_x, double start_y, double target_x, double target_y)
{
    ROS_INFO("trajectory_plan_meters");

    geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;
    double distance;

    std::vector<double> backoffGoal, backoffGoal2;
    bool backoff_flag = smartGoalBackoff(target_x,target_y, costmap_global_ros_, &backoffGoal);
    bool backoff_flag2 = smartGoalBackoff(start_x,start_y, costmap_global_ros_, &backoffGoal2);

    startPointSimulated.header.seq = start_point_simulated_message++;	// increase the sequence number
    startPointSimulated.header.stamp = ros::Time::now();
    startPointSimulated.header.frame_id = move_base_frame;
    if(backoff_flag2 == true)
    {
        startPointSimulated.pose.position.x = backoffGoal2.at(0);
        startPointSimulated.pose.position.y = backoffGoal2.at(1);
    }
    else
    {
        startPointSimulated.pose.position.x = start_x;
        startPointSimulated.pose.position.y = start_y;
    }
    startPointSimulated.pose.position.z = 0;
    startPointSimulated.pose.orientation.x = 0;
    startPointSimulated.pose.orientation.y = 0;
    startPointSimulated.pose.orientation.z = 0;
    startPointSimulated.pose.orientation.w = 1;

    goalPointSimulated.header.seq = goal_point_simulated_message++;	// increase the sequence number
    goalPointSimulated.header.stamp = ros::Time::now();
    goalPointSimulated.header.frame_id = move_base_frame;
    if(backoff_flag == true)
    {
        goalPointSimulated.pose.position.x = backoffGoal.at(0);
        goalPointSimulated.pose.position.y = backoffGoal.at(1);
    }
    else
    {
        goalPointSimulated.pose.position.x = target_x;
        goalPointSimulated.pose.position.y = target_y;
    }
    goalPointSimulated.pose.position.z = 0;
    goalPointSimulated.pose.orientation.x = 0;
    goalPointSimulated.pose.orientation.y = 0;
    goalPointSimulated.pose.orientation.z = 0;
    goalPointSimulated.pose.orientation.w = 1;

    std::vector<geometry_msgs::PoseStamped> global_plan;

    //acquire_mutex(&costmap_mutex, __FUNCTION__);
//    ROS_INFO("computing path");   
        
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_global_ros_->getCostmap()->getMutex()));
        
    bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);
//    distance_computer->computeDistance(1,2,3,4); //TODO use this for execution and testing...
    
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> unlock(*(costmap_global_ros_->getCostmap()->getMutex()));
//    ROS_INFO("path computed");
    //release_mutex(&costmap_mutex, __FUNCTION__);
    
    //ROS_ERROR("%d", successful);
    //ros::Duration(2).sleep();
    
    if(successful == true)
    {
//        ROS_ERROR("Path from (%f, %f) to (%f, %f)", startPointSimulated.pose.position.x, startPointSimulated.pose.position.y, goalPointSimulated.pose.position.x, goalPointSimulated.pose.position.y);
//        //distance =  global_plan.size();
//        for(int i=0; i < global_plan.size(); i++)
//            ROS_ERROR("(%f, %f)", global_plan[i].pose.position.x, global_plan[i].pose.position.y);
        
        distance = 0;
        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
        geometry_msgs::PoseStamped prev_point = (*it);
        it++;
        for(; it != global_plan.end(); it++) {
//            distance += euclidean_distance(prev_point.pose.position.x, prev_point.pose.position.y, (*it).pose.position.x, (*it).pose.position.y);
            distance += sqrt( (prev_point.pose.position.x - (*it).pose.position.x) * (prev_point.pose.position.x - (*it).pose.position.x) + (prev_point.pose.position.y - (*it).pose.position.y) * (prev_point.pose.position.y - (*it).pose.position.y) ); //* costmap_global_ros_->getCostmap()->getResolution();
            prev_point = (*it);
        }
        
        //ROS_ERROR("%f", distance);
        
//        for(int i=0; i < ds_list.size(); i++) {
//            if( fabs(target_x - ds_list[i].x) <= 0.3 && fabs(target_y - ds_list[i].y) <= 0.3)
//                ROS_ERROR("ok %.1f, %.1f", target_x, target_y);
//        }
        
//       ROS_INFO("end trajectory_plan_meters successfully");
        return distance;
    }
    else
    {
        // It should never happen that trajectory_plan_meters is called on a goal that is not reachable...
//        double wx, wy;
//        unsigned int mx, my;
//        costmap_global_ros_->getCostmap()->worldToMap(wx, wy, mx, my);
//        double world_x = (mx - costmap_global_ros_->getCostmap()->getSizeInCellsX() / 2) * costmap_global_ros_->getCostmap()->getResolution();
//        double world_y = (my - costmap_global_ros_->getCostmap()->getSizeInCellsY() / 2) * costmap_global_ros_->getCostmap()->getResolution();
        ROS_WARN("makePlan() failed for goal (%.1f, %.1f) from start (%.1f, %.1f) (Stage coord.s); returning -1...", goalPointSimulated.pose.position.x + robot_home_world_x,  goalPointSimulated.pose.position.y + robot_home_world_y, startPointSimulated.pose.position.x + robot_home_world_x, startPointSimulated.pose.position.y + robot_home_world_y);
        
//        ROS_ERROR("makePlan() failed for goal (%.1f, %.1f) from start (%.1f, %.1f) (Stage coord.s); returning -1...", goalPointSimulated.pose.position.x,  goalPointSimulated.pose.position.y, startPointSimulated.pose.position.x, startPointSimulated.pose.position.y);
        return -1;
    }
}

double ExplorationPlanner::simplifiedDistanceFromDs(unsigned int ds_index, unsigned int frontier_index) {
    
    acquire_mutex(&mutex_erase_frontier, __FUNCTION__);
    mutex_ds.lock();
    if(frontiers.size() <= frontier_index || ds_list.size() <= ds_index) {
        ROS_FATAL("invalid index in simplifiedDistanceFromDs()");
        ROS_FATAL("frontiers: size: %u, index: %u", frontiers.size(), frontier_index);
        ROS_FATAL("ds: size: %u; index: %u", ds_list.size(), ds_index);
    }
    mutex_ds.unlock();    
    
    double distance;
    if(ds_index >= frontiers.at(frontier_index).list_distance_from_ds.size() || frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) < 0)
    {
        double ds_x, ds_y, f_x, f_y;
        ds_x = ds_list.at(ds_index).x;
        ds_y = ds_list.at(ds_index).y;
        f_x = frontiers.at(frontier_index).x_coordinate;
        f_y = frontiers.at(frontier_index).y_coordinate;
        distance = trajectory_plan_meters(ds_x, ds_y, f_x, f_y); //TODO allow user to select if in this case we should use the real distance or not
    } 
    else
        distance = frontiers.at(frontier_index).list_distance_from_ds.at(ds_index);
    
    release_mutex(&mutex_erase_frontier, __FUNCTION__);
    return distance;
}

bool ExplorationPlanner::reachable_target(double x, double y) {
    if(costmap_ros_ == NULL) {
        ROS_DEBUG("Costmap is not ready yet: cannot check reachability of the target");
        return false;
    }
    
    geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;

    startPointSimulated.header.seq = 1; //TODO correct??
    startPointSimulated.header.stamp = ros::Time::now();
    startPointSimulated.header.frame_id = move_base_frame;
    startPointSimulated.pose.position.x = robotPose.getOrigin().getX();
    startPointSimulated.pose.position.y = robotPose.getOrigin().getY();
    startPointSimulated.pose.position.z = 0;
    startPointSimulated.pose.orientation.x = 0;
    startPointSimulated.pose.orientation.y = 0;
    startPointSimulated.pose.orientation.z = 0;
    startPointSimulated.pose.orientation.w = 1;

    goalPointSimulated.header.seq = 1; //TODO correct??
    goalPointSimulated.header.stamp = ros::Time::now();
    goalPointSimulated.header.frame_id = move_base_frame;  
    goalPointSimulated.pose.position.x = x;
    goalPointSimulated.pose.position.y = y;
    goalPointSimulated.pose.position.z = 0;
    goalPointSimulated.pose.orientation.x = 0;
    goalPointSimulated.pose.orientation.y = 0;
    goalPointSimulated.pose.orientation.z = 0;
    goalPointSimulated.pose.orientation.w = 1;

    std::vector<geometry_msgs::PoseStamped> global_plan;

    //acquire_mutex(&costmap_mutex, __FUNCTION__);
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_global_ros_->getCostmap()->getMutex()));
    bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> unlock(*(costmap_global_ros_->getCostmap()->getMutex()));
    //release_mutex(&costmap_mutex, __FUNCTION__);
    
    if(successful == true) {
        double final_x, final_y;
        double acc = 1;
        final_x = global_plan.at(global_plan.size()-1).pose.position.x;
        final_y = global_plan.at(global_plan.size()-1).pose.position.y;
        
        //for(int i =0; i < global_plan.size(); i++)
        //    ROS_ERROR("point of plan: (%f, %f)", global_plan.at(i).pose.position.x, global_plan.at(i).pose.position.y);
        
        //ROS_ERROR("finals: (%f, %f) - target: (%f, %f)", final_x, final_y, x, y);
        
        if(final_x > x) {
            if(final_x - x > acc)
                return false;
            else if(x - final_x > acc)
                return false;
        }
        if(final_y > y) {
            if(final_y - y > acc)
                return false;
            else if(y - final_y > acc)
                return false;
        }
        return true;
    }
    else {
//        ROS_ERROR("the target is for the moment unreachable");
        // The target is (at least at the moment) unreachable, in the sense that there is not a path to it. Notice that it's not possible to understand if there was a failure in the computation of the path caused by the global planner or if a path does not really exists.
        return false;
   }
}

void ExplorationPlanner::updateDistances(double max_available_distance, bool use_heuristic) {
    for(unsigned int i=0; i< ds_list.size(); i++)
        ds_list.at(i).has_EOs = false;

    for(unsigned int frontier_index = frontiers.size() - 1; frontier_index >= 0; frontier_index--) { //start from the bottom not to penalize the newest frontiers
        for(unsigned int ds_index=0; ds_index < ds_list.size(); ds_index++) {
            acquire_mutex(&mutex_erase_frontier, __FUNCTION__);
            
            // in case meanwhile some frontiers have been deleted
            if(frontier_index >= frontiers.size()) {
                release_mutex(&mutex_erase_frontier, __FUNCTION__);
                return;
            }
            
            while(frontiers.at(frontier_index).list_distance_from_ds.size() < ds_list.size())
                frontiers.at(frontier_index).list_distance_from_ds.push_back(-1);

            if(frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) > 0 && use_heuristic)
                ;
            else if(frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) > 0 && frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) < max_available_distance)
                ;
            else {
                double distance;
                distance = euclidean_distance(ds_list.at(ds_index).x, ds_list.at(ds_index).y, frontiers.at(frontier_index).x_coordinate, frontiers.at(frontier_index).y_coordinate);
                if(distance*2 > max_available_distance)
                    frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) = distance;
                else {
                     distance = trajectory_plan_meters(ds_list.at(ds_index).x, ds_list.at(ds_index).y, frontiers.at(frontier_index).x_coordinate, frontiers.at(frontier_index).y_coordinate);
                    if(distance < 0)
                        ;
                    else
                        frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) = distance;
                }
            }

            if (frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) > 0 && (frontiers.at(frontier_index).list_distance_from_ds.at(ds_index) * 2) < max_available_distance)
                ds_list.at(ds_index).has_EOs = true;

            release_mutex(&mutex_erase_frontier, __FUNCTION__);
        }
    }
}
