#include "frontier_selector.h"

FrontierSelector::FrontierSelector() {
    loadParameters();
}

void FrontierSelector::loadParameters() {
    getParam("w1", w1);
    getParam("w2", w2);
    getParam("w3", w3);
    getParam("w4", w4);
    int tmp_num_robots;
    getParam("num_robots", tmp_num_robots);
    num_robots = (unsigned int)tmp_num_robots;
}

template <typename T>
void FrontierSelector::getParam(std::string param_name, T &param) {
    ros::NodeHandle private_nh("~");
    if(!private_nh.getParam("w1", w1))
        ROS_FATAL("Parameter %s not found!", param_name.c_str());
}

bool FrontierSelector::selectFrontier( double available_distance, std::vector<double> *final_goal, int count, std::vector<std::string> *robot_str_name, int actual_cluster_id, bool energy_above_th) {
//    errors = 0;
//    frontier_selected = false;
//    start_time = ros::Time::now();
//    winner_of_auction = true;
//    this->available_distance = available_distance;
 
    sorted_frontiers.clear();

    ROS_FATAL("MISSING");
//    my_sort_cost_0(energy_above_th, w1, w2, w3, w4);

//    ROS_DEBUG("sorted_frontiers size: %u", sorted_frontiers.size());
//    if(sorted_frontiers.size() == 0) {
//        my_error_counter = 0;
//        return false;
//    }

    unsigned int skipped = 0;
    bool frontier_selected = false;
    for(unsigned int i=0; i < sorted_frontiers.size(); i++)
    {
        ROS_FATAL("MISSING");
//        if(!my_check_efficiency_of_goal(available_distance, &sorted_frontiers.at(i))) {
//            ROS_INFO("frontier currentl unreachable: skipping");
//            continue;
//        }

        my_selected_frontier = &sorted_frontiers.at(i);

        //start auction
        my_bid = sorted_frontiers.at(i).cost;

        if(i == sorted_frontiers.size()-1 && skipped >= num_robots - 1)
            ROS_INFO("this is the only frontier for the robot: no auctioning");
        else 
        {
            ROS_INFO("start frontier negotiation");
            frontier_under_negotiation.x_coordinate = my_selected_frontier->x_coordinate;
            frontier_under_negotiation.y_coordinate = my_selected_frontier->y_coordinate;
            my_negotiate();
//         
            for(int j = 0; j < auction_timeout; j++) {
                ros::Duration(1).sleep();
                ros::spinOnce();
            }
            
            if(!winner_of_auction) {
                ROS_INFO("frontier under auction: skip");
                skipped++;
                continue;
            }
        }
        
        frontiers_under_auction.clear();
        
        sendSelectedFrontier();
        
        ROS_INFO("selected goal: %.2f, %.2f", my_selected_frontier->x_coordinate, my_selected_frontier->y_coordinate);
        final_goal->push_back(my_selected_frontier->x_coordinate);
        final_goal->push_back(my_selected_frontier->y_coordinate);            
        final_goal->push_back(my_selected_frontier->detected_by_robot);
        final_goal->push_back(my_selected_frontier->id);

//        robot_str_name->push_back(robot_name_str); 

//        my_error_counter = 0;
                  
        break;
    }

//    my_error_counter = 0;
    return frontier_selected;

}

void FrontierSelector::sendSelectedFrontier() {
    ROS_FATAL("MISSING");
//    adhoc_communication::ExpFrontier negotiation_list;
//    adhoc_communication::ExpFrontierElement negotiation_element;
//    negotiation_element.detected_by_robot = robot_name;
//    negotiation_element.x_coordinate = my_selected_frontier->x_coordinate;
//    negotiation_element.y_coordinate = my_selected_frontier->y_coordinate;
//    negotiation_element.id = my_selected_frontier->id;
//        
//    negotiation_list.frontier_element.push_back(negotiation_element);

//    my_sendToMulticast("mc_", negotiation_list, "send_next_robot_goal");
}

void FrontierSelector::my_negotiate() {
    ROS_FATAL("MISSING");
}


//double ExplorationPlanner::frontier_cost(frontier_t *frontier) {
//    return frontier_cost_0(frontier);
//}

//double ExplorationPlanner::frontier_cost_0(frontier_t *frontier) {

//    // frontier position
//    double frontier_x = frontier->x_coordinate;
//    double frontier_y = frontier->y_coordinate;

//    // calculate d_g
//    double d_g = euclidean_distance(frontier_x, frontier_y, robot_x, robot_y);

//    // calculate d_gbe
//    double d_gb;
//    if(optimal_ds_set)
//        d_gb = euclidean_distance(frontier_x, frontier_y, optimal_ds_x, optimal_ds_y);
//    else
//        d_gb = euclidean_distance(frontier_x, frontier_y, robot_home_x, robot_home_y);
//    double d_gbe;
//    if(my_energy_above_th)
//    {
//        d_gbe = -d_gb;
//    }
//    else
//    {
//        d_gbe = d_gb;
//    }
//    
//    // calculate d_r
//    double d_r = 0;
//    ros::Time time_now = ros::Time::now();
//    mutex_last_robot_auctioned_frontier_list.lock();
//    for(unsigned int i=0; i<last_robot_auctioned_frontier_list.size(); i++) {
//    
//        // remove auctioned frontiers that are too old //TODO should be better doing this in another place... but it may be inefficient
//        if(time_now - last_robot_auctioned_frontier_list.at(i).timestamp > ros::Duration(VALIDITY_INTERVAL)) {
//            ROS_INFO("expired");
//            last_robot_auctioned_frontier_list.erase(last_robot_auctioned_frontier_list.begin() + i);
//        }
//        else {
//            double distance = euclidean_distance(frontier_x, frontier_y, last_robot_auctioned_frontier_list.at(i).x_coordinate, last_robot_auctioned_frontier_list.at(i).y_coordinate);
//            if(distance < 0)
//                continue;
//            if(distance < d_r || d_r == 0) 
//                d_r = distance;        
//        }
//    }
//    mutex_last_robot_auctioned_frontier_list.unlock();
//    d_r = -d_r;    

//    // calculate theta
//    double theta;
//    if(use_theta)
//        theta = computeTheta(frontier_x, frontier_y);
//    else
//        theta = 0;
//    //DEBUGGING
//    frontier->_theta = theta;

//    // calculate cost function
//    return w1 * d_g + w2 * d_gbe + w3 * d_r + w4 * theta;
// 
//}

//void ExplorationPlanner::my_sort_cost_0(bool energy_above_th, int w1, int w2, int w3, int w4)
//{
//    acquire_mutex(&store_frontier_mutex, __FUNCTION__);
//    
//    my_energy_above_th = energy_above_th;
//    this->w1 = w1;
//    this->w2 = w2;
//    this->w3 = w3;
//    this->w4 = w4;
//    
//    if(frontiers.size() > 0)
//    {     
//        for(unsigned int j = 0; j < frontiers.size(); j++)
//        {
//            if(!my_quick_check_efficiency_of_goal(this->available_distance, &frontiers.at(j)))
//                continue;
//            // calculate cost function
//            frontiers.at(j).cost = frontier_cost_0(&frontiers.at(j));
//            add_to_sorted_fontiers_list_if_convinient(frontiers.at(j));        
//        }
//        robot_last_x = robot_x;
//        robot_last_y = robot_y;
//        use_theta = true;
//    }
//    else
//    {
//        ROS_INFO("Sorting not possible, no frontiers available");
//    }
//    
//    release_mutex(&store_frontier_mutex, __FUNCTION__);
//    ROS_INFO("finished my_sort_cost_0");
//  
//}

//void quickSort(int arr[], int left, int right) {
//      int i = left, j = right;
//      int tmp;
//      int pivot = arr[(left + right) / 2];

//      /* partition */
//      while (i <= j) {
//            while (arr[i] < pivot)
//                  i++;
//            while (arr[j] > pivot)
//                  j--;
//            if (i <= j) {
//                  tmp = arr[i];
//                  arr[i] = arr[j];
//                  arr[j] = tmp;
//                  i++;
//                  j--;
//            }
//      };
// 
//      /* recursion */
//      if (left < j)
//            quickSort(arr, left, j);
//      if (i < right)
//            quickSort(arr, i, right);
//}
