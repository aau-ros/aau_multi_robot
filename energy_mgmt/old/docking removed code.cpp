    /*
    int arr[5][5] = {
        { 0, 1, 0, 6, 0 }, { 1, 0, 1, 8, 5 }, { 0, 1, 0, 1, 7 }, { 6, 8, 1, 0, 1
    }, { 0, 5, 7, 1, 0 },
    };
    std::vector < std::vector <int > > graph;
    for(int i=0; i < 5; i++) {
        std::vector<int> temp;
        for(int j=0; j < 5; j++)
            temp.push_back(arr[i][j]);
        graph.push_back(temp);
    }
    */

    /* VERSION 1 */
    /*
bool computed_new_optimal_ds = false;
std::vector<ds_t>::iterator it = ds.begin();
for (; it != ds.end(); it++)
    // if(optimal_ds_computed_once) {

    //TODO here I should consider that maybe there is a pending update of the DS...
    if ((best_ds.x - x) * (best_ds.x - x) + (best_ds.y - y) * (best_ds.y - y) >
        ((*it).x - x) * ((*it).x - x) + ((*it).y - y) * ((*it).y - y))
    {
        computed_new_optimal_ds = true;
        next_optimal_ds = *it;
    }
    else
        ;  // ROS_ERROR("\e[1;34m!!!!\e[0m");
//} else {
//    ROS_ERROR("\e[1;34mFirst computation of optimal DS: ds %d at (%f, %f)\e[0m", best_ds.id, best_ds.x,
//    best_ds.y);
//    optimal_ds_computed_once = true;
//    best_ds = *it;
//}

if(computed_new_optimal_ds)
   if(participating_to_auction == 0 && !update_state_required) {
        ROS_ERROR("\e[1;34mNew optimal DS: ds%d (%f, %f)\e[0m", next_optimal_ds.id, next_optimal_ds.x,
next_optimal_ds.y);
        best_ds = next_optimal_ds;
        geometry_msgs::PointStamped msg1;
        msg1.point.x = best_ds.x;
        msg1.point.y = best_ds.y;
        //pub_new_target_ds.publish(msg1);
    }
    else
        ROS_ERROR("\e[1;34mNext optimal DS: ds%d (%f, %f)\e[0m", next_optimal_ds.id, next_optimal_ds.x,
next_optimal_ds.y);
 else
    ROS_DEBUG("Optimal DS unchanged");
    */
