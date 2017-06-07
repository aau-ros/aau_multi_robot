struct frontier_t
{
    int id;
    int detected_by_robot;
    std::string detected_by_robot_str;
    double robot_home_x;
    double robot_home_y;
    double x_coordinate; //world coordinate
    double y_coordinate; //world coordinate
    int distance_to_robot;
    int dist_to_robot;
    double my_distance_to_robot;
    double my_distance_to_optimal_ds;
    float cost;
    int cluster_id; //the id of the cluster in which the frontiers has been inserted; -1 if it is in no cluster
};
