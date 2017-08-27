#include "distance_computer_concrete.h"

DistanceComputerConcrete::DistanceComputerConcrete() {
    std::string my_prefix = ""; //TODO
    sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target");
}

void DistanceComputerConcrete::checkReachability(double x, double y) {
    explorer::DistanceFromRobot srv_msg;
    srv_msg.request.x = x;
    srv_msg.request.y = y;
    
    //ros::service::waitForService("explorer/reachable_target");
    for(int j = 0; j < 10 && !sc_reachable_target; j++) {
        ROS_FATAL("NO MORE CONNECTION!");
        ros::Duration(1).sleep();
        //sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target", true);
    }

    if (sc_reachable_target.call(srv_msg))
        reachable = srv_msg.response.reachable;
    else
    {
        ROS_ERROR("Unable to check if %s is reachable, retrying later...", dsToStr(*it).c_str());
        continue;
    }
}

double DistanceComputerConcrete::euclideanDistanceFromRobot(double goal_x, double goal_y)
{
    ROS_FATAL("MISSING");
    return -1;
//    return distance(robot->x, robot->y, goal_x, goal_y, euclidean);
}

double DistanceComputerConcrete::actualDistanceFromRobot(double goal_x, double goal_y)
{
    ROS_FATAL("MISSING");
    return -1;
//    return distance(robot->x, robot->y, goal_x, goal_y, euclidean);
}

double DistanceComputerConcrete::euclideanDistance(double start_x, double start_y, double goal_x, double goal_y)
{
    double dx = (goal_x - start_x);
    double dy = (goal_y - start_y);
    return sqrt(dx * dx + dy * dy);
}

double DistanceComputerConcrete::actualDistance(double start_x, double start_y, double goal_x, double goal_y)
{
    ROS_FATAL("MISSING");
    /* Use euclidean distance if required by the caller */
//    if (euclidean)
//    {
//        double dx = (goal_x - start_x);
//        double dy = (goal_y - start_y);
//        return sqrt(dx * dx + dy * dy);
//    }

//    /* Otherwise, use actual distance: ask explorer node to compute it (using its costmap) */    
//    explorer::Distance srv_msg;
//    srv_msg.request.x1 = start_x;
//    srv_msg.request.y1 = start_y;
//    srv_msg.request.x2 = goal_x;
//    srv_msg.request.y2 = goal_y;
//    
//    //ros::service::waitForService("explorer/distance");   
//    for(int i = 0; i < 10 && !sc_distance; i++) {
//        ROS_FATAL("NO MORE CONNECTION!");
//        ros::Duration(1).sleep();
//        //sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance", true);
//    }
//    for (int i = 0; i < 10; i++)
//        if (sc_distance.call(srv_msg) && srv_msg.response.distance >= 0) {
//            return srv_msg.response.distance;
//        } else
//            ros::Duration(1).sleep();
//    
//    /* If the service is not working at the moment, return invalid value */ //TODO(minor) raise exception?
//    ROS_ERROR("Unable to compute distance at the moment...");
    return -1;
}
