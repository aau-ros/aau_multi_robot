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
