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
