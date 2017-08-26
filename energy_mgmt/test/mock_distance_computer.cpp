#include "mock_distance_computer.h"

MockDistanceComputer::MockDistanceComputer() {}

void MockDistanceComputer::addReachablePoint(double x, double y) {
    point_t point;
    point.x = x; point.y = y;
    reachable_points.push_back(point);
}

bool MockDistanceComputer::checkReachability(double x, double y) {
    for(auto it = reachable_points.begin(); it != reachable_points.end(); it++)
        if(sqrt((it->x, x)*(it->x, x) + (it->y, y)*(it->y, y)) < TOLLERANCE)
            return true;
    return false;
}
