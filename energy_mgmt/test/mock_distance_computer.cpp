#include "mock_distance_computer.h"
#include <ros/ros.h>

MockDistanceComputer::MockDistanceComputer() {}

void MockDistanceComputer::addReachablePoint(double x, double y) {
    point_t point;
    point.x = x; point.y = y;
    reachable_points.push_back(point);
}

void MockDistanceComputer::addDistanceFromRobot(double x, double y, double distance) { //TODO raise expection / handle case of already inserted item
    distance_from_robot_t dist;
    dist.x = x; dist.y = y;
    dist.distance = distance;
    distances_from_robot.push_back(dist);
}

void MockDistanceComputer::updateDistanceFromRobot(double x, double y, double distance) {
    for(auto it = distances_from_robot.begin(); it != distances_from_robot.end(); it++)
        if(sqrt((it->x - x)*(it->x - x) + (it->y - y)*(it->y - y)) < TOLLERANCE)
            it->distance = distance;

    //TODO raise exception / return false if not found
}

bool MockDistanceComputer::checkReachability(double x, double y) {
    for(auto it = reachable_points.begin(); it != reachable_points.end(); it++)
        if(sqrt((it->x - x)*(it->x - x) + (it->y - y)*(it->y - y)) < TOLLERANCE)
            return true;
    return false;
}

double MockDistanceComputer::actualDistanceFromRobot(double x, double y) {
    for(auto it = distances_from_robot.begin(); it != distances_from_robot.end(); it++)
        if(sqrt((it->x - x)*(it->x - x) + (it->y - y)*(it->y - y)) < TOLLERANCE)
            return it->distance;
    return -1; //TODO raise exception
}
