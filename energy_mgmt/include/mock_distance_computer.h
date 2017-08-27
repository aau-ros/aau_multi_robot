#ifndef MOCK_DISTANCE_COMPUTER_H
#define MOCK_DISTANCE_COMPUTER_H

#include <vector>
#include <unordered_map>
#include <math.h>   // sqrt
#include "distance_computer_interface.h"

#define TOLLERANCE 0.001

struct point_t {
    double x, y;
};

struct distance_from_robot_t {
    double x, y;
    double distance;
};

class MockDistanceComputer : public DistanceComputerInterface
{
public:
    MockDistanceComputer();
    bool checkReachability(double x, double y) override;
    double actualDistanceFromRobot(double x, double y) override;
    void addReachablePoint(double x, double y);
    void addDistanceFromRobot(double x, double y, double distance);
    void updateDistanceFromRobot(double x, double y, double distance);

private:
    //TODO use unordered_map
    std::vector<point_t> reachable_points;
    std::vector<distance_from_robot_t> distances_from_robot;
};

#endif // MOCK_DISTANCE_COMPUTER_H
