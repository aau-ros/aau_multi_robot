#ifndef MOCK_DISTANCE_COMPUTER_H
#define MOCK_DISTANCE_COMPUTER_H

#include <vector>
#include <math.h>   // sqrt
#include "distance_computer_interface.h"

#define TOLLERANCE 0.001

struct point_t {
    double x, y;
};

class MockDistanceComputer : public DistanceComputerInterface
{
public:
    MockDistanceComputer();
    bool checkReachability(double x, double y) override;
    void addReachablePoint(double x, double y);

private:
    std::vector<point_t> reachable_points;
};

#endif // MOCK_DISTANCE_COMPUTER_H
