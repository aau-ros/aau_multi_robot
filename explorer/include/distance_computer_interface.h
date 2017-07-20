#ifndef DISTANCE_COMPUTER_INTERFACE_H
#define DISTANCE_COMPUTER_INTERFACE_H

#include <ros/ros.h>

class DistanceComputerInterface
{
public:

    virtual ros::Time simulationTimeNow() = 0;
    virtual double computeDistance(double start_x, double start_y, double target_x, double target_y) = 0;

};

#endif /* DISTANCE_COMPUTER_INTERFACE_H */
