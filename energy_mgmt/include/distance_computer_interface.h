#ifndef DISTANCE_COMPUTER_INTERFACE_H
#define DISTANCE_COMPUTER_INTERFACE_H

class DistanceComputerInterface
{
public:
    virtual bool checkReachability(double x, double y) = 0;
    virtual double actualDistanceFromRobot(double x, double y) = 0;
//    virtual double euclideanDistanceFromRobot(double x, double y) = 0;
//    virtual double actualDistance(double x1, double y1, double x2, double y2) = 0;
//    virtual double euclideanDistance(double x1, double y1, double x2, double y2) = 0;
};

#endif // DISTANCE_COMPUTER_INTERFACE_H
