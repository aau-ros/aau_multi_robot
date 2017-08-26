#ifndef DISTANCE_COMPUTER_INTERFACE_H
#define DISTANCE_COMPUTER_INTERFACE_H

class DistanceComputerInterface
{
public:
    virtual bool checkReachability(double x, double y) = 0;
};

#endif // DISTANCE_COMPUTER_INTERFACE_H
