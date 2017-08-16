#ifndef BID_COMPUTER_H
#define BID_COMPUTER_H

#include <ros/ros.h>

class BidComputer {
public:
    virtual double getBid() = 0;
    virtual void updateLlh() = 0;
    virtual void processMessages() = 0;
};

#endif // BID_COMPUTER_H
