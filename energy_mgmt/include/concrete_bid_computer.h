#ifndef CONCRETE_BID_COMPUTER_H
#define CONCRETE_BID_COMPUTER_H

#include "bid_computer.h"

class ConcreteBidComputer : public BidComputer {
public:
    ConcreteBidComputer();
    double getBid();
};

#endif // CONCRETE_BID_COMPUTER_H
