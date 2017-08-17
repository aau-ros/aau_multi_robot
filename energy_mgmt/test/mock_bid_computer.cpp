#include "mock_bid_computer.h"

MockBidComputer::MockBidComputer() {
    bid_index = 0;
}

double MockBidComputer::getBid() {
    double return_value = bids.at(bid_index);
    bid_index++;
    return return_value;
}

void MockBidComputer::addBid(double bid) {
    bids.push_back(bid);
}

void MockBidComputer::processMessages() {}

void MockBidComputer::updateLlh() {}

void MockBidComputer::logMetadata() {}
