#ifndef MOCK_AUCTION_MANAGER_H
#define MOCK_AUCTION_MANAGER_H

#include "auction_manager_interface.h"

class MockAuctionManager : public AuctionManagerInterface {
public:
    auction_t current_auction_test;
    bool participating_test;
    bool auction_started_test;
    bool winner_test;

    MockAuctionManager();
    void tryToAcquireDs();
    bool isRobotParticipatingToAuction();
    bool isRobotWinnerOfMostRecentAuction();
    auction_t getCurrentAuction();
    void lock();
    void unlock();
    void preventParticipationToAuctions();
    void allowParticipationToAuctions();
    void setOptimalDs(unsigned int optimal_ds_id);
};

#endif // MOCK_AUCTION_MANAGER_H
