# include "mock_auction_manager.h"

MockAuctionManager::MockAuctionManager() {
    auction_started_test = false;
    winner_test = false;
}

void MockAuctionManager::tryToAcquireDs() {
    auction_started_test = true;
}

bool MockAuctionManager::isRobotParticipatingToAuction() {
    return participating_test;
}

bool MockAuctionManager::isRobotWinnerOfMostRecentAuction() {
    return winner_test;
}

auction_t MockAuctionManager::getCurrentAuction() {
    return current_auction_test;
}

void MockAuctionManager::lock() {}

void MockAuctionManager::unlock() {}
