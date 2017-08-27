#include "mock_docking_station_manager.h"

MockDockingStationManager::MockDockingStationManager() {}

std::vector<ds_t> MockDockingStationManager::getDockingStations() {
    return docking_stations;
}

void MockDockingStationManager::addDockingStation(ds_t ds) {
    docking_stations.push_back(ds);
}

void MockDockingStationManager::setOptimalDockingStation(unsigned int id) {
    optimal_ds_id = id;
}

void MockDockingStationManager::setOptimalDockingStation(ds_t ds) {}

ds_t MockDockingStationManager::getOptimalDockingStation() {}

unsigned int MockDockingStationManager::getOptimalDockingStationId() {
    return optimal_ds_id;
}
