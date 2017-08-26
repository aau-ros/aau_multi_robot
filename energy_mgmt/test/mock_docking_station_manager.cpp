#include "mock_docking_station_manager.h"

MockDockingStationManager::MockDockingStationManager() {}

void MockDockingStationManager::getDockingStations() {
    
}

std::vector<ds_t> MockDockingStationManager::getDockingStations2() {
    return docking_stations;
}

void MockDockingStationManager::addDockingStation(ds_t ds) {
    docking_stations.push_back(ds);
}
