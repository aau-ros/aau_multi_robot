#ifndef MOCK_DOCKING_STATION_MANAGER_H
#define MOCK_DOCKING_STATION_MANAGER_H

#include <vector>
#include "docking_station_manager_interface.h"

class MockDockingStationManager : public DockingStationManagerInterface
{
public:
    MockDockingStationManager();
    void getDockingStations() override;
    std::vector<ds_t> getDockingStations2();
    void addDockingStation(ds_t ds) override;

private:
    std::vector<ds_t> docking_stations;
};

#endif // MOCK_DOCKING_STATION_MANAGER_H
