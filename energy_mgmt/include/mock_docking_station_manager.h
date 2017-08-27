#ifndef MOCK_DOCKING_STATION_MANAGER_H
#define MOCK_DOCKING_STATION_MANAGER_H

#include <vector>
#include "docking_station_manager_interface.h"

class MockDockingStationManager : public DockingStationManagerInterface
{
public:
    MockDockingStationManager();
    std::vector<ds_t> getDockingStations() override;
    void addDockingStation(ds_t ds) override;
    void setOptimalDockingStation(unsigned int id) override;
    void setOptimalDockingStation(ds_t ds) override;
    ds_t getOptimalDockingStation() override;

    std::vector<ds_t> getDockingStations2();
    unsigned int getOptimalDockingStationId();

private:
    std::vector<ds_t> docking_stations;
};

#endif // MOCK_DOCKING_STATION_MANAGER_H
