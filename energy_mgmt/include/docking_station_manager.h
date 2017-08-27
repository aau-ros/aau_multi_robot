#ifndef DOCKING_STATION_MANAGER_H
#define DOCKING_STATION_MANAGER_H

#include "ds_structs.h"

class DockingStationManager {
public:
    DockingStationManager();
    std::vector<ds_t> getDockingStations() override;
    void addDockingStation(ds_t ds) override;
    void setOptimalDockingStation(unsigned int id) override;
    void setOptimalDockingStation(ds_t ds) override;
    ds_t getOptimalDockingStation() override; //TODO put how to handle the fact that iterator can be invalidated?
};

#endif // DOCKING_STATION_MANAGER_H
