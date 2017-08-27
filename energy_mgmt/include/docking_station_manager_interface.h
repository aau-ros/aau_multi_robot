#ifndef DOCKING_STATION_MANAGER_INTERFACE_H
#define DOCKING_STATION_MANAGER_INTERFACE_H

#include "ds_structs.h"

class DockingStationManagerInterface
{
public:
    virtual std::vector<ds_t> getDockingStations() = 0;
    virtual void addDockingStation(ds_t ds) = 0;
    virtual void setOptimalDockingStation(unsigned int id) = 0;
    virtual void setOptimalDockingStation(ds_t ds) = 0;
    virtual ds_t getOptimalDockingStation() = 0;

protected:
    unsigned int optimal_ds_id;
    std::vector<ds_t> docking_stations;
};

#endif // DOCKING_STATION_MANAGER_INTERFACE_H
