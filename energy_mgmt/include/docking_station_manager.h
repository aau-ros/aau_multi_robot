#ifndef DOCKING_STATION_MANAGER_H
#define DOCKING_STATION_MANAGER_H

#include "ds_structs.h"

class DockingStationManager {
public:
    DockingStationManager();
    ds_t getOptimalDockingStation();
    void setOptimalDockingStation(ds_t ds);
    void getDockingStationIterator(); //TODO put how to handle the fact that iterator can be invalidated?
    void getDockingStationVector(); //TODO put how to handle the fact that iterator can be invalidated?
};

#endif // DOCKING_STATION_MANAGER_H
