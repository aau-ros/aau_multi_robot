#ifndef DOCKING_STATION_MANAGER_H
#define DOCKING_STATION_MANAGER_H

struct ds_t
{
    int id;
    double x, y; // coordinates of the DS in the /map frame
    double world_x, world_y; // coordinates of the DS in the /world frame (i.e., in case of a simulation, in the reference system of the simulator)
    bool vacant;
    double timestamp;
    bool has_EOs;
};

class DockingStationManager {
public:
    DockingStationManager();
    ds_t getOptimalDockingStation();
    setOptimalDockingStation(ds_t ds);
    void getDockingStationIterator(); //TODO put how to handle the fact that iterator can be invalidated?
    void getDockingStationVector(); //TODO put how to handle the fact that iterator can be invalidated?
};

#endif // DOCKING_STATION_MANAGER_H
