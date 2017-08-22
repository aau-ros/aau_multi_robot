#ifndef OPTIMAL_DOCKING_STATION_SELECTOR_H
#define OPTIMAL_DOCKING_STATION_SELECTOR_H

class OptimalDockingStationSelector {
public:
    OptimalDockingStationSelector();
    void setDockingStationManager();

private:
    unsigned int num_robots;
    unsigned int docking_station_selection_policy;

    loadParameters();
}

#endif // OPTIMAL_DOCKING_STATION_SELECTOR_H
