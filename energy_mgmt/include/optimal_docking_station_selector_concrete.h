#ifndef OPTIMAL_DOCKING_STATION_SELECTOR_CONCRETE_H
#define OPTIMAL_DOCKING_STATION_SELECTOR_CONCRETE_H

#include <limits>
#include <ros/ros.h>
#include "ds_structs.h"
#include "docking_station_manager_interface.h"

class OptimalDockingStationSelectorConcrete {
public:
    OptimalDockingStationSelectorConcrete();
    void setDockingStationManager(DockingStationManagerInterface *docking_station_manager);

private:
    DockingStationManagerInterface *docking_station_manager;
    unsigned int docking_station_selection_policy;
    std::vector<ds_t> ds;
    unsigned int next_optimal_ds_id;

    void loadParameters();
    void computeOptimalDs();
    void closestPolicy();
    double distance_from_robot(double goal_x, double goal_y, bool euclidean);
    double distance(double start_x, double start_y, double goal_x, double goal_y, bool euclidean);
    void abs_to_rel(double absolute_x, double absolute_y, double *relative_x, double *relative_y);
    void rel_to_abs(double relative_x, double relative_y, double *absolute_x, double *absolute_y);
    void vacantPolicy();
    void opportuneOptimalDs();
    void currentOptimalDs();
    void flockingOptimalDs();
};

#endif // OPTIMAL_DOCKING_STATION_SELECTOR_CONCRETE_H
