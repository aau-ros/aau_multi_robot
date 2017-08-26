#ifndef DOCKING_STATION_DETECTOR_H
#define DOCKING_STATION_DETECTOR_H

#include <ros/ros.h>
#include <explorer/DistanceFromRobot.h>
#include "ds_structs.h"

#define SSTR(x) static_cast<std::ostringstream &>((std::ostringstream() << std::dec << x)).str()

class DockingStationDetector
{
public:
    DockingStationDetector();

private:
    std::vector<ds_t> undiscovered_dss;
    ros::ServiceClient sc_reachable_target;

    void preloadDockingStations();
    void detectNewDockingStations();
    std::string dsToStr(ds_t ds);
    void abs_to_rel(double absolute_x, double absolute_y, double *relative_x, double *relative_y);
    void rel_to_abs(double relative_x, double relative_y, double *absolute_x, double *absolute_y);
};

#endif // DOCKING_STATION_DETECTOR_H
