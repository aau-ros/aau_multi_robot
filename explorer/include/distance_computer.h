#ifndef DISTANCE_COMPUTER_H
#define DISTANCE_COMPUTER_H

#include "distance_computer_interface.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

class DistanceComputer : public DistanceComputerInterface
{
public:
    /**
     * Constructor.
     */
    DistanceComputer(costmap_2d::Costmap2DROS *costmap);

    ros::Time simulationTimeNow();
    double computeDistance(double start_x, double start_y, double target_x, double target_y);
    
private:
    costmap_2d::Costmap2DROS *costmap;

};

#endif /* DISTANCE_COMPUTER_H */
