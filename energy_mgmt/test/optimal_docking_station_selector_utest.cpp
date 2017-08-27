#include <ros/ros.h>
#include <gtest/gtest.h>
#include "optimal_docking_station_selector_concrete.h"
#include "mock_docking_station_manager.h"
#include "mock_distance_computer.h"

TEST(TestOptimalDockingStationSelector, testClosestPolicy)
{
    OptimalDockingStationSelectorConcrete odss;
    MockDockingStationManager dsm;
    MockDistanceComputer dc;
    odss.setDockingStationManager(&dsm);
    odss.setDistanceComputer(&dc);

    ds_t new_ds;
    new_ds.id = 1;
    new_ds.x = 10;
    new_ds.y = 10;
    dsm.addDockingStation(new_ds);
    dc.addDistanceFromRobot(10, 10, 10);

    odss.computeOptimalDs();
    EXPECT_EQ(1, dsm.getOptimalDockingStationId());

    new_ds.id = 5;
    new_ds.x = 20;
    new_ds.y = 20;
    dsm.addDockingStation(new_ds);
    dc.addDistanceFromRobot(20, 20, 5);
    new_ds.id = 3;
    new_ds.x = 30;
    new_ds.y = 30;
    dsm.addDockingStation(new_ds);
    dc.addDistanceFromRobot(30, 30, 30);

    odss.computeOptimalDs();
    EXPECT_EQ(5, dsm.getOptimalDockingStationId());

    dc.updateDistanceFromRobot(20, 20, 40);

    odss.computeOptimalDs();
    EXPECT_EQ(1, dsm.getOptimalDockingStationId());

    dc.updateDistanceFromRobot(30, 30, 5);

    odss.computeOptimalDs();
    EXPECT_EQ(3, dsm.getOptimalDockingStationId());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "optimal_docking_station_selector_utest");
    ros::NodeHandle nh;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
