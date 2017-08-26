#include <ros/ros.h>
#include <gtest/gtest.h>

#include "docking_station_detector.h"
#include "mock_docking_station_manager.h"
#include "mock_distance_computer.h"

#define NEAR_ERROR 0.001

TEST(TestDockingStationDetector, testDetectDockingStations)
{
    MockDockingStationManager dsm;
    MockDistanceComputer dc;
    DockingStationDetector dsd;
    dsd.setDockingStationManager(&dsm);
    dsd.setDistanceComputer(&dc);

    dc.addReachablePoint(4, 2);
    dc.addReachablePoint(-1, 1.5);

    dsd.preloadDockingStations();
    dsd.detectNewDockingStations();
    dsd.detectNewDockingStations();

    std::vector<ds_t> docking_stations = dsm.getDockingStations2();
    ASSERT_EQ(2, docking_stations.size());
    EXPECT_EQ(0, docking_stations.at(0).id);
    EXPECT_NEAR(4, docking_stations.at(0).x, NEAR_ERROR);
    EXPECT_NEAR(2, docking_stations.at(0).y, NEAR_ERROR);
    EXPECT_EQ(3, docking_stations.at(1).id);
    EXPECT_NEAR(-1, docking_stations.at(1).x, NEAR_ERROR);
    EXPECT_NEAR(1.5, docking_stations.at(1).y, NEAR_ERROR);

    dc.addReachablePoint(-2, 4.5);

    dsd.detectNewDockingStations();
    docking_stations = dsm.getDockingStations2();
    ASSERT_EQ(3, docking_stations.size());
    EXPECT_EQ(0, docking_stations.at(0).id);
    EXPECT_NEAR(4, docking_stations.at(0).x, NEAR_ERROR);
    EXPECT_NEAR(2, docking_stations.at(0).y, NEAR_ERROR);
    EXPECT_EQ(3, docking_stations.at(1).id);
    EXPECT_NEAR(-1, docking_stations.at(1).x, NEAR_ERROR);
    EXPECT_NEAR(1.5, docking_stations.at(1).y, NEAR_ERROR);
    EXPECT_EQ(2, docking_stations.at(2).id);
    EXPECT_NEAR(-2, docking_stations.at(2).x, NEAR_ERROR);
    EXPECT_NEAR(4.5, docking_stations.at(2).y, NEAR_ERROR);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "docking_station_detector_utest");
    ros::NodeHandle nh;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
