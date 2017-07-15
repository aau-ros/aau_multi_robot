#include "ExplorationPlanner.h"

#include <ros/ros.h>
#include <gtest/gtest.h>



// Declare a test
TEST(TestSuite, testCase1)
{
    int i = 10;
    EXPECT_NE(i, 1);
    ; //<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
}

TEST(TestSuite, testCase2)
{
    explorationPlanner::ExplorationPlanner ep(1,true,"robot");
    int i = 10;
    EXPECT_NE(i, 1);
    //explorationPlanner::frontier_t f;
    //f.x_coordinate = 10;
    //f.y_coordinate = 20;
    //ep.pushFrontier(f);
    //std::vector<double> final_goal;
    //ep.my_determine_goal_staying_alive(1, 2, available_distance, &final_goal, 1, "robot", -1, battery_charge > 50, w1, w2, w3, w4);
    //EXPECT_EQ(final_goal.at(0), f.x_coordinate);
    //EXPECT_EQ(final_goal.at(1), f.y_coordinate);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "energy_mgmt");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
