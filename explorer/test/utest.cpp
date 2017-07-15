#include "ExplorationPlanner.h"

#include <ros/ros.h>
#include <gtest/gtest.h>

double euclidean_distance(double x1, double y1, double x2, double y2) {
    return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

namespace testing
{
 namespace internal
 {
  enum GTestColor {
      COLOR_DEFAULT,
      COLOR_RED,
      COLOR_GREEN,
      COLOR_YELLOW
  };

  extern void ColoredPrintf(GTestColor color, const char* fmt, ...);
 }
}
#define PRINTF(...)  do { testing::internal::ColoredPrintf(testing::internal::COLOR_GREEN, "[          ] "); testing::internal::ColoredPrintf(testing::internal::COLOR_YELLOW, __VA_ARGS__); } while(0)

// C++ stream interface
class TestCout : public std::stringstream
{
public:
    ~TestCout()
    {
        PRINTF("%s",str().c_str());
    }
};

#define TEST_COUT  TestCout()

// Declare a test
TEST(TestSuite, testCase1)
{
    int i = 10;
    EXPECT_NE(i, 1);
    ; //<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
}

TEST(TestSuite, testCase2)
{
    std::string robot_name = "robot";
    explorationPlanner::ExplorationPlanner *ep = new explorationPlanner::ExplorationPlanner(1,true,robot_name);
    
    ep->setTestMode(true);
    ep->setRobotPosition(0, 0);
    ep->setOptimalDs(0, -1, -1);

    explorationPlanner::frontier_t f;
    f.x_coordinate = 10;
    f.y_coordinate = 20;
    ep->pushFrontier(f);
    std::vector<double> final_goal;
    
    ep->addDistance(10, 20, 0, 0, 1);
    ep->addDistance(10, 20, -1, -1, 1);
    
    int w1 = 1;
    int w2 = 1;
    int w3 = 1;
    int w4 = 1;
    double available_distance = 10;
    unsigned int battery_charge = 60;
    std::vector<std::string> name;
    
    bool success = ep->my_determine_goal_staying_alive(1, 2, available_distance, &final_goal, 1, &name, -1, battery_charge > 50, w1, w2, w3, w4);
    EXPECT_FALSE(success);
}

TEST(TestSuite, testCase3)
{
    std::string robot_name = "robot";
    explorationPlanner::ExplorationPlanner *ep = new explorationPlanner::ExplorationPlanner(1,true,robot_name);
    
    ep->setTestMode(true);
    ep->setRobotPosition(1, 1);
    ep->setOptimalDs(0, -1, -1);

    explorationPlanner::frontier_t f;
    f.x_coordinate = 10;
    f.y_coordinate = 20;
    ep->pushFrontier(f);
    std::vector<double> final_goal;
    
    ep->addDistance(10, 20, 1, 1, euclidean_distance(10,20,1,1));
    ep->addDistance(10, 20, -1, -1, euclidean_distance(10,20,-1,-1));
    
    EXPECT_EQ(ep->getOptimalDsX(), -1);
    
    int w1 = 1;
    int w2 = 1;
    int w3 = 1;
    int w4 = 1;
    double available_distance = euclidean_distance(10,20,1,1) + euclidean_distance(10,20,-1,-1) + 10;
    unsigned int battery_charge = 60;
    std::vector<std::string> name;
    
    bool success = ep->my_determine_goal_staying_alive(1, 2, available_distance, &final_goal, 1, &name, -1, battery_charge > 50, w1, w2, w3, w4);
    
    TEST_COUT << ep->_b1 << std::endl;
    TEST_COUT << ep->_f1 << std::endl;
    TEST_COUT << ep->_f2 << std::endl;
    TEST_COUT << ep->_f3 << std::endl;
    TEST_COUT << ep->_f4 << std::endl;
    TEST_COUT << ep->_f5 << std::endl;
    TEST_COUT << ep->_f6 << std::endl;
    TEST_COUT << available_distance << std::endl;
    TEST_COUT << ep->_f7 << std::endl;
//    TEST_COUT << std::to_string(ep->_i1) << std::endl;
    
    EXPECT_TRUE(success);
    EXPECT_EQ(final_goal.at(0), f.x_coordinate);
    EXPECT_EQ(final_goal.at(1), f.y_coordinate);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "energy_mgmt");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
