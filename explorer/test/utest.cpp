#include <ros/ros.h>
#include <gtest/gtest.h>

#define TEST_FRIENDS \
    friend class ExplorationPlannerTest_testCase5_Test; \
    friend class ExplorationPlannerTest_testCase1_Test;

#include "ExplorationPlanner.h"

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


namespace explorationPlanner {
    class ExplorationPlannerTest : public ::testing::Test {
    };

// Declare a test
TEST_F(ExplorationPlannerTest, testCase1)
{
    int i = 10;
    EXPECT_EQ(i, 10); // 1st arg.: expected value; 2nd arg.: actual value (i.e., value returned from the tested code, which should be equal to the expected one)
}

TEST_F(ExplorationPlannerTest, testCase2)
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

TEST_F(ExplorationPlannerTest, testCase3)
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
    
    ASSERT_TRUE(success);
    EXPECT_EQ(final_goal.at(0), f.x_coordinate);
    EXPECT_EQ(final_goal.at(1), f.y_coordinate);
}

TEST_F(ExplorationPlannerTest, testCase4)
{
    std::string robot_name = "robot";
    explorationPlanner::ExplorationPlanner *ep = new explorationPlanner::ExplorationPlanner(1,true,robot_name);
    
    ep->setTestMode(true);
    ep->setRobotPosition(0, 0); //to later set the last robot position
    ep->setRobotPosition(1, 1);
    ep->setOptimalDs(0, -1, -1);
    ep->use_theta = true;

    explorationPlanner::frontier_t f1, f2;
    f1.x_coordinate = 10;
    f1.y_coordinate = 20;
    f2.x_coordinate = -10;
    f2.y_coordinate = -20;
    ep->pushFrontier(f2);
    ep->pushFrontier(f1);
    
    std::vector<double> final_goal;
    
    ep->addDistance(10, 20, 1, 1, euclidean_distance(10,20,1,1));
    ep->addDistance(10, 20, -1, -1, euclidean_distance(10,20,-1,-1));
    ep->addDistance(-10, -20, 1, 1, euclidean_distance(-10,-20,1,1));
    ep->addDistance(-10, -20, -1, -1, euclidean_distance(-10,-20,-1,-1));
    
    EXPECT_EQ(ep->getOptimalDsX(), -1);
    
    int w1 = 1;
    int w2 = 1;
    int w3 = 1;
    int w4 = 1;
    double available_distance = euclidean_distance(10,20,1,1) + euclidean_distance(10,20,-1,-1) + 10;
    unsigned int battery_charge = 60;
    std::vector<std::string> name;
    
    bool success = ep->my_determine_goal_staying_alive(1, 2, available_distance, &final_goal, 1, &name, -1, battery_charge > 50, w1, w2, w3, w4);
    
    ASSERT_TRUE(success);
    EXPECT_EQ(10, final_goal.at(0));
    EXPECT_EQ(20, final_goal.at(1));
    EXPECT_EQ(1/M_PI * (M_PI - fabs(fabs(atan2(0-1,0-1) - atan2(1-20, 1-10)) - M_PI)), final_goal.at(4));
    EXPECT_NEAR(1/M_PI * (atan2(20-1, 10-1) - atan2(1-0,1-0)), final_goal.at(4), 0.001);
    
    
    TEST_COUT << ep->_f1 << std::endl;
    TEST_COUT << ep->_f2 << std::endl;
    TEST_COUT << ep->_f3 << std::endl;
    TEST_COUT << ep->_f4 << std::endl;
    TEST_COUT << ep->_f5 << std::endl;
    TEST_COUT << ep->_f6 << std::endl;
    TEST_COUT << available_distance << std::endl;
    TEST_COUT << ep->_f7 << std::endl;
    
}

TEST_F(ExplorationPlannerTest, testCase5)
{
    std::string robot_name = "robot";
    explorationPlanner::ExplorationPlanner *ep = new explorationPlanner::ExplorationPlanner(1,true,robot_name);
    
    ep->setTestMode(true);
    ep->computeTheta(0, 0); //to later set the last robot position
    
}

TEST_F(ExplorationPlannerTest, testCase6)
{
    
    std::string robot_name = "robot";
    explorationPlanner::ExplorationPlanner *ep = new explorationPlanner::ExplorationPlanner(1,true,robot_name);
    ep->setTestMode(true);
    explorationPlanner::frontier_t f1;
    f1.x_coordinate = 10, f1.y_coordinate = 20, ep->pushFrontier(f1);

    ds_t ds1, ds2;
    ds1.x = 0, ds1.y = 0, ep->ds_list.push_back(ds1);
    ds2.x = 10, ds2.y = 10, ep->ds_list.push_back(ds2);

    ep->addDistance(f1.x_coordinate, f1.y_coordinate, ds1.x, ds1.y, euclidean_distance(f1.x_coordinate, f1.y_coordinate, ds1.x, ds1.y));

    ep->updateDistances(100);

    EXPECT_EQ(euclidean_distance(f1.x_coordinate, f1.y_coordinate, ds1.x, ds1.y), ep->frontiers.at(0).list_distance_from_ds.at(0));

    ep->addDistance(f1.x_coordinate, f1.y_coordinate, ds2.x, ds2.y, euclidean_distance(f1.x_coordinate, f1.y_coordinate, ds2.x, ds2.y));
    
    ep->updateDistances(100);

    EXPECT_EQ(euclidean_distance(f1.x_coordinate, f1.y_coordinate, ds1.x, ds1.y), ep->frontiers.at(0).list_distance_from_ds.at(0));
    EXPECT_EQ(euclidean_distance(f1.x_coordinate, f1.y_coordinate, ds2.x, ds2.y), ep->frontiers.at(0).list_distance_from_ds.at(1));
    
}




} /* namespace */

int main(int argc, char **argv){
  ros::init(argc, argv, "energy_mgmt");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}




