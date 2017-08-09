#include <ros/ros.h>
#include <gtest/gtest.h>
#include <explorer/battery_state.h>
#include "battery_state_updater.h"
#include "mock_time_manager.h"


#define SENDING_SLEEP_TIME 1
#define FLOAT_ABSOLUTE_ERROR 0.001

//TODO load from launch file 
const double power_charging =              300;
const double power_moving_fixed_cost =      10;
const double power_per_speed =               8;
const double power_microcontroller =        40;
const double power_sonar =                   5;
const double power_laser =                  15;
const double power_basic_computations =     20;
const double power_advanced_computations =  30;
const double maximum_traveling_distance =  100;

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

TEST(TestSuite, testCase2)
{
    MockTimeManager mtm;
    explorer::battery_state bt;
    BatteryStateUpdater bsu(&bt);
    bsu.setTimeManager(&mtm);
    EXPECT_TRUE(true);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "energy_mgmt");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
