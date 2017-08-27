#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(TestSample, test)
{

}

int main(int argc, char **argv){
    ros::init(argc, argv, "sample_utest");
    ros::NodeHandle nh; // to print error messages
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
