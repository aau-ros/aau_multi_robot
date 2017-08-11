#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(TestAuctionManager, test1)
{
    ros::Duration(10).sleep();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "energy_mgmt");
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
