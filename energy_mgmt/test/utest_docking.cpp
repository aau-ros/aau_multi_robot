#include "docking.h"
#include <ros/ros.h>
#include <gtest/gtest.h>

#define SENDING_SLEEP_TIME 1 //s
#define FLOAT_ABSOLUTE_ERROR 0.001 //(adimensional)



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

enum state_t
    {
        exploring,  // the robot is computing which is the next frontier to be
                    // explored

        going_charging,  // the robot has the right to occupy a DS to recharge

        charging,  // the robot is charging at a DS

        finished,  // the robot has finished the exploration

        fully_charged,  // the robot has recently finished a charging process; notice
                        // that the robot is in this state even if it is not really
                        // fully charged (since just after a couple of seconds after
                        // the end of the recharging process the robot has already
                        // lost some battery energy, since it consumes power even
                        // when it stays still

        stuck,

        in_queue,  // the robot is in a queue, waiting for a DS to be vacant

        auctioning,  // auctioning: the robot has started an auction; notice that if
                     // the robot is aprticipating to an auction that it was not
                     // started by it, its state is not equal to auctioning!!!
                     
        auctioning_2,

        going_in_queue,  // the robot is moving near a DS to later put itself in
                         // in_queue state

        going_checking_vacancy,  // the robot is moving near a DS to check if it
                                 // vacant, so that it can occupy it and start
                                 // recharging

        checking_vacancy,  // the robot is currently checking if the DS is vacant,
                           // i.e., it is waiting information from the other robots
                           // about the state of the DS

        moving_to_frontier_before_going_charging,  // TODO hmm...

        moving_to_frontier,  // the robot has selected the next frontier to be
                             // reached, and it is moving toward it
        leaving_ds,          // the robot was recharging, but another robot stopped
        dead,
        moving_away_from_ds,
        auctioning_3
    };

//namespace dockingTest {
//    class DockingTest : public ::testing::Test {
//    };

double euclidean_distance(double x1, double y1, double x2, double y2) {
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

TEST(DockingTest, testCase1)
{
    docking *doc = new docking();
    doc->robot_state = static_cast<docking::state_t>(exploring);
    doc->test_mode = true;
    double robot_x = 30, robot_y = 30;
    doc->robot->x = robot_x;
    doc->robot->y = robot_y;
    doc->num_ds = 2;
    ds_t ds0, ds1;
    ds0.id = 0;
    ds0.x = 10;
    ds0.y = 10;
    ds1.id = 1;
    ds1.x = 20;
    ds1.y = 20;
    doc->ds.push_back(ds0);
    doc->ds.push_back(ds1);
    doc->compute_optimal_ds();
    EXPECT_EQ(ds1.id, doc->optimal_ds_id); //expected, actual

}

TEST(DockingTest, testCase2)
{
    docking *doc = new docking();
    doc->robot_state = static_cast<docking::state_t>(exploring);
    doc->test_mode = true;
    doc->maximum_travelling_distance = 30;
    double robot_x = 30, robot_y = 30;
    doc->robot->x = robot_x;
    doc->robot->y = robot_y;
    doc->num_ds = 2;
    ds_t ds0, ds1;
    ds0.id = 0;
    ds0.x = 10;
    ds0.y = 10;
    ds1.id = 1;
    ds1.x = 20;
    ds1.y = 20;
    doc->ds.push_back(ds0);
    doc->ds.push_back(ds1);
    doc->addDistance(ds0.x, ds0.y, ds1.x, ds1.y, euclidean_distance(ds0.x, ds0.y, ds1.x, ds1.y));
    doc->addDistance(ds0.x, ds0.y, robot_x, robot_y, euclidean_distance(ds0.x, ds0.y, robot_x, robot_y));
    doc->addDistance(ds1.x, ds1.y, robot_x, robot_y, euclidean_distance(ds1.x, ds1.y, robot_x, robot_y));
     for (int i = 0; i < doc->num_ds; i++)
        {
            std::vector<int> temp;
            std::vector<float> temp_f;
            for (unsigned int j = 0; j < (unsigned int)doc->num_ds; j++) {
                temp.push_back(-1);
                temp_f.push_back(-1);
            }
            doc->ds_mst.push_back(temp);
            doc->ds_graph.push_back(temp_f);
        }
    doc->update_ds_graph();
    doc->goal_ds_path_id = 0;
    doc->simple_compute_and_publish_path_on_ds_graph();
    EXPECT_EQ(doc->path.at(0), ds1.id);
    EXPECT_EQ(doc->path.at(1), ds0.id);

}

//TEST(DockingTest, testCase3)
//{
//    docking *doc = new docking();
//    doc->robot_state = static_cast<docking::state_t>(exploring);
//    doc->test_mode = true;
//    doc->maximum_travelling_distance = 30;
//    double robot_x = 80, robot_y = 0;
//    doc->robot->x = robot_x;
//    doc->robot->y = robot_y;
//    doc->num_ds = 3;

//    ds_t ds0; ds0.id = 0; ds0.x = 20; ds0.y = 0; doc->ds.push_back(ds0);
//    ds_t ds1; ds1.id = 1; ds1.x = 40; ds1.y = 0; doc->ds.push_back(ds1);
//    ds_t ds2; ds2.id = 2; ds2.x = 60; ds2.y = 0; doc->ds.push_back(ds2);

//    for(unsigned int i=0; i < doc->ds.size(); i++) {
//        for(unsigned int j=i; j < doc->ds.size(); j++)
//            doc->addDistance(doc->ds.at(i).x, doc->ds.at(i).y, doc->ds.at(j).x, doc->ds.at(j).y, euclidean_distance(doc->ds.at(i).x, doc->ds.at(i).y, doc->ds.at(j).x, doc->ds.at(j).y));
//        doc->addDistance(doc->ds.at(i).x, doc->ds.at(i).y, robot_x, robot_y, euclidean_distance(doc->ds.at(i).x, doc->ds.at(i).y, robot_x, robot_y));
//    }
//     for (int i = 0; i < doc->num_ds; i++)
//        {
//            std::vector<int> temp;
//            std::vector<float> temp_f;
//            for (unsigned int j = 0; j < (unsigned int)doc->num_ds; j++) {
//                temp.push_back(-1);
//                temp_f.push_back(-1);
//            }
//            doc->ds_mst.push_back(temp);
//            doc->ds_graph.push_back(temp_f);
//        }
//    doc->update_ds_graph();
//    doc->goal_ds_path_id = 0;
//    doc->simple_compute_and_publish_path_on_ds_graph();
//    EXPECT_EQ(doc->path.at(0), ds2.id);
//    EXPECT_EQ(doc->path.at(1), ds1.id);
//    EXPECT_EQ(doc->path.at(2), ds0.id);
//    EXPECT_EQ(doc->path.size(), 3);

//}

//TEST(DockingTest, testCase4)
//{
//    docking *doc = new docking();
//    doc->robot_state = static_cast<docking::state_t>(exploring);
//    doc->test_mode = true;
//    doc->maximum_travelling_distance = 30;
//    double robot_x = 50, robot_y = 50;
//    doc->robot->x = robot_x;
//    doc->robot->y = robot_y;
//    doc->num_ds = 5;

//    /*
//    R: robot
//    0: goal DS
//    1-4: DS
//                    R
//                   /
//                  /
//                 2
//                / 
//               /   
//              3
//              |  
//              |
//              |  
//        0 --- 1 --- 4
//    */
//    ds_t ds0; ds0.id = 0; ds0.x =  0; ds0.y =  0; doc->ds.push_back(ds0);
//    ds_t ds1; ds1.id = 1; ds1.x = 25; ds1.y =  0; doc->ds.push_back(ds1);
//    ds_t ds2; ds2.id = 2; ds2.x = 40; ds2.y = 40; doc->ds.push_back(ds2);
//    ds_t ds3; ds3.id = 3; ds3.x = 25; ds3.y = 25; doc->ds.push_back(ds3);
//    ds_t ds4; ds4.id = 4; ds4.x = 50; ds4.y =  0; doc->ds.push_back(ds4);

//    for(unsigned int i=0; i < doc->ds.size(); i++) {
//        for(unsigned int j=i; j < doc->ds.size(); j++)
//            doc->addDistance(doc->ds.at(i).x, doc->ds.at(i).y, doc->ds.at(j).x, doc->ds.at(j).y, euclidean_distance(doc->ds.at(i).x, doc->ds.at(i).y, doc->ds.at(j).x, doc->ds.at(j).y));
//        doc->addDistance(doc->ds.at(i).x, doc->ds.at(i).y, robot_x, robot_y, euclidean_distance(doc->ds.at(i).x, doc->ds.at(i).y, robot_x, robot_y));
//    }
//    
//    for (int i = 0; i < doc->num_ds; i++)
//    {
//        std::vector<int> temp;
//        std::vector<float> temp_f;
//        for (unsigned int j = 0; j < (unsigned int)doc->num_ds; j++) {
//            temp.push_back(-1);
//            temp_f.push_back(-1);
//        }
//        doc->ds_mst.push_back(temp);
//        doc->ds_graph.push_back(temp_f);
//    }
//    doc->update_ds_graph();
//    doc->goal_ds_path_id = 0;
//    doc->simple_compute_and_publish_path_on_ds_graph();
//    EXPECT_EQ(doc->path.at(0), ds2.id);
//    EXPECT_EQ(doc->path.at(1), ds3.id);
//    EXPECT_EQ(doc->path.at(2), ds1.id);
//    EXPECT_EQ(doc->path.at(3), ds0.id);
//    EXPECT_NE(doc->path.at(0), ds4.id); // to avoid warning on unused variable
//    EXPECT_EQ(doc->path.size(), 4);

//}

//TEST(DockingTest, testCase5)
//{
//    docking *doc = new docking();
//    doc->robot_state = static_cast<docking::state_t>(exploring);
//    doc->test_mode = true;
//    doc->maximum_travelling_distance = 30;
//    double robot_x = 75, robot_y = 0;
//    doc->robot->x = robot_x;
//    doc->robot->y = robot_y;
//    doc->num_ds = 5;

//    /*
//    R: robot
//    0: goal DS
//    1-4: DS
//                 2
//                / 
//               /   
//              3
//              |  
//              |
//              |  
//        0 --- 1 --- 4 --- R
//    */
//    ds_t ds0; ds0.id = 0; ds0.x =  0; ds0.y =  0; doc->ds.push_back(ds0);
//    ds_t ds1; ds1.id = 1; ds1.x = 25; ds1.y =  0; doc->ds.push_back(ds1);
//    ds_t ds2; ds2.id = 2; ds2.x = 40; ds2.y = 40; doc->ds.push_back(ds2);
//    ds_t ds3; ds3.id = 3; ds3.x = 25; ds3.y = 25; doc->ds.push_back(ds3);
//    ds_t ds4; ds4.id = 4; ds4.x = 50; ds4.y =  0; doc->ds.push_back(ds4);

//    for(unsigned int i=0; i < doc->ds.size(); i++) {
//        for(unsigned int j=i; j < doc->ds.size(); j++)
//            doc->addDistance(doc->ds.at(i).x, doc->ds.at(i).y, doc->ds.at(j).x, doc->ds.at(j).y, euclidean_distance(doc->ds.at(i).x, doc->ds.at(i).y, doc->ds.at(j).x, doc->ds.at(j).y));
//        doc->addDistance(doc->ds.at(i).x, doc->ds.at(i).y, robot_x, robot_y, euclidean_distance(doc->ds.at(i).x, doc->ds.at(i).y, robot_x, robot_y));
//    }
//    
//    for (int i = 0; i < doc->num_ds; i++)
//    {
//        std::vector<int> temp;
//        std::vector<float> temp_f;
//        for (unsigned int j = 0; j < (unsigned int)doc->num_ds; j++) {
//            temp.push_back(-1);
//            temp_f.push_back(-1);
//        }
//        doc->ds_mst.push_back(temp);
//        doc->ds_graph.push_back(temp_f);
//    }
//    doc->update_ds_graph();
//    doc->goal_ds_path_id = 0;
//    doc->simple_compute_and_publish_path_on_ds_graph();
//    EXPECT_EQ(doc->path.at(0), ds4.id);
//    EXPECT_EQ(doc->path.at(1), ds1.id);
//    EXPECT_EQ(doc->path.at(2), ds0.id);
//    EXPECT_NE(doc->path.at(0), ds2.id);
//    EXPECT_NE(doc->path.at(0), ds3.id); // to avoid warning on unused variable
//    EXPECT_EQ(doc->path.size(), 3);

//}

TEST(DockingTest, testCase6) {
    docking *doc = new docking();
    ds_t ds0; ds0.id = 0; ds0.x =   0; ds0.y =   0; ds0.vacant = true;  doc->ds.push_back(ds0);
    ds_t ds1; ds1.id = 1; ds1.x =  20; ds1.y =  20; ds1.vacant = false; doc->ds.push_back(ds1);
    ds_t ds2; ds2.id = 2; ds2.x = 110; ds2.y = 110; ds2.vacant = true;  doc->ds.push_back(ds2);
    double robot_x = 50, robot_y = 50;
    doc->robot->x = robot_x;
    doc->robot->y = robot_y;
    doc->ds_selection_policy = 1;

    //doc->compute_optimal_ds();
    //EXPECT_EQ(doc->optimal_ds_id, ds0.id);

}

TEST(DockingTest, testCase7) {
    docking *doc = new docking();
    ds_t ds0; ds0.id = 0; ds0.x =   0; ds0.y =   0; ds0.vacant = false;  doc->ds.push_back(ds0);
    ds_t ds1; ds1.id = 1; ds1.x =  20; ds1.y =  20; ds1.vacant = false; doc->ds.push_back(ds1);
    ds_t ds2; ds2.id = 2; ds2.x = 110; ds2.y = 110; ds2.vacant = false;  doc->ds.push_back(ds2);
    double robot_x = 50, robot_y = 50;
    doc->robot->x = robot_x;
    doc->robot->y = robot_y;
    doc->ds_selection_policy = 1;

    //doc->compute_optimal_ds();
    //EXPECT_EQ(doc->optimal_ds_id, ds1.id);
}

TEST(DockingTest, testCase8) {
    docking *doc = new docking();
    ds_t ds0; ds0.id = 0; ds0.x =   0; ds0.y =   0; ds0.vacant = true;  doc->ds.push_back(ds0);
    ds_t ds1; ds1.id = 1; ds1.x =  20; ds1.y =  20; ds1.vacant = true; doc->ds.push_back(ds1);
    ds_t ds2; ds2.id = 2; ds2.x = 110; ds2.y = 110; ds2.vacant = true;  doc->ds.push_back(ds2);
    double robot_x = 50, robot_y = 50;
    doc->robot->x = robot_x;
    doc->robot->y = robot_y;
    doc->ds_selection_policy = 1;

    //doc->compute_optimal_ds();
    //EXPECT_EQ(doc->optimal_ds_id, ds1.id);
}



//} // end namespace dockingTest

int main(int argc, char **argv){
  ros::init(argc, argv, "energy_mgmt");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
