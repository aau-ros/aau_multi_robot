#include "ros/ros.h"
#include <explorer/switchExplRend.h>
#include <explorer/getPosition.h>
#include <explorer/MissionFinished.h>
#include <adhoc_communication/SendString.h>
#include <adhoc_communication/RecvString.h>
//#include <adhoc_communication/SendRendezvous.h>
//#include <adhoc_communication/RzvPoint.h>
#include <adhoc_communication/SendMmPoint.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/GetPlanRequest.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "rendezvous/RendezvousPoint.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <navfn/navfn_ros.h>
//#include <navfn/navfn.h>
//#include <navfn/MakeNavPlan.h>

#define EXPLORER 1
#define RELAY 2
#define BASE 3

#define A 0
#define B 1
#define C 2
#define D 3

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct RendezvousPoint{
  double x;
  double y;
  bool visited;
};

class Rendezvous
{
public:
	Rendezvous();
    ros::NodeHandle *nh;
    int iAm;
    std::string myBuddy_prefix;
    ros::Publisher pub;
    ros::Subscriber sub_hallo;
    ros::Subscriber sub_rendezvous;

    navfn::NavfnROS nav;

    // parameter
    std::string robot_prefix;
    int waitForResult;
    std::string move_base_frame;
    double maxWaitTime;
    double explorationTime;
    double rendezvousTime;

    bool move_robot(double x, double y);
    void relayRobot();
    void exploreRobot();

    void stopStartExplorer();
    void commute();
    void relayRobot_unlimited();
    void exploreRobot_unlimited();
    void test_hallo();
    void base_station();
    void test_relay_base_station();

    void callback_hallo(const adhoc_communication::RecvString msg);
    void callback_rendezvous(const adhoc_communication::MmPoint msg);

private:
	double home_x, home_y;
	ros::ServiceClient expl_client;
    ros::ServiceClient position_client;
    ros::ServiceClient mission_client;
    ros::ServiceClient hallo_client;
    ros::ServiceClient rendezvous_client;
    ros::ServiceClient plan_client;

    bool missionFinished;
    int numberMeetings;
    bool timeoutAtRendezvous;
    bool teamMemberInRange;
    bool rendezvousSuccessful;

    int rendezvous_state;
    RendezvousPoint possibleRendezvous;
    RendezvousPoint nextRendezvous;


    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2DROS *costmap_global_ros_;

    std::vector<RendezvousPoint> *rendezvousPoints;
    //RendezvousPoint currentRendezvous;

    geometry_msgs::PointStamped rendezvousPoint;
    int rendezvous_point_message;
    ros::Publisher pub_rendezvous;

    geometry_msgs::PointStamped visited_rendezvousPoint;
    int visited_rendezvous_point_message;
    ros::Publisher pub_visited_rendezvous;

    geometry_msgs::PointStamped unreachable_rendezvousPoint;
    int unreachable_rendezvous_point_message;
    ros::Publisher pub_unreachable_rendezvous;

    bool hallo(double max_wait_time);
    bool rendezvous();
    void addRendezvous(double new_x, double new_y);
    void printRendezvousPoints();
    void visualize_rendezvous(double x, double y);
    void visualize_visited_rendezvous(double x, double y);
    void visualize_unreachable_rendezvous(double x, double y);

    bool reachable(double x, double y);
    void callbackMoveToRendezvous_unlimited();
    void new_Rendezvous_available(const rendezvous::RendezvousPointConstPtr& msg);


};


