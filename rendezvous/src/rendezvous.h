#include "ros/ros.h"
#include <explorer/switchExplRend.h>
#include <explorer/getPosition.h>
#include <explorer/MissionFinished.h>
#include <adhoc_communication/SendString.h>
#include <adhoc_communication/RecvString.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "rendezvous/RendezvousPoint.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>

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
    ros::Subscriber sub;
    ros::Subscriber sub_hallo;

    // parameter
    std::string robot_prefix;
    int waitForResult;
    std::string move_base_frame;

    bool move_robot(double x, double y);
    void relayRobot();
    void exploreRobot();

    void stopStartExplorer();
    void commute();
    void relayRobot_unlimited();
    void exploreRobot_unlimited();
    void test_hallo();

     void callback_hallo(const adhoc_communication::RecvString msg);

private:
	double home_x, home_y;
	ros::ServiceClient expl_client;
    ros::ServiceClient position_client;
    ros::ServiceClient mission_client;
    ros::ServiceClient hallo_client;
    ros::ServiceClient rendezvous_client;

    bool missionFinished;
    int numberMeetings;
    bool timeoutAtRendezvous;
    bool teamMemberInRange;

    std::vector<RendezvousPoint> *rendezvousPoints;
    RendezvousPoint currentRendezvous;

    geometry_msgs::PointStamped rendezvousPoint;
    int rendezvous_point_message;
    ros::Publisher pub_rendezvous;

    geometry_msgs::PointStamped visited_rendezvousPoint;
    int visited_rendezvous_point_message;
    ros::Publisher pub_visited_rendezvous;

    geometry_msgs::PointStamped unreachable_rendezvousPoint;
    int unreachable_rendezvous_point_message;
    ros::Publisher pub_unreachable_rendezvous;


    void callbackMoveToRendezvous();
    bool hallo();
    void rendezvous();
    void addRendezvous(double new_x, double new_y);
    void printRendezvousPoints();
    void visualize_rendezvous(double x, double y);
    void visualize_visited_rendezvous(double x, double y);
    void visualize_unreachable_rendezvous(double x, double y);

    bool reachable(double x, double y);
    void callbackMoveToRendezvous_unlimited();
    void new_Rendezvous_available(const rendezvous::RendezvousPointConstPtr& msg);


};


