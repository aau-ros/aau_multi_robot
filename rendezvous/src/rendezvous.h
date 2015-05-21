#include "ros/ros.h"
#include <explorer/switchExplRend.h>
#include <explorer/getPosition.h>
#include <explorer/MissionFinished.h>
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

    // parameter
    std::string robot_prefix;
    int waitForResult;
    std::string move_base_frame;

    bool move_robot(double x, double y);
    void relayRobot();
    void exploreRobot();

    void printRendezvousPoints();
    void new_Rendezvous_available(const rendezvous::RendezvousPointConstPtr& msg);

    void stopStartExplorer();


private:
	double home_x, home_y;
	ros::ServiceClient expl_client;
    ros::ServiceClient position_client;
    ros::ServiceClient mission_client;

    bool missionFinished;

    std::vector<RendezvousPoint> *rendezvousPoints;
    RendezvousPoint currentRendezvous;

    geometry_msgs::PointStamped rendezvousPoint;
    int rendezvous_point_message;
    ros::Publisher pub_rendezvous;

    void callbackMoveToRendezvous();
    void addRendezvous(double new_x, double new_y);
    void visualize_rendezvous(double x, double y);
    bool reachable(double x, double y);

};


