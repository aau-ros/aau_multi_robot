#include "ros/ros.h"
#include <explorer/switchExplRend.h>
#include <explorer/getPosition.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

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

    // parameter
    std::string robot_prefix;
    int waitForResult;
    std::string move_base_frame;

    bool move_robot(double x, double y);
    void relayRobot();
    void exploreRobot();

private:
	double home_x, home_y;
	ros::ServiceClient expl_client;
    ros::ServiceClient position_client;

    std::vector<RendezvousPoint> *rendezvousPoints;
    RendezvousPoint currentRendezvous;

    void callbackMoveToRendezvous();
    void addRendezvous(double new_x, double new_y);
};


