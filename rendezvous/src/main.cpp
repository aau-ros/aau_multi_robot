#include "rendezvous.h"

#define EXPLORER 1
#define RELAY 2


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rendezvous");
    Rendezvous *rendzv = new Rendezvous();

    //rendzv->relayRobot();
    //rendzv->exploreRobot();
    //rendzv->stopStartExplorer();

    //replace by classification() method then
    std::string robot0 = "/robot_0";
    std::string robot1 = "/robot_1";

    if(robot1.compare(rendzv->robot_prefix) == 0)
    {
        rendzv->iAm = EXPLORER;
        rendzv->myBuddy_prefix = "/robot_0";
        ROS_DEBUG("%s is an EXPLORER", rendzv->robot_prefix.c_str());
    }
    else if(robot0.compare(rendzv->robot_prefix) == 0)
    {
        rendzv->iAm = RELAY;
        rendzv->myBuddy_prefix = "/robot_1";
        ROS_DEBUG("%s is an RELAY", rendzv->robot_prefix.c_str());
    }
    // end replace


    if(rendzv->iAm == EXPLORER)
    {
        std::string topic = rendzv->robot_prefix + "/rendezvous/RendezvousPoints";
        rendzv->pub = rendzv->nh->advertise<rendezvous::RendezvousPoint>(topic, 10);
        rendzv->exploreRobot();
    }
    else if(rendzv->iAm == RELAY)
    {
        // subscribe to rendezvous points topic of my EXPLORE robot
        std::string topic_name = rendzv->myBuddy_prefix + std::string("/rendezvous/RendezvousPoints");
        rendzv->sub = rendzv->nh->subscribe(topic_name, 10, &Rendezvous::new_Rendezvous_available, rendzv);
        rendzv->relayRobot();
    }



}
