#include "rendezvous.h"

#define EXPLORER 1
#define RELAY 2

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rendezvous");
    Rendezvous *rendzv = new Rendezvous();

    rendzv->relayRobot();
    //rendzv->exploreRobot();

//    //replace by classification() method then
//    std::string robot0 = "/robot_0";
//    std::string robot1 = "/robot_1";

//    if(robot0.compare(rendzv->robot_prefix) == 0)
//    {
//        rendzv->iAm = EXPLORER;
//    }
//    else if(robot1.compare(rendzv->robot_prefix) == 0)
//    {
//        rendzv->iAm = RELAY;
//    }

//    if(rendzv->iAm == EXPLORER)
//    {
//        rendzv->exploreRobot();
//    }
//    else if(rendzv->iAm == RELAY)
//    {
//        rendzv->relayRobot();
//    }



}
