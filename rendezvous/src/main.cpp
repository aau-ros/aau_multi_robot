#include "rendezvous.h"

#define EXPLORER 1
#define RELAY 2

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rendezvous");
    Rendezvous *rendzv = new Rendezvous();

    int temp;
    rendzv->nh->getParam("robot_prefix", temp);

    if(temp == 0)
    {
        rendzv->iAm = RELAY;
    }

    if(rendzv->iAm == RELAY)
    {
       rendzv->relayRobot();
    }

}
