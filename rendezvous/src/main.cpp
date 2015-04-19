#include "rendezvous.h"

#define EXPLORER 1
#define RELAY 2

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rendezvous");
    Rendezvous *rendzv = new Rendezvous();

   rendzv->relayRobot();

}
