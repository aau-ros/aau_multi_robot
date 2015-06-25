#include "rendezvous.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rendezvous");
    Rendezvous *rendzv = new Rendezvous();

    std::string robot0 = "/robot_0";
    std::string robot1 = "/robot_1";
    std::string robot2 = "/robot_2";

    std::string topic = rendzv->robot_prefix + std::string("/rendezvous/checkCommunicationRange");
    std::string rendezvous_topic = rendzv->robot_prefix + std::string("/rendezvous/rendezvousAgreement");

    rendzv->sub_hallo = rendzv->nh->subscribe(topic, 1, &Rendezvous::callback_hallo, rendzv);
    rendzv->sub_rendezvous = rendzv->nh->subscribe(rendezvous_topic, 1, &Rendezvous::callback_rendezvous, rendzv);

    if(robot1.compare(rendzv->robot_prefix) == 0)
    {
        rendzv->iAm = EXPLORER;
        rendzv->myBuddy_prefix = "/robot_0";
        ROS_DEBUG("%s is explorer of %s", rendzv->robot_prefix.c_str(), rendzv->myBuddy_prefix.c_str());

        rendzv->exploreRobot();
        //rendzv->test_relay_base_station();

    }
    else if(robot0.compare(rendzv->robot_prefix) == 0)
    {
        rendzv->iAm = RELAY;
        rendzv->myBuddy_prefix = "/robot_1";
        ROS_DEBUG("%s is relay of %s", rendzv->robot_prefix.c_str(), rendzv->myBuddy_prefix.c_str());

        rendzv->relayRobot();
        //rendzv->test_relay_base_station();
    }
    else if(robot2.compare(rendzv->robot_prefix) == 0)
    {
        rendzv->iAm = BASE;
        rendzv->myBuddy_prefix = "/robot_0";
        ROS_DEBUG("%s is the base station", rendzv->robot_prefix.c_str());

        //rendzv->relayRobot();
        rendzv->base_station();
    }



// --------------------------------------   for unlimited communication ------------------------------------
//    if(rendzv->iAm == EXPLORER)
//    {
//        std::string topic = rendzv->robot_prefix + "/rendezvous/RendezvousPoints";
//        rendzv->pub = rendzv->nh->advertise<rendezvous::RendezvousPoint>(topic, 10);
//        rendzv->exploreRobot_unlimited();
//    }
//    else if(rendzv->iAm == RELAY)
//    {
//        // subscribe to rendezvous points topic of my explore robot
//        std::string topic_name = rendzv->myBuddy_prefix + std::string("/rendezvous/RendezvousPoints");
//        rendzv->sub = rendzv->nh->subscribe(topic_name, 10, &Rendezvous::new_Rendezvous_available, rendzv);
//        rendzv->relayRobot_unlimited();
//    }
// -----------------------------------------------------------------------------------------------------------

// --------------------------------------  using ad_hoc_communication node -----------------------------------

    //std::string topic = rendzv->robot_prefix + std::string("/rendezvous/checkCommunicationRange");
    //rendzv->sub_hallo = rendzv->nh->subscribe(topic, 1, &Rendezvous::callback_hallo, rendzv);

//    if(rendzv->iAm == EXPLORER)
//    {
//        // join_mc_group of relay
//        rendzv->exploreRobot();
//    }
//    else if(rendzv->iAm == RELAY)
//    {
//        rendzv->relayRobot();
//    }

    //rendzv->test_hallo();

// -----------------------------------------------------------------------------------------------------------

}
