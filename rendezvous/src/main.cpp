#include "rendezvous.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rendezvous");
    Rendezvous *rendzv = new Rendezvous();

    boost::thread thr_log(boost::bind(&Rendezvous::log_info, rendzv));

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

    thr_log.interrupt();
    thr_log.join();
}
