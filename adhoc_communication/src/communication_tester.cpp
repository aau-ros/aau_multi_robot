/*
 * communication_tester.cpp
 *
 *  Created on: 13.07.2013
 *      Author: Günther Cwioro
 *
 * Description:
 * This node demonstrate how to send data with the node adhoc_communication over network.
 * It is also showed how to send any ROS messages over network.
 *
 *
 * Node instruction:
 *
 * PARAMETERS:
 *
 *
 *
 * 		PARAMETER NAME: 		-d {$DESTINATION}
 * 		DESCTIPTION:			The hostname of the destination
 *								NOTE: Use \"\" for broadcasts
 *
 *
 * 		PARAMETER NAME: 		-s {$SOURCE}
 * 		DESCTIPTION:			The hostname of the source
 *								NOTE: Only needed, if the adhoc node is in \"simulation_mode\"
 *
 *
 * 		PARAMETER NAME: 		-c {$COUNT}
 * 		DESCTIPTION:			Diffenes how often the node will send a message
 *								NOTE: If zero, or not set the node will send infinity messages
 *
 * 		PARAMETER NAME: 		-l
 * 		DESCTIPTION:			If set the node will call the get_neighbor service of the adhoc node and will print out the result on the screen
 *
 *
 * 		PARAMETER NAME: 		-n
 * 		DESCTIPTION:			If set, the node will print out changes of the neighbor list in real time
 *
 *
 * 		PARAMETER:				-p {$PREFIX}
 * 		DESCTIPTION:			Defines the prefix the node will use to call services
 *  							NOTE: Only relevant in the simulation mode
 *
 * 		PARAMETER:				-j
 * 		DESCTIPTION:			Will call srv to join mc group.
 *  							NOTE: The name of the mc group will be the value of the -d parameter
 *
 * 		PARAMETER:				-k
 * 		DESCTIPTION:			Defines the size in kilobytes of the data that will be send
 *
 *
 *  EXAMPLES:
 *
 *  	NON SIMULATION:
 *  			Send 5 messages to a certain host:
 *  			adhoc_communication communication_tester -r dest_host -c 5
 *
 *  			Get information about new/lost neighbors in realtime:
 *  			adhoc_communication communication_tester -n
 *

 *
 *  	SIMULATION:
 *  			Send infinite messages from robot_1 to robot_3 a certain host
 *  			adhoc_communication communication_tester -s robot_0 -r robot_3
 *
 *  			Get information about new/lost neighbors in realtime from robot_2:
 *  			adhoc_communication communication_tester -p robot_2 -n
 *
 *  			Print neighbor list on screen from robot_2:
 *  			adhoc_communication communication_tester -p robot_2 -l
 */



#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"



/* Custom MSG Types*/
#include "adhoc_communication/ExpAuction.h"
#include "adhoc_communication/ExpFrontierElement.h"
#include "adhoc_communication/MmListOfPoints.h"
#include "adhoc_communication/MmRobotPosition.h"
#include "adhoc_communication/ExpFrontier.h"
#include "adhoc_communication/MmMapUpdate.h"
#include "adhoc_communication/RecvString.h"
#include "adhoc_communication/MmControl.h"
#include "adhoc_communication/MmPoint.h"
#include "adhoc_communication/ExpCluster.h"

/* Custom SRV Types*/
#include "adhoc_communication/ChangeMCMembership.h"
#include "adhoc_communication/SendExpCluster.h"
#include "adhoc_communication/SendMmMapUpdate.h"
#include "adhoc_communication/SendOccupancyGrid.h"
#include "adhoc_communication/GetNeighbors.h"
#include "adhoc_communication/SendExpFrontier.h"
#include "adhoc_communication/SendMmPoint.h"
#include "adhoc_communication/SendQuaternion.h"
#include "adhoc_communication/SendExpAuction.h"
#include "adhoc_communication/SendMmControl.h"
#include "adhoc_communication/SendMmRobotPosition.h"
#include "adhoc_communication/SendString.h"
#include "adhoc_communication/GetGroupState.h"
#include "functions.h"


#include <cstdlib>

#include <sstream>
#include <netinet/in.h>
#include <list>

void getMessage(const nav_msgs::OccupancyGrid::ConstPtr& map);
void getPosition(const adhoc_communication::MmRobotPosition::ConstPtr& pos);
void getAnyMessage(const adhoc_communication::RecvString::ConstPtr& st);

void joinGroup(std::string receiver);
void sendData(std::string receiver);
void removeRobot(const std_msgs::String::ConstPtr& msg);
void newRobot(const std_msgs::String::ConstPtr& msg);
void callGetNeighborService();
ros::ServiceClient sM;
ros::ServiceClient sG;
ros::ServiceClient sP;
ros::ServiceClient sS;

ros::ServiceClient gN;

ros::ServiceClient sJ;
ros::NodeHandle *n_priv;

std::string prefix = "";
uint32_t size = 1; //kilo byte

int main(int argc, char **argv)
{


    ros::init(argc, argv, "communication_tester");
    ros::NodeHandle nh;
    n_priv = &nh;



    /*GET PARAMETER*/

    std::string topic_new_robot = "new_robot";
    std::string topic_remove_robot = "remove_robot";
    bool simulation_mode = false;
    bool advertise_srv_neighbor = false;
    bool advertise_msg_neighbor = false;
    bool join_group = false;
    int robots_in_simulation = 10;
    int count_send = 0;
    std::string group_name = "";



    n_priv->param("robots_in_simulation", robots_in_simulation, 10);
    n_priv->param("topic_new_robot", topic_new_robot, std::string("new_robot"));
    n_priv->param("topic_remove_robot", topic_remove_robot, std::string("remove_robot"));


    std::string sender = "";
    std::string receiver = "";

    //--s Sender --r Receiver --l true --n true

    bool receiver_set = false;

    for (int i = 1; i < argc; i++)
    {

        if (std::string("-s").compare(argv[i]) == 0)
        {
            simulation_mode = true;
            sender = argv[++i];
        }
        else if (std::string("-d").compare(argv[i]) == 0)
        {
            receiver = argv[++i];
            receiver_set = true;
        }
        else if (std::string("-l").compare(argv[i]) == 0)
            advertise_srv_neighbor = true;
        else if (std::string("-p").compare(argv[i]) == 0)
        {
            simulation_mode = true;
            prefix = "/" + std::string(argv[++i]);
        }
        else if (std::string("-n").compare(argv[i]) == 0)
            advertise_msg_neighbor = true;
        else if (std::string("-c").compare(argv[i]) == 0)
        {
            count_send = atoi(argv[++i]);
        }
        else if (std::string("-j").compare(argv[i]) == 0)
        {
            join_group = true;
        }
        else if (std::string("-g").compare(argv[i]) == 0)
        {
            group_name = argv[++i];
        }
        else if (std::string("-k").compare(argv[i]) == 0)
        {
            size = atoi(argv[++i]);
        }
        else
        {
            ROS_FATAL("UNKNOWN PARAMETER: %s", argv[i]);
            ROS_FATAL("VALID PARAMETERS ARE:");
            ROS_FATAL(" ");


            ROS_FATAL("PARAMETER: 	-d {$DESTINATION}");
            ROS_FATAL("DESCTIPTION:	The hostname of the destination");
            ROS_FATAL("			NOTE: Use \"\" for broadcasts");
            ROS_FATAL(" ");

            ROS_FATAL("PARAMETER: 	-s {$SOURCE}");
            ROS_FATAL("DESCTIPTION:	The hostname of the source");
            ROS_FATAL("			NOTE: Only needed, if the adhoc node is in \"simulation_mode\"");
            ROS_FATAL(" ");

            ROS_FATAL("PARAMETER: 	-c {$COUNT}");

            ROS_FATAL("DESCTIPTION:	Diffenes how often the node will send a message");
            ROS_FATAL("			NOTE: If zero, or not set the node will send infinity messages");
            ROS_FATAL(" ");

            ROS_FATAL("PARAMETER: 	-l");
            ROS_FATAL("DESCTIPTION:	If set the node will call the get_neighbor service of the adhoc node and will print out the result on the screen");
            ROS_FATAL(" ");

            ROS_FATAL("PARAMETER: 	-n");
            ROS_FATAL("DESCTIPTION:	If set, the node will print out changes of the neighbor list in real time");
            ROS_FATAL(" ");

            ROS_FATAL("PARAMETER: 	-p {$PREFIX}");
            ROS_FATAL("DESCTIPTION:	Defines the prefix the node will use to call services");
            ROS_FATAL("			NOTE: Only relevant in the simulation mode");

            ROS_FATAL("PARAMETER: 	-j");
            ROS_FATAL("DESCTIPTION:	Will call srv to join mc group");
            ROS_FATAL("			NOTE: The name of the mc group will be the value of the -d parameter");

            ROS_FATAL("PARAMETER: 	-g");
            ROS_FATAL("DESCTIPTION:	Will give you the information of a certain group");
            ROS_FATAL("	");

            ROS_FATAL("PARAMETER: 	-k");
            ROS_FATAL("DESCTIPTION:	Defines the size in kilobytes of the data that will be send");



            return 0;
        }



    }






    std::string node_prefix = "adhoc_communication/";
ros::spinOnce();
    ros::Subscriber topic1 = n_priv->subscribe(prefix + "/topic_map", 1000, getMessage);
    ros::Subscriber topic2 = n_priv->subscribe(prefix + "/position", 1000, getPosition);
    ros::Subscriber topic3 = n_priv->subscribe(prefix + "/ros_is_cool", 1000, getAnyMessage);
ros::spinOnce();
    ros::Subscriber sub_new_neighbor;
    ros::Subscriber sub_remove_neighbor;

    if (advertise_msg_neighbor)
    {
        sub_new_neighbor = n_priv->subscribe(prefix +"/" +node_prefix + topic_new_robot, 1000, newRobot);
        sub_remove_neighbor = n_priv->subscribe(prefix + "/" +node_prefix + topic_remove_robot, 1000, removeRobot);
        std::string t = prefix + node_prefix+ "/" + topic_new_robot;
        ROS_ERROR("%s", t.c_str());
    }
   ros::spinOnce();





    sM = n_priv->serviceClient<adhoc_communication::SendOccupancyGrid>("/" + sender + "/" + node_prefix + std::string("send_map"));
    ROS_ERROR("%s",("/" + sender + "/" + node_prefix + "get_group_state").c_str());
    sG = n_priv->serviceClient<adhoc_communication::GetGroupState>("/" + sender + "/" + node_prefix + "get_group_state");
    sP = n_priv->serviceClient<adhoc_communication::SendMmRobotPosition>("/" + sender + "/" + node_prefix + "send_position");
    sS = n_priv->serviceClient<adhoc_communication::SendString>("/" + sender + "/" + node_prefix + "send_string");

    sJ = n_priv->serviceClient<adhoc_communication::ChangeMCMembership>("/" + sender + "/" + node_prefix + "join_mc_group");

    if (advertise_srv_neighbor)
        gN = n_priv->serviceClient<adhoc_communication::GetNeighbors>("/" + sender + "/" + node_prefix + "get_neighbors");
    ros::spinOnce();

    if (simulation_mode)
        ROS_INFO("Robot: %s", prefix.c_str());

    if (join_group)
        joinGroup(receiver);

    if (group_name.compare("") != 0)
    {
        adhoc_communication::GetGroupState group_state; 
        group_state.request.group_name = group_name; 
        

       

        if (sG.call(group_state))
        {
            
            ROS_INFO("GROUP NAME: %s",group_state.request.group_name.c_str());
            ROS_INFO("UPLINK: %s",group_state.response.route_uplink.c_str());
            if(!group_state.response.downlinks.empty())
            {
            ROS_INFO("DOWNLINKS:" );
            int count = 0;
            for(std::vector<std::string>::iterator i = group_state.response.downlinks.begin(); i != group_state.response.downlinks.end(); ++i)
            {
                count++;
                std::string mem =*i;
                ROS_INFO("%s:: %s",getIntAsString(count).c_str(), mem.c_str());
                
            }
            }
            else  ROS_INFO("NO DOWNLINKS" );
            ROS_INFO("CONNECTED: [%s] MEMBER[%s] ROOT[%s] ACTIVATED[%s] JOINING[%s] ",getBoolAsString(group_state.response.connected).c_str(),getBoolAsString(group_state.response.member).c_str(),getBoolAsString(group_state.response.root).c_str(),getBoolAsString(group_state.response.activated).c_str(),getBoolAsString(group_state.response.joining).c_str());
           
        }
        else
        {
            ROS_INFO("Failed call service to get group info..");
        }
    }

    int count = 0;
    while (ros::ok())
    {
 	ros::spinOnce();   
        if (!join_group)
        {


            if (count < count_send && receiver_set)
                sendData(receiver);

            if (advertise_srv_neighbor)
                callGetNeighborService();


	            
	    ros::Duration(0.10f).sleep();

            

            if (count_send > 0)
                count++;
        }


    }

    return 0;
}

void getMessage(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    ROS_INFO("GOT MAP: HEIGHT[%i] WIDTH[%i]", map->info.height, map->info.width);
}

void getPosition(const adhoc_communication::MmRobotPosition::ConstPtr& pos)
{
    ROS_INFO("GOT POSITION: SOURCE[%s] X[%f] Y[%f] Z[%f]", pos->src_robot.c_str(), pos->position.pose.position.x, pos->position.pose.position.y, pos->position.pose.position.z);
}

uint32_t msg_got = 0;
void getAnyMessage(const adhoc_communication::RecvString::ConstPtr& st)
{
    /*How to de-serialize a ROS message: */

    ROS_INFO("GOT MSG[%u]: SIZE BYTES[%u]", ++msg_got,(int) st->data.length());
    /*
    geometry_msgs::PointStamped point_stamp_from_network = geometry_msgs::PointStamped(); //create empty message

    ros::serialization::IStream stream((unsigned char*)st->data.data(), st->data.length()); // st->data includes the serialized message
    ros::serialization::deserialize(stream, point_stamp_from_network);
     */
    /*Now the PointStamped message is de-serialized and saved in point_stamp_from_network */
    //ROS_INFO("GOT POINT: SOURCE[%s] X[%f] Y[%f] Z[%f]",st->sourceHost.c_str(), point_stamp_from_network.point.x,point_stamp_from_network.point.y,point_stamp_from_network.point.z);


}

void newRobot(const std_msgs::String::ConstPtr& msg)
{


    ROS_INFO("%s: NOW CONNECTED WITH %s", prefix.c_str(), msg->data.c_str());
}

void removeRobot(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("%s: LOST CONNECTION FROM %s", prefix.c_str(), msg->data.c_str());
}

void sendData(std::string receiver)
{
    /*How to send OCCupancyGrid:*/
    /*
    adhoc_communication::sendOccupancyGrid request_map;
    request_map.request.topic = "topic_map";
    request_map.request.map.info.height = 3;
    request_map.request.map.info.width = 4;
    request_map.request.destinationHost = receiver;



    if (sM.call(request_map))
    {
            ROS_INFO("Successfully call service to send map..");

            if(request_map.response.successfullySend)
                    ROS_INFO("Map has been sent successfully!");
            else
                    ROS_INFO("Failed to send map!");

    }
    else
    {
            ROS_INFO("Failed call service to send map..");
    }
     */

    /*How to send PostStamped:*/
    /*
                              adhoc_communication::sendMmPosition request_pos;
                              request_pos.request.destinationHost =receiver; //;
                              geometry_msgs::PoseStamped ps = geometry_msgs::PoseStamped();
                              request_pos.request.position.pose.position.x = (double)12;
                              request_pos.request.position.pose.position.y = (double)13;
                              request_pos.request.position.pose.position.z = (double)14;

                              request_pos.request.topic = "position";
                              if (sP.call(request_pos))
                              {
                                      ROS_INFO("Successfully call service to send position->.");

                                      if(request_pos.response.successfullySend)
                                              ROS_INFO("Position has been sent successfully!");
                                      else
                                              ROS_INFO("Failed to send position!");

                              }
                              else
                              {
                                      ROS_INFO("Failed call service to send position->.");
                              }



     */

    /*How to send any data.
     * In this case i will send my own serialized ROS message over network.
     * The type of the message will be a geometry_msgs/PointStamped.
     */

    /*Create Message*/

    geometry_msgs::PointStamped point = geometry_msgs::PointStamped();
    point.point.x = 1;
    point.point.y = 2;
    point.point.z = 3;

    /*Serialize Message using ros serialization*/

    uint32_t serial_size = ros::serialization::serializationLength(point);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ros::serialization::OStream streamOut(buffer.get(), serial_size);
    ros::serialization::serialize(streamOut, point);

    std::string serializedObj = "";
    serializedObj.append((const char*) buffer.get(), serial_size);

    /*Now the string 'serializedObj' includes the serialized message */

    adhoc_communication::SendString request_any; // create request of type any+
    request_any.request.dst_robot = receiver; // init destination host
    std::string chars_10 = "ROSisCool!";
    for (uint32_t i = 0; i < size * 100; i++)
    {
        request_any.request.data += chars_10;
    }

    //  request_any.request.data = serializedObj; // set the serialized object as payload of the packet
    /* On the destination node its necessary to listen on the topic and re-serialize the object again->
     * This is explained in the function "getAnyMessage()"
     */

    request_any.request.topic = "ros_is_cool";

    if (sS.call(request_any))
    {
        ROS_INFO("Successfully call service to send serialized message..");

        if (request_any.response.status)
            ROS_INFO("Serialization string has been sent successfully!");
        else
            ROS_INFO("Failed to send serialization string!");
    }
    else
    {
        ROS_INFO("Failed call service to send serialized message..");
    }

}

void callGetNeighborService()
{
    adhoc_communication::GetNeighbors request_neighbors;

    if (gN.call(request_neighbors))
    {
        ROS_INFO("Successfully call service to get neighbors..");
        ROS_INFO("Neigbors:");
        for (std::vector<std::string>::iterator it = request_neighbors.response.neigbors.begin(); it != request_neighbors.response.neigbors.end(); ++it)
        {
            std::string neighbor = *it;
            ROS_INFO("[%s]", neighbor.c_str());
        }


    }
    else
    {
        ROS_INFO("Failed call service to get neighbors..");
    }
    ros::Duration(2.00f).sleep();
    ros::spinOnce();
}

void joinGroup(std::string receiver)
{

    adhoc_communication::ChangeMCMembership join_group;
    join_group.request.action = true;
    join_group.request.group_name = receiver;

    if (sJ.call(join_group))
    {
        ROS_INFO("Successfully call service to join group..");


    }
    else
    {
        ROS_INFO("Failed call service to get neighbors..");
    }
    ros::Duration(2.00f).sleep();
    ros::spinOnce();

}

