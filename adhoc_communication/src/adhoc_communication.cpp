/*
 * ad_hoc_communication.cpp
 *
 *  Created on: 03.07.2013
 *      Author: Günther Cwioro
 *              g.cwioro@gmx.net
 *
 * Description:
 * This node implements a dynamic source routing protocol, which allows you to send data through a network of independent nodes.
 * Its important that the node always run with root rights, because the protocol is implement with raw sockets and without root rights you can't open a raw socket.
 *
 * Node instruction:
 *
 * The node advertise three services:
 * 1) 	name: send_string type: sendString
 * 		This service is to send any data in form from a string over the network.
 * 		Publish type: getData
 *
 * 2)	name: send_map type: sendOccupancyGrid
 * 		This service sends serialized ROS messages
 * 		Publish type: nav_msgs/OccupancyGrid
 *
 * 3)  	name: send_position type: sendPosition
 * 		This service sends serialized ROS messages
 * 		Publish type: positionExchange
 *
 * Every service request has a field for the hostname of the destination and a field for the topic to publish the message.
 * If you want to broadcast a message you must set the destination hostname as an empty string.
 * NOTE: Broadcast don't include acknowledgments and can only be send to direct neighbors and if just one frame of the broadcast packet will be lost over transmission, the whole packet is useless.
 *
 * The node 'communication_tester' of this package demonstrate how you can use the services.
 * There is also a tutorial to adapt this node to implement a service to send your own ROS messages.
 *
 * The node have a lot of parameter and defines that can be changed. There is a list of all parameters and defines in the docu foldere.
 *
 *Special Notes: (not important for users)
 *
 * For debugging reasons there is an option to set the hostname and the mac of the node manually.
 * There is also the opportunity to give the node only a few of reachable hosts.
 * These option can be set when you start the node. Here an example:
 *
 * I want to start a node with hostname "my_hostname" and the mac "84:a6:c8:43:1e:3a". The node should only be able to reach two hosts:
 * neighbor_one 10:00:00:00:00:00
 * neighbor_two 20:00:00:00:00:00
 * To reach this you must start the node like this:
 *
 * rosrun ad_hoc_comm ad_hoc_communication wlan0 my_hostname 84:a6:c8:43:1e:3a neighbor_one 10:00:00:00:00:00 neighbor_two 20:00:00:00:00:00
 *
 * NOTE: If you don't use this feature you can start the node normally and all reachable neighbor will be detected automatically. If you want to use this feature, make sure DEBUG is defined.
 */


#define DEBUG


#include "header.h"

bool finished_exploration;

void shutDown()
{
    ROS_ERROR("shutDown");
    ros::shutdown();
    exit(0);
}

bool shutDownRos(adhoc_communication::ShutDown::Request &req, adhoc_communication::ShutDown::Response &res)
{
    ROS_ERROR("shutDownRos");
    boost::thread d(shutDown);
    return true;
}

bool getNeighbors(adhoc_communication::GetNeighbors::Request &req, adhoc_communication::GetNeighbors::Response & res)
{
    //ROS_ERROR("getNeighbors");
    /* Description:
     * Service call to get a list of all direct connected neighbors of the node
     */
    boost::unique_lock<boost::mutex> lock(mtx_neighbors);
    for (std::list<hostname_mac>::iterator it = neighbors_l.begin(); it != neighbors_l.end(); ++it)
    {
        hostname_mac current_frame = *it;
        string hn = current_frame.hostname;
        //ROS_ERROR("%s", hn.c_str());

        //TODO very bad here...
        //if (current_frame.reachable)
            res.neigbors.push_back(hn);
    }

    return true;
}

bool getGroupStateF(adhoc_communication::GetGroupState::Request &req, adhoc_communication::GetGroupState::Response &res)
{
    ROS_ERROR("getGroupStateF");
    boost::unique_lock<boost::mutex> lock(mtx_mc_groups);
    McTree *t = mc_handler.getMcGroup(&req.group_name);

    if (t != NULL)
    {
        res.activated = t->activated;
        res.connected = t->connected;
        res.root = t->root;
        res.member = t->member;

        for (std::list<mac*>::iterator it = t->downlinks_l_.begin(); it != t->downlinks_l_.end(); ++it)
        {
            mac* m = *it;
            res.downlinks.push_back(getHostnameFromMac(m->mac_adr));
        }

        if (!t->root)
            res.route_uplink = "Next hop:" + getHostnameFromMac(t->route_uplink_->next_hop) + " RD:" + getIntAsString(t->route_uplink_->root_distance) + " HOBS:" + getIntAsString(t->route_uplink_->hobs) + " CH:" + getIntAsString(t->route_uplink_-> current_hop);

        else
            res.route_uplink = "";

        return true;
    }
    else
    {
        res.route_uplink = "No mc entry found!";
    }
}

bool SendEmAuction(adhoc_communication::SendEmAuction::Request &req, adhoc_communication::SendEmAuction::Response &res)
{
    //ROS_ERROR("\n\t\e[1;34mSending auction...\e[0m\n");
    /* Description:
     * Service call to send an auction for energy management.
     */
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif

    ROS_DEBUG("Service called to send energy management auction...");
    string s_msg = getSerializedMessage(req.auction);
    /*Call the function sendPacket and with the serialized object and the frame payload type as parameter*/
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_EM_AUCTION, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendEmAuction", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return true;
}

bool SendEmAuctionResult(adhoc_communication::SendEmAuctionResult::Request &req, adhoc_communication::SendEmAuctionResult::Response &res)
{
    //ROS_ERROR("\n\t\e[1;34mSending auction...\e[0m\n");
    /* Description:
     * Service call to send an auction for energy management.
     */
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif

    ROS_DEBUG("Service called to send energy management auction...");
    string s_msg = getSerializedMessage(req.auction_result);
    /*Call the function sendPacket and with the serialized object and the frame payload type as parameter*/
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_EM_AUCTION, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendEmAuctionResult", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return true;
}

bool SendEmDockingStation(adhoc_communication::SendEmDockingStation::Request &req, adhoc_communication::SendEmDockingStation::Response &res)
{
    /* Description:
     * Service call to send the location and state of a docking station.
     */
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif

    //ROS_DEBUG("Service called to send location and state of a docking station..."); //F
    //ROS_ERROR("Service called to send location and state of a docking station...");
    string s_msg = getSerializedMessage(req.docking_station);
    /*Call the function sendPacket and with the serialized object and the frame payload type as parameter*/
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_EM_DOCKING_STATION, req.topic);
    //ROS_ERROR("%d", res.status); //F

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendEmDockingStation", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return true;
}

bool SendEmRobot(adhoc_communication::SendEmRobot::Request &req, adhoc_communication::SendEmRobot::Response &res)
{
    /* Description:
     * Service call to send the state of a robot.
     */
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif

    ROS_DEBUG("Service called to send state of a robot...");
    string s_msg = getSerializedMessage(req.robot);
    /*Call the function sendPacket and with the serialized object and the frame payload type as parameter*/ //F
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_EM_ROBOT, req.topic);
    //res.status = sendBroadcast(req.topic, s_msg, FRAME_DATA_TYPE_EM_ROBOT, 100);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendEmRobot", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return true;
}

/*tutorial*/
bool sendQuaternion(adhoc_communication::SendQuaternion::Request &req,
        adhoc_communication::SendQuaternion::Response & res)
{
    /* Description:
     * Service call to send QUATERNION.
     */
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif

    ROS_DEBUG("Service called to QUATERNION...");
    string s_msg = getSerializedMessage(req.quaternion);
    /*Call the function sendPacket and with the serialized object and the frame payload type as parameter*/
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_QUATERNION, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendQuaternion", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return true;
}

bool sendMap(adhoc_communication::SendOccupancyGrid::Request &req, adhoc_communication::SendOccupancyGrid::Response & res)
{
    /* Description:
     * Service call to send OccupancyGrid.
     */
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif

    ROS_DEBUG("Service called to send map..");
    string s_msg = getSerializedMessage(req.map);
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_MAP, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendOccupancyGrid", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return res.status;
}

bool sendBroadcast(string& topic, string& data, uint8_t type, uint16_t range)
{
    ROS_ERROR("sendBroadcast");
    MultiHopBroadcastFrame b(topic, data, hostname, type, range);
    string n = b.getFrameAsNetworkString(src_mac);
    if (n.length() > ETHER_MAX_LEN)
    {
        ROS_ERROR("BROADCAST FRAME IS TOO BIG!");
    }
    socketSend(n);
    return true;
}

bool sendBroadcastString(adhoc_communication::BroadcastString::Request &req, adhoc_communication::BroadcastString::Response & res)
{
    ROS_ERROR("sendBroadcastString");
    /* Description:
     * Service call to send OccupancyGrid.
     */
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif
    ROS_DEBUG("Service called to send broadcast string..");

    res.status = sendBroadcast(req.topic, req.data, FRAME_DATA_TYPE_ANY, req.hop_limit);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("BroadcastString", time_call, getMillisecondsTime(), req.data.size(), res.status);
#endif
    return res.status;
}

bool sendBroadcastCMgrRobotUpdate(adhoc_communication::BroadcastCMgrRobotUpdate::Request &req, adhoc_communication::BroadcastCMgrRobotUpdate::Response & res)
{
    ROS_ERROR("sendBroadcastCMgrRobotUpdate");
    /* Description:
     * Service call to send OccupancyGrid.
     */
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif
    ROS_DEBUG("Service called to send broadcast update..");

    string s_msg = getSerializedMessage(req.update);


    res.status = sendBroadcast(req.topic, s_msg, FRAME_DATA_TYPE_ROBOT_UPDATE, req.hop_limit);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("BroadcastCMgrRobotUpdate", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return res.status;
}

bool sendRobotUpdate(adhoc_communication::SendCMgrRobotUpdate::Request &req, adhoc_communication::SendCMgrRobotUpdate::Response & res)
{
    /* Description:
     * Service call to send OccupancyGrid.
     */
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif
    ROS_DEBUG("Service called to send robot update..");

    string s_msg = getSerializedMessage(req.update);
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_ROBOT_UPDATE, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendCMgrRobotUpdate", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return res.status;
}

bool sendTwist(adhoc_communication::SendTwist::Request &req, adhoc_communication::SendTwist::Response & res)
{
    /* Description:
     * Service call to send OccupancyGrid.
     */
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif
    ROS_DEBUG("Service called to send twist..");
    string s_msg = getSerializedMessage(req.twist);

    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_TWIST, req.topic);
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendTwist", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return res.status;
}

bool sendPosition(adhoc_communication::SendMmRobotPosition::Request &req, adhoc_communication::SendMmRobotPosition::Response & res)
{
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif
    /* Description:
     * Service call to send PoseStamped.
     */
    ROS_DEBUG("Service called to send position..");

    string s_msg = getSerializedMessage(req.position);
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_POSITION, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendMmRobotPosition", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return res.status;
}

bool sendAuction(adhoc_communication::SendExpAuction::Request &req, adhoc_communication::SendExpAuction::Response & res)
{
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif
    /* Description:
     * Service call to send Auction.
     */

    ROS_DEBUG("Service called to send Auction..");

    string s_msg = getSerializedMessage(req.auction);
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_AUCTION, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendExpAuction", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return res.status;
}

bool sendCluster(adhoc_communication::SendExpCluster::Request &req, adhoc_communication::SendExpCluster::Response & res)
{
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif
    /* Description:
     * Service call to send Cluster.
     */

    ROS_DEBUG("Service called to send cluster..");

    string s_msg = getSerializedMessage(req.cluster);
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_CLUSTER, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendExpCluster", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return res.status;
}

bool sendString(adhoc_communication::SendString::Request &req, adhoc_communication::SendString::Response & res)
{
    /* Description:
     * Service call to send a string.
     */
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
    if (hostname.compare(req.dst_robot) == 0)
        return false;
#endif
    ROS_DEBUG("Service called to send string..");

    res.status = sendPacket(req.dst_robot, req.data, FRAME_DATA_TYPE_ANY, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS

    Logging::logServiceCalls("SendString", time_call, getMillisecondsTime(), req.data.size(), res.status);
#endif
    return res.status;
}

bool sendPoint(adhoc_communication::SendMmPoint::Request &req, adhoc_communication::SendMmPoint::Response & res)
{
    /* Description:
     * Service call to send PoseStamped.
     */

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif
    ROS_DEBUG("Service called to send point..");

    string s_msg = getSerializedMessage(req.point);
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_POINT, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendMmPoint", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return res.status;
}

bool sendControlMessage(adhoc_communication::SendMmControl::Request &req, adhoc_communication::SendMmControl::Response & res)
{
    /* Description:
     * Service call to send PoseStamped.
     */
    ROS_DEBUG("Service called to send control message..");
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif
    string s_msg = getSerializedMessage(req.msg);
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_CONTROLL_MSG, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendMmControl", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return res.status;
}

bool sendMapUpdate(adhoc_communication::SendMmMapUpdate::Request &req, adhoc_communication::SendMmMapUpdate::Response & res)
{
    /* Description:
     * Service call to send PoseStamped.
     */

    ROS_DEBUG("Service called to send map update..");
#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif
    string s_msg = getSerializedMessage(req.map_update);
    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_MAP_UPDATE, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendMmMapUpdate", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return res.status;
}

bool sendFrontier(adhoc_communication::SendExpFrontier::Request &req, adhoc_communication::SendExpFrontier::Response & res)
{
    /* Description:
     * Service call to send PoseStamped.
     */

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    unsigned long time_call = getMillisecondsTime();
#endif
    string s_msg = getSerializedMessage(req.frontier);
    ROS_DEBUG("Service called to send frontier..");


    res.status = sendPacket(req.dst_robot, s_msg, FRAME_DATA_TYPE_FRONTIER, req.topic);

#ifdef PERFORMANCE_LOGGING_SERVICE_CALLS
    Logging::logServiceCalls("SendExpFrontier", time_call, getMillisecondsTime(), s_msg.size(), res.status);
#endif
    return res.status;
}


bool sendPacket(std::string &hostname_destination, std::string& payload, uint8_t data_type_, std::string& topic)
{
    packet_id_++;
    
    fake_network::SendMessage msg;
    msg.request.source_robot = hostname;
    msg.request.payload = payload;
    msg.request.data_type = data_type_;
    msg.request.topic = topic;
    
    sc_send_message.call(msg);

    return true;
}

bool joinMCGroup(adhoc_communication::ChangeMCMembership::Request &req, adhoc_communication::ChangeMCMembership::Response &res)
{
    ROS_ERROR("joinMCGroup");
    return false;
}



bool publishMessageFromFakeNetwork(fake_network::SendMessage::Request &req, fake_network::SendMessage::Response &res) {
    ROS_ERROR("Called!");
    return true;
}

void publish_topic(const fake_network::NetworkMessage network_msg) {
    //ROS_ERROR("Called!");
    //ROS_ERROR("%s", network_msg.topic.c_str());

    try
    {
        std::string payload = network_msg.payload;
        //ROS_ERROR("PUBLISH PACKET: %04x", network_msg.data_type); //F
        if (network_msg.data_type == FRAME_DATA_TYPE_MAP)
        {
            nav_msgs::OccupancyGrid map;

            desializeObject((unsigned char*) payload.data(),
                    payload.length(), &map);

            publishMessage(map, network_msg.topic);
        }

        if (network_msg.data_type == FRAME_DATA_TYPE_POSITION)
        {

            adhoc_communication::MmRobotPosition pos =
                    adhoc_communication::MmRobotPosition();
            //pos.position = getPoseStampFromNetworkString(
            //		(unsigned char*) payload.data(), payload.length());
            desializeObject((unsigned char*) payload.data(), payload.length(), &pos.position);
            pos.src_robot = network_msg.source_robot;
            publishMessage(pos, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_ANY)
        {
            adhoc_communication::RecvString data = adhoc_communication::RecvString();
            data.src_robot = network_msg.source_robot;
            data.data = payload;
            publishMessage(data, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_POINT)
        {
            adhoc_communication::MmPoint point;
            desializeObject((unsigned char*) payload.data(),
                    payload.length(), &point);
            point.src_robot = network_msg.source_robot;
            publishMessage(point, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_CONTROLL_MSG)
        {
            adhoc_communication::MmControl msg;
            desializeObject(
                    (unsigned char*) payload.data(), payload.length(), &msg);
            msg.src_robot = network_msg.source_robot;
            publishMessage(msg, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_MAP_UPDATE)
        {
            adhoc_communication::MmMapUpdate mapUpdate;
            desializeObject(
                    (unsigned char*) payload.data(), payload.length(), &mapUpdate);
            mapUpdate.src_robot = network_msg.source_robot;
            publishMessage(mapUpdate, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_FRONTIER)
        {
            adhoc_communication::ExpFrontier frontier;
            desializeObject(
                    (unsigned char*) payload.data(), payload.length(), &frontier);

            publishMessage(frontier, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_CLUSTER)
        {
            adhoc_communication::ExpCluster cluster;
            desializeObject(
                    (unsigned char*) payload.data(), payload.length(), &cluster);

            publishMessage(cluster, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_AUCTION)
        {
            adhoc_communication::ExpAuction auc;
            desializeObject(
                    (unsigned char*) payload.data(), payload.length(), &auc);

            publishMessage(auc, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_TWIST)
        {
            geometry_msgs::Twist twist;
            desializeObject(
                    (unsigned char*) payload.data(), payload.length(), &twist);

            publishMessage(twist, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_ROBOT_UPDATE)
        {
            adhoc_communication::CMgrRobotUpdate r_up;
            desializeObject((unsigned char*) payload.data(), payload.length(), &r_up);

            publishMessage(r_up, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_EM_AUCTION)
        {
            adhoc_communication::EmAuction data;
            desializeObject((unsigned char*) payload.data(), payload.length(), &data);

            publishMessage(data, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_EM_AUCTION_RESULT)
        {
            adhoc_communication::EmAuctionResult data;
            desializeObject((unsigned char*) payload.data(), payload.length(), &data);

            publishMessage(data, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_EM_DOCKING_STATION)
        {
            adhoc_communication::EmDockingStation data;
            desializeObject((unsigned char*) payload.data(), payload.length(), &data);

            publishMessage(data, network_msg.topic);
        }
        else if (network_msg.data_type == FRAME_DATA_TYPE_EM_ROBOT)
        {
            adhoc_communication::EmRobot data;
            desializeObject((unsigned char*) payload.data(), payload.length(), &data);

            publishMessage(data, network_msg.topic);
        }
        else
            ROS_ERROR("UNKNOWN DATA TYPE!");

    } catch (int e)
    {

        ROS_ERROR("NODE THROWS EXCEPTION: %u", e);
    }    
}

void finished_exploration_callback(const std_msgs::Empty msg) {
//    ROS_INFO("finalize");
//    finished_exploration = true;
}

int main(int argc, char **argv)
{
    /* Description:
     * Initialize parameters, sockets, hostname and reachable hosts.
     * Start Threads.
     * Check if i got the frame already.
     * If not process the frame.
     */
    ros::init(argc, argv, "adhoc_communication");

    ros::NodeHandle b("~");
    ros::NodeHandle b_pub;

    sc_send_message = b_pub.serviceClient<fake_network::SendMessage>("fake_network/send_message");
    

#ifdef DEBUG
    // ENABLE DEBUGGING OUTPUT
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
       ros::console::notifyLoggerLevelsChanged();
    }
#endif


    n_priv = &b;
    n_pub = &b_pub;

    initParams(n_priv);
    initFrameTypes();

    /*INIT HOSTNAME*/

    if (hostname.compare("") == 0)
    {
        char hostname_c[1024];
        hostname_c[1023] = '\0';
        gethostname(hostname_c, 1023);
        hostname = std::string(hostname_c);
    }

    /*Advertise services and listeners*/
    std::string robot_prefix = "";
    if (simulation_mode)
    {
        robot_prefix = "/" + hostname + "/";
    }

    ros::ServiceServer sendRobotUpdateS = n_pub->advertiseService(robot_prefix + node_prefix + "send_robot_update",
            sendRobotUpdate);

    ros::ServiceServer sendTwistS = n_pub->advertiseService(robot_prefix + node_prefix + "send_twist",
            sendTwist);
    ros::ServiceServer sendMapS = n_pub->advertiseService(robot_prefix + node_prefix + "send_map", sendMap);
    ros::ServiceServer sendPostionS = n_pub->advertiseService(robot_prefix + node_prefix + "send_position", sendPosition);
    ros::ServiceServer sendStringS = n_pub->advertiseService(robot_prefix + node_prefix + "send_string", sendString);
    ros::ServiceServer getNeighborsS = n_pub->advertiseService(robot_prefix + node_prefix + "get_neighbors", getNeighbors);
    ros::ServiceServer sendPointS = n_pub->advertiseService(robot_prefix + node_prefix + "send_point", sendPoint);
    ros::ServiceServer sendMapUpdateS = n_pub->advertiseService(robot_prefix + node_prefix + "send_map_update", sendMapUpdate);
    ros::ServiceServer sendControlMsgS = n_pub->advertiseService(robot_prefix + node_prefix + "send_control_message", sendControlMessage);
    ros::ServiceServer joinMCGroupS = n_pub->advertiseService(robot_prefix + node_prefix + "join_mc_group", joinMCGroup);
    ros::ServiceServer sendFrontierS = n_pub->advertiseService(robot_prefix + node_prefix + "send_frontier", sendFrontier);
    ros::ServiceServer sendClusterS = n_pub->advertiseService(robot_prefix + node_prefix + "send_cluster", sendCluster);
    ros::ServiceServer sendAuctionS = n_pub->advertiseService(robot_prefix + node_prefix + "send_auction", sendAuction);
    ros::ServiceServer sendBcastS = n_pub->advertiseService(robot_prefix + node_prefix + "broadcast_robot_update", sendBroadcastCMgrRobotUpdate);
    ros::ServiceServer sendBcastStringS = n_pub->advertiseService(robot_prefix + node_prefix + "broadcast_string", sendBroadcastString);

    ros::ServiceServer shutDownRosS = n_pub->advertiseService(robot_prefix + node_prefix + "shut_down", shutDownRos);

    ros::ServiceServer getGroupStatusS = n_pub->advertiseService(robot_prefix + node_prefix + "get_group_state", getGroupStateF);
    
    ros::ServiceServer sendEmRobotS = n_pub->advertiseService(node_prefix + "/send_em_robot", SendEmRobot);
    ros::ServiceServer sendEmAuctionS = n_pub->advertiseService(node_prefix + "/send_em_auction", SendEmAuction);
    ros::ServiceServer sendEmAuctionResultS = n_pub->advertiseService(node_prefix + "/send_em_auction_result", SendEmAuctionResult);
    ros::ServiceServer sendEmDockingStationS = n_pub->advertiseService(node_prefix + "/send_em_docking_station", SendEmDockingStation);
        //ROS_ERROR("%s", sendEmDockingStationS.getService().c_str()); //F

    publishers_l.push_front(n_pub->advertise<std_msgs::String>(robot_prefix + node_prefix + topic_new_robot, 1000, true));
    publishers_l.push_front(n_pub->advertise<std_msgs::String>(robot_prefix + node_prefix + topic_remove_robot, 1000, true));
    publishers_l.push_front(n_pub->advertise<adhoc_communication::EmDockingStation>(node_prefix + "/send_em_docking_station", 1000, true));
    publishers_l.push_front(n_pub->advertise<adhoc_communication::EmAuction>(node_prefix + "/send_em_auction", 1000, true));

    //publishers_l.push_front(n_pub->advertise<adhoc_communication::MmControl>("control", 1000, true));
    //publishers_l.push_front(n_pub->advertise<adhoc_communication::MmMapUpdate>("map_other", 1000, true));
    //publishers_l.push_front(n_pub->advertise<adhoc_communication::MmRobotPosition>("position_other_robots ", 1000, true));
    
    
    ros::ServiceServer ss_publish_message = b_pub.advertiseService(robot_prefix + node_prefix + "/publish_message", publishMessageFromFakeNetwork);
    //ROS_ERROR("%s", ss_publish_message.getService().c_str());
    
    ros::Subscriber sub = b_pub.subscribe("adhoc_communication/publish_message_topic", 1000, publish_topic);
    
    signal(SIGSEGV, handler); // install handler
    
    //int *p = 0;    
    //int a = *p;

/*
    if (simulation_mode)
    {
        my_sim_position = new PositionSubscriber();
        my_sim_position->robot_name_ = hostname;
        std::string topic = "/" + hostname;
        topic.append("/base_pose_ground_truth");

        const char* robot_number_pos = hostname.data() + 6; // Example: robot_1 or robot_2 -> robot_i
        try
        {
            my_sim_position->robot_number_ = boost::lexical_cast<uint32_t>(std::string(robot_number_pos));
        } catch (boost::bad_lexical_cast const&)
        {
            ROS_FATAL("If parameter simulation mode is set, the hostname of the robot must be like the robot names of the stage robot names, e.g., robot_0 or robot_1 etc.");
            return 0;
        }

        for (int i = 0; i < robots_in_simulation; i++)
        {
            if ((unsigned) i != my_sim_position->robot_number_)
            {
                std::string i_as_str = getIntAsString(i);
                std::string topic_to_sub = "/robot_";
                topic_to_sub.append(i_as_str);

                topic_to_sub.append("/base_pose_ground_truth");
                PositionSubscriber* sub = new PositionSubscriber();
                sub->robot_name_ = std::string("robot_").append(i_as_str);
                sub->robot_number_ = i;
                sub_robot_pos_l.push_front(n_pub->subscribe(topic_to_sub, 1, &PositionSubscriber::Subscribe, &*sub));
                robot_positions_l.push_front(sub);
            }
            else
                sub_robot_pos_l.push_front(n_pub->subscribe(topic, 1, &PositionSubscriber::Subscribe, &*my_sim_position));
        }
    }
*/

    /* Create own mc group */
    /*
    {
        ROS_INFO("Create own group! %s ", std::string("mc").append("_").append(hostname).c_str());
        boost::unique_lock<boost::mutex> lock(mtx_mc_groups);
        mc_handler.createGroupAsRoot(&std::string("mc").append("_").append(hostname));
    }
    */

    // testPacket();
    
    ros::NodeHandle nh;
    ros::Subscriber sub_wait = nh.subscribe("finished_exploration", 10, finished_exploration_callback);
    
    finished_exploration = false;

    while (ros::ok() && !finished_exploration)
    {
        ros::spinOnce();
    }
    
    ros::Duration(3).sleep();
    ROS_INFO("Node terminating...");
    ros::shutdown();

    return 0;
}

template<class message>
void publishMessage(message m, string topic)
{
    //ROS_ERROR("e[1;34mPublishing a message...\e[0m"); //F
    /* Description:
     * Publish any ROS message on a given topic.
     * Maximal different topics to publish are limited by MAX_DIFFERENT_TOPICS -> defines.h
     */

    try
    {
        bool pubExsists = false;
        std::string topic_w_prefix = "/" + hostname + "/" + topic;
        if (simulation_mode)
            topic = topic_w_prefix;
        // ROS_ERROR("\n\e[1;34mRequested topic: %s\e[0m", topic.c_str());   //F
        for (std::list<ros::Publisher>::iterator i = publishers_l.begin(); i != publishers_l.end(); i++)
        {
            //ROS_ERROR("\n\e[1;34mTopic: %s\e[0m", (*i).getTopic().c_str()); //F
            if ((*i).getTopic().compare(topic) == 0)
            {
                //ROS_ERROR("\n\e[1;34mPublishing on topic: %s\e[0m", (*i).getTopic().c_str()); //F
                (*i).publish(m);
                pubExsists = true;
            }
        }
        if (!pubExsists)
        {
            publishers_l.push_front(n_pub->advertise<message>(topic, 1000, true));
            publishers_l.front().publish(m);
            //ROS_ERROR("\n\e[1;34mPublishing on topic: %s\e[0m", publishers_l.front().getTopic().c_str()); //F
            //todo check if ledge = true is working
        }
    }
    catch (...)
    {
        ROS_ERROR("IN PUBLISH MESSAGE: NODE THROWS EXCEPTION!");
    }
}

template<class message>
void sendMessage(message m, string topic) {
    ;
}



void publishPacket(Packet * p)
{

    /* Description:
     * Simply publish a complete packet
     */



    /* check if packet has already been published*/
    stc_packet pack(p->hostname_source_, p->mc_group_, p->id_);
    unsigned long now = getMillisecondsTime();
    for (list<stc_packet>::iterator it = published_packets_l.begin(); it != published_packets_l.end();)
    {

        if (pack == *it)
        {
            //        ROS_ERROR("DONT PUBLISH PACKET: ID[%u] SRC[%s/%s]", pack.id, pack.src.c_str(), pack.mc_group.c_str());
            return;
        }
        else if (now - (*it).ts >= MAX_TIME_CACHE_PUBLISHED_PACKETS)
            it = published_packets_l.erase(it);
        else
            it++;
    }


    published_packets_l.push_back(pack);

    /* Check if the robot is member of the mc group*/
    bool mc_member = false;
    {
        boost::unique_lock<boost::mutex> lock(mtx_mc_groups);
        McTree* group = mc_handler.getMcGroup(&p->mc_group_);
        if (group != NULL && group->member)
            mc_member = true;
    }

    ROS_INFO("PUBLISH PACKET: ID[%u] SRC[%s/%s]", pack.id, pack.src.c_str(), pack.mc_group.c_str());

    if (p->mc_group_.compare("") == 0 || mc_member)
    {
        //  ROS_ERROR("GOT PACKET: ID[%u] SOURCE[%s] SIZE[%u]", p->id_, p->hostname_source_.c_str(), p->size_);
        try
        {
            std::string payload = p->getPayload();
            //ROS_ERROR("PUBLISH PACKET: %04x", p->data_type_); //F
            if (p->data_type_ == FRAME_DATA_TYPE_MAP)
            {
                nav_msgs::OccupancyGrid map;

                desializeObject((unsigned char*) payload.data(),
                        payload.length(), &map);

                publishMessage(map, p->topic_);
            }

            if (p->data_type_ == FRAME_DATA_TYPE_POSITION)
            {

                adhoc_communication::MmRobotPosition pos =
                        adhoc_communication::MmRobotPosition();
                //pos.position = getPoseStampFromNetworkString(
                //		(unsigned char*) payload.data(), payload.length());
                desializeObject((unsigned char*) payload.data(), payload.length(), &pos.position);
                pos.src_robot = p->hostname_source_;
                publishMessage(pos, p->topic_);
            }
            else if (p->data_type_ == FRAME_DATA_TYPE_ANY)
            {
                adhoc_communication::RecvString data = adhoc_communication::RecvString();
                data.src_robot = p->hostname_source_;
                data.data = payload;
                publishMessage(data, p->topic_);
            }
            else if (p->data_type_ == FRAME_DATA_TYPE_POINT)
            {
                adhoc_communication::MmPoint point;
                desializeObject((unsigned char*) payload.data(),
                        payload.length(), &point);
                point.src_robot = p->hostname_source_;
                publishMessage(point, p->topic_);
            }
            else if (p->data_type_ == FRAME_DATA_TYPE_CONTROLL_MSG)
            {
                adhoc_communication::MmControl msg;
                desializeObject(
                        (unsigned char*) payload.data(), payload.length(), &msg);
                msg.src_robot = p->hostname_source_;
                publishMessage(msg, p->topic_);
            }
            else if (p->data_type_ == FRAME_DATA_TYPE_MAP_UPDATE)
            {
                adhoc_communication::MmMapUpdate mapUpdate;
                desializeObject(
                        (unsigned char*) payload.data(), payload.length(), &mapUpdate);
                mapUpdate.src_robot = p->hostname_source_;
                publishMessage(mapUpdate, p->topic_);
            }
            else if (p->data_type_ == FRAME_DATA_TYPE_FRONTIER)
            {
                adhoc_communication::ExpFrontier frontier;
                desializeObject(
                        (unsigned char*) payload.data(), payload.length(), &frontier);

                publishMessage(frontier, p->topic_);
            }
            else if (p->data_type_ == FRAME_DATA_TYPE_CLUSTER)
            {
                adhoc_communication::ExpCluster cluster;
                desializeObject(
                        (unsigned char*) payload.data(), payload.length(), &cluster);

                publishMessage(cluster, p->topic_);
            }
            else if (p->data_type_ == FRAME_DATA_TYPE_AUCTION)
            {
                adhoc_communication::ExpAuction auc;
                desializeObject(
                        (unsigned char*) payload.data(), payload.length(), &auc);

                publishMessage(auc, p->topic_);
            }
            else if (p->data_type_ == FRAME_DATA_TYPE_TWIST)
            {
                geometry_msgs::Twist twist;
                desializeObject(
                        (unsigned char*) payload.data(), payload.length(), &twist);

                publishMessage(twist, p->topic_);
            }
            else if (p->data_type_ == FRAME_DATA_TYPE_ROBOT_UPDATE)
            {
                adhoc_communication::CMgrRobotUpdate r_up;
                desializeObject((unsigned char*) payload.data(), payload.length(), &r_up);

                publishMessage(r_up, p->topic_);
            }
            else if (p->data_type_ == FRAME_DATA_TYPE_EM_AUCTION)
            {
                adhoc_communication::EmAuction data;
                desializeObject((unsigned char*) payload.data(), payload.length(), &data);

                publishMessage(data, p->topic_);
            }
            else if (p->data_type_ == FRAME_DATA_TYPE_EM_DOCKING_STATION)
            {
                adhoc_communication::EmDockingStation data;
                desializeObject((unsigned char*) payload.data(), payload.length(), &data);

                publishMessage(data, p->topic_);
            }
            else if (p->data_type_ == FRAME_DATA_TYPE_EM_ROBOT)
            {
                adhoc_communication::EmRobot data;
                desializeObject((unsigned char*) payload.data(), payload.length(), &data);

                publishMessage(data, p->topic_);
            }
            else
                ROS_ERROR("UNKNOWN DATA TYPE!");

        } catch (int e)
        {

            ROS_ERROR("NODE THROWS EXCEPTION: %u", e);
        }
        /*Tutorial*/
        /*
        else if(p->data_type_ == FRAME_DATA_TYPE_QUATERNION)
        {

        geometry_msgs::Quaternion quaternion = getQuaternionFromNetworkString((unsigned char*)payload.data(),payload.length());
        publishMessage(quaternion, p->topic_);
        }
         */
    }


}

