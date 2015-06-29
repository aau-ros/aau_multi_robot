/*!
 * \file adhoc_communication.cpp
 * \author Günther Cwioro
 * \brief Main file for the ad hoc communication node including the main function.
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


#include "header.h"

bool getGroupStateF(adhoc_communication::GetGroupState::Request &req, adhoc_communication::GetGroupState::Response &res)
{
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


/*!
 * \brief Shuts down the rode node.
 */
void shutDown()
{
    ros::shutdown();
    exit(0);
}

bool shutDownRos(adhoc_communication::ShutDown::Request &req, adhoc_communication::ShutDown::Response &res)
{
    boost::thread d(shutDown);
    return true;
}

/*!
 * \brief Service to tell the node to attempt to connect to a multicast group.
 * \param [in] req The service request object.
 * \param [out] res The service response object.
 *
 * Implements the ROS service join_mc_group to tell a node to connect to a multicast group.
 */
bool joinMCGroup(adhoc_communication::ChangeMCMembership::Request &req, adhoc_communication::ChangeMCMembership::Response &res)
{
    /* Description:
     * Service call join a mc group
     */
    if (req.action)
    {
        ROS_INFO("Try to join group: %s", req.group_name.c_str());
        /* Check if i am already a node in the tree */
        res.status = false;

        boost::unique_lock<boost::mutex> lock_mc_groups(mtx_mc_groups);
        McTree *t = mc_handler.getMcGroup(&req.group_name);

        if (t != NULL && t->connected && t->activated)
        {

            t->member = true;
            res.status = true;
            //ROS_DEBUG("CHANGE MC STATUS IN GROUP [%s] FROM CONNECTOR TO MEMBER ", req.group_name.c_str());
            return true;
        }
        else if (t != NULL && t->outgoing_request_ != NULL && getMillisecondsTime() - t->time_stamp_ < INTERVAL_WAIT_FOR_MCROUTES)
        {
            /* In this case the join request has been invoked by another node and is now running */
            lock_mc_groups.unlock();

            sleepMS(INTERVAL_WAIT_FOR_MCROUTES - (getMillisecondsTime() - t->time_stamp_));

            sleepMS(INTERVAL_WAIT_FOR_MCROUTES * 2);
            lock_mc_groups.lock();

            t = mc_handler.getMcGroup(&req.group_name);

            if (t->activated)
            {

                res.status = true;
                t->member = true;

                return true;
            }
            else
            {
                t->resetTmpFields();
                res.status = false;
            }
            /* look if request was successfully*/
        }
        else if (t != NULL && (!t->connected || !t->activated))
        {
            t->resetTmpFields();
            res.status = false;
        }
        else if (t != NULL)
            t->printTree();

        if (t == NULL || (t != NULL && t->outgoing_request_ == NULL))
        {
            mc_handler.addGroup(&req.group_name);
            {
                boost::unique_lock<boost::mutex> lock(mtx_routing_table);
                Logging::logRoutingTable(&routing_table_l, &mc_groups_l);
            }

            uint8_t join_attempts = 0;

            /* create route request for logging*/
            route_request rreq_logging;
            rreq_logging.id = RouteRequest::req_count_stat;
            rreq_logging.hostname_source = hostname;
            rreq_logging.is_mc = true;
            rreq_logging.hostname_destination = req.group_name;

            while (join_attempts < MAX_INTERN_JOIN_ATTEMPS && res.status == false)
            {
                RouteRequest* rr = new RouteRequest(hostname, req.group_name, hop_limit_min, true);

                /*SAFE route request*/
                route_request rreq_out;
                rreq_out.id = rr->header_.id;
                rreq_out.hop_limit = hop_limit_min;
                rreq_out.hostname_source = hostname;
                rreq_out.is_mc = true;
                rreq_out.hostname_destination = req.group_name;

                {
                    lock_mc_groups.unlock();
                    boost::unique_lock<boost::mutex> lock(mtx_route_requests);
                    route_requests_l.push_front(rreq_out);
                    lock_mc_groups.lock();
                }

                /*SEND route request*/
                routing_entry route;

                ROS_INFO("SEND JOIN REQUEST: HOST[%s] ID[%u]", req.group_name.c_str(), rr->header_.id);

                t = mc_handler.getMcGroup(&req.group_name);

                t->safeOutgoingRequest(rr);
                string network_string = rr->getRequestAsNetworkString(src_mac);
                socketSend(network_string);

                lock_mc_groups.unlock();
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
                Logging::increaseProperty("num_mc_rrep_sent");
                Logging::increaseProperty("num_mc_total_bytes_sent ", network_string.length());
#endif
                sleepMS(INTERVAL_WAIT_FOR_MCROUTES);
                lock_mc_groups.lock();

                res.status = !t->routing_entries_l_.empty() || (t->activated && t->connected);

                rreq_logging.retransmitted = join_attempts;
                join_attempts++;
            }


            if (res.status && t->activateBestRoute(&rreq_logging))
            {
                t->member = true;
                t->connected = true;
                t->activated = true;

                /* Activate route*/
                McRouteActivationFrame r_act_frame = McRouteActivationFrame(t->route_uplink_->next_hop, t->group_name_, t->route_uplink_->id, t->route_uplink_->hostname_source);
                string network_string = r_act_frame.getFrameAsNetworkString(src_mac);

                resendUnackLinkFrame("", r_act_frame.header_.id, r_act_frame.header_.mac_destination, network_string, FRAME_TYPE_MC_ACTIVATION);

                ROS_INFO("ACTIVATE MC ROUTE: MC GROUP[%s] NEXT HOP[%s]", t->group_name_.c_str(), getMacAsStr(t->route_uplink_->next_hop).c_str());
                ROS_INFO("SUCCESSFULLY JOINED GROUP MC: [%s]", req.group_name.c_str());

                socketSend(network_string);
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
                Logging::increaseProperty("num_mc_ract_sent");
                Logging::increaseProperty("num_mc_total_bytes_sent ", network_string.length());
#endif

                res.status = true;

                /* Send response to waiting requests */
                while (!t->waiting_requests_l_.empty())
                {

                    RouteRequest *req = t->waiting_requests_l_.front();
                    t->waiting_requests_l_.pop_front();
                    lock_mc_groups.unlock();
                    processRouteRequest(*req);
                    lock_mc_groups.lock();

                    delete req;
                }

                return true;
            }
            else
            {
                ROS_WARN("COULD NOT JOIN GROUP MC: [%s]", req.group_name.c_str());
            }
        }
        else
        {
            ROS_WARN("UNEXPECTED FAILURE: 2345 %s", req.group_name.c_str());
        }
    }
    else
    {
        /*disconnect from mc group*/
        boost::unique_lock<boost::mutex> lock_mc_group(mtx_mc_groups);
        McTree* t = mc_handler.getMcGroup(&req.group_name);

        if (t != NULL)
        {
            if (t->downlinks_l_.empty())
            {
                if (t->root)
                {
                    ROS_ERROR("CANNOT DISCONNECT FROM OWN GROUP: [%s]", req.group_name.c_str());
                    return true;
                }
                ROS_WARN("DISCONNECT FROM MC TREE: MC GROUP[%s] NEXT HOP[%s]", t->group_name_.c_str(), getMacAsStr(t->route_uplink_->next_hop).c_str());

                McDisconnectFrame disc_frame(t->route_uplink_->next_hop, req.group_name);
                disc_frame.disconnect_uplink = true;


                string network_string = disc_frame.getFrameAsNetworkString(src_mac);
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY

                Logging::increaseProperty("num_mc_total_frames_sent");
                Logging::increaseProperty("num_mc_total_bytes_sent ", network_string.length());

#endif
                /*SAFE the mc disconnect frame as unacknowledged*/
                resendUnackLinkFrame("", disc_frame.header_.id, disc_frame.header_.mac_destination, network_string, FRAME_TYPE_MC_DISCONNECT);

                socketSend(network_string);

                /*erease mc tree*/
                mc_handler.removeGroup(&req.group_name);
            }
            else
            {
                t->member = false;
                ROS_INFO("SET MEMBERSHIP TO FALSE: GROUP[%s]", t->group_name_.c_str());
            }
        }
        else
        {
            ROS_INFO("NOT A MC MEMBER!");
        }
    }

    return false;
}

bool getNeighbors(adhoc_communication::GetNeighbors::Request &req, adhoc_communication::GetNeighbors::Response & res)
{
    /* Description:
     * Service call to get a list of all direct connected neighbors of the node
     */
    boost::unique_lock<boost::mutex> lock(mtx_neighbors);
    for (std::list<hostname_mac>::iterator it = neighbors_l.begin(); it != neighbors_l.end(); ++it)
    {
        hostname_mac current_frame = *it;
        string hn = current_frame.hostname;

        if (current_frame.reachable)
            res.neigbors.push_back(hn);
    }

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

bool sendRouteRequest(string* dst, routing_entry* route)
{
    RouteRequest rr = RouteRequest(hostname, *dst, hop_limit_min, false);

    /*SAFE route request*/
    route_request out_req;
    out_req.id = rr.header_.id;
    out_req.hop_limit = hop_limit_min;
    out_req.is_mc = false;
    out_req.hostname_source = hostname;
    out_req.ts = getMillisecondsTime();
    out_req.retransmitted = 0;
    out_req.hostname_destination = *dst;
    out_req.response_sent = false;
    {
        boost::unique_lock<boost::mutex> lock(mtx_route_requests);
        route_requests_l.push_front(out_req);
    }

    /*SEND route request*/
    ROS_INFO("SEND ROUTE REQUEST: HOST[%s] ID[%u]", dst->c_str(), rr.header_.id);
    string network_string = rr.getRequestAsNetworkString(src_mac);
    socketSend(network_string);
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY    
    Logging::increaseProperty("num_rreq_sent");
    Logging::increaseProperty("num_total_bytes_sent", network_string.length());
#endif

    // Wait for the route response
    {
        boost::unique_lock<boost::mutex> lock(mtx_notify);
        /*Start thread to re-send requests.*/
        boost::thread resendRouteRequestT(resendRouteRequest, boost::ref(out_req), boost::ref(got_request_response));

        got_request_response.wait(lock); //wait for response
        resendRouteRequestT.interrupt();
    }

    /*CHECK if the request was successfully*/
    bool route_found = getRoute(*route, *dst);

    /* LOGGING rreq_sender.csv*/
    if (route_found)
        Logging::logRRequestInitiater(&out_req, route);
    else
        Logging::logRRequestInitiater(&out_req, NULL);

    return route_found;
}

bool gotAckRoutedFrame(routing_entry& route)
{
    unsigned long start_time = getMillisecondsTime();
    {
        boost::unique_lock<boost::mutex> lockAckRF(mtx_notify);
        boost::thread resendRoutedFrameT(resendRoutedFrame, boost::ref(got_rframe_ack), route);
        got_rframe_ack.wait(lockAckRF); //wait for ack
        resendRoutedFrameT.interrupt();
    }
    unsigned long end_time = getMillisecondsTime();

    boost::unique_lock<boost::mutex> loc(mtx_unack_routed_frame);
    if (unack_routed_frame->frame_is_ack)
    {
        //   ROS_ERROR("SUCCESSFULLY SENT ROUTED FRAME: ID[%u] DESTINATION HOST[%s]", unack_routed_frame->frame.header_.frame_id, route.hostname_destination.c_str());
    }
    else
    {
        ROS_WARN("Could not sent frame: ID[%u] destination host[%s]", unack_routed_frame->frame.header_.packet_id, route.hostname_destination.c_str());
        {
            loc.unlock();
            boost::unique_lock<boost::mutex> loc_rt(mtx_routing_table);
            routing_table_l.remove(route);
        }

        loc.lock();
    }
    Logging::logTransportFrame(unack_routed_frame, &route, start_time, end_time, unack_routed_frame->frame_is_ack);
    return unack_routed_frame->frame_is_ack;
}

bool sendPacket(std::string &hostname_destination, std::string& payload, uint8_t data_type_, std::string& topic)
{
    packet_id_++;

    /* Description:
     * Send a routed frame.
     * Behavior of the function is documented within the function.
     *
     * Returns:
     * 		(true) if sending was successfully/
     *
     * 		(false) if frames has been lost
     *
     * 		NOTE: Broadcasts are always true.
     */

    // SVN version before changes: 7840

    // check if I'm the destination
    if (hostname_destination.compare(hostname) == 0)
    {
        ROS_WARN("Robot cannot send data to itself!");
        return false;
    }

    /* broadcast: empty hostname_destination
     * multicast: an entry for hostname_destination is in mc_handler, i.e., this host joined the mc_group hostname_destination
     * unicast: otherwise
     */
    bool bcast = (hostname_destination.compare("") == 0); /// is broadcast
    bool send_successfully;
    bool mc_frame = false; /// is multicast
    routing_entry route;

    if (!bcast)
    {
        ROS_DEBUG("DESTINATION: [%s]", hostname_destination.c_str());
        /* Try to find unicast route */
        send_successfully = getRoute(route, hostname_destination);

        if (!send_successfully)
        {
            /* Try to find mc route */
            boost::unique_lock<boost::mutex> lock_mc(mtx_mc_groups);
            McTree* mc_t = mc_handler.getMcGroup(&hostname_destination);
            if (mc_t != NULL && mc_t->connected && mc_t->activated)
            {

                ROS_DEBUG("DESTINATION IS A MC GROUP: [%s]", mc_t->group_name_.c_str());
                mc_frame = true;
                send_successfully = true;
                bcast = true;
                memcpy(route.next_hop, bcast_mac, 6);
                route.hostname_destination = hostname_destination;
                route.hostname_source = hostname;

                if (mc_t->root && mc_t->downlinks_l_.size() == 0)
                {
                    ROS_DEBUG("THERE ARE NO OTHER MEMBERS IN THE MC GROUP [%s]", mc_t->group_name_.c_str());
                    return true;
                }
            }
        }

        if (!send_successfully)
        {
            send_successfully = sendRouteRequest(&hostname_destination, &route);
        }
    }
    else
    {
        /*BROADCAST*/
        memcpy(route.next_hop, bcast_mac, 6);
        route.hostname_destination = "";
        route.hostname_source = hostname;
        send_successfully = true;
    }

    if (!send_successfully)
    {
        ROS_WARN("NO ROUTED FOUND TO [%s]", hostname_destination.c_str());
        return false;
    }

    /*PREPARE SENDING IF ROUTE WAS FOUND*/

    send_successfully = false;
    uint32_t header_len = RoutedFrame::HEADER_FIXED_LEN;
    header_len += hostname.length();
    header_len += topic.length();
    header_len += mc_frame ? route.hostname_destination.length() : 0; //if multicast


    uint32_t payload_len = payload.length();
    uint32_t payload_space = (max_frame_size - header_len); //NOTE: payload space must be at least 1 to send data over network
    /*Calculate amount of needed frames*/
    float frames_count_f = payload_len / (float) payload_space; //Amount of frames with no payload
    uint32_t frames_count = frames_count_f; // get integer of the float (if frames_count_f is 9.34, frames_count is 9.00 )
    if (frames_count_f > frames_count) // example: if frames_count_f is 9.34 i need 10 frames
        frames_count++;

    uint32_t packet_size = payload_len + (frames_count * header_len);

    ROS_DEBUG("PACKET INFO: PL[%u] PL SPACE[%u] HEADER LEN[%u] SIZE[%u]", payload_len, payload_space, header_len, frames_count);
    /*Check if packet is not too large and the frame have also space for payload data*/
    if (packet_size >= (unsigned) max_packet_size)
    {
        ROS_ERROR("Packet[%uBYTES] is too big! Try to increase the 'max_packet_size' parameter [%uBYTES]", packet_size, max_packet_size);
        return false;
    }
    else if (header_len >= (unsigned) max_frame_size)
    {
        ROS_ERROR("Header so long that there is no space for payload! Try to increase the 'max_frame_size' parameter!");
        return false;
    }

    /*SEGMENTATION*/
    Packet packet;
    list<RoutedFrame> frames;
    //list<string> network_strings;
    int pos = 0;

    /* create frame list and network string list */
    /*  this is needed for multicast if a node will request a frame which has not been sent yet */
    for (unsigned int i = 0; i < frames_count; i++)
    {
        std::string p(payload, pos, payload_space);
        pos += payload_space;
        RoutedFrame f = RoutedFrame(topic, p, data_type_, packet_id_, i, frames_count);

        f.mc_flag = mc_frame;
        f.negative_ack_type = frames_count > Packet::NACK_THRESHOLD;
        f.mc_g_name_ = mc_frame ? route.hostname_destination : "";
        f.hostname_source_ = hostname;

        /* re-init packet*/
        if (i == 0)
            packet = Packet(f);

        if (f.negative_ack_type)
            packet.frames_l_.push_back(f);
        frames.push_back(f);

    }


    if (packet.isNack())
    {
        boost::unique_lock<boost::mutex> lock(mtx_cached_mc_packets);
        packet.ts_ = getMillisecondsTime() + packet.size_ * 10; // fake time stamp to prevent that the packet would not be too early deleted
        packet.hostname_source_ = hostname;

        cached_mc_packets_l.push_front(packet);
    }

    bool cannot_send_frame = false;

    for (unsigned int i = 0; i < frames_count && cannot_send_frame == false; i++)
    {
        RoutedFrame f = frames.front();
        string network_string = f.getFrameAsNetworkString(route, src_mac);

        /* POS ACK MC*/
        if (packet.isMcFrame() && !packet.isNack())
        {
            McPosAckObj* ack_obj = NULL;
            {
                /* safe an McPosAckObj before sending, because after sending it might be too late, because the acknowledgment of the receiver could already be here
                   the same for the unicast*/
                boost::unique_lock<boost::mutex> lock_mc(mtx_mc_groups);
                McTree* mc_t = mc_handler.getMcGroup(&hostname_destination);
                ack_obj = new McPosAckObj(&f, mc_t);
            }
            {
                boost::unique_lock<boost::mutex> lock(mtx_mc_ack_l);
                mc_ack_l.push_back(ack_obj);
            }
        } /* UNICAST */
        else if (!packet.isMcFrame())
        {
            /*SAFE routed frame as unacknowledged*/
            {
                boost::unique_lock<boost::mutex> lock(mtx_unack_routed_frame);

                unack_routed_frame->frame = f;
                unack_routed_frame->frame_is_ack = false;
                unack_routed_frame->time_stamp = getMillisecondsTime();
                unack_routed_frame->retransmitted = 0;
                memcpy(unack_routed_frame->frame.header_.mac_destination_, route.next_hop, 6);
                unack_routed_frame->hostname_destination = hostname_destination;
                memcpy(unack_routed_frame->mac, f.header_.mac_destination_, 6);

            }
            // if (route.hobs > 2)
            {
                /*SAFE link frame as unacknowledged*/

                resendUnackLinkFrame(hostname, f.header_.frame_id, route.next_hop, network_string, FRAME_TYPE_TRANSPORT_DATA);
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
                Logging::increaseProperty("num_unique_data_frames_sent");

                Logging::increaseProperty("num_total_bytes_sent", network_string.length());
#endif
            }
        }

        /* Wait a time for every nack mc frame to prevent network jam */
        if (packet.isNack() && f.header_.packet_sequence_num % 10 == 0)//todo 
            sleepMS(5);

        socketSend(network_string);
        //ROS_INFO("SEND ROUTED FRAME: ID[%u] DESTINATION HOST[%s] NEXT HOP[%s] ROUTE ID[%u]", f.header_.frame_id, route.hostname_destination.c_str(), getMacAsStr(route.next_hop).c_str(), route.id);
#ifdef DELAY
        if (simulation_mode)
            boost::this_thread::sleep(boost::posix_time::milliseconds(delay)); //usleep(delay);
#endif     


#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
        if (f.mc_flag)
        {
            Logging::increaseProperty("num_mc_unique_data_frames_sent");
            Logging::increaseProperty("num_mc_total_bytes_sent ", network_string.length());
        }
#endif

        /* POS_ACK_MULTICAST */
        if (packet.isMcFrame() && !packet.isNack())
        {
            stc_RoutedFrame* stc_rf = new stc_RoutedFrame();
            stc_rf->frame = f;
            stc_rf->frame_is_ack = false;
            stc_rf->hostname_destination = hostname_destination;
            stc_rf->retransmitted = 0;
            stc_rf->time_stamp = getMillisecondsTime();

            /* check if frame is already acknowledged*/
            McPosAckObj* mc_ack = getMcAckObj(&f.mc_g_name_, f.header_.packet_id, f.header_.packet_sequence_num);
            if (mc_ack != NULL && mc_ack->missing_acks_l.empty())
            {
                stc_rf->frame_is_ack = true;
            }
            else
            {
                boost::thread resendMcAckFrameT(deliverMcAckFrame, boost::ref(*stc_rf), boost::ref(route), false);
                resendMcAckFrameT.join();
            }


            if (stc_rf->frame_is_ack)
            {
                send_successfully = true;
            }
            else
            {
                ROS_WARN("Could not sent pos ack packet with ID[%u] to group[%s]", stc_rf->frame.header_.packet_id, hostname_destination.c_str());

                return false;
            }
            delete stc_rf;
        }

        if (!bcast)
        {
            if (!gotAckRoutedFrame(route))
            {
                /* try to find another route*/
                routing_entry new_route;
                if (sendRouteRequest(&hostname_destination, &new_route))
                {
                    if (new_route.samePath(route) == false)
                    {
                        route = routing_entry(new_route);
                        i--;
                        continue;
                    }

                }

                ROS_WARN("Could not sent unicast packet: ID[%u] destination host[%s]", unack_routed_frame->frame.header_.packet_id, route.hostname_destination.c_str());
                return false;
            }
            else
                send_successfully = true;
        }

        frames.pop_front();
    }
    if (!bcast)
        ROS_INFO("SUCCESSFULLY SEND PACKET: ID[%u] DEST[%s]", packet_id_, hostname_destination.c_str());
    else if (packet.isMcFrame())
    {
        send_successfully = true;
        if (packet.isNack())
        {
            ROS_INFO("SUCCESSFULLY SEND MC NEGATIVE ACK PACKET: ID[%u] GROUP[%s]", packet_id_, hostname_destination.c_str());
        }
        else
        {
            ROS_INFO("SUCCESSFULLY SEND MC POSITIVE ACK PACKET: ID[%u] GROUP[%s]", packet_id_, hostname_destination.c_str());
        }
    }
    else
    {
        send_successfully = true;
        ROS_INFO("SEND BROADCAST PACKET: ID[%u]", packet_id_);
    }

    return send_successfully;
}

void checkMutexAvailability(boost::mutex* m, string name)
{
    while (ros::ok())
    {
        sleepMS(100);

        if (!mtx_packets_incomplete.try_lock())
            ROS_ERROR("mutex %s is not free", name.c_str());
        else
            mtx_packets_incomplete.unlock();
    }
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


    n_priv = &b;
    n_pub = &b_pub;

    unack_routed_frame = new stc_RoutedFrame();

    initParams(n_priv);
    initFrameTypes();

    /*CREATE SOCKETS */
    eth_raw_init();

    /*INIT HOSTNAME*/

    if (hostname.compare("") == 0)
    {
        char hostname_c[1024];
        hostname_c[1023] = '\0';
        gethostname(hostname_c, 1023);
        hostname = std::string(hostname_c);
    }

#ifdef PRINT_PARAMS
    ROS_ERROR("NODE STARTED WITH FOLLOWING SETTINGS:");
    ROS_ERROR("HOSTNAME[%s] MAC[%s] INTERFACE[%s]", hostname.c_str(), getMacAsStr(src_mac).c_str(), interface);
    ROS_ERROR("num_link_retrans=%u num_e2e_retrans=%u num_rreq=%u", num_link_retrans, num_e2e_retrans, num_rreq);
    ROS_ERROR("max_packet_size=%u max_frame_size=%u hop_limit_min=%u hop_limit_max=%u", max_packet_size, max_frame_size, hop_limit_min, hop_limit_max);
    ROS_ERROR("hop_limit_increment=%u beacon_interval=%u", hop_limit_increment, beacon_interval);
    ROS_ERROR("rebuild_mc_tree=%s", getBoolAsString(rebuild_mc_tree).c_str());
#endif


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

    publishers_l.push_front(n_pub->advertise<std_msgs::String>(robot_prefix + node_prefix + topic_new_robot, 1000, true));
    publishers_l.push_front(n_pub->advertise<std_msgs::String>(robot_prefix + node_prefix + topic_remove_robot, 1000, true));

    Logging::init(n_pub, &hostname);

    signal(SIGSEGV, handler); // install handler  

    /*Tutorial*/
    //ros::ServiceServer sendQuaternionS = n->advertiseService("send_quaternion", sendQuaternion);
    /*INIT THREADS*/

    boost::thread receiveFramesT(receiveFrames);
    boost::thread processIncomingFramesT(processIncomingFrames);
    // boost::thread resendFrameT(&resendFrames); // this thread re-sends frames ineffective and too less accurate
    boost::thread deleteObsoleteRequestsT(deleteObsoleteRequests);
    // boost::thread reconnectToMcGroupsT(&reconnectToMcGroups);
    boost::thread requestPendingFramesT(requestPendingFrames);
    boost::thread sendBeaconsT(sendBeacons);
    boost::thread deleteOldPacketsT(deleteOldPackets);
    boost::thread resendRequestedFramesT(resendRequestedFrames);

    // boost::thread checkMutexAvailabilityT(&checkMutexAvailability, &mtx_packets_incomplete, "packets incomplete");
    // boost::thread checkMutexAvailabilisastyT(&checkMutexAvailability, &mtx_cached_mc_packets, "mtx_cached_mc_packets");

    list<ros::Subscriber> sub_robot_pos_l;

    boost::thread log_mem_consumption_packets(&Logging::logMemoryConsumptionPackets, 5000,
            &mtx_packets_incomplete, &packets_incomplete_l,
            &mtx_cached_mc_packets, &cached_mc_packets_l,
            &mtx_unack_link_frames, &unack_link_frames_l,
            &mtx_unack_cr_frames, &unack_cr_frames_l
            );


#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
    boost::thread log_uc_frames(Logging::logUcPacketsSummary, 5000);
#endif

#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
    boost::thread log_mc_frames(Logging::logMcPacketsSummary, 5000);
#endif


#ifdef PERFORMANCE_LOGGING
    boost::thread periodix(Logging::periodicLog, 10000);
#endif

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

    /* Create own mc group */
    {
        ROS_INFO("Create own group! %s ", std::string("mc").append("_").append(hostname).c_str());
        boost::unique_lock<boost::mutex> lock(mtx_mc_groups);
        mc_handler.createGroupAsRoot(&std::string("mc").append("_").append(hostname));
    }




#ifdef JOIN_ALL_GROUPS
    if (simulation_mode)
    {
        boost::thread joinT(&joinAllMcGroups);
#ifdef DEBUG_OUTPUT
        //  printMcConnections(&mc_trees_l);
#endif


    }
#endif

    // testPacket();

    while (ros::ok())
    {
        ros::spin();
    }
    Logging::log();

    sleepMS(1000);

    delete unack_routed_frame;

    /*Wait for threads*/
    receiveFramesT.join();
    sendBeaconsT.join();



    /*Close sockets..*/
    close_raw_socket();


    ROS_INFO("Node terminating...");

    return 0;
}

bool gotAck(AckLinkFrame* inc_frame)
{
    /* Description:
     * Process acknowledgment of link frames
     * If CR is enabled, also the unacknowledged CR frames will be managed by this function
     *
     * Return:
     * 		(true) if got acknowledgment of unacknowledged link frame
     *
     * 		(false) if got acknowledgment of frame that is not in unack_link_frames_l
     */


    stc_frame f;
    f.type = inc_frame->header_.ack_frame_type;
    memcpy(f.mac, inc_frame->header_.mac_confirmer, 6);
    f.hostname_source = inc_frame->hostname_source_;
    f.frame_id = inc_frame->header_.frame_id;
    f.mc_group = "";

    {
        boost::unique_lock<boost::mutex> lock(mtx_unack_link_frames);
        std::list<stc_frame>::iterator i = std::find(unack_link_frames_l.begin(), unack_link_frames_l.end(), f);
        if (i != unack_link_frames_l.end())
        {
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
            if (compareMac(inc_frame->header_.mac_confirmer, inc_frame->eh_h_.eh_source))
                Logging::increaseProperty("num_acknowledgments_received_directly");
            else
                Logging::increaseProperty("num_acknowledgments_received_relay");

            Logging::increaseProperty("num_total_bytes_received", inc_frame->buffer_str_len_);
#endif

#ifdef PERFORMANCE_LOGGING_UC_LINK_FRAMES
            Logging::logUcLinkTransmission(*i);
#endif
            ROS_INFO("GOT ACK OF LINK FRAME: ID[%u] SOURCE MAC[%s] TYPE[%s]", (*i).frame_id, getMacAsStr((*i).mac).c_str(), frame_types[(*i).type].c_str());
            unack_link_frames_l.remove(*i);
            return true;
        }
    }

    {
        boost::unique_lock<boost::mutex> lock(mtx_unack_cr_frames);

        for (std::list<ack_cr_info>::iterator i = unack_cr_frames_l.begin(); i != unack_cr_frames_l.end();)
        {
            if ((*i).id == inc_frame->header_.frame_id && (*i).source_host.compare(inc_frame->hostname_source_) == 0 && compareMac((*i).frame_dst_mac, inc_frame->header_.mac_confirmer))
            {
                if (compareMac((*i).frame_src_mac, inc_frame->header_.mac_destination_))
                    ROS_INFO("GOT ACK OF CR FRAME: ID[%u] SOURCE MAC[%s]", (*i).id, getMacAsStr(inc_frame->eh_h_.eh_source).c_str());
                else if (compareMac(inc_frame->header_.mac_destination_, src_mac))
                {
                    ROS_INFO("GOT ACK OF RETRANSMITTED CR FRAME: ID[%u] SOURCE MAC[%s]", (*i).id, getMacAsStr(inc_frame->eh_h_.eh_source).c_str());
                    sendLinkAck((*i).frame_src_mac, (*i).frame_dst_mac, (*i).id, (*i).source_host, false, (*i).frame_type);
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
                    Logging::increaseProperty("num_acknowledgments_relayed");
                    Logging::increaseProperty("num_total_bytes_sent", AckLinkFrame::HEADER_FIXED_LEN + (*i).source_host.length());
#endif
                }


#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
                Logging::increaseProperty("num_acknowledgments_received_relay");
                Logging::increaseProperty("num_total_bytes_received", inc_frame->buffer_str_len_);
#endif

                unack_cr_frames_l.erase(i);
                return true;
            }
            else if (getMillisecondsTime() - (*i).ts >= MAX_TIME_CACHE_UNACK_CR_FRAMES)
            {
                ROS_WARN("DROP UNACKNOWLEGED CR FRAME: ID[%u] SOURCE MAC[%s]", (*i).id, getMacAsStr(inc_frame->eh_h_.eh_source).c_str());
                i = unack_cr_frames_l.erase(i);
            }
            else
                i++;
        }
    }
    return false;
}

void receiveFrames()
{

    /* Description:
     * This function receive all incoming frames on the interface and process them.
     * Behavior of the function will be explained within the function.
     *
     * NOTE: This function runs as own thread.
     */
    unsigned char* buffer_incoming = new unsigned char[ETHER_MAX_LEN]; //ETHER_MAX_LEN]; /* Buffer for ethernet frame (maximum size)*/

    uint8_t max_safed_frames = 1;
    std::vector<std::string> frames_as_strings;
    uint8_t current_buffer_index = 0;
    uint16_t buffer_size = 0;
    for (uint8_t i = 0; i < max_safed_frames; i++)
        frames_as_strings.push_back(std::string(" "));


    while (ros::ok())
    {


        //WAIT for incoming packet.../
        recvfrom(raw_socket, buffer_incoming, ETHER_MAX_LEN, 0, NULL, NULL);

        /*
         * 	int i = recvfrom(raw_socket, buffer_incoming, ETHER_MAX_LEN, 0, NULL, NULL);
         if (i == -1 && ros::ok())
         {
         perror("recvfrom():");
         exit(1);
         }
         */

        ethernet_header = (struct eh_header *) buffer_incoming;

        /*CHECK if the incoming frame has the Ethernet type of the routing protocol*/
        if (ethernet_header->eh_protocol_id != ntohs(ETH_TYPE))
            continue;


        /* continue if frame is from myself*/
        if (compareMac(ethernet_header->eh_source, src_mac))
            continue;

        uint8_t frame_type;
        unsigned char* frameTypeP = buffer_incoming + 14;
        memcpy((unsigned char*) &frame_type, (unsigned char*) frameTypeP, 1);

        /*Check if packet is for me*/
        unsigned char* dest_mac_p = buffer_incoming + 15;
        bool packet_4_me = compareMac(dest_mac_p, src_mac);
        bool packet_is_bcast = compareMac(dest_mac_p, bcast_mac);
        bool process_frame = true;


        /*
         * This option is for testing the re-sent features of the protocol.
         * It can be defined how many percentages of all incoming protocol frames will be dropped. (PERCENTAGE_OF_PACKET_LOST) -> defines.h
         *
         */

        if (loss_ratio > 0)
        {
            if (incoming_frame_count == 0)
            {
                incoming_frame_count++;
                ROS_ERROR("------------------------------------------------------------------------------");
                ROS_ERROR("----------------------LOSE FRAMES IS ACTIVATED!!------------------------------");
                ROS_ERROR("------------------------------------------------------------------------------");
            }
            if (frame_type == FRAME_TYPE_TRANSPORT_DATA || frame_type == FRAME_TYPE_TRANSPORT_ACK)
            {
                incoming_frame_count++;

                if ((float) incoming_frames_lost / (float) incoming_frame_count <= loss_ratio)
                {
                    incoming_frames_lost++;
                    continue;
                }
            }
        }

#ifdef USE_CHANNEL_MODEL                 
        process_frame = isReachable(ethernet_header->eh_source);
#endif                  
        if (!process_frame)
        {
            continue;
        }


        /* Check if the current frame is not equal to the last ones */
        uint8_t buffer_index = current_buffer_index - 1 >= 0 ? current_buffer_index - 1 : max_safed_frames - 1;
        //process_frame = !isBufferInList(buffer_incoming, &frames_as_strings, buffer_index); todo


        if (!process_frame)
        {
            continue;
        }

        buffer_size = 250; // todo create buffer_str_len member for all other frame classes and set it after frame is created (like mc maintenace frame)
        /*frame*/

        if (frame_type == FRAME_TYPE_MC_ACK) //MC ACK PACKET
        {
            McAckFrame frame(buffer_incoming);

            /*Negative ack*/
            if (frame.correct_crc_)
            {
                processMcAckFrame(&frame);
            }
            else
                ROS_ERROR("MC ACK FRAME: CRC INCORECCT!");

        }
        else if (frame_type == FRAME_TYPE_MC_NACK)
        {
            McNackFrame frame(buffer_incoming);

            /*Negative ack*/
            if (frame.correct_crc_)
            {

                if (iAmMember(frame.mc_group_))
                {
                    boost::unique_lock<boost::mutex> lock(mtx_requested_frames);
                    requested_frames_l.push_back(frame);

                }
            }
            else
                ROS_ERROR("MC NACK FRAME: CRC INCORECCT!");
        }
        else if (frame_type == FRAME_TYPE_ACK) //ACK PACKET
        {
            AckLinkFrame inc_ack_lframe(buffer_incoming);
            bool frame_is_4_me = compareMac(src_mac, inc_ack_lframe.header_.mac_destination_);

            buffer_size = inc_ack_lframe.buffer_str_len_;

            /* continue if cr is disabled and frame is an neg ack*/
            if (enable_cooperative_relaying == false && !frame_is_4_me && inc_ack_lframe.pos_ack_flag_)
            {
                continue;
            }
            if (inc_ack_lframe.pos_ack_flag_)
            {
                gotAck(&inc_ack_lframe);
            }
        }/*PROCESS Got route request*/
        else if (frame_type == FRAME_TYPE_REQUEST)
        {
            RouteRequest req(buffer_incoming);
            if (req.correct_crc_)
            {
                buffer_size = req.buffer_str_len_;

                processRouteRequest(req);
            }
            else
                ROS_ERROR("ROUTE REQUEST: CRC INCORECCT!");
        }/*PROCESS  route response*/
        else if (frame_type == FRAME_TYPE_REPLY)
        {
            RouteResponse rr(buffer_incoming);

            if (rr.correct_crc_)
                processRouteResponse(rr);
            else
                ROS_WARN("ROUTE RESPONSE: CRC INCORECCT!");
        }/*PROCESS routed frame*/
        else if (frame_type == FRAME_TYPE_TRANSPORT_DATA) // && (packet_4_me || packet_is_bcast))
        {
            RoutedFrame frame(buffer_incoming);
            buffer_size = frame.buffer_str_len_;
            if (packet_is_bcast)
                processBroadcastFrame(&frame);
            else
                processRoutedFrame(&frame);
        }/*PROCESS acknowledgment of routed frame*/
        else if (frame_type == FRAME_TYPE_TRANSPORT_ACK)
        {
            AckRoutedFrame rf(buffer_incoming);
            buffer_size = rf.buffer_str_len_;

            processAckRoutedFrame(&rf);
        }/*PROCESS Beacon*/
        else if (frame_type == FRAME_TYPE_BEACON)
        {
            Beacon hf(buffer_incoming);
            if (hf.correct_crc_)
                processBeacon(&hf);
            else
                ROS_ERROR("BEACON: CRC INCORECCT!");

        }/*PROCESS McActivation*/
        else if (frame_type == FRAME_TYPE_MC_ACTIVATION) // && packet_4_me )
        {

            McRouteActivationFrame mc_m_f(buffer_incoming);

            if (mc_m_f.correct_crc_)
            {
                buffer_size = mc_m_f.buffer_str_len_;
                processMcActivationFrame(&mc_m_f);
            }
            else
                ROS_ERROR("MC ROUTE ACTIVATION: CRC INCORECCT!");
        }/*PROCESS FRAME_TYPE_MC_DISCONNECT*/
        else if (frame_type == FRAME_TYPE_MC_DISCONNECT)
        {
            McDisconnectFrame mc_d(buffer_incoming);

            if (mc_d.correct_crc_)
            {
                buffer_size = mc_d.buffer_str_len_;

                if (mc_d.disconnect_uplink)
                    processMcDisconnectUplink(&mc_d);
                else if (mc_d.disconnect_downlink)
                    processMcDisconnectDownlink(&mc_d);
                else
                    ROS_ERROR("UNKNOWN MC DISCONNECT FRAME!");

                // processMcMaintenanceFrame(&mc_m_f);
            }
            else
                ROS_ERROR("McDisconnectionFrame: CRC INCORECCT!");


        }
        else if (frame_type == FRAME_TYPE_BROADCAST)
        {
            MultiHopBroadcastFrame bcast(buffer_incoming);

            if (bcast.correct_crc_ == false)
            {
                ROS_ERROR("MultiHopBroadcastFrame: CRC INCORECCT!");
                continue;
            }

            bcasts bc_str(bcast.header_.id, bcast.hostname_source_);
            if (std::find(bcasts_l.begin(), bcasts_l.end(), bc_str) == bcasts_l.end() && bcast.hostname_source_.compare(hostname) != 0)
            {
                bcasts_l.push_back(bc_str);

                buffer_size = bcast.buffer_str_len_;
                if (bcast.header_.payload_type == FRAME_DATA_TYPE_ROBOT_UPDATE)
                {
                    adhoc_communication::CMgrRobotUpdate r_up;
                    if (bcast.payload_.compare("") == 0)
                    {
                        ROS_ERROR("ERROR: PAYLOAD OF BCAST FRAME IS EMPTY!");
                        continue;
                    }
                    desializeObject((unsigned char*) bcast.payload_.data(), bcast.payload_.length(), &r_up);
                    publishMessage(r_up, bcast.topic_);
                }
                else if (bcast.header_.payload_type == FRAME_DATA_TYPE_ANY)
                {

                    if (bcast.payload_.compare("") == 0)
                    {
                        ROS_ERROR("ERROR: PAYLOAD OF BCAST FRAME IS EMPTY!");
                        continue;
                    }
                    adhoc_communication::RecvString data;
                    data.src_robot = bcast.hostname_source_;
                    data.data = bcast.payload_;                  
                    publishMessage(data, bcast.topic_);
                }

                if (bcast.rebroadcast)
                {
                    socketSend(bcast.getFrameAsNetworkString(src_mac));
                }
            }
        }
        else
            ROS_ERROR("UNKNOWN FRAMETYPE: [%u] source %s dest %s", frame_type, getMacAsStr(ethernet_header->eh_source).c_str(), getMacAsStr(dest_mac_p).c_str());


        if (frame_type != FRAME_TYPE_BEACON)
        {
            current_buffer_index = current_buffer_index + 1 < frames_as_strings.size() ? current_buffer_index + 1 : 0;
            frames_as_strings[current_buffer_index] = std::string((const char*) buffer_incoming, buffer_size);
        }

    } //end while

    free(buffer_incoming);
}

template<class message>
void publishMessage(message m, string topic)
{
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

        for (std::list<ros::Publisher>::iterator i = publishers_l.begin(); i != publishers_l.end(); i++)
        {
            if ((*i).getTopic().compare(topic) == 0)
            {
                (*i).publish(m);
                pubExsists = true;
            }
        }
        if (!pubExsists)
        {
            publishers_l.push_front(n_pub->advertise<message>(topic, 1000, true));
            publishers_l.front().publish(m);
            //todo check if ledge = true is working
        }
    }    catch (...)
    {
        ROS_ERROR("IN PUBLISH MESSAGE: NODE THROWS EXCEPTION!");
    }
}

void sendBeacons()
{
    /* Description:
     * Send hello frames.
     * Increase 'no_hello_got' variable of all other neighbors.
     * Delete neighbors if no_hello_got reaches a maximum.
     *
     * NOTE: The neighbor table is only used for debugging reasons. The protocol will also work without neighbor table.
     * 		 This function runs as a thread.
     */

    Beacon hf = Beacon(src_mac, hostname);
    string beacon = hf.getFrameAsNetworkString();
    uint16_t beacon_len = beacon.length();
    while (ros::ok())
    {
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
        Logging::increaseProperty("num_beacons_sent");
        Logging::increaseProperty("num_total_bytes_sent", beacon_len);
#endif

        socketSend(beacon);
        {
            boost::unique_lock<boost::mutex> lock_neighbors(mtx_neighbors);

            std::list<hostname_mac>::iterator i = neighbors_l.begin();
            while (i != neighbors_l.end())
            {
                hostname_mac & neighbor(*i);


                if (getMillisecondsTime() - neighbor.ts >= TIME_BEFORE_REMOVE_NEIGHBOR && neighbor.reachable)
                {
                    ROS_ERROR("REMOVE NEIGHBOR: HOSTNAME[%s]", neighbor.hostname.c_str());
                    std_msgs::String msg;
                    msg.data = neighbor.hostname;
                    lock_neighbors.unlock();

                    publishMessage(msg, node_prefix + topic_remove_robot);
                    lock_neighbors.lock();
                    neighbor.reachable = false;

                    /* MC Group  */
                    hostname_mac n;
                    n.hostname = neighbor.hostname;
                    memcpy((void*) n.mac, neighbor.mac, 6);


                    boost::thread t(&mcLostConnection, n);
                }
                else
                {
                    ROS_DEBUG("NEIGHBOR: HOSTNAME[%s] LAST BEACON GOT[%lu]", neighbor.hostname.c_str(), neighbor.ts);
                }
                i++;
            }
        }

        //boost::this_thread::sleep(boost::posix_time::milliseconds(beacon_interval));
        sleepMS(&beacon_interval);
    }
}

McPosAckObj * getMcAckObj(std::string* group, uint32_t p_id, uint32_t seq)
{
    boost::unique_lock<boost::mutex> lock(mtx_mc_ack_l);
    for (list<McPosAckObj*>::iterator i = mc_ack_l.begin(); i != mc_ack_l.end(); i++)
    {
        McPosAckObj* ack_obj = *i;
        if (ack_obj->group_name.compare(*group) == 0 && p_id == ack_obj->packet_id && ack_obj->sequence_num == seq)
        {
            return ack_obj;
        }
    }
    return NULL;
}

void sendLinkAck(unsigned char* dest, unsigned char* confirmer_mac, uint32_t id, string source, bool cr, uint8_t type)
{
    AckLinkFrame ack = AckLinkFrame(src_mac, confirmer_mac, dest, id, source, type);
    ack.cr_flag_ = cr;
    //   ROS_ERROR("send link ack:");
    //   ack.print_frame();
    string network_string = ack.getFrameAsNetworkString();
    socketSend(network_string);
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
    Logging::increaseProperty("num_acknowledgments_sent");
    Logging::increaseProperty("num_total_bytes_sent", network_string.length());
#endif

}

void processMcAckFrame(McAckFrame* f)
{
    /*MULTICAST POSITIVE ACK*/
    McPosAckObj* ack_obj = getMcAckObj(&f->mc_group_, f->header_.packet_id, f->header_.frame_seq_num);
    boost::unique_lock<boost::mutex> lock(mtx_mc_ack_l);
    if (ack_obj != NULL)
    {
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
        Logging::increaseProperty("num_mc_acknowledgments_received_directly");
        Logging::increaseProperty("num_mc_total_bytes_received ", f->buffer_str_len_);
#endif
        // ROS_ERROR("GOT MULTICAST ACK: GROUP[%s] HOST[%s] PACKET[%u] SEQUENCE[%u] FROM [%s]", f->mc_group_.c_str(), f->hostname_source_.c_str(), f->header_.packet_id, f->header_.frame_seq_num, getMacAsStr(f->eh_h_.eh_source).c_str());
        if (ack_obj->GotAck(f))
        {
            boost::unique_lock<boost::mutex> lockd(mtx_notify);
            got_rframe_ack.notify_all();

            ROS_DEBUG("GOT ALL MULTICAST ACK: GROUP[%s] HOST[%s] PACKET[%u] SEQUENCE[%u]", f->mc_group_.c_str(), f->hostname_source_.c_str(), f->header_.packet_id, f->header_.frame_seq_num);

            if (f->hostname_source_.compare(hostname) != 0)
            {
                /*send ack*/
                McAckFrame arf = McAckFrame(src_mac, f->eh_h_.eh_source, f->hostname_source_, f->mc_group_, f->header_.packet_id, f->header_.frame_seq_num);
                string network_string = arf.getFrameAsNetworkString();
                socketSend(network_string);
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
                Logging::increaseProperty("num_mc_acknowledgments_sent");
                Logging::increaseProperty("num_mc_total_bytes_sent ", network_string.length());
#endif
            }
        }
    }


}

void processAckRoutedFrame(AckRoutedFrame * in_a_rf)
{
    /* Description:
     * Process acknowledgment of routed frames.
     * Behavior of the function is explained within the function
     */

    /*create struct of a frame, to safe it and make sure that every frame will only received once*/
    stc_frame frame_to_safe;
    frame_to_safe.frame_id = in_a_rf->header_.frame_id;
    memcpy(frame_to_safe.mac, in_a_rf->eh_h_.eh_source, 6);
    frame_to_safe.hostname_source = std::string(in_a_rf->hostname_source_);
    frame_to_safe.mc_group = "";

    /*Check if I am the destination of the frame*/
    bool frame_is_for_me = compareMac(in_a_rf->header_.mac_destination_, src_mac);

    if (frame_is_for_me)
    {
        /*Send acknowledgment of link frame*/
        sendLinkAck(in_a_rf->eh_h_.eh_source, src_mac, frame_to_safe.frame_id, frame_to_safe.hostname_source, in_a_rf->cr_flag, in_a_rf->header_.frame_type);
    }

    if (!frame_is_for_me && !enable_cooperative_relaying)
    {
        ROS_DEBUG("DONT PROCESS FRAME BECAUSE IT IS NOT FOR ME AND COOP_RELAYING IS TURNED OFF!");
        return;
    }

    if (!in_a_rf->correct_crc_)
    {
        ROS_ERROR("ACK ROUTED FRAME: CRC INCORRECT!");
        return;
    }

    /* UNICAST */
    routing_entry route;
    {
        route.hostname_source = in_a_rf->hostname_source_;
        route.id = in_a_rf->header_.route_id;

        boost::unique_lock<boost::mutex> lock(mtx_routing_table);
        std::list<routing_entry>::iterator find_route = std::find(routing_table_l.begin(), routing_table_l.end(), route);
        /*Search the route in the routing table*/
        if (find_route == routing_table_l.end())
        {
            ROS_DEBUG("ROUTE FOR THE ACK FRAME NOT FOUND: ID[%u] HOSTNAME SOURCE[%s] ROUTE ID[%u]", in_a_rf->header_.frame_id, in_a_rf->hostname_source_.c_str(), in_a_rf->header_.route_id);
            return;
        }

        route.id = (*find_route).id;
        route.hostname_destination = (*find_route).hostname_destination;
        route.hostname_source = (*find_route).hostname_source;
        route.cr_entry = (*find_route).cr_entry;
        memcpy(route.next_hop, (*find_route).next_hop, 6);
        memcpy(route.previous_hop, (*find_route).previous_hop, 6);
    }

    if (frame_is_for_me)
    {
        /*Check if i am the source*/
        if (route.hostname_source.compare(hostname) == 0)
        {

            ROS_DEBUG("GOT ACKNOWLEGEMENT OF ROUTED FRAME: HOSTNAME[%s] ID[%u]", route.hostname_destination.c_str(), in_a_rf->header_.frame_id);
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
            Logging::increaseProperty("num_data_frames_received_directly");
            Logging::increaseProperty("num_unique_data_frames_received_directly");
            Logging::increaseProperty("num_total_bytes_received", in_a_rf->buffer_str_len_);
#endif

            {
                boost::unique_lock<boost::mutex> lock(mtx_unack_routed_frame);

                /* Check if the incoming ack is for the unuacknowledged frame*/
                if (in_a_rf->header_.frame_id == unack_routed_frame->frame.header_.frame_id && route.hostname_destination.compare(unack_routed_frame->hostname_destination) == 0)
                {
                    unack_routed_frame->frame_is_ack = true;
                    boost::unique_lock<boost::mutex> lockd(mtx_notify);
                    got_rframe_ack.notify_all();
                }
            }

        }/*FORWARD frame*/
        else
        {
            // ROS_ERROR("FORWARD ACK OF ROUTED FRAME: FROM[%s]->TO[%s]", getMacAsStr(in_a_rf->eh_h_.eh_source).c_str(),  getMacAsStr(route.previous_hop).c_str());

            string network_string = in_a_rf->getFrameAsNetworkString(route, src_mac);

            /*SAFE the packet as unacknowledged*/
            resendUnackLinkFrame(in_a_rf->hostname_source_, in_a_rf->header_.frame_id, route.previous_hop, network_string, FRAME_TYPE_TRANSPORT_ACK);

#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
            Logging::increaseProperty("num_unique_data_frames_forwarded");
            Logging::increaseProperty("num_total_bytes_sent", network_string.length());
#endif

            socketSend(network_string);
#ifdef DELAY

            if (simulation_mode)
                boost::this_thread::sleep(boost::posix_time::milliseconds(delay));
#endif
        }
    }
}

void processRoutedFrame(RoutedFrame * f)
{
    /* Description:
     * Process routed frame
     * Behavior of the function is explained within the function
     */

    RoutedFrame inc_frame = *f;

    if (!inc_frame.correct_crc_)
    {
        ROS_ERROR("ROUTED FRAME: CRC INCORECCT!");
        return;
    }

    /*Check if I am the destination of the frame*/
    bool frame_is_for_me = compareMac(inc_frame.header_.mac_destination_, src_mac);

    if (frame_is_for_me)
    {
        /*Send acknowledgment */
        sendLinkAck(inc_frame.eh_h_.eh_source, src_mac, inc_frame.header_.frame_id, inc_frame.hostname_source_, f->cr_flag, f->header_.frame_type);
    }

    if (!frame_is_for_me)
    {
        ROS_DEBUG("DONT PROCESS FRAME BECAUSE IT IS NOT FOR ME AND COOP_RELAYING IS TURNED OFF!");
        return;
    }

    bool cr_route = true;
    routing_entry route;
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
    Logging::increaseProperty("num_data_frames_received_directly");
    Logging::increaseProperty("num_total_bytes_received", f->buffer_str_len_);
#endif

    /*CHECK if i got the frame already*/

    if (gotFrameRecently(inc_frame))
    {
        // ROS_ERROR("GOT R FRAME ALREADY %u", inc_frame.header_.packet_sequence_num);
        return;
    }

    safeFrame(inc_frame);

#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
    if (f->cr_flag)
        Logging::increaseProperty("num_unique_data_frames_received_relay");
    else
        Logging::increaseProperty("num_unique_data_frames_received_directly");
#endif



    /*SEARCH the route in the routing table*/
    routing_entry route_f;
    route_f.id = inc_frame.header_.route_id;
    route_f.hostname_source = inc_frame.hostname_source_;
    {
        boost::unique_lock<boost::mutex> lock(mtx_routing_table);
        std::list<routing_entry>::iterator find_route = std::find(routing_table_l.begin(), routing_table_l.end(), route_f);
        if (find_route == routing_table_l.end())
        {
            ROS_DEBUG("ROUTE FOR THE ROUTED FRAME NOT FOUND: ID[%u] HOSTNAME SOURCE[%s] ROUTE ID[%u]", inc_frame.header_.frame_id, inc_frame.hostname_source_.c_str(), inc_frame.header_.route_id);
            return;
        }

        routing_entry & tmp(*find_route);
        route.id = tmp.id;
        route.hostname_destination = tmp.hostname_destination;
        route.hostname_source = tmp.hostname_source;
        memcpy(route.next_hop, tmp.next_hop, 6);
        memcpy(route.previous_hop, tmp.previous_hop, 6);
        route.cr_entry = tmp.cr_entry;

    }

    /*Check if i am the destination*/
    if (route.hostname_destination.compare(hostname) == 0)
    {
        ROS_DEBUG("GOT ROUTED FRAME: HOSTNAME[%s] ID[%u]", route.hostname_destination.c_str(), inc_frame.header_.frame_id);


        /*SEND ack to the root source */
        AckRoutedFrame arf = AckRoutedFrame(inc_frame);
        std::string network_string = arf.getFrameAsNetworkString(route, src_mac);
#ifdef DELAY
        if (simulation_mode)
            boost::this_thread::sleep(boost::posix_time::milliseconds(delay));
#endif

        resendUnackLinkFrame(arf.hostname_source_, arf.header_.frame_id, arf.header_.mac_destination_, network_string, FRAME_TYPE_TRANSPORT_ACK);

        socketSend(network_string);
        ROS_DEBUG("SEND ACKNOWLEGEMENT OF ROUTED FRAME: ID[%u] DESTINATION[%s] NEXT HOP[%s]", inc_frame.header_.frame_id, route.hostname_destination.c_str(), getMacAsStr(route.previous_hop).c_str());

        boost::unique_lock<boost::mutex> lock(mtx_inc_rf_frames);
        inc_frames_l.push_back(inc_frame);

    }
    else/*forward frame*/
    {
        string network_string = inc_frame.getFrameAsNetworkString(route, src_mac);

        /*Safe the frame as unacknowledged*/
        stc_frame frame_to_ack = inc_frame.getFrameStruct();
        frame_to_ack.network_string = network_string;

        resendUnackLinkFrame(f->hostname_source_, f->header_.frame_id, inc_frame.header_.mac_destination_, network_string, FRAME_TYPE_TRANSPORT_DATA);

        ROS_DEBUG("FORWARD ROUTED FRAME: FROM[%s] TO[%s]", getMacAsStr(route.previous_hop).c_str(), getMacAsStr(route.next_hop).c_str());
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
        Logging::increaseProperty("num_unique_data_frames_forwarded");
        Logging::increaseProperty("num_total_bytes_sent ", network_string.length());
#endif

        socketSend(network_string);

    }
}

void sendMcAck(unsigned char* dst, string h_source, string g_name, uint32_t packet_id, uint32_t seq_num)
{
    ROS_DEBUG("SEND POS MC ACKNOWLEGEMENT: ID[%u] GROUP[%s] SOURCE[%s] FROM[%s]", packet_id, g_name.c_str(), h_source.c_str(), getHostnameFromMac(dst).c_str());
    /*SEND ack to the root source */

    McAckFrame arf = McAckFrame(src_mac, dst, h_source, g_name, packet_id, seq_num);
    string network_string = arf.getFrameAsNetworkString();
    socketSend(network_string);
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
    Logging::increaseProperty("num_mc_acknowledgments_sent");
    Logging::increaseProperty("num_mc_total_bytes_sent ", network_string.length());
#endif   
}

void deliverMcAckFrame(stc_RoutedFrame& stc_rf, routing_entry& r, bool send_ack)
{
    Logging::increaseProperty("running_multicast_threads");
    /* wait for acknowledgement and resend frame */

    while (stc_rf.frame_is_ack == false && stc_rf.retransmitted <= num_e2e_retrans)
    {
        {
            boost::unique_lock<boost::mutex> lockAckRF(mtx_notify);
            boost::thread* resendMcFrameT = new boost::thread(resendMcFrame, boost::ref(got_rframe_ack), &stc_rf, r);
            got_rframe_ack.wait(lockAckRF); //wait for response
            resendMcFrameT->interrupt();
            delete resendMcFrameT;
        }

        McPosAckObj* mc_ack = getMcAckObj(&stc_rf.frame.mc_g_name_, stc_rf.frame.header_.packet_id, stc_rf.frame.header_.packet_sequence_num);

        boost::unique_lock<boost::mutex> ack_lock(mtx_mc_ack_l);
        if (mc_ack != NULL && mc_ack->missing_acks_l.empty())
        {
            stc_rf.frame_is_ack = true;
        }
    }

    if (send_ack && stc_rf.frame_is_ack)
    {

        sendMcAck(stc_rf.frame.eh_h_.eh_source, stc_rf.frame.hostname_source_, stc_rf.frame.mc_g_name_, stc_rf.frame.header_.packet_id, stc_rf.frame.header_.packet_sequence_num);
    }

    if (stc_rf.frame_is_ack == false)
    {
        ROS_WARN("COULD NOT SENT MC ACK FRAME: SOURCE[%s] GROUP[%s] PACK_ID[%u] SEQ_N[%u]", stc_rf.frame.hostname_source_.c_str(), stc_rf.frame.mc_g_name_.c_str(), stc_rf.frame.header_.packet_id, stc_rf.frame.header_.packet_sequence_num);
    }
    Logging::decreaseProperty("running_multicast_threads");
}

void processBroadcastFrame(RoutedFrame * f)
{
    /* Description:
     * Process a Broadcast
     * Check if i got the frame already.
     * If not process the frame.
     */

    RoutedFrame inc_frame(*f);



    if (inc_frame.mc_flag)
    {
        boost::unique_lock<boost::mutex> lock(mtx_mc_groups);
        McTree* g = mc_handler.getMcGroup(&inc_frame.mc_g_name_);

        if (g != NULL && (!g->activated || !g->connected || !g->processFrame(inc_frame.eh_h_.eh_source)))
        {
            //     ROS_ERROR("break %u %u %u",!g->activated, !g->connected , !g->processFrame(inc_frame.eh_h_.eh_source));
            return;
        }
    }

    if (!inc_frame.correct_crc_)
    {
        ROS_ERROR("BROADCAST FRAME: CRC INCORECCT!");
        return;
    }

    // dont process own frames
    if (inc_frame.hostname_source_.compare(hostname) == 0)
        return;


#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
    if (!inc_frame.mc_flag)
    {
        Logging::increaseProperty("num_data_frames_received_directly");
        Logging::increaseProperty("num_total_bytes_received", f->buffer_str_len_);
    }

#endif

#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
    if (inc_frame.mc_flag)
    {
        Logging::increaseProperty("num_mc_data_frames_received_directly");
        Logging::increaseProperty("num_mc_total_bytes_received ", f->buffer_str_len_);
    }
#endif

    if (gotFrameRecently(inc_frame))
    {
        //ROS_ERROR("got bcast frame recently %u", inc_frame.header_.packet_sequence_num);
        if (f->negative_ack_type == false || (f->cr_flag && f->mc_flag))
            sendMcAck(inc_frame.eh_h_.eh_source, inc_frame.hostname_source_, inc_frame.mc_g_name_, inc_frame.header_.packet_id, inc_frame.header_.packet_sequence_num);

        return;
    }

#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
    if (!inc_frame.mc_flag)
        Logging::increaseProperty("num_unique_data_frames_received_directly");
#endif

#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
    if (inc_frame.mc_flag)
        Logging::increaseProperty("num_mc_data_frames_received_directly");
#endif

    safeFrame(inc_frame);

    /* MULTICAST */
    if (inc_frame.mc_flag)
    {
        bool propagated = false;
        {
            boost::unique_lock<boost::mutex> lock_groups(mtx_mc_groups);
            McTree* mc_t = mc_handler.getMcGroup(&inc_frame.mc_g_name_);

            if (mc_t == NULL) // no entry
                return;

            if (!mc_t->activated)
            {
                ROS_ERROR("MC ROUTE IS NOT ACTIVE [%s]", inc_frame.mc_g_name_.c_str());
                return;
            }
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
            Logging::increaseProperty("num_mc_unique_data_frames_received_directly");
#endif


            propagated = mc_t->propagateFrame(inc_frame.eh_h_.eh_source);
        }
        ROS_INFO("GOT MC FRAME: ID[%u] GROUP[%s] SOURCE[%s] FROM[%s] P_ID[%u] SEQ_N[%u]", inc_frame.header_.frame_id, inc_frame.mc_g_name_.c_str(), inc_frame.hostname_source_.c_str(), getHostnameFromMac(inc_frame.eh_h_.eh_source).c_str(), inc_frame.header_.packet_id, inc_frame.header_.packet_sequence_num);


        /* NEGATIVE ACK */
        if (inc_frame.negative_ack_type)
        {

            cacheNackMcFrame(inc_frame);

        }/* POSITIVE ACK */
        else
        {
            /* If the frame was propagated, the node is a end node and can send the acknowledgement*/
            if (!propagated || recursive_mc_ack == false)
            {
                sendMcAck(inc_frame.eh_h_.eh_source, inc_frame.hostname_source_, inc_frame.mc_g_name_, inc_frame.header_.packet_id, inc_frame.header_.packet_sequence_num);
            }

            if (propagated)
            {
                /* RESEND FRAME */

                stc_RoutedFrame stc_rf;
                stc_rf.frame = *f;
                stc_rf.frame_is_ack = false;
                stc_rf.retransmitted = 0;
                stc_rf.time_stamp = getMillisecondsTime();

                routing_entry route;
                route.hostname_source = f->hostname_source_;
                memcpy(route.next_hop, bcast_mac, 6);
                route.id = -1;

                boost::unique_lock<boost::mutex> lock(mtx_mc_ack_l);
                boost::unique_lock<boost::mutex> lock_groups(mtx_mc_groups);
                McTree* mc_t = mc_handler.getMcGroup(&inc_frame.mc_g_name_);
                McPosAckObj* ack_obj = new McPosAckObj(&inc_frame, mc_t);

                mc_ack_l.push_front(ack_obj);

                boost::thread resendMcAckFrameT(deliverMcAckFrame, stc_rf, route, recursive_mc_ack);
            }
        }


        if (propagated)
        {
            memcpy((unsigned char*) inc_frame.header_.mac_destination_, (unsigned char*) bcast_mac, 6);
            inc_frame.resend_flag = false;
            string network_string = inc_frame.getFrameAsNetworkString(inc_frame.header_.route_id, inc_frame.header_.mac_destination_, inc_frame.hostname_source_, src_mac);
            socketSend(network_string);
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
            Logging::increaseProperty("num_mc_unique_data_frames_forwarded");
            Logging::increaseProperty("num_mc_total_bytes_sent ", network_string.length());
#endif
            ROS_DEBUG("FORWARD MC FRAME: ID[%u] GROUP[%s] SOURCE[%s]", inc_frame.header_.frame_id, inc_frame.mc_g_name_.c_str(), inc_frame.hostname_source_.c_str());
        }
        else
            ROS_DEBUG("Don't propergate mc frame: ID[%u] group[%s] source[%s]", inc_frame.header_.frame_id, inc_frame.mc_g_name_.c_str(), inc_frame.hostname_source_.c_str());

        {
            boost::unique_lock<boost::mutex> lock(mtx_inc_rf_frames);
            inc_frames_l.push_back(inc_frame);
        }


    }/* BROADCAST */
    else
    {
        //ROS_DEBUG("GOT BROADCAST: SOURCE HOST[%s] ID[%u]",getMacAsCStr(inc_frame.mac_source_),inc_frame.header_.frame_id);

#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
        Logging::increaseProperty("num_unique_data_frames_received_directly");
        Logging::increaseProperty("num_total_bytes_received", f->buffer_str_len_);
#endif

        boost::unique_lock<boost::mutex> lock(mtx_inc_rf_frames);
        inc_frames_l.push_back(inc_frame);
    }



}

void processRouteRequest(RouteRequest req)
{
    /* Description:
     * Process a Route Request
     * Behavior of the function is documented within the function
     */

    /*check if i got the request already*/
    route_request inc_req;
    inc_req.id = req.header_.id;
    inc_req.hostname_source = req.hostname_source_;
    bool route_req_exsits = true;
    {
        std::list<route_request>::iterator findReq;
        {
            boost::unique_lock<boost::mutex> lock_rreq(mtx_route_requests);
            findReq = std::find(route_requests_l.begin(), route_requests_l.end(),
                    inc_req);
            route_req_exsits = findReq != route_requests_l.end();
        }

        /* LOGGING */

        boost::unique_lock<boost::mutex> lock_mc_groups(mtx_mc_groups);
        McTree* mc_g = mc_handler.getMcGroup(&req.hostname_destination_);
        if ((req.hostname_destination_.compare(hostname) == 0 || (mc_g != NULL && mc_g->connected && mc_g->activated)))
        {
            lock_mc_groups.unlock();
            uint16_t counter = 1;
            if (route_req_exsits)
            {
                boost::unique_lock<boost::mutex> lock_rreq(mtx_route_requests);
                route_request & route_req(*findReq);
                counter = ++route_req.counter;
            }

            mac m_adr(src_mac);
            req.path_l_.push_back(m_adr);
            lock_mc_groups.lock();
            {

                boost::unique_lock<boost::mutex> lock_rreq(mtx_route_requests);
                Logging::logRRequestReceiver(req.hostname_destination_, req.header_.id, counter, req.header_.hop_count - 1, counter == 1, req.path_l_);

            }
            req.path_l_.pop_back();
        }
    }


    /*Check if the incoming request is not from myself*/
    if (req.hostname_source_.compare(hostname) != 0 && (req.header_.hop_count + 1 <= req.header_.hop_limit || req.header_.hop_limit == 0) && !route_req_exsits)//(!routeReqestExsists || req.mc_flag_))
    {
        if (req.mc_flag_ == false)
        {
            /*Check if I am the destination for the request*/
            if (req.hostname_destination_.compare(hostname) == 0)
            {
                inc_req.counter = 1;
                inc_req.response_sent = true;
                RouteResponse rr = RouteResponse(req, src_mac, 0);

                ROS_INFO("GOT ROUTE REQUEST FOR ME: SOURCE[%s] ID[%u]", req.hostname_source_.c_str(), req.header_.id);

                /*Safe route */
                routing_entry new_route;
                new_route.hostname_destination = hostname;
                new_route.current_hop = rr.current_hop_;
                new_route.hobs = req.header_.hop_count - 1;
                new_route.hostname_source = req.hostname_source_;
                new_route.id = req.header_.id;
                new_route.cr_entry = false;
                memcpy(new_route.previous_hop, ethernet_header->eh_source, 6);
                memcpy(new_route.next_hop, src_mac, 6);
                ROS_INFO("ADD ROUTE: ID[%u] SOURCE[%s] DEST[%s] NEXT HOP[%s] PREVIUS HOP[%s] PATH:[%s]", new_route.id, new_route.hostname_source.c_str(), new_route.hostname_destination.c_str(), getMacAsStr(new_route.next_hop).c_str(), getMacAsStr(new_route.previous_hop).c_str(), getPathAsStr(rr.path_l_).c_str());

                /* Logging */
                {
                    boost::unique_lock<boost::mutex> lock(mtx_routing_table);
                    cleanRoutingTable();
                    routing_table_l.push_front(new_route);
                    boost::unique_lock<boost::mutex> lock_g(mtx_mc_groups);
                    Logging::logRoutingTable(&routing_table_l, &mc_groups_l);
                }


                /* If the source and the destination are direct neighbors, 
                 * the request process duration is too fast for the "request resend thread".
                 * The source gets the response before that thread can be started and so the thread will resend the request for no reason  
                 * To avoid this problem, a little delay must be added for this case
                 */
                if (simulation_mode && new_route.hobs <= 1)
                    sleepMS(10);

                string network_string = rr.getResponseAsNetworkString(src_mac);

                socketSend(network_string);
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
                Logging::increaseProperty("num_rreq_received");
                Logging::increaseProperty("num_rrep_sent");
                Logging::increaseProperty("num_total_bytes_received", req.buffer_str_len_);
                Logging::increaseProperty("num_total_bytes_sent", network_string.length());

#endif


            }/*If not: forward the request*/
            else
            {
                ROS_INFO("FORWARD REQUEST: SOURCE HOST[%s] DESTINATION HOST[%s] MAX HOPS[%u] CURRENT HOP[%u]", req.hostname_source_.c_str(), req.hostname_destination_.c_str(), req.header_.hop_limit, req.header_.hop_count + 1);
                //DEBUG
                inc_req.response_sent = false;
                string network_string = req.getRequestAsNetworkString(src_mac);
                socketSend(network_string);

                /*Logging */
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
                Logging::increaseProperty("num_rreq_sent");
                Logging::increaseProperty("num_total_bytes_sent", network_string.length());
#endif
                Logging::logRRequestIntermediate(&req);
            }
        }
        else if (req.mc_flag_)
        {
            boost::unique_lock<boost::mutex> lock(mtx_mc_groups);
            McTree* mc_t = mc_handler.getMcGroup(&req.hostname_destination_);

            /* Check if I have a entry of the mc group */
            if (mc_t != NULL && mc_t->connected && mc_t->activated) // || mc_t.connected == false)
            {
                /* return if request is from the uplink */
                if (!mc_t->root && compareMac(mc_t->route_uplink_->next_hop, req.eh_h_.eh_source))
                    return;
                inc_req.counter = 1;
                inc_req.response_sent = true;


                routing_entry * route = new routing_entry;
                route->id = req.header_.id;
                route->hostname_source = req.hostname_source_;
                route->hostname_destination = req.hostname_destination_;
                memcpy((void*) route->previous_hop, (void*) req.eh_h_.eh_source, 6);
                route->mac_path_l = std::list<mac>(req.path_l_);
                // x frames in y seconds then timeout. timout is time 4 nacks

                mc_handler.addDownlinkRoute(route);

                RouteResponse rr = RouteResponse(req, src_mac, mc_t->route_uplink_->root_distance + req.header_.hop_count);

                // ROS_DEBUG("Path[%s]", getPathAsCStr(req.path_l_));
                ROS_INFO("GOT MC REQUEST FOR ME AS DEST: MC GROUP[%s] SOURCE[%s] ID[%u] HOPS[%u] RD[%u]", mc_t->group_name_.c_str(), req.hostname_source_.c_str(), req.header_.id, req.header_.hop_count, mc_t->route_uplink_->root_distance);
                string network_string = rr.getResponseAsNetworkString(src_mac);
                socketSend(network_string);

#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
                Logging::increaseProperty("num_mc_rrep_sent");
                Logging::increaseProperty("num_mc_total_bytes_sent ", network_string.length());
#endif
            }
            else if (mc_t != NULL && mc_t->outgoing_request_ != NULL && getMillisecondsTime() - mc_t->time_stamp_ < INTERVAL_WAIT_FOR_MCROUTES)
            {
                RouteRequest* request = new RouteRequest(req);

                if (mc_t->addWaitingRequest(request, src_mac))
                {
                    //ROS_ERROR("ADD REQUEST IN WAITING LIST: DEST[%s] PATH:[%s]", request->hostname_destination_.c_str(), getPathAsStr(request->path_l_).c_str());
                    //  printRouteRequest(request);
                }
                else
                {
                    // ROS_ERROR("unexpected failure: 2399923");
                    //  delete request;
                }

                return;
            }
            else if (!route_req_exsits) // only forward request if I have not already forwarded the req
            {
                ROS_INFO("FORWARD MC REQUEST: SOURCE HOST[%s] DESTINATION GROUP[%s] MAX HOPS[%u] CURRENT HOP[%u]", req.hostname_source_.c_str(), req.hostname_destination_.c_str(), req.header_.hop_limit, req.header_.hop_count + 1);
                inc_req.response_sent = false;

                mc_handler.addGroup(&req.hostname_destination_);
                mc_handler.getMcGroup(&req.hostname_destination_)->safeOutgoingRequest(new RouteRequest(req));
                {
                    boost::unique_lock<boost::mutex> lock(mtx_routing_table);
                    Logging::logRoutingTable(&routing_table_l, &mc_groups_l);
                }
                string network_string = req.getRequestAsNetworkString(src_mac);
                socketSend(network_string);

                /*Logging */
                Logging::logRRequestIntermediate(&req);

#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
                Logging::increaseProperty("num_mc_rreq_sent");
                Logging::increaseProperty("num_mc_total_bytes_sent ", network_string.length());
#endif
            }
            else
            {
                if (mc_t != NULL)
                {
                    ROS_ERROR("mc_t != NULL");
                    mc_t->printTree();
                }
                else
                {
                    ROS_ERROR("return");
                }
                /* return the function to prevent that the request will be stored if routeRequest already exists */
                return;
            }
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
            Logging::increaseProperty("num_mc_rreq_received");
            Logging::increaseProperty("num_mc_total_bytes_received ", req.buffer_str_len_);
#endif
        }

        /*Safe request*/
        memcpy((void*) inc_req.receiver_mac, (void*) req.eh_h_.eh_source, 6);
        inc_req.hostname_destination = req.hostname_destination_;
        inc_req.is_mc = req.mc_flag_;

        boost::unique_lock<boost::mutex> lock(mtx_route_requests);
        route_requests_l.push_front(inc_req);
    }
}

void processRouteResponse(RouteResponse rr)
{
    /* Description:
     * Process a Route Response
     * Behavior of the function is documented within the function
     */
    /*Check if i already sent an response */
    route_request incoming_req;
    incoming_req.id = rr.request_id_;
    incoming_req.hostname_source = rr.hostname_source_;

    boost::unique_lock<boost::mutex> lock_route_req(mtx_route_requests);
    std::list<route_request>::iterator find_rreq = std::find(route_requests_l.begin(), route_requests_l.end(), incoming_req); // find the response

    /*Check if was involved in the route request process, so that i can forward the response*/
    if (find_rreq != route_requests_l.end())
    {
        route_request & request(*find_rreq); // safe a reference of the request
        routing_entry new_route;
        /*Check if i am the sender to forward and if I already forward the response*/
        //  std::string currenthop = getHostnameFromMac( rr.mac_current_hop_);
        //  ROS_DEBUG("current host: %s %u",currenthop.c_str(),request.response_sent);
        if (compareMac(rr.mac_current_hop_, src_mac) && request.response_sent == false)
        {
            /*Create route */
            new_route.hobs = rr.hop_count_ - 1;
            new_route.mac_path_l = rr.path_l_;
            new_route.ts = getMillisecondsTime();
            new_route.hostname_source = rr.hostname_source_;
            new_route.id = rr.request_id_;
            new_route.cr_entry = false;
            new_route.current_hop = rr.current_hop_;
            memcpy(new_route.next_hop, ethernet_header->eh_source, 6);
            RouteResponse resp_2_get_next_hop =
                    RouteResponse(
                    (unsigned char*) ((std::string) rr.getResponseAsNetworkString(
                    src_mac)).data());
            memcpy((unsigned char*) new_route.previous_hop,
                    (void*) resp_2_get_next_hop.mac_current_hop_, 6);

            if (rr.mc_flag_ == false)
            {
                /*The response is for me */
                if (rr.current_hop_ == rr.hop_count_ && rr.hostname_source_ == hostname)
                {
                    ROS_INFO("GOT ROUTE RESPONSE FROM[%s]", request.hostname_destination.c_str());
                    ROS_INFO("PATH[%s]", getPathAsStr(rr.path_l_).c_str());

                    new_route.hostname_destination = request.hostname_destination;

                    route_requests_l.remove(request);

                    {
                        boost::unique_lock<boost::mutex> lock(mtx_routing_table);
                        cleanRoutingTable();
                        routing_table_l.push_front(new_route);
                        boost::unique_lock<boost::mutex> lock_g(mtx_mc_groups);
                        Logging::logRoutingTable(&routing_table_l, &mc_groups_l);

                    }

                    boost::unique_lock<boost::mutex> lock(mtx_notify);
                    got_request_response.notify_all();

#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
                    Logging::increaseProperty("num_rrep_received");
                    Logging::increaseProperty("num_total_bytes_received", rr.hostname_source_.length() + RouteResponse::HEADER_FIXED_LEN);
#endif
                }/*Forward the response*/
                else
                {

                    ROS_INFO("FORWARD RESPONSE FROM[%s]: SOURCE HOST[%s] DESTINATION HOST[%s] NEXT HOP[%s]", getMacAsStr(new_route.next_hop).c_str(), new_route.hostname_source.c_str(), new_route.hostname_destination.c_str(), getMacAsStr(new_route.previous_hop).c_str());
                    ROS_DEBUG("PATH[%s]", getPathAsStr(rr.path_l_).c_str());

                    request.response_sent = true;
                    new_route.hostname_destination = "";
                    {
                        boost::unique_lock<boost::mutex> lock(
                                mtx_routing_table);
                        cleanRoutingTable();
                        routing_table_l.push_front(new_route);
                        boost::unique_lock<boost::mutex> lock_g(mtx_mc_groups);
                        Logging::logRoutingTable(&routing_table_l, &mc_groups_l);
                    }
                    string network_string = rr.getResponseAsNetworkString(src_mac);
                    socketSend(network_string);
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
                    Logging::increaseProperty("num_rrep_sent");
                    Logging::increaseProperty("num_total_bytes_received", network_string.length());
#endif
                }
            }/* MULTICAST */
            else
            {
                /* Check if the mc tree has not been activated*/
                {
                    boost::unique_lock<boost::mutex> lock(mtx_mc_groups);
                    McTree* tree = mc_handler.getMcGroup(&request.hostname_destination);
                    if (tree != NULL && tree->activated && tree->connected)
                        return;
                }

                routing_entry* route = new routing_entry;
                route->hostname_destination = request.hostname_destination;
                route->hostname_source = request.hostname_source;
                route->current_hop = rr.current_hop_;
                route->hobs = rr.current_hop_ - 1;
                route->root_distance = rr.root_distance - (rr.hop_count_ - rr.current_hop_);
                route->cr_entry = false;
                route->id = rr.request_id_;
                route->mac_path_l = rr.path_l_;

                memcpy(route->previous_hop, new_route.previous_hop, 6);
                memcpy(route->next_hop, new_route.next_hop, 6);
                {
                    boost::unique_lock<boost::mutex> lock(mtx_mc_groups);
                    if (!mc_handler.addUplinkRoute(route))
                    {
                        delete route;
                        return;
                    }
                }

                if (rr.hostname_source_.compare(hostname) == 0)
                {
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
                    Logging::increaseProperty("num_mc_rrep_received");
                    Logging::increaseProperty("num_mc_total_bytes_received", rr.hostname_source_.length() + RouteResponse::HEADER_FIXED_LEN);
#endif
                    //ROS_ERROR("%s", getPathAsCStr(rr.path_l_));

                    ROS_INFO("GOT MC RESPONSE: ROOT DISTANCE[%u] FROM[%s] MC GROUP[%s]", route->root_distance, getMacAsStr(rr.eh_.h_source).c_str(), request.hostname_destination.c_str());
                    //ROS_INFO("PATH: %s", getPathAsCStr(rr.path_l_));
                }
                else
                {
                    /*forward*/
                    ROS_INFO("FORWARD MC RESPONSE FROM[%s]: SOURCE HOST[%s] DESTINATION MC[%s] NEXT HOP[%s]", getMacAsStr(rr.eh_.h_source).c_str(), rr.hostname_source_.c_str(), request.hostname_destination.c_str(), getMacAsStr(new_route.previous_hop).c_str());
                    //  ROS_DEBUG("PATH[%s]",getPathAsCStr(rr.path_l_));

                    string network_string = rr.getResponseAsNetworkString(src_mac);
                    socketSend(network_string);
                    request.response_sent = true;

#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
                    Logging::increaseProperty("num_mc_rrep_sent");
                    Logging::increaseProperty("num_mc_total_bytes_sent ", network_string.length());
#endif
                }
            }
        }
        else if (request.response_sent)
            ROS_INFO("I SENT ALREADY A RESPONSE: ID[%u] SOURCE HOST[%s]", rr.request_id_, rr.hostname_source_.c_str());
        else if (compareMac(rr.mac_current_hop_, src_mac) == false)
            ROS_ERROR("I AM NOT THE CURRENT HOP[%s] TO FORWARD THE RESPONSE!", getMacAsStr(rr.mac_current_hop_).c_str());



    }
    else
        ROS_DEBUG("WAS NOT INVOLVED IN ROUTING PROCESS: ID[%u] SOURCE HOST[%s]", rr.request_id_, rr.hostname_source_.c_str());

}

void processBeacon(Beacon * beacon)
{
    /* Description:
     * Process a Hello Frame.
     * Add new neighbor in list if new neighbor is detected.
     * If neighbor already exists, reset the no_hello_hot counter to zero.
     */

#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
    Logging::increaseProperty("num_beacons_received");
    Logging::increaseProperty("num_total_bytes_received", beacon->HEADER_FIXED_LEN + beacon->hostname_.length());
#endif
    // ROS_ERROR("GOT BEACON FROM [%s]", getMacAsCStr(beacon->mac_source_));
    hostname_mac hm;
    hm.hostname = beacon->hostname_;
    hm.reachable = true;
    memcpy(hm.mac, beacon->mac_source_, 6);

    bool new_neighbor = true;
    bool neighbor_reconnect = false;

    boost::unique_lock<boost::mutex> lock_neighbors(mtx_neighbors);

    std::list<hostname_mac>::iterator current_neighbor = std::find(neighbors_l.begin(), neighbors_l.end(), hm); // checks if the frame already exsists. return pointer to end if not
    if (current_neighbor != neighbors_l.end())
    {
        new_neighbor = false;

        (*current_neighbor).stamp(); // set the hello count of all neighbors that already exsit to 0

        if (!(*current_neighbor).reachable)
        {
            neighbor_reconnect = true;
        }

        (*current_neighbor).reachable = true;
    }

    if (new_neighbor || neighbor_reconnect)
    {
        std_msgs::String msg;
        msg.data = hm.hostname;
        lock_neighbors.unlock();
        publishMessage(msg, node_prefix + topic_new_robot);
        lock_neighbors.lock();
        ROS_ERROR("NEW NEIGHBOR: NAME[%s]", hm.hostname.c_str());

        if (new_neighbor)
        {
            neighbors_l.push_front(hm);
        }
    }
}

void processMcActivationFrame(McRouteActivationFrame * f)
{

    if (compareMac(f->header_.mac_destination, src_mac) == false)
        return;

    boost::unique_lock<boost::mutex> lock_mc_groups(mtx_mc_groups);

    sendLinkAck(f->eh_h_.eh_dest, src_mac, f->header_.id, "", false, FRAME_TYPE_MC_ACTIVATION);

#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
    Logging::increaseProperty("num_mc_ract_received");
    Logging::increaseProperty("num_mc_total_bytes_received ", f->buffer_str_len_);
#endif

    McTree* mc_t = mc_handler.getMcGroup(&f->mc_group_);

    /* active member*/
    if (mc_t != NULL && ((mc_t->connected && mc_t->activated) || mc_t->root))
    {
        mac* downlink = new mac(f->eh_h_.eh_source);
        if (mc_t->addDownlinkAsMember(downlink))
        {

            ROS_INFO("ADD NEW LEAFE TO MC TREE: GROUP[%s] LEAFE MAC[%s]", f->mc_group_.c_str(), getMacAsStr(f->eh_h_.eh_source).c_str());
            //mc_handler.printMcGroups();
        }
        else
        {
            //ROS_ERROR("unexpected failure: could not add member[%s] as downlink", getMacAsCStr(downlink->mac_adr));

            //  mc_t->printTree();
            //delete downlink;
        }
    } /* connector*/
    else if ((mc_t = mc_handler.getMcGroup(&f->hostname_source_, &f->header_.route_id)) != NULL)
    {

        mac* downlink = new mac;
        memcpy((void*) downlink->mac_adr, (void*) f->eh_h_.eh_source, 6);
        if (mc_t->addDownlinkAsConnector(downlink))
        {
            ROS_INFO("ACT AS CONNECTOR FOR MC TREE: GROUP[%s]", mc_t->group_name_.c_str());
            if (!mc_t->activateRoute(&f->hostname_source_, &f->header_.route_id, f->eh_h_.eh_source))
                ROS_ERROR("unexpected failure: could not activate mc tree as connector");


            McRouteActivationFrame r_act_frame = McRouteActivationFrame(mc_t->route_uplink_->next_hop, mc_t->group_name_, mc_t->route_uplink_->id, mc_t->route_uplink_->hostname_source);

            string network_string = r_act_frame.getFrameAsNetworkString(src_mac);
            lock_mc_groups.unlock();
            /*SAFE the mc activation as unacknowledged*/
            resendUnackLinkFrame("", r_act_frame.header_.id, r_act_frame.header_.mac_destination, network_string, FRAME_TYPE_MC_ACTIVATION);

            socketSend(network_string);
            lock_mc_groups.lock();


#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
            Logging::increaseProperty("num_mc_ract_sent");
            Logging::increaseProperty("num_mc_total_bytes_sent ", network_string.length());
#endif

            /* Send response to waiting requests */
            while (!mc_t->waiting_requests_l_.empty())
            {

                RouteRequest *req = mc_t->waiting_requests_l_.front();
                mc_t->waiting_requests_l_.pop_front();
                lock_mc_groups.unlock();
                processRouteRequest(*req);
                lock_mc_groups.lock();

                delete req;
            }
        }
        else
            ROS_ERROR("unexpected failure: could not add downlink as connector");
    }
    else if ((mc_t = mc_handler.getMcGroup(&f->mc_group_)) != NULL)
    {
        ROS_ERROR("act from: %s", getMacAsStr(f->eh_h_.eh_source).c_str());
        mc_t->printTree();
    }
    else if (mc_t == NULL)
    {
        ROS_ERROR("ERROR: 87");
        mc_t->printTree();
    }



}

void processIncomingFrames()
{
    /* Description:
     * Process a Packet.
     * Creates a new packet if the frame belongs to a new packet. If packet already exists, the frame will be added to the packet.
     * This function also delete obsolete incomplete packets.
     * If the packet is complete it will be published.
     */

    while (ros::ok())
    {
        list<RoutedFrame> frames;
        bool list_is_empty = true;
        {
            frames.clear();
            boost::unique_lock<boost::mutex> lock(mtx_inc_rf_frames);
            frames = list<RoutedFrame>(inc_frames_l);
            inc_frames_l.clear();
            list_is_empty = inc_frames_l.empty();
        }





        //   ROS_ERROR("%u", frames.size());
        while (frames.size() >= 1)
        {

            {
                unsigned long time_lock = getMillisecondsTime();
                boost::unique_lock<boost::mutex> lock_packets_inco(mtx_packets_incomplete);
               // if (getMillisecondsTime() - time_lock >= 100)
               //     ROS_ERROR("dont got lock 1");
                for (std::list<Packet>::iterator p_i = packets_incomplete_l.begin(); p_i != packets_incomplete_l.end();)
                {

                    bool packet_is_complete = false;
                    Packet & p(*p_i);

                    for (std::list<RoutedFrame>::iterator f_i = frames.begin(); f_i != frames.end();)
                    {

                        RoutedFrame & frame(*f_i);

                        if (p.id_ == frame.header_.packet_id
                                && frame.header_.packet_size > 1
                                && p.hostname_source_.compare(
                                frame.hostname_source_) == 0)
                        {

                            if (p.addFrame(frame))
                            {
                                publishPacket(&p);
                                packet_is_complete = true;
                            }

                            f_i = frames.erase(f_i);
                        }
                        else
                            f_i++;
                    }

                    if (packet_is_complete)
                    {

                        Packet p_comp = Packet(*p_i);
                        p_comp.hostname_source_ = (*p_i).hostname_source_;
                        if (p_comp.isMcFrame())
                        {

                            boost::unique_lock<boost::mutex> lock(mtx_cached_mc_packets);

                            for (std::list<Packet>::iterator it = cached_mc_packets_l.begin(); it != cached_mc_packets_l.end(); it++)
                            {
                                Packet pa = *it;
                                if (pa.id_ == p_comp.id_
                                        && pa.hostname_source_.compare(p_comp.hostname_source_) == 0
                                        && pa.mc_group_.compare(p_comp.mc_group_) == 0)
                                {
                                    /* add the ordered completed one*/
                                    cached_mc_packets_l.push_back(p_comp);
                                    /*remove the unordered uncompleted one*/
                                    cached_mc_packets_l.erase(it);

                                    break;
                                }
                            }
                        }

                        //ROS_DEBUG("packet is full %u",p_comp.id_);
                        p_i = packets_incomplete_l.erase(p_i);

                    }
                    else if (getMillisecondsTime() - p.ts_ > INTERVAL_DELETE_OLD_PACKETS)
                    {
                        ROS_INFO("DROP UNFINISHED PACKET: ID[%u] SOURCE[%s] SIZE[%u/%lu]", p.id_, p.hostname_source_.c_str(), p.size_, p.frames_l_.size());
                        p_i = packets_incomplete_l.erase(p_i);
                    }
                    else
                        p_i++;

                }
            }


            for (std::list<RoutedFrame>::iterator f_i = frames.begin(); f_i != frames.end();)
            {

                RoutedFrame & frame(*f_i);

                bool new_packet = true;

                /* Look in incomplete packets */
                {
                    unsigned long time_lock = getMillisecondsTime();
                    boost::unique_lock<boost::mutex> lock_packets_inco(mtx_packets_incomplete);
                    if (getMillisecondsTime() - time_lock >= 100)
                        ROS_WARN("did not got lock 2");
                    for (std::list<Packet>::iterator it = packets_incomplete_l.begin(); it != packets_incomplete_l.end(); it++)
                    {
                        Packet p = *it;
                        if (p.id_ == frame.header_.packet_id
                                && p.hostname_source_.compare(frame.hostname_source_) == 0)
                        {
                            new_packet = false;
                            break;
                        }
                    }
                }

                bool got_packet_already = false;
                /* Look in complete packets */
                if (new_packet)
                {
                    unsigned long time_lock = getMillisecondsTime();
                    boost::unique_lock<boost::mutex> lock(mtx_cached_mc_packets);
                    if (getMillisecondsTime() - time_lock >= 100)
                        ROS_WARN("did not got lock 3");



                    for (std::list<Packet>::iterator it_pack = cached_mc_packets_l.begin(); it_pack != cached_mc_packets_l.end(); it_pack++)
                    {
                        if ((*it_pack).id_ == frame.header_.packet_id && (*it_pack).hostname_source_.compare(frame.hostname_source_) == 0 && (*it_pack).mc_group_.compare(frame.mc_g_name_) == 0 && (*it_pack).size_ <= (*it_pack).frames_l_.size() && (*it_pack).size_ > 1)
                        {
                            new_packet = false;
                            //f_i = frames.erase(f_i);
                            got_packet_already = true;

                            break;
                        }
                    }
                }

                if (new_packet)
                {

                    Packet p = Packet(frame);
                    ROS_DEBUG("ADD NEW PACKET: ID[%u] GROUP[%s] SOURCE[%s] SIZE[%u]", p.id_, p.mc_group_.c_str(), p.hostname_source_.c_str(), p.size_);

                    if (p.addFrame(frame))
                        publishPacket(&p);
                    else
                    {
                        boost::unique_lock<boost::mutex> lock_packets_inco(mtx_packets_incomplete);
                        packets_incomplete_l.push_back(p);
                    }



                    f_i = frames.erase(f_i);


                }
                else if (got_packet_already)
                    f_i = frames.erase(f_i);

                else
                {
                    // ROS_WARN("warn");

                    f_i++;
                }


            }




        }


        //  if (list_is_empty)
        sleepMS(10);
    }

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
                desializeObject((unsigned char*) payload.data(), payload.length(), &pos);
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

void resendRouteRequest(route_request &req, boost::condition_variable & con)
{
    /* Description:
     * Re-Send a route_request and notify if the request has its max retransmissions reached.
     * Also the hop_limit_max will be increased if i get no response. The thread stops also, if he will be interrupted.
     * The thread is supposed to be interrupted if the node gets an response of the route request
     *
     * NOTE: This functions will be executed as a own thread.
     */

    sleepMS(INTERVAL_RESEND_ROUTE_REQUEST);

    while (boost::this_thread::interruption_requested() == false)
    {

        if (req.retransmitted >= (unsigned) num_rreq)
        {
            if (req.hop_limit < (unsigned) hop_limit_max)
            {
                req.hop_limit += hop_limit_increment;
                req.retransmitted = 0;
                ROS_INFO("INCREASE HOP COUNT OF ROUTE REQUEST: ID[%u] DESTINAION HOST[%s] MAX HOPS[%u]", req.id, req.hostname_destination.c_str(), req.hop_limit);
            }
            else
            {
                ROS_WARN("DROP ROUTE REQUEST: ID[%u] DESTINAION HOST[%s]", req.id, req.hostname_destination.c_str());

                {
                    boost::unique_lock<boost::mutex> lock(mtx_route_requests);
                    route_requests_l.remove(req);
                }

                boost::unique_lock<boost::mutex> lock(mtx_notify);
                con.notify_all();

                return;
            }
        }
        else
        {
            req.id++;
            ROS_INFO("RESEND ROUTE REQUEST: ID[%u] DESTINAION HOST[%s] MAX HOPS[%u]", req.id, req.hostname_destination.c_str(), req.hop_limit);
            RouteRequest rr = RouteRequest(req);
            rr.req_count_stat++; // static count

            req.ts = getMillisecondsTime();
            req.retransmitted++;
            {
                boost::unique_lock<boost::mutex> lock(mtx_route_requests);
                route_requests_l.push_front(req);
            }
            string network_string = rr.getRequestAsNetworkString(src_mac);
            socketSend(network_string);
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
            Logging::increaseProperty("num_rreq_sent");
            Logging::increaseProperty("num_total_bytes_sent", network_string.length());
#endif    
        }

        sleepMS(INTERVAL_RESEND_ROUTE_REQUEST);
    }
}

void resendRoutedFrame(boost::condition_variable &condi, routing_entry route)
{

    /* Description:
     * Re-Send the unacknowledged routed frame and notify if the routed frame has its max retransmissions reached.
     * The thread stops also, if it will be interrupted.
     * The thread is supposed to be interrupted if the node gets an response of the route request
     *
     * NOTE: This functions will be executed as a own thread.
     */

    Logging::increaseProperty("running_unicast_transport_threads");
    sleepMS(INTERVAL_RESEND_ROUTED_FRAME);

    while (boost::this_thread::interruption_requested() == false)
    {
        {
            boost::unique_lock<boost::mutex> lock(mtx_unack_routed_frame);

            if (unack_routed_frame->retransmitted >= num_e2e_retrans)
            {
                boost::unique_lock<boost::mutex> lock(mtx_notify);
                unack_routed_frame->frame_is_ack = false;
                condi.notify_all();
                Logging::decreaseProperty("running_unicast_transport_threads");
                return;
            }
            else
            {
                unack_routed_frame->frame.header_.frame_id = unack_routed_frame->frame.frame_count_stat;
                unack_routed_frame->frame.frame_count_stat++;
                unack_routed_frame->time_stamp = getMillisecondsTime();
                unack_routed_frame->retransmitted++;
                unack_routed_frame->frame.resend_flag = true;

                string network_string = unack_routed_frame->frame.getFrameAsNetworkString(route, src_mac);
                //ROS_DEBBUG("RESEND ROUTED FRAME: ID[%u] DESTINAIONT HOST[%s] NEXT HOP[%s]", unack_routed_frame->frame.header_.frame_id, unack_routed_frame->hostname_destination.c_str(), getMacAsStr(unack_routed_frame->frame.header_.mac_destination_).c_str());
                socketSend(network_string);



#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
                Logging::increaseProperty("num_data_frames_resent");
                Logging::increaseProperty("num_total_bytes_sent", network_string.length());
#endif
            }
        }
        sleepMS(INTERVAL_RESEND_ROUTED_FRAME);
    }

    Logging::decreaseProperty("running_unicast_transport_threads");

    return;
}

void resendMcFrame(boost::condition_variable &condi, stc_RoutedFrame* rf, routing_entry route)
{
    Logging::increaseProperty("running_multicast_threads");

    sleepMS(INTERVAL_RESEND_ROUTED_FRAME);

    if (!boost::this_thread::interruption_requested())
    {
        rf->retransmitted++;
        rf->frame.resend_flag = true;

        // ROS_DEBUG("RESEND MC FRAME: ID[%u] SRC HOST[%s] NEXT HOP[%s]", rf->frame.header_.frame_id, rf->frame.hostname_source_.c_str(), getMacAsStr(rf->frame.header_.mac_destination_).c_str());
        string network_string = rf->frame.getFrameAsNetworkString(route, src_mac);
        socketSend(network_string);
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
        Logging::increaseProperty("num_mc_data_frames_resent");
        Logging::increaseProperty("num_mc_total_bytes_sent ", network_string.length());
#endif
    }

    if (!boost::this_thread::interruption_requested())
    {
        boost::unique_lock<boost::mutex> lock(mtx_notify);
        condi.notify_all();
    }

    Logging::decreaseProperty("running_multicast_threads");
}

void deleteObsoleteRequests()
{
    /* Description:
     * This function delete obsolete route request from route_requests_l.
     *
     * NOTE: This functions will be executed as a own thread.
     */

    while (ros::ok())
    {
        {
            boost::unique_lock<boost::mutex> lock(mtx_route_requests);
            std::list<route_request>::iterator r = route_requests_l.begin();

            while (r != route_requests_l.end()) // check if the route request is from me
            {
                route_request & currentRequest(*r);
                if (getMillisecondsTime() - currentRequest.ts > MAX_TIME_CACHE_ROUTE_REQUEST)
                {
                    ROS_DEBUG("DROP CACHED ROUTE REQUEST: ID[%u] DESTuINAION HOST[%s]", currentRequest.id, currentRequest.hostname_destination.c_str());
                    r = route_requests_l.erase(r);
                }
                else
                    r++;
            }
        }
        sleepMS(INTERVAL_UPTDATE_THREAD_TO_DELETE_OBSOLETE_REQUESTS);
    }
}

void deleteOldPackets()
{
    /* Description:
     * This function delete obsolete route request from route_requests_l.
     *
     * NOTE: This functions will be executed as a own thread.
     */

    while (ros::ok())
    {
        {
            boost::unique_lock<boost::mutex> lock(mtx_cached_mc_packets);
            std::list<Packet>::iterator r = cached_mc_packets_l.begin();

            while (r != cached_mc_packets_l.end())
            {
                Packet & p(*r);
                if ((p.size_ <= p.frames_l_.size() && getMillisecondsTime() > p.ts_ && getMillisecondsTime() - p.ts_ >= INTERVAL_DELETE_OLD_PACKETS) || !p.isMcFrame())
                {
                    ROS_INFO("DROP OLD PACKET: ID[%u] GROUP[%s] SRC HOST[%s]", p.id_, p.mc_group_.c_str(), p.hostname_source_.c_str());
                    r = cached_mc_packets_l.erase(r);
                }
                else
                    r++;
            }
        }

        {
            boost::unique_lock<boost::mutex> lock(mtx_packets_incomplete);
            std::list<Packet>::iterator r = packets_incomplete_l.begin();

            while (r != packets_incomplete_l.end())
            {
                Packet & p(*r);
                if ((getMillisecondsTime() > p.ts_ && getMillisecondsTime() - p.ts_ >= INTERVAL_DELETE_OLD_PACKETS) && p.isMcFrame())
                {
                    ROS_INFO("DROP OLD UNFINISHED PACKET: ID[%u] GROUP[%s] SRC HOST[%s]", p.id_, p.mc_group_.c_str(), p.hostname_source_.c_str());
                    r = packets_incomplete_l.erase(r);
                }
                else
                    r++;
            }
        }

        sleepMS(INTERVAL_UPTDATE_THREAD_TO_DELETE_OBSOLETE_REQUESTS);
    }
}

void resendLinkFrame(stc_frame f)
{

    if (num_link_retrans == 0)
        return;

    Logging::increaseProperty("running_unicast_link_threads");
    f.retransmitted = 0;
    bool got_ack = false;
    do
    {
        sleepMS(INTERVAL_TO_RETRANSMIT_LINK_FRAMES);

        bool frame_successfully_acknowleged = false;

        {
            boost::unique_lock<boost::mutex> lock(mtx_unack_link_frames);
            std::list<stc_frame>::iterator cr_frame_it = std::find(unack_link_frames_l.begin(), unack_link_frames_l.end(), f);
            frame_successfully_acknowleged = cr_frame_it == unack_link_frames_l.end();
        }

        if (frame_successfully_acknowleged == false)
        {
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY

            if (f.type == FRAME_TYPE_TRANSPORT_ACK || f.type == FRAME_TYPE_TRANSPORT_DATA)
            {
                if (f.hostname_source.compare(hostname) == 0)
                    Logging::increaseProperty("num_data_frames_resent");
                else
                    Logging::increaseProperty("num_data_frames_reforwarded");
            }

            if (f.mc_group.compare("") == 0)
                Logging::increaseProperty("num_total_bytes_sent", f.network_string.length());
#endif

#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
            if (f.mc_group.compare("") != 0)
                Logging::increaseProperty("num_mc_total_bytes_sent ", f.network_string.length());
#endif

            //    ROS_ERROR("RESEND LINK FRAME: ID[%u] CONFIRMER MAC[%s] SOURCE HOST[%s] TYPE[%s] RELAY[%u]", f.frame_id, getMacAsStr(f.mac).c_str(), f.hostname_source.c_str(), frame_types[f.type].c_str(), f.cr);
            socketSend(f.network_string);
            f.retransmitted++;
        }
        else
        {
            got_ack = true;
        }
    } while (f.retransmitted < num_link_retrans && !got_ack && ros::ok());

    if (!got_ack)
    {
        {
            boost::unique_lock<boost::mutex> lock(mtx_unack_link_frames);
            std::list<stc_frame>::iterator cr_frame_it = std::find(unack_link_frames_l.begin(), unack_link_frames_l.end(), f);
            if (cr_frame_it != unack_link_frames_l.end())
            {
                unack_link_frames_l.erase(cr_frame_it);
                ROS_WARN("DROP FRAME: ID[%u] CONFIRMER MAC[%s] SOURCE HOST[%s] TYPE[%s]", f.frame_id, getMacAsStr(f.mac).c_str(), f.hostname_source.c_str(), frame_types[f.type].c_str());
            }
        }
        if (f.type == FRAME_TYPE_MC_ACTIVATION)
        {
            hostname_mac n;
            memcpy(n.mac, f.mac, 6);
            n.hostname = getHostnameFromMac(f.mac);

            //  mcLostConnection(n);
            boost::thread t(&mcLostConnection, n);
        }
    }

    Logging::decreaseProperty("running_unicast_link_threads");
}

