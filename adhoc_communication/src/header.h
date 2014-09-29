/*
 * \header.h
 *
 * \date 13.08.2013
 * \author Günhter Cwioro
 * g.cwioro@gmx.net
 */

#ifndef HEADER_H_
#define HEADER_H_


/*General includes*/
#include "ros/ros.h"

//#include <iwlib.h>
//#include <wireless.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include <stdio.h>
#include <sys/ioctl.h>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/crc.hpp>
using std::list;
using std::string;


/*Network specific includes*/
#include <sys/socket.h>
#include <netinet/in.h>
#include <linux/if.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <asm/types.h>

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
#include "adhoc_communication/CMgrDimensions.h"
#include "adhoc_communication/CMgrRobotUpdate.h"

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
#include "adhoc_communication/SendTwist.h"
#include "adhoc_communication/GetGroupState.h"
#include "adhoc_communication/SendCMgrRobotUpdate.h"
#include "adhoc_communication/ShutDown.h"
#include "adhoc_communication/BroadcastCMgrRobotUpdate.h"
#include "adhoc_communication/BroadcastString.h"


#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include "functions.h"

/*Defines for timers and retransmits)*/
#include "defines.h"


#include "EthernetFrame.h"
#include "EthernetFrame.cpp"
/*Struct defines*/
#include "structs.h"

/*Classes for the AD-HOC protocol*/

#include "Beacon.cpp"
#include "Beacon.h"
#include "AckLinkFrame.h"
#include "AckLinkFrame.cpp"
#include "McAckFrame.h"
#include "McAckFrame.cpp"
#include "RouteRequest.h"
#include "RouteRequest.cpp"
#include "RouteResponse.cpp"
#include "RouteResponse.h"
#include "AckRoutedFrame.h"
#include "AckRoutedFrame.cpp"
#include "McRouteActivationFrame.h"
#include "McRouteActivationFrame.cpp"
#include "Packet.cpp"
#include "PositionSubscriber.h"
#include "PositionSubscriber.cpp"
#include "McDisconnectFrame.h"
#include "McDisconnectFrame.cpp"
#include "MultiHopBroadcastFrame.h"
#include "MultiHopBroadcastFrame.cpp"
#include "McNackFrame.h"
#include "McNackFrame.cpp"

#include "McHandler.h"
#include "McHandler.cpp"
#include "McPosAckObj.h"
#include "McPosAckObj.cpp"


#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>



std::map<uint16_t, std::string> frame_types;

void handler(int sig)
{
    void *array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);

    // print out all the frames to stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}



/*VARS DECLARATION*/
unsigned char* interface = NULL; // using outgoing interface
std::string hostname; // hostname of the node
std::string mac_as_string = "";
std::string interface_as_string = "";
uint32_t packet_id_ = -1;


void socketSend(string network_string);

void deliverMcAckFrame(stc_RoutedFrame& stc_rf, routing_entry& r, bool send_ack);
geometry_msgs::PoseStamped getPoseStampFromNetworkString(unsigned char* serializedPoseStamp, uint32_t length);
nav_msgs::OccupancyGrid getMapFromNetworkString(unsigned char* serializedMap, uint32_t length);
bool joinMCGroup(adhoc_communication::ChangeMCMembership::Request &req, adhoc_communication::ChangeMCMembership::Response &res);

#include "Logging.h"
#include "Logging.cpp"

void processMcAckFrame(McAckFrame* f);

list<McTree*> mc_groups_l;
McHandler mc_handler(&mc_groups_l);
list<McPosAckObj*> mc_ack_l;

list<stc_frame> unack_link_frames_l;
list<ack_cr_info> unack_cr_frames_l;
list<ack_cr_info> unack_mc_cr_frames_l;
list<bcasts> bcasts_l;

stc_RoutedFrame* unack_routed_frame; // there can be just one unacknowledged routed frame at a time, because the protocol waits for the ack until the next routed frame will be send
list<route_request> route_requests_l;
list<routing_entry> routing_table_l;

std::map<unsigned char*, string> mac_hname_dict;
list<hostname_mac> neighbors_l; // list with all reachable direct neighbors. This list is not used for the routing process. Nodes detect each other by "hello frames"
list<Packet> packets_incomplete_l; // list to cache incomplete packets
list<Packet> cached_mc_packets_l;
list<RoutedFrame> inc_frames_l;
list<McNackFrame> requested_frames_l;

list<stc_packet> published_packets_l;

list<PositionSubscriber*> robot_positions_l;
PositionSubscriber* my_sim_position;




RoutedFrame frames_received_recently_a[MAX_FRAMES_CACHED]; // static array to cache frames incoming frames. This is needed, because frames are always received twice by the receive function
uint16_t frr_index = 0; // current index to insert incoming frame.
uint16_t frr_count = 0; // count to know how many frames are cached. This is only important at the beginning, later frr_count will always be MAX_FRAMES_CACHED

/*MUTEXES to share data over different threads*/
boost::mutex mtx_relays;
boost::mutex mtx_routing_table;
boost::mutex mtx_cr_entries;
boost::mutex mtx_route_requests;
boost::mutex mtx_unack_routed_frame;
boost::mutex mtx_neighbors;
boost::mutex mtx_unack_link_frames;
boost::mutex mtx_unack_cr_frames;
boost::mutex mtx_unack_mc_cr_frames;

boost::mutex mtx_frames_received_recently;
boost::mutex mtx_inc_rf_frames;
boost::mutex mtx_requested_frames;
boost::mutex mtx_packets_incomplete;
boost::mutex mtx_cached_mc_packets;

boost::mutex mtx_mc_ack_l;
boost::mutex mtx_mc_groups;

// Vars for communication
int raw_socket = 0; // Socket descriptor
struct sockaddr_ll socket_address;
unsigned char src_mac[6]; /*Local host NIC MAC address*/
struct eh_header* ethernet_header;


//VARS for the configurable parameter
int num_link_retrans = 3;
int num_e2e_retrans = 10; //1;
int num_rreq = 1;
int max_frame_size = 1500;
int hop_limit_min = 0;
int hop_limit_max = 0;
int hop_limit_increment = 3;
int max_packet_size = 10000;
int beacon_interval = 500;
bool enable_cooperative_relaying = false;
bool rebuild_mc_tree = false;
bool recursive_mc_ack = false;
double loss_ratio = 0;
std::string topic_new_robot = "new_robot";
std::string topic_remove_robot = "remove_robot";
std::string log_path = "";
std::string sim_robot_macs = "";

/*Simulation*/
bool simulation_mode = false;
int robots_in_simulation = 10;
int p_thres = -75;
int p_tx = 15; //[dBm]
int n_model = 4;
int l_0_model = 33;
#ifdef DELAY
int delay = 1; //[us]
#endif

std::list<ros::Publisher> publishers_l;

ros::NodeHandle *n_priv;
ros::NodeHandle *n_pub;

boost::mutex mtx_notify;

boost::condition_variable got_request_response;
boost::condition_variable got_rframe_ack;


std::string node_prefix = "adhoc_communication/";


std::string getHostnameFromMac(unsigned char mac[6]);

bool isReachable(unsigned char mac[6]);
void initParams(ros::NodeHandle* n);

bool getRoute(routing_entry& r, std::string hostname);
void safeFrame(RoutedFrame incomingFrame);

void sendLinkAck(unsigned char* dest, unsigned char* confirmer_mac, uint32_t id, string source, bool cr, uint8_t type);
bool gotAck(AckLinkFrame* frame);
bool gotFrameRecently(RoutedFrame incomingFrame);
void publishMap(nav_msgs::OccupancyGrid map, std::string topic);
void publishPacket(Packet* p);

void addDownlinkToMcTree(mc_tree tree, mac downlink);
mc_tree getMCTree(std::string mc_name);
void mcLostConnection(hostname_mac host);
bool connectedWith(unsigned char mac[6]);

void resendLinkFrame(stc_frame f);
void processAckRoutedFrame(AckRoutedFrame* fr);
void processRoutedFrame(RoutedFrame* f);
void processBroadcastFrame(RoutedFrame* f);
void processRouteRequest(RouteRequest req);
void processRouteResponse(RouteResponse rr);
void processBeacon(Beacon* helloF);
void processMcActivationFrame(McRouteActivationFrame * f);

void resendMcFrame(boost::condition_variable &condi, stc_RoutedFrame* rf, routing_entry route);
void resendRouteRequest(route_request &req, boost::condition_variable &con);
void resendRoutedFrame(boost::condition_variable &condi, routing_entry route);
void reconnectToMcGroup(std::string group_name);
void deleteObsoleteRequests();
void deleteOldPackets();

void sendBeacons();
void receiveFrames();
void processIncomingFrames();
void close_raw_socket();
int eth_raw_init();

McPosAckObj* getMcAckObj(std::string* group, uint32_t p_id, uint32_t seq);

void resendRequestedFrame(uint32_t* seq_num, uint32_t* packet_id, string* source_h, string* mc_group);
void resendRequestedFrames();


bool sendPacket(std::string &hostname_destination, std::string& payload, uint8_t data_type_, std::string& topic);

void initRobotMacList(std::string* robot_mac);
template<class message>
void publishMessage(message m, string topic);


std::string getListAsString(std::list<uint32_t> l);

#ifdef DEBUG
#include "DebugFunctions.h"
#endif

uint32_t incoming_frame_count = 0;
uint32_t incoming_frames_lost = 0;

int eth_raw_init()
{


    /* Vars */
    int i;
    struct ifreq ifr;
    int ifindex = 0; /*Ethernet Interface index*/

    /* Open socket */
    raw_socket = socket(PF_PACKET, SOCK_RAW, htons(ETH_TYPE));
    if (raw_socket == -1)
    {
        perror("socket():");
        exit(1);
    }
    ROS_INFO("Successfully opened socket: %i\n", raw_socket);

    /*Get ethernet interface index*/
    strncpy(ifr.ifr_name, (const char*) interface, IFNAMSIZ);
    if (ioctl(raw_socket, SIOCGIFINDEX, &ifr) == -1)
    {
        perror("SIOCGIFINDEX");
        exit(1);
    }
    ifindex = ifr.ifr_ifindex;
    ROS_INFO("Successfully got interface index: %i\n", ifindex);

    /*retrieve corresponding MAC*/
    if (ioctl(raw_socket, SIOCGIFHWADDR, &ifr) == -1)
    {
        perror("SIOCGIFINDEX");
        exit(1);
    }

    if (mac_as_string.compare("") == 0)
    {
        for (i = 0; i < 6; i++)
        {
            src_mac[i] = ifr.ifr_hwaddr.sa_data[i];
        }
        ROS_INFO("Host MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);
    }

    /*prepare sockaddr_ll*/
    socket_address.sll_family = PF_INET;
    socket_address.sll_protocol = htons(ETH_TYPE);
    socket_address.sll_ifindex = ifindex;
    //socket_address.sll_hatype   = ARPHRD_ETHER;
    socket_address.sll_pkttype = PACKET_BROADCAST;
    socket_address.sll_halen = ETH_ALEN;
    socket_address.sll_addr[0] = bcast_mac[0];
    socket_address.sll_addr[1] = bcast_mac[1];
    socket_address.sll_addr[2] = bcast_mac[2];
    socket_address.sll_addr[3] = bcast_mac[3];
    socket_address.sll_addr[4] = bcast_mac[4];
    socket_address.sll_addr[5] = bcast_mac[5];
    socket_address.sll_addr[6] = 0x00;
    socket_address.sll_addr[7] = 0x00;

    return 0;

}

void close_raw_socket()
{
    /*Clean up.......*/

    struct ifreq ifr;

    if (raw_socket == -1)
        return; /* No socket has beeen created */

    strncpy(ifr.ifr_name, (const char*) interface, IFNAMSIZ);
    ioctl(raw_socket, SIOCGIFFLAGS, &ifr);
    ifr.ifr_flags &= ~IFF_PROMISC;
    ioctl(raw_socket, SIOCSIFFLAGS, &ifr);
    close(raw_socket);

}

bool getRoute(routing_entry& r, std::string hostname)
{
    /* Description:
     * Search a route to the destination host in the routing table.
     * The parameter of the type routing_entry will be initialized, if a route can be found.
     *
     * Returns:
     * 		(true)	If route has been found
     *
     * 		(false) If route has not been found
     */
    boost::unique_lock<boost::mutex> lock(mtx_routing_table);

    for (std::list<routing_entry>::iterator it = routing_table_l.begin(); it != routing_table_l.end(); ++it)
    {

        if ((*it).hostname_destination.compare(hostname) == 0)
        {
            r = *it;
            r.ts = getMillisecondsTime();
            return true;
        }
    }
    return false;
}

void socketSend(string network_string)
{
    sendto(raw_socket, network_string.data(), network_string.length(), 0, (struct sockaddr*) &socket_address, sizeof (socket_address));

#ifdef DELAY

    if (simulation_mode)

        boost::this_thread::sleep(boost::posix_time::milliseconds(delay)); //usleep(delay);
#endif
}

void resendUnackLinkFrame(string hostname_src, uint32_t id, unsigned char* mac, string network_string, uint8_t type)
{
    /*SAFE the mc activation as unacknowledged*/
    stc_frame unack_link_frame;
    unack_link_frame.frame_id = id;
    memcpy(unack_link_frame.mac, mac, 6);
    unack_link_frame.hostname_source = hostname_src;
    unack_link_frame.type = type;
    unack_link_frame.network_string = network_string;

    {
        boost::unique_lock<boost::mutex> lock(mtx_unack_link_frames);
        unack_link_frames_l.push_back(unack_link_frame);
    }

    boost::thread cr(&resendLinkFrame, unack_link_frame);
}

void cleanRoutingTable()
{
    unsigned long now = getMillisecondsTime();
    for (list<routing_entry>::iterator i = routing_table_l.begin(); i != routing_table_l.end();)
    {

        if (now - (*i).ts >= MAX_TIME_CACHE_UNUSED_ROUTES)
            i = routing_table_l.erase(i);
        else
            i++;
    }
}

void initFrameTypes()
{
    frame_types[0x61] = "Link Ack";
    frame_types[0x42] = "Beacon";
    frame_types[0x52] = "RReq";
    frame_types[0x72] = "RRep";
    frame_types[0x44] = "Data";
    frame_types[0x41] = "Transport Ack";
    frame_types[0x64] = "Relay Detection";
    frame_types[0x53] = "Reley Delection";
    frame_types[0x5d] = "MC ACK";
    frame_types[0x59] = "MC ACTIVATION";
    frame_types[0x50] = "MC DISCONNECTION";
}

/* disconnection frame that comes from a downlink to disconnect from an uplink*/
void processMcDisconnectUplink(McDisconnectFrame* f)
{
    if (!compareMac(f->header_.mac_destination, src_mac))
        return;

    sendLinkAck(f->eh_h_.eh_dest, src_mac, f->header_.id, "", false, FRAME_TYPE_MC_DISCONNECT);

    boost::unique_lock<boost::mutex> groups_lock(mtx_mc_groups);

    McTree* group = mc_handler.getMcGroup(&f->mc_group_);

    if (group == NULL)
    {
        return;
    }

#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
    groups_lock.unlock();
    Logging::increaseProperty("num_mc_total_frames_received");
    Logging::increaseProperty("num_mc_total_bytes_received ", f->buffer_str_len_);
    groups_lock.lock();
#endif

    if (!group->removeMacIfExsists(f->eh_h_.eh_source))
    {
        ROS_ERROR("Unexpected Error: %s is not a downlink for group %s", getMacAsStr(f->eh_h_.eh_source).c_str(), f->mc_group_.c_str());
        return;
    }

    if (group->member)
    {
        groups_lock.unlock();
       // ROS_ERROR("GOT DISCONNECT MSG FROM DOWNLINK: GROUP[%s] DOWNLINK[%s]", f->mc_group_.c_str(), getHostnameFromMac(f->eh_h_.eh_source).c_str());
        groups_lock.lock();
    } else
    {
        adhoc_communication::ChangeMCMembership::Request req;
        adhoc_communication::ChangeMCMembership::Response res;
        req.action = false;
        req.group_name = f->mc_group_;
        groups_lock.unlock();
        joinMCGroup(req, res);
        groups_lock.lock();
    }
}

/* disconnection frame that comes from a uplink to rebuild the whole mc path*/
void processMcDisconnectDownlink(McDisconnectFrame* f)
{
    sendLinkAck(f->eh_h_.eh_dest, src_mac, f->header_.id, "", false, FRAME_TYPE_MC_DISCONNECT);

    boost::unique_lock<boost::mutex> groups_lock(mtx_mc_groups);
    McTree* group = mc_handler.getMcGroup(&f->mc_group_);

    if (group == NULL)
        return;

#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
    groups_lock.unlock();
    Logging::increaseProperty("num_mc_total_frames_received");
    Logging::increaseProperty("num_mc_total_bytes_received ", f->buffer_str_len_);
    groups_lock.lock();
#endif

    if (!compareMac(f->eh_h_.eh_source, group->route_uplink_->next_hop))
    {
        return;
    }

    group->connected = false;
    //ROS_ERROR("GOT MC PRUNE MESSAGE GROUP[%s] HOST[%s]", f->mc_group_.c_str(), getHostnameFromMac(f->eh_h_.eh_source).c_str());

    if (!group->downlinks_l_.empty())
    {
      //  ROS_ERROR("FORWARD MC PRUNE GROUP[%s] HOST[%s]", group->group_name_.c_str(), getHostnameFromMac(f->eh_h_.eh_source).c_str());
        group->downlinks_l_.clear();
        mtx_mc_groups.unlock();
        McDisconnectFrame dis_f(bcast_mac, f->mc_group_);
        dis_f.disconnect_downlink = true;
        string network_string = dis_f.getFrameAsNetworkString(src_mac);

        socketSend(network_string);
        mtx_mc_groups.lock();
    }

    /*Try to reconn*/
    if (group->member)
    {
        /* set connection to false*/
        boost::thread tryToJoin(&reconnectToMcGroup, f->mc_group_);

    } else
        mc_handler.removeGroup(&f->mc_group_);
}

void testPacket()
{
    int packet_size = 10;
    int packet_id = 0;
    Packet p;
    std::vector<RoutedFrame> l;

    for (int packet_seq = 0; packet_seq < packet_size; packet_seq++)
    {
        RoutedFrame f("topic", "random_data", 2, packet_id, packet_seq, packet_size);
        l.push_back(f);

        if (packet_seq == 0)
            p = Packet(f);

        if (packet_seq % 2 == 0)
        {
            ROS_ERROR("add %u", packet_seq);
            p.addFrame(f);
        } else
            ROS_ERROR("skip %u", packet_seq);
    }

    ROS_ERROR("missings seq: %s %u %lu %lu", getListAsString(p.missed_sequences_l_).c_str(), p.size_, p.frames_l_.size(), p.missed_sequences_l_.size());

    p.refreshLists();

    ROS_ERROR("missings seq: %s %u %lu %lu", getListAsString(p.missed_sequences_l_).c_str(), p.size_, p.frames_l_.size(), p.missed_sequences_l_.size());
}

bool gotFrameRecently(RoutedFrame rf)
{
    /* Description:
     * Checks if frame is in frames_received_recently_a.
     *
     * Returns:
     * 		(true) 	If got frame already
     *
     * 		(false) If frame is new
     */
    //ROS_DEBUG("");

    /* Allways process frames with the resend flag*/
    if (rf.resend_flag && rf.mc_flag == false)
    {
        return false;
    }

    boost::unique_lock<boost::mutex> lock(mtx_frames_received_recently);

    int interations = 0;
    //	ROS_DEBUG("COMPARE FRAME SEQ NUM[%u] PACKET ID[%u] SOURCE[%s] ",rf.header_.packet_sequence_num,rf.header_.packet_id,rf.hostname_source_.c_str());
    /*The first index to search will be the last of insert this makes search faster, because it happens often that a frame will be received twice from the network interface*/
    for (int i = frr_index > 0 ? frr_index - 1 : frr_count - 1; interations < frr_count;)
    {

        //	ROS_DEBUG("IT SEQ[%u] PACKET ID[%u] SOURCE[%s] ",frames_received_recently_a[i].header_.packet_sequence_num,frames_received_recently_a[i].header_.packet_id,frames_received_recently_a[i].hostname_source_.c_str());
        /* UNICAST */
        if (!rf.mc_flag && !rf.resend_flag &&
                frames_received_recently_a[i].header_.packet_sequence_num == rf.header_.packet_sequence_num &&
                frames_received_recently_a[i].header_.packet_id == rf.header_.packet_id &&
                frames_received_recently_a[i].hostname_source_.compare(rf.hostname_source_) == 0)// &&
        {
            //	ROS_DEBUG("GOT FRAME RECENTLY: SEQ NUM[%u] PACKET ID[%u] SOURCE[%s] ",rf.header_.packet_sequence_num,rf.header_.packet_id,rf.hostname_source_.c_str());
            return true;
        }

        if (rf.mc_flag && frames_received_recently_a[i].header_.packet_id == rf.header_.packet_id &&
                frames_received_recently_a[i].header_.packet_sequence_num == rf.header_.packet_sequence_num &&
                frames_received_recently_a[i].hostname_source_.compare(rf.hostname_source_) == 0 &&
                frames_received_recently_a[i].mc_g_name_.compare(rf.mc_g_name_) == 0
                )
        {
            //ROS_DEBUG("GOT MC FRAME RECENTLY: SEQ NUM[%u] PACKET ID[%u] SOURCE[%s] ",rf.header_.packet_sequence_num,rf.header_.packet_id,rf.hostname_source_.c_str());
            return true;
        }

        interations++;

        if (i - 1 > 0)
            i--;
        else
            i = frr_count - 1;

    }

    return false;
}

void safeFrame(RoutedFrame incomingFrame)
{
    /* Description:
     * Safe frame in frames_received_recently_a
     */

    boost::unique_lock<boost::mutex> lock(mtx_frames_received_recently);
    //	ROS_DEBUG("safe frame %u %s",incomingFrame.frame_id,incomingFrame.hostname_source.c_str());

    frames_received_recently_a[frr_index].header_.packet_sequence_num = incomingFrame.header_.packet_sequence_num;
    frames_received_recently_a[frr_index].header_.packet_id = incomingFrame.header_.packet_id;
    frames_received_recently_a[frr_index].header_.frame_id = incomingFrame.header_.frame_id;
    frames_received_recently_a[frr_index].hostname_source_ = incomingFrame.hostname_source_;
    frames_received_recently_a[frr_index].mc_g_name_ = incomingFrame.mc_g_name_;
    memcpy(frames_received_recently_a[frr_index++].header_.mac_destination_, incomingFrame.header_.mac_destination_, 6);


    //frames_received_recently_a[frr_index] = RoutedFrame(incomingFrame);

    if (frr_count < MAX_FRAMES_CACHED)
        frr_count++;

    if (frr_index >= MAX_FRAMES_CACHED)
        frr_index = 0;

}

std::string getHostnameFromMac(unsigned char mac[6])
{
    /* Description:
     * Returns a hostname of a mac using neighbor table.
     *
     * NOTE: This function is not used in this version.. but in earlier implementations.
     */

    // boost::unique_lock<boost::mutex> lock(mtx_neighbors);
    hostname_mac hm;
    for (std::list<hostname_mac>::iterator it = neighbors_l.begin(); it != neighbors_l.end(); ++it)
    {
        hm = *it;

        if (compareMac(hm.mac, mac))
        {
            return hm.hostname;
        }
    }
    return getMacAsStr(mac);
}

void initParams(ros::NodeHandle* n)
{
    /*GET PRIVATE PARAMETER*/
    /*		NAME						VARIABLE					DEFAULT VALUE*/

    n->param("interface", interface_as_string, std::string("wlan0"));
    n->param("mac", mac_as_string, std::string(""));
    n->param("num_link_retrans", num_link_retrans, 10);
    n->param("num_e2e_retrans", num_e2e_retrans, 10);
    n->param("num_rreq", num_rreq, 1);
    n->param("max_frame_size", max_frame_size, 1500);
    n->param("hop_limit_min", hop_limit_min, 0);
    n->param("hop_limit_max", hop_limit_max, 0);
    n->param("robot_name", hostname, hostname);
    n->param("hop_limit_increment", hop_limit_increment, 3);
    n->param("beacon_interval", beacon_interval, 250);
    n->param("max_packet_size", max_packet_size, 10000000);
    n->param("simulation_mode", simulation_mode, false);
    n->param("robots_in_simulation", robots_in_simulation, 10);
    n->param("p_thres", p_thres, -75);
    n->param("p_tx", p_tx, 15);
    n->param("n_model", n_model, 4);
    n->param("l_0_model", l_0_model, 33);
    n->param("topic_new_robot", topic_new_robot, std::string("new_robot"));
    n->param("topic_remove_robot", topic_remove_robot, std::string("remove_robot"));
   
    n->param("rebuild_mc_tree", rebuild_mc_tree, false);
    n->param("recursive_mc_ack", recursive_mc_ack, false);
    n->param("loss_ratio", loss_ratio, loss_ratio);
    n->param("nack_threshold", Packet::NACK_THRESHOLD, Packet::NACK_THRESHOLD);

    n->param("sim_robot_macs", sim_robot_macs, std::string("")); // exp: "robot_0,00:11:00:00:00:00!robot_0,00:11:00:00:00:00
    RoutedFrame::enable_cooperative_relaying = enable_cooperative_relaying;

    //ROS_ERROR("mac from param %s",mac_as_string.c_str());

    if (mac_as_string.compare("") != 0)
    {

        initMacFromString(src_mac, mac_as_string.data());
    }
    //  ROS_ERROR("parameters int: %s mac: %s real mac %s",interface_as_string.c_str(),mac_as_string.c_str(), getMacAsCStr(src_mac));

    interface = (unsigned char*) interface_as_string.data();

    initRobotMacList(&sim_robot_macs);

    /*SET TX POWER ON HARDWARE IF SIMULATION MODE IS OFF*/
    if (simulation_mode == false)
    {
        std::string command = "iwconfig " + std::string((const char*) interface) + " txpower ";
        command.append(getIntAsString(p_tx) + "db");
        if (system(command.data()) == 0)
            ROS_INFO("TX POWER CHANGED IN HARDWARE TO %u dbm", p_tx);

    }
}

bool isReachable(unsigned char mac[6])
{
    if (simulation_mode)
    {
        PositionSubscriber other_robot;
        other_robot.robot_name_ = getHostnameFromMac(mac);

        for (std::list<PositionSubscriber*>::iterator it = robot_positions_l.begin(); it != robot_positions_l.end(); ++it)
        {
            if (other_robot.robot_name_.compare((*it)->robot_name_) == 0)
            {
                double d = my_sim_position->calcDistance(*it);

                //ROS_ERROR("d: %f",d);
                if (d == -1)
                {

                    return false;
                }

                double p_rx = p_tx - (l_0_model + 10 * n_model * log10(d));


                if (p_thres <= p_rx)
                {
                    //ROS_DEBUG("connected with %s %f %f",other_robot->robot_name_.c_str(),d,p_rx);
                    return true;
                } else
                    return false;

            }
        }

    } else if (sim_robot_macs.compare("") != 0)
    {
        boost::unique_lock<boost::mutex> lock(mtx_neighbors);
        hostname_mac n(mac);

        std::list<hostname_mac>::iterator searchNeighbor = std::find(neighbors_l.begin(), neighbors_l.end(), n);

        return searchNeighbor != neighbors_l.end();

    } else
        return true;
}

bool connectedWith(unsigned char mac[6])
{
    hostname_mac n;
    memcpy((unsigned char*) n.mac, (unsigned char*) mac, 6);
    boost::unique_lock<boost::mutex> lock(mtx_neighbors);
    std::list<hostname_mac>::iterator searchNeighbor = std::find(neighbors_l.begin(), neighbors_l.end(), n);

    // ROS_DEBUG("reachable %u %s", ((hostname_mac) * searchNeighbor).reachable, getMacAsCStr((unsigned char*) mac));

    return searchNeighbor != neighbors_l.end() && ((hostname_mac) * searchNeighbor).reachable;
}

void disconnectDownlinks(McTree* t)
{
    McDisconnectFrame disc_f(bcast_mac, t->group_name_);
    disc_f.disconnect_downlink = true;
    string network_string = disc_f.getFrameAsNetworkString(src_mac);

    for (list<mac*>::iterator i = t->downlinks_l_.begin(); i != t->downlinks_l_.end(); i++)
    {
        resendUnackLinkFrame("", disc_f.header_.id, (*i)->mac_adr, network_string, FRAME_TYPE_MC_DISCONNECT);
    }

    t->downlinks_l_.clear();

    socketSend(network_string);
}

void mcLostConnection(hostname_mac host)
{
    std::string mc_name = "";

    boost::unique_lock<boost::mutex> lock_g(mtx_mc_groups);

    std::vector<McTree*> trees = mc_handler.lostConnectionDownlinks(host.mac);

    while (!trees.empty())
    {
        McTree* t = trees.back();

        ROS_ERROR("REMOVE DOWNLINK: GROUP[%s]  HOST[%s]", t->group_name_.c_str(), host.hostname.c_str());
        trees.pop_back();

        /*disconnent from uplink if tree is just a connector*/
        if (t->downlinks_l_.empty() && !t->member)
        {
            adhoc_communication::ChangeMCMembership::Request req;
            adhoc_communication::ChangeMCMembership::Response res;
            req.action = false;
            req.group_name = t->group_name_;
            lock_g.unlock();
            joinMCGroup(req, res);
            lock_g.lock();
        }
    }
    trees = mc_handler.lostConnectionUplinks(host.mac);
    while (!trees.empty())
    {
        McTree* t = trees.back();
        trees.pop_back();

       // ROS_ERROR("LOST CONNECTION TO UPLINK GROUP[%s] HOST[%s]", t->group_name_.c_str(), host.hostname.c_str());

        if (rebuild_mc_tree)
        {
            /*disconnect from downlinks*/
            if (!t->downlinks_l_.empty())
            {
                ROS_INFO("SEND PRUNE MESSAGE TO TREE LEAFES GROUP[%s]", t->group_name_.c_str());
                disconnectDownlinks(t);
            } else
            {
                ROS_INFO("NO OTHER DOWNLINKS: GROUP[%s]", t->group_name_.c_str());
            }

            if (t->member)
            {
                // ROS_ERROR("try rejoin");
                t->connected = false;
                boost::thread tryToJoin(&reconnectToMcGroup, std::string(t->group_name_));
            } else
            {
                //  ROS_ERROR("delete group");
                mc_handler.removeGroup(&t->group_name_);
            }
        } else
        {
            t->connected = false;
            boost::thread tryToJoin(&reconnectToMcGroup, std::string(t->group_name_));
        }
    }
}

void cacheNackMcFrame(RoutedFrame rf)
{
    bool packet_exsists = false;
    //ROS_DEBUG("safe frame with %u",rf.packet_sequence_num_);
    boost::unique_lock<boost::mutex> lock(mtx_cached_mc_packets);

    for (std::list<Packet>::iterator it_p = cached_mc_packets_l.begin(); it_p != cached_mc_packets_l.end(); ++it_p)
    {
        Packet & p(*it_p);
        
        if (p.id_ == rf.header_.packet_id)
        {
            packet_exsists = true;
            if (rf.hostname_source_.compare(p.hostname_source_) == 0 && rf.mc_g_name_.compare(p.mc_group_) == 0)
            {
                bool frame_exsists = false;
                for (std::list<RoutedFrame>::iterator it_f = p.frames_l_.begin(); it_f != p.frames_l_.end(); ++it_f)
                {
                    RoutedFrame cur_f = *it_f;
                    if (cur_f.header_.packet_sequence_num == rf.header_.packet_sequence_num)
                    {
                        frame_exsists = true;
                        break;
                    }
                }
                //ROS_DEBUG("push front %u",p.frames_l_.size());
                if (!frame_exsists)
                    p.frames_l_.push_front(rf);

                return;
            }
        }
    }

    /*create new packet*/
    if (!packet_exsists)
    {
        Packet p = Packet(rf);
        p.addFrame(rf);
        cached_mc_packets_l.push_front(p);
    }
}

bool iAmMember(string group)
{
    boost::unique_lock<boost::mutex> lock_groups(mtx_mc_groups);
    McTree* t = mc_handler.getMcGroup(&group);

    return (t != NULL && t->activated && t->member);
}

void resendRequestedFrameFromPacket(McNackFrame nack)
{
    boost::unique_lock<boost::mutex> lock(mtx_cached_mc_packets);
    //list<Packet> cachedc_mc_packets_copy = list<Packet>(cached_mc_packets_l);

    for (std::list<Packet>::iterator it_p = cached_mc_packets_l.begin(); it_p != cached_mc_packets_l.end(); ++it_p)
    {
        /* packet found*/
        if ((*it_p).id_ == nack.header_.packet_id && nack.hostname_source_.compare((*it_p).hostname_source_) == 0 && nack.mc_group_.compare((*it_p).mc_group_) == 0)
        {
            bool one_frame_resent = false;

            std::vector<RoutedFrame> frames_in_packet((*it_p).frames_l_.begin(), (*it_p).frames_l_.end());
            lock.unlock();

            //ROS_DEBUG("pid %u size %u",p.id_,p.frames_l_.size());
            for (std::vector<RoutedFrame>::iterator it_f = frames_in_packet.begin(); it_f != frames_in_packet.end(); ++it_f)
            {
                RoutedFrame f = *it_f;

                list<uint32_t> l(nack.req_seq_nums_.begin(), nack.req_seq_nums_.end());

                //    ROS_ERROR("%s", getListAsString(l).c_str());

                if (std::find(nack.req_seq_nums_.begin(), nack.req_seq_nums_.end(), f.header_.packet_sequence_num) != nack.req_seq_nums_.end())
                {
                    one_frame_resent = true;
                    f.resend_flag = true;
                    string network_string = f.getFrameAsNetworkString(f.header_.route_id, bcast_mac, f.hostname_source_, src_mac);
                    //    ROS_ERROR("RESEND REQUESTED FRAME: SEQ NUM[%u] PACKET ID[%u] GROUP[%s] SOURCE[%s]", f.header_.packet_sequence_num, f.header_.packet_id, f.mc_g_name_.c_str(), f.hostname_source_.c_str());
                    socketSend(network_string);
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
                    Logging::increaseProperty("num_mc_data_frames_resent");
                    Logging::increaseProperty("num_mc_total_bytes_sent", network_string.length());
#endif          
                }
            }

            lock.lock();

            if (one_frame_resent)
            {
                (*it_p).ts_ = getMillisecondsTime();
            }

            return;
        }
    }
    // ROS_ERROR("COULD NOT FIND REQ PACKET: PACKET ID[%u] GROUP[%s] SOURCE[%s]", nack.header_.packet_id, nack.mc_group_.c_str(), nack.hostname_source_.c_str());
}

void resendRequestedFrames()
{
    while (ros::ok())
    {
        list<McNackFrame> frames;
        {
            frames.clear();
            boost::lock_guard<boost::mutex> lock(mtx_requested_frames);
            frames = list<McNackFrame>(requested_frames_l);
            requested_frames_l.clear();

            //	ROS_DEBUG("list :%u",frames.size());
        }

        if (frames.size() > 0)
        {
            list<McNackFrame>::iterator it = frames.begin();
            while (!frames.empty())
            {
                McNackFrame f = *it;
                resendRequestedFrameFromPacket(f);
                // resendRequestedFrame(&f.header_.frame_seq_num, &f.header_.packet_id, &f.hostname_source_, &f.mc_group_);
                it = frames.erase(it);
                // ros::Duration(0, INTERVAL_BETWEEN_RESEND_REQUESTED_FRAMES * 1000).sleep();
            }
        } else
        {
            ros::Duration(0, INTERVAL_BETWEEN_RESEND_REQUESTED_FRAMES * 1000).sleep();
        }
    }
}
//todo unicast_link_transmission.csv -> check // nur gesendete data frames lala


void reconnectToMcGroup(std::string group_name)
{
    if (rebuild_mc_tree)
    {
        //if a node gets a prune message, it will will a little to give the network time for synconization
        sleepMS(INTERVAL_WAIT_MC_RECONN);
    }

    int reconnect_tries = 0;
    adhoc_communication::ChangeMCMembership::Request req;
    adhoc_communication::ChangeMCMembership::Response res;
    req.action = false;
    while (ros::ok() && reconnect_tries <= MAX_JOIN_ATTEMPS)
    {
        {
            boost::unique_lock<boost::mutex> lock_groups(mtx_mc_groups);
            McTree* t = mc_handler.getMcGroup(&group_name);

            if (t != NULL && t->activated && (!t->downlinks_l_.empty() || t->member))
            {
                if (!t->connected)
                {
                    ROS_INFO("TRY RECONNECTION TO MC GROUP[%s]", group_name.c_str());
                    reconnect_tries++;
                    req.group_name = group_name;
                    req.action = true;
                    lock_groups.unlock();
                    joinMCGroup(req, res);
                    lock_groups.lock();

                } else
                    return;
            }
        }
        sleepMS(INTERWAL_RECONNECT_MC_G);
    }

    if (reconnect_tries <= MAX_JOIN_ATTEMPS)
    {
        /*downlink disconnection*/
        ROS_ERROR("STOP TRYING TO CONNECT TO MC GROUP: NAME[%s]", group_name.c_str());
        routing_entry route;
        route.hostname_source = group_name;
        memcpy(route.next_hop, bcast_mac, 6);

        boost::unique_lock<boost::mutex> lock_groups(mtx_mc_groups);
        McTree* t = mc_handler.getMcGroup(&group_name);
        if (t != NULL)
        {
            disconnectDownlinks(t);
            req.action = false;
            lock_groups.unlock();
            joinMCGroup(req, res);
            lock_groups.lock();
        }
    }
}

void joinAllMcGroups()
{
    ros::Duration(3, 0).sleep();
    adhoc_communication::ChangeMCMembership::Request req;
    req.action = true;
    adhoc_communication::ChangeMCMembership::Response res;
    for (int c = 0; c < robots_in_simulation; c++)
    { //robots_in_simulation



        req.group_name = "mc_robot_" + getIntAsString(c);

        ROS_ERROR("join %s", req.group_name.c_str());
        joinMCGroup(req, res);
        sleepMS(100);

        {
            boost::unique_lock<boost::mutex> lock_mc_groups(mtx_mc_groups);

            if (!mc_handler.getMcGroup(&req.group_name)->member)
            {
                lock_mc_groups.unlock();
                joinMCGroup(req, res);
                lock_mc_groups.lock();
            }
        }
    }
    ROS_ERROR("JOINING FINISHED!");
    mc_handler.printMcGroups();
#ifdef TRY_REJOIN

    mc_handler.printMcGroups();

    /*
   ros::Duration(2, 0).sleep();
   for (int c = 3; c < 4; c++) {//robots_in_simulation


       std::string g_name = "mc_robot_" + convertInt(c);
       McTree* mc_t = mc_handler.getMcGroup(&g_name);
       if(mc_t == NULL )
           ROS_ERROR("FATAL: %s", g_name.c_str());
       else if(!mc_t->activated)
            ROS_ERROR("NOT CONNECTED: %s", g_name.c_str());
      /*
       uint8_t joinAttemps = 0;
       while ((t.group_name.compare("") == 0 || !t.member) && joinAttemps++ < 10) {
           req.action = true;
           req.group_name = "mc_robot_" + convertInt(c);
           joinMCGroup(req, res);
           t = getActivatedMCTree("mc_robot_" + convertInt(c));
           ;
           ROS_ERROR("Join mc_robot_%u again", c);
       }
   }*/
#endif


}

struct frame_packet_info
{
    string source;
    string group_name;
    uint32_t seq_num;
    uint32_t packet_id;

    bool operator ==(const frame_packet_info& st)
    {
        if (seq_num == st.seq_num && packet_id == st.packet_id && group_name.compare(st.group_name) == 0 && source.compare(st.source))
            return true;

        return false;
    }
};

std::vector<McNackFrame> getMissingFrames(Packet * p)
{
    std::vector<McNackFrame> l;

    if (p->missed_sequences_l_.empty())
        p->refreshLists();

    std::vector<uint32_t> list_4_nack;

    for (std::list<uint32_t>::iterator i = p->missed_sequences_l_.begin(); i != p->missed_sequences_l_.end(); i++)
    {
        list_4_nack.push_back(*i);

        if (list_4_nack.size() % MAX_REQUESTED_FRAMES_IN_NACK == 0)
        {
            McNackFrame nack(src_mac, bcast_mac, p->hostname_source_, p->mc_group_, p->id_, list_4_nack);
            l.push_back(nack);
            list_4_nack.clear();
        }
    }

    if (list_4_nack.size() > 0)
    {
        McNackFrame nack(src_mac, bcast_mac, p->hostname_source_, p->mc_group_, p->id_, list_4_nack);
        l.push_back(nack);
    }

    return l;
}

Packet * getFirstMcFrame(list<Packet>* packets)
{
    for (std::list<Packet>::iterator p = packets->begin(); p != packets->end(); p++)
    {
        if ((*p).isNack() && (*p).size_ > (*p).frames_l_.size())
            return &(*p);
    }

    return NULL;
}

std::string getListAsString(std::vector<uint32_t> l)
{
    string r = "";

    for (std::vector<uint32_t>::iterator i = l.begin(); i != l.end(); i++)
        r.append(getIntAsString(*i) + ",");

    return r;
}

std::string getListAsString(std::list<uint32_t> l)
{
    string r = "";

    for (std::list<uint32_t>::iterator i = l.begin(); i != l.end(); i++)
        r.append(getIntAsString(*i) + ",");

    return r;
}

void updateTsRequestFrames(Packet p)
{
    boost::unique_lock<boost::mutex> lock(mtx_packets_incomplete);
    for (std::list<Packet>::iterator pi = packets_incomplete_l.begin(); pi != packets_incomplete_l.end(); pi++)
    {
        if ((*pi).id_ == p.id_ && (*pi).hostname_source_.compare(p.hostname_source_) == 0 && (*pi).mc_group_.compare(p.mc_group_) == 0)
        {
            (*pi).ts_last_frame_request = getMillisecondsTime();
            return;
        }
    }
}

void requestPendingFrames()
{
    std::list<Packet> pack_inco_copy;
    while (ros::ok())
    {
        sleepMS(INTERVAL_REQUEST_FRAMES);

        int requested_frames = 0;

        {
            pack_inco_copy.clear();
            boost::unique_lock<boost::mutex> lock(mtx_packets_incomplete);
            pack_inco_copy = std::list<Packet>(packets_incomplete_l);
        }

        for (std::list<Packet>::iterator p = pack_inco_copy.begin(); p != pack_inco_copy.end(); p++)
        {
            if (!(*p).isNack() || (*p).size_ <= (*p).frames_l_.size())
                continue;

            float perc_frames_got = (float) (*p).frames_l_.size() / (float) (*p).size_;
            //ROS_DEBUG("%f %u",perc_frames_got,p.id_);
            // if ((perc_frames_got >= START_RQUESTING_PACKET_FILL_LEVEL || getMillisecondsTime() - (*p).ts_ >= START_RQUESTING_INTERVAL_AFTER_LAST_FRAME_GOT) || !(*p).missed_sequences_l_.empty())

            if (getMillisecondsTime() - (*p).ts_ >= START_RQUESTING_INTERVAL_AFTER_LAST_FRAME_GOT || (!(*p).missed_sequences_l_.empty() && getMillisecondsTime() - (*p).ts_last_frame_request >= 200))
            {
                std::vector<McNackFrame> nacks = getMissingFrames(&(*p));

                while (!nacks.empty())
                {
                    updateTsRequestFrames(*p);

                    //   ROS_ERROR("%u %u %u ", (*p).size_, (*p).frames_l_.size(), (*p).missed_sequences_l_.size());

                    //  ROS_ERROR("REQUEST FRAMES: PACKET ID[%u] SEQ[%s] GROUP[%s] SOURCE[%s]", (*p).id_, getListAsString(nacks.back().req_seq_nums_).c_str(), (*p).mc_group_.c_str(), (*p).hostname_source_.c_str());

                    string network_string = nacks.back().getFrameAsNetworkString();
                    //    ROS_ERROR("%u %u %u %u", network_string.length(), nacks.back().req_seq_nums_.size(), nacks.size(), (*p).missed_sequences_l_.size());
                    nacks.pop_back();
                    socketSend(network_string);
#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
                    Logging::increaseProperty("num_mc_acknowledgments_sent");
                    Logging::increaseProperty("num_mc_total_bytes_sent", network_string.length());
#endif      
                    sleepMS(10);

                }
            }
        }
    }
}

void initRobotMacList(std::string * robot_mac)
{
    if (robot_mac->compare("") == 0)
        return;

    //input string should have the following format: hostname,mac!hostname2,mac2

    robot_mac->insert(0, "!");

    std::vector<std::string> hostnames_macs;
    boost::split(hostnames_macs, *robot_mac, boost::is_any_of("!"));

    for (std::vector<std::string>::iterator i = hostnames_macs.begin(); i != hostnames_macs.end(); i++)
    {
        std::string & h(*i);
        if (h.compare("") == 0)
            continue;

        try
        {
            //ROS_ERROR("hostname macs: %s", h.c_str());
            std::vector<std::string> hostname_mac_l;
            boost::split(hostname_mac_l, h, boost::is_any_of(","));
            //macToString(hostname_mac[1]);
            unsigned char* mac_p = new unsigned char[6];
            hostname_mac hm;
            hm.reachable = false;
            hm.hostname = hostname_mac_l[0];
            initMacFromString(hm.mac, hostname_mac_l[1].data());

            neighbors_l.push_back(hm);
            ROS_ERROR("ADD TO WHITELIST: %s [%s]", hm.hostname.c_str(), getMacAsStr(hm.mac).c_str());

        } catch (const std::out_of_range& oor)
        {
            ROS_ERROR("SYNTAX ERROR IN PARAMETER 'sim_robot_macs' %s ", robot_mac->c_str());
        }
    }
}

template <class t>
void desializeObject(unsigned char* serialized_pose_stamp, uint32_t length, t * obj)
{
    /* Description:
     * de-serialized a ROS message from a buffer.
     */
    // Fill buffer with a serialized UInt32
    try
    {
        ros::serialization::IStream stream(serialized_pose_stamp, length);
        ros::serialization::deserialize(stream, *obj);
    } catch (ros::serialization::StreamOverrunException e)
    {
        ROS_ERROR("IN desializeObject: NODE THROWS EXCEPTION: %s ", e.what());
        ROS_ERROR("PARAMETERS: length=[%u] object type=[%s]", length, typeid (*obj).name());
    }
}


#endif /* HEADER_H_ */
