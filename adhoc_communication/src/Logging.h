/* 
 * File:   Logging.h
 * Author: cwioro
 *
 * Created on August 8, 2014, 10:35 AM
 */

#ifndef LOGGING_H
#define	LOGGING_H

#include "Logging.h"
#include "ros/ros.h"
#include "structs.h"
#include "functions.h"
#include <boost/filesystem.hpp>

class Logging
{
    

public:
    Logging();
    Logging(const Logging& orig);
    static void init(ros::NodeHandle* n, std::string* robot_name);

    static void log();
    static bool createLogPath();
    static void periodicLog(unsigned long interval_ms);
    static void logServiceCalls(string service_name, unsigned long time_call, unsigned long time_res, unsigned long size, bool ret_val);
    static void logUcPacketsSummary(unsigned long interval_ms);
    static void logMcPacketsSummary(unsigned long interval_ms);
    static void logMemoryConsumptionPackets(unsigned long interval_ms, boost::mutex* mtx_pack, list<Packet>* packets, boost::mutex* mtx_cached_mc, list<Packet>* cached_mc, boost::mutex* mtx_unack_link_frames, list<stc_frame>* unack_link_frames, boost::mutex* mtx_unack_cr_frames, list<ack_cr_info>* unack_cr_frames);
    static void logRoutingTable(std::list<routing_entry>* u_entries, std::list<McTree*>* m_entries);
    static void logRRequestInitiater(route_request* req, routing_entry* route);
    static void logRRequestIntermediate(RouteRequest* r);
    static void logTransportFrame(stc_RoutedFrame* f, routing_entry* route, unsigned long start_time, unsigned long end_time, bool success);
    static void logUcLinkTransmission(stc_frame f);
    static void logRRequestReceiver(std::string h_src, uint32_t id, uint16_t counter, uint16_t hobs, bool selected, std::list<mac> path_l);
    // static void logRouteRequestReceiverUpdate(RouteResponse* res);
    static void createLogFile(std::list<std::string>* table, std::string* filename);
    static void safeLogs();

    static uint32_t getProperty(std::string prog_name);
    static void increaseProperty(std::string prop_name);
    static void increaseProperty(std::string prop_name, uint32_t value);
    static void decreaseProperty(std::string prop_name);
    static void decreaseProperty(std::string prop_name, uint32_t value);

    static ros::NodeHandle* n;
    //  static uint32_t running_unicast_cr_threads, running_multicast_threads, running_unicast_transport_threads, running_unicast_link_threads; // running threads   
    static boost::mutex mtx_logging;

    static std::map<std::string, uint32_t> uint32_properties;



    static std::list<std::string> entries_r_table, entries_rreq_initi, entries_rreq_recv, entries_rreq_interm, entries_mem_consumption, entries_uc_frames, entries_mc_frames, entries_link_frames, entries_service_calls, entries_transport_frames;
    static std::string log_path, routing_table_file, route_req_src_file, route_req_dst_file, route_req_iterm_file, mem_consumption_file, uc_frames_file, mc_frames_file, link_frames_file, sercice_calls_file, transport_frames_file;
    static std::string* robot_name;


    virtual ~Logging();
private:
    static bool removeRouteRequestReceiver(RouteResponse* res);

};
ros::NodeHandle* Logging::n = NULL;
std::list<std::string> Logging::entries_r_table;
std::list<std::string> Logging::entries_rreq_initi;
std::list<std::string> Logging::entries_rreq_interm;
std::list<std::string> Logging::entries_rreq_recv;
std::list<std::string> Logging::entries_mem_consumption;
std::list<std::string> Logging::entries_uc_frames;
std::list<std::string> Logging::entries_mc_frames;
std::list<std::string> Logging::entries_link_frames;
std::list<std::string> Logging::entries_service_calls;
std::list<std::string> Logging::entries_transport_frames;


std::map<std::string, uint32_t> Logging::uint32_properties;

std::string Logging::log_path = "";
std::string* Logging::robot_name = NULL;

std::string Logging::routing_table_file = "routing_table.csv";
std::string Logging::route_req_src_file = "rreq_initiater.csv";
std::string Logging::route_req_dst_file = "rreq_receiver.csv";
std::string Logging::route_req_iterm_file = "rreq_intermediate.csv";
std::string Logging::mem_consumption_file = "packet_memory_consumption.csv";
std::string Logging::uc_frames_file = "unicast_datalink_transmission_summary.csv";
std::string Logging::mc_frames_file = "multicast.csv";
std::string Logging::link_frames_file = "unicast_link_transmission.csv";
std::string Logging::sercice_calls_file = "services.csv";
std::string Logging::transport_frames_file = "transport_frames.csv";

/* 
uint32_t Logging::running_unicast_cr_threads = 0;
uint32_t Logging::running_multicast_threads = 0;
uint32_t Logging::running_unicast_transport_threads = 0;
uint32_t Logging::running_unicast_link_threads = 0;

unicast frames sent 
uint32_t Logging::num_unique_data_frames_sent = 0;
uint32_t Logging::num_unique_data_frames_forwarded = 0;
uint32_t Logging::num_data_frames_resent = 0;
uint32_t Logging::num_data_frames_reforwarded = 0;
uint32_t Logging::num_data_frames_relayed = 0;
uint32_t Logging::num_acknowledgments_sent = 0;
uint32_t Logging::num_acknowledgments_relayed = 0;
uint32_t Logging::num_rreq_sent = 0;
uint32_t Logging::num_rrep_sent = 0;
uint32_t Logging::num_beacons_sent = 0;
uint32_t Logging::num_relay_selection_sent = 0;
uint32_t Logging::num_relay_detection_sent = 0;
uint32_t Logging::num_total_frames_sent = 0;


/* unicast received sent 
uint32_t Logging::num_unique_data_frames_received_directly = 0;
uint32_t Logging::num_unique_data_frames_received_relay = 0;
uint32_t Logging::num_data_frames_received_directly = 0;
uint32_t Logging::num_data_frames_received_relay = 0;
uint32_t Logging::num_acknowledgments_received_directly = 0;
uint32_t Logging::num_acknowledgments_received_relay = 0;
uint32_t Logging::num_rreq_received = 0;
uint32_t Logging::num_rrep_received = 0;
uint32_t Logging::num_relay_selection_received = 0;
uint32_t Logging::num_relay_detection_received = 0;
uint32_t Logging::num_duplicate_frames_received = 0;
uint32_t Logging::num_beacons_received = 0;
uint32_t Logging::num_total_frames_received = 0;

uint32_t Logging::num_total_bytes_sent = 0;
uint32_t Logging::num_total_bytes_received = 0;

/* mc frames sent 
uint32_t Logging::num_mc_unique_data_frames_sent = 0;
uint32_t Logging::num_mc_unique_data_frames_forwarded = 0;
uint32_t Logging::num_mc_data_frames_resent = 0;
uint32_t Logging::num_mc_data_frames_reforwarded = 0;
uint32_t Logging::num_mc_data_frames_relayed = 0;
uint32_t Logging::num_mc_acknowledgments_sent = 0;
uint32_t Logging::num_mc_acknowledgments_relayed = 0;
uint32_t Logging::num_mc_rreq_sent = 0;
uint32_t Logging::num_mc_rrep_sent = 0;
uint32_t Logging::num_mc_ract_sent = 0;
uint32_t Logging::num_mc_relay_selection_sent = 0;
uint32_t Logging::num_mc_total_frames_sent = 0;


/* mc frames sent 
uint32_t Logging::num_mc_unique_data_frames_received_directly = 0;
uint32_t Logging::num_mc_unique_data_frames_received_relay = 0;
uint32_t Logging::num_mc_data_frames_received_directly = 0;
uint32_t Logging::num_mc_data_frames_received_relay = 0;
uint32_t Logging::num_mc_acknowledgments_received_directly = 0;
uint32_t Logging::num_mc_acknowledgments_received_relay = 0;
uint32_t Logging::num_mc_rreq_received = 0;
uint32_t Logging::num_mc_rrep_received = 0;
uint32_t Logging::num_mc_ract_received = 0;
uint32_t Logging::num_mc_relay_selection_received = 0;
uint32_t Logging::num_mc_duplicate_frames_received = 0;
uint32_t Logging::num_mc_total_frames_received = 0;
uint32_t Logging::num_mc_total_bytes_sent = 0;
uint32_t Logging::num_mc_total_bytes_received = 0;
 */
boost::mutex Logging::mtx_logging;



#endif	/* LOGGING_H */
