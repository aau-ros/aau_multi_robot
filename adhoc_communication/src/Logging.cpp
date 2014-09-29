/* 
 * File:   Logging.cpp
 * Author: cwioro
 * 
 * Created on August 8, 2014, 10:35 AM
 */

#include "Logging.h"
#include "RouteRequest.h"
#include "RouteResponse.h"
#include "McTree.h"
#include "EthernetFrame.h"
#include "Packet.h"
#include "defines.h"
#include <boost/algorithm/string.hpp>

Logging::Logging()
{

}

Logging::Logging(const Logging& orig)
{

}

Logging::~Logging()
{

}

void Logging::periodicLog(unsigned long interval_ms)
{
    while (ros::ok())
    {
        sleepMS(interval_ms);
        log();
    }
}

void Logging::init(ros::NodeHandle* n, std::string* robot_n)
{
#ifndef PERFORMANCE_LOGGING
    return;
#endif
    Logging::n = n;
    Logging::robot_name = robot_n;
    entries_r_table.push_front("#time,num_cr_entries,num_unicast_entries,num_multicast_entries,size");
    entries_rreq_initi.push_front("#time,rreq_id,dst,start_time,end_time,num_rrep_rcvd,num_rreq_sent,route_length,route,multicast");
    entries_rreq_recv.push_front("#time,rreq_id,src,counter,route_length,selected,route");
    entries_rreq_interm.push_front("#time,rreq_id,src,rcvd_from");
    entries_mem_consumption.push_front("#time,size_multicast_packets_complete,size_multicast_packets_incomplete,size_unicast_packets_incomplete,size_unack_relay_frames,running_unicast_link_threads,running_multicast_threads,running_unicast_cr_threads,running_unicast_transport_threads");
    entries_link_frames.push_front("#time,src,frame_id,mac_src, mac_dst,time_sent,time_ack,retransmissions");

    /* UNCIAST FRAMES HEADER */
    string uc_header = "#time,num_unique_data_frames_sent,num_unique_data_frames_forwarded,num_data_frames_resent,num_data_frames_reforwarded,num_data_frames_relayed,"
            "num_acknowledgments_sent,num_acknowledgments_relayed,num_rreq_sent,num_rrep_sent,num_beacons_sent,num_relay_selection_sent,num_relay_detecion_sent,num_total_frames_sent";
    uc_header += ",num_unique_data_frames_received_directly,num_unique_data_frames_received_relay,num_data_frames_received_directly,num_data_frames_received_relay,"
            "num_acknowledgments_received_directly,num_acknowledgments_received_relay,num_rreq_received,num_rrep_received,num_relay_selection_received,num_relay_detecion_received,num_duplicate_frames_received,num_beacons_received,num_total_frames_received,num_total_bytes_sent,num_total_bytes_received";

    entries_uc_frames.push_front(uc_header);

    /* MULTICAST FRAMES HEADER */
    string mc_header = "#time,num_mc_unique_data_frames_sent,num_mc_unique_data_frames_forwarded,num_mc_data_frames_resent,num_mc_data_frames_reforwarded";
    mc_header += ",num_mc_data_frames_relayed,num_mc_acknowledgments_sent,num_mc_acknowledgments_relayed,num_mc_rreq_sent";
    mc_header += ",num_mc_rrep_sent,num_mc_ract_sent,num_mc_relay_selection_sent,num_mc_total_frames_sent";

    mc_header += ",num_mc_unique_data_frames_received_directly,num_mc_unique_data_frames_received_relay,num_mc_data_frames_received_directly";
    mc_header += ",num_mc_data_frames_received_relay,num_mc_acknowledgments_received_directly,num_mc_acknowledgments_received_relay,num_mc_rreq_received";
    mc_header += ",num_mc_rrep_received,num_mc_ract_received,num_mc_relay_selection_received,num_mc_duplicate_frames_received,num_mc_total_frames_received";
    mc_header += ",num_mc_total_bytes_sent,num_mc_total_bytes_received";

    entries_mc_frames.push_front(mc_header);
    entries_service_calls.push_back("#time,service,time_call,time_res,data_size,return");
    entries_transport_frames.push_back("#time,id,dst,retransmissions,start_time,end_time,success,route_length,route,multicast");
}

void Logging::logUcPacketsSummary(unsigned long interval_ms)
{
#ifndef PERFORMANCE_LOGGING
    return;
#endif
#ifndef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
    return;
#endif

    while (ros::ok())
    {
        sleepMS(interval_ms);



        string entry = getIntAsString(getMillisecondsTime()) + ",";
        {
            boost::unique_lock<boost::mutex> lock(Logging::mtx_logging);
            uint32_properties["num_total_frames_sent"] = 0;
            uint32_properties["num_total_frames_received"] = 0;
        }

        /* num_unique_data_frames_sent */
        entry += getIntAsString(getProperty("num_unique_data_frames_sent")) + ",";
        increaseProperty("num_total_frames_sent", getProperty("num_unique_data_frames_sent"));

        /* getProperty("num_unique_data_frames_forwarded*/
        entry += getIntAsString(getProperty("num_unique_data_frames_forwarded")) + ",";
        increaseProperty("num_total_frames_sent", getProperty("num_unique_data_frames_forwarded"));

        /* num_data_frames_resent*/
        entry += getIntAsString(getProperty("num_data_frames_resent")) + ",";
        increaseProperty("num_total_frames_sent", getProperty("num_data_frames_resent"));


        /* num_data_frames_reforwarded*/
        entry += getIntAsString(getProperty("num_data_frames_reforwarded")) + ",";
        increaseProperty("num_total_frames_sent", getProperty("num_data_frames_reforwarded"));

        /* num_data_frames_relayed*/
        entry += getIntAsString(getProperty("num_data_frames_relayed")) + ",";
        increaseProperty("num_total_frames_sent", getProperty("num_data_frames_relayed"));

        /*num_acknowledgments_sent */
        entry += getIntAsString(getProperty("num_acknowledgments_sent")) + ",";
        increaseProperty("num_total_frames_sent", getProperty("num_acknowledgments_sent"));

        /* getProperty("num_acknowledgments_relayed*/
        entry += getIntAsString(getProperty("num_acknowledgments_relayed")) + ",";
        increaseProperty("num_total_frames_sent", getProperty("num_acknowledgments_relayed"));

        /* num_rreq_sent*/
        entry += getIntAsString(getProperty("num_rreq_sent")) + ",";
        increaseProperty("num_total_frames_sent", getProperty("num_rreq_sent"));

        /* num_rrep_sent*/
        entry += getIntAsString(getProperty("num_rrep_sent")) + ",";
        increaseProperty("num_total_frames_sent", getProperty("num_rrep_sent"));

        /*num_beacons_sent */
        entry += getIntAsString(getProperty("num_beacons_sent")) + ",";
        increaseProperty("num_total_frames_sent", getProperty("num_beacons_sent"));

        /*num_relay_selection_sent */
        entry += getIntAsString(getProperty("num_relay_selection_sent")) + ",";
        increaseProperty("num_total_frames_sent", getProperty("num_relay_selection_sent"));

        /*num_relay_detection_sent */
        entry += getIntAsString(getProperty("num_relay_detection_sent")) + ",";
        increaseProperty("num_total_frames_sent", getProperty("num_relay_detection_sent"));

        /*num_total_frames_sent */
        entry += getIntAsString(getProperty("num_total_frames_sent")) + ",";


        //      num_total_frames_received = 0;


        /*num_unique_data_frames_received_directly */
        entry += getIntAsString(getProperty("num_unique_data_frames_received_directly")) + ",";

        /*num_unique_data_frames_received_relay */
        increaseProperty("num_total_frames_received", getProperty("num_unique_data_frames_received_relay"));
        entry += getIntAsString(getProperty("num_unique_data_frames_received_relay")) + ",";

        /*num_data_frames_received_directly */
        increaseProperty("num_total_frames_received", getProperty("num_data_frames_received_directly"));
        entry += getIntAsString(getProperty("num_data_frames_received_directly")) + ",";

        /*num_data_frames_received_relay */
        increaseProperty("num_total_frames_received", getProperty("num_data_frames_received_relay"));
        entry += getIntAsString(getProperty("num_data_frames_received_relay")) + ",";

        /*num_acknowledgments_received_directly */
        increaseProperty("num_total_frames_received", getProperty("num_acknowledgments_received_directly"));
        entry += getIntAsString(getProperty("num_acknowledgments_received_directly")) + ",";

        /*num_acknowledgments_received_relay */
        increaseProperty("num_total_frames_received", getProperty("num_acknowledgments_received_relay"));
        entry += getIntAsString(getProperty("num_acknowledgments_received_relay")) + ",";

        /*num_rreq_received */
        increaseProperty("num_total_frames_received", getProperty("num_rreq_received"));
        entry += getIntAsString(getProperty("num_rreq_received")) + ",";

        /*num_rrep_received */
        increaseProperty("num_total_frames_received", getProperty("num_rrep_received"));
        entry += getIntAsString(getProperty("num_rrep_received")) + ",";

        /*num_relay_selection_received */
        increaseProperty("num_total_frames_received", getProperty("num_relay_selection_received"));
        entry += getIntAsString(getProperty("num_relay_selection_received")) + ",";

        /*num_relay_detection_received */
        increaseProperty("num_total_frames_received", getProperty("num_relay_detection_received"));
        entry += getIntAsString(getProperty("num_relay_detection_received")) + ",";

        /*num_duplicate_frames_received */
        increaseProperty("num_total_frames_received", getProperty("num_duplicate_frames_received"));
        entry += getIntAsString(getProperty("num_duplicate_frames_received")) + ",";

        /*num_beacons_received */
        increaseProperty("num_total_frames_received", getProperty("num_beacons_received"));
        entry += getIntAsString(getProperty("num_beacons_received")) + ",";

        /*um_total_frames_received */
        entry += getIntAsString(getProperty("num_total_frames_received")) + ",";

        /*num_total_bytes_sent */
        entry += getIntAsString(getProperty("num_total_bytes_sent")) + ",";

        /*num_total_bytes_received */
        entry += getIntAsString(getProperty("num_total_bytes_received")) + ",";


        entries_uc_frames.push_back(entry);

    }
}

void Logging::logMcPacketsSummary(unsigned long interval_ms)
{
#ifndef PERFORMANCE_LOGGING
    return;
#endif

#ifndef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
    return;
#endif


    while (ros::ok())
    {
        sleepMS(interval_ms);



        string entry = getIntAsString(getMillisecondsTime()) + ",";


        /*num_mc_unique_data_frames_sent */
        entry += getIntAsString(getProperty("num_mc_unique_data_frames_sent")) + ",";
        increaseProperty("num_mc_total_frames_sent", getProperty("num_mc_unique_data_frames_sent"));

        /* num_mc_unique_data_frames_forwarded*/
        entry += getIntAsString(getProperty("num_mc_unique_data_frames_forwarded")) + ",";
        increaseProperty("num_mc_total_frames_sent", getProperty("num_mc_unique_data_frames_forwarded"));

        /* num_mc_data_frames_resent*/
        entry += getIntAsString(getProperty("num_mc_data_frames_resent")) + ",";
        increaseProperty("num_mc_total_frames_sent", getProperty("num_mc_data_frames_resent"));

        /* num_mc_data_frames_reforwarded*/
        entry += getIntAsString(getProperty("num_mc_data_frames_reforwarded")) + ",";
        increaseProperty("num_mc_total_frames_sent", getProperty("num_mc_data_frames_reforwarded"));

        /* num_mc_data_frames_relayed*/
        entry += getIntAsString(getProperty("num_mc_data_frames_relayed")) + ",";
        increaseProperty("num_mc_total_frames_sent", getProperty("num_mc_data_frames_relayed"));

        /*num_mc_acknowledgments_sent */
        entry += getIntAsString(getProperty("num_mc_acknowledgments_sent")) + ",";
        increaseProperty("num_mc_total_frames_sent", getProperty("num_mc_acknowledgments_sent"));

        /* num_mc_acknowledgments_relayed*/
        entry += getIntAsString(getProperty("num_mc_acknowledgments_relayed")) + ",";
        increaseProperty("num_mc_total_frames_sent", getProperty("num_mc_acknowledgments_relayed"));

        /* num_mc_rreq_sent*/
        entry += getIntAsString(getProperty("num_mc_rreq_sent")) + ",";
        increaseProperty("num_mc_total_frames_sent", getProperty("num_mc_rreq_sent"));

        /* num_mc_rrep_sent*/
        entry += getIntAsString(getProperty("num_mc_rrep_sent")) + ",";
        increaseProperty("num_mc_total_frames_sent", getProperty("num_mc_rrep_sent"));

        /*num_mc_ract_sent */
        entry += getIntAsString(getProperty("num_mc_ract_sent")) + ",";
        increaseProperty("num_mc_total_frames_sent", getProperty("num_mc_ract_sent"));

        /*num_mc_relay_selection_sent */
        entry += getIntAsString(getProperty("num_mc_relay_selection_sent")) + ",";
        increaseProperty("num_mc_total_frames_sent", getProperty("num_mc_relay_selection_sent"));

        /*num_mc_total_frames_sent */
        entry += getIntAsString(getProperty("num_mc_total_frames_sent")) + ",";





        /*num_mc_unique_data_frames_received_directly */
        increaseProperty("num_mc_total_frames_received", getProperty("num_mc_unique_data_frames_received_directly"));
        entry += getIntAsString(getProperty("num_mc_unique_data_frames_received_directly")) + ",";

        /*num_mc_unique_data_frames_received_relay */
        increaseProperty("num_mc_total_frames_received", getProperty("num_mc_unique_data_frames_received_relay"));
        entry += getIntAsString(getProperty("num_mc_unique_data_frames_received_relay")) + ",";

        /*num_mc_data_frames_received_directly */
        increaseProperty("num_mc_total_frames_received", getProperty("num_mc_data_frames_received_directly"));
        entry += getIntAsString(getProperty("num_mc_data_frames_received_directly")) + ",";

        /*num_mc_data_frames_received_relay */
        increaseProperty("num_mc_total_frames_received", getProperty("num_mc_data_frames_received_relay"));
        entry += getIntAsString(getProperty("num_mc_data_frames_received_relay")) + ",";

        /*num_mc_acknowledgments_received_directly */
        increaseProperty("num_mc_total_frames_received", getProperty("num_mc_acknowledgments_received_directly"));
        entry += getIntAsString(getProperty("num_mc_acknowledgments_received_directly")) + ",";

        /*num_mc_acknowledgments_received_relay */
        increaseProperty("num_mc_total_frames_received", getProperty("num_mc_acknowledgments_received_relay"));
        entry += getIntAsString(getProperty("num_mc_acknowledgments_received_relay")) + ",";

        /*num_mc_rreq_received */
        increaseProperty("num_mc_total_frames_received", getProperty("num_mc_rreq_received"));
        entry += getIntAsString(getProperty("num_mc_rreq_received")) + ",";

        /*num_mc_rrep_received */
        increaseProperty("num_mc_total_frames_received", getProperty("num_mc_rrep_received"));
        entry += getIntAsString(getProperty("num_mc_rrep_received")) + ",";

        /*num_mc_ract_received */
        increaseProperty("num_mc_total_frames_received", getProperty("num_mc_ract_received"));
        entry += getIntAsString(getProperty("num_mc_ract_received")) + ",";

        /*num_mc_relay_selection_received */
        increaseProperty("num_mc_total_frames_received", getProperty("num_mc_relay_selection_received"));
        entry += getIntAsString(getProperty("num_mc_relay_selection_received")) + ",";

        /*num_mc_duplicate_frames_received */
        increaseProperty("num_mc_total_frames_received", getProperty("num_mc_duplicate_frames_received"));
        entry += getIntAsString(getProperty("num_mc_duplicate_frames_received")) + ",";

        /*num_mc_total_frames_received */
        entry += getIntAsString(getProperty("num_mc_total_frames_received")) + ",";

        /*num_mc_total_frames_received */
        entry += getIntAsString(getProperty("num_mc_total_bytes_sent")) + ",";

        /*num_mc_total_frames_received */
        entry += getIntAsString(getProperty("num_mc_total_bytes_received")) + ",";


        entries_mc_frames.push_back(entry);

    }
}

void Logging::log()
{
#ifndef PERFORMANCE_LOGGING

    return;
#endif
    Logging::createLogPath();

    Logging::createLogFile(&Logging::entries_rreq_initi, &Logging::route_req_src_file);
    Logging::createLogFile(&Logging::entries_rreq_recv, &Logging::route_req_dst_file);
    Logging::createLogFile(&Logging::entries_rreq_interm, &Logging::route_req_iterm_file);
    Logging::createLogFile(&Logging::entries_r_table, &Logging::routing_table_file);
    Logging::createLogFile(&Logging::entries_mem_consumption, &Logging::mem_consumption_file);
    Logging::createLogFile(&Logging::entries_transport_frames, &Logging::transport_frames_file);

#ifdef PERFORMANCE_LOGGING_UC_LINK_FRAMES
    Logging::createLogFile(&Logging::entries_link_frames, &Logging::link_frames_file);
#endif
#ifdef PERFORMANCE_LOGGING_UC_LINK_SUMMARY
    Logging::createLogFile(&Logging::entries_uc_frames, &Logging::uc_frames_file);
#endif

#ifdef PERFORMANCE_LOGGING_MC_LINK_SUMMARY
    Logging::createLogFile(&Logging::entries_mc_frames, &Logging::mc_frames_file);
#endif


    Logging::createLogFile(&Logging::entries_service_calls, &Logging::sercice_calls_file);


}

void Logging::createLogFile(std::list<std::string>* table, std::string* filename)
{
#ifndef PERFORMANCE_LOGGING
    return;
#endif
    std::ofstream myfile;
    std::string file_name = Logging::log_path + *filename;
    myfile.open(file_name.c_str());

    //ROS_ERROR("output file %s", file_name.c_str());
    for (std::list<std::string>::iterator i = table->begin(); i != table->end(); i++)
    {

        std::string line = *i;
        myfile << line << "\n";
    }
    myfile.close();
}

bool Logging::createLogPath()
{
#ifndef PERFORMANCE_LOGGING
    return false;
#endif
    Logging::n->param("log_path", Logging::log_path, std::string(""));
    Logging::log_path += "/adhoc_communication/" + *robot_name + "/";

    ROS_INFO("Creating log path \"%s\".", Logging::log_path.c_str());
    boost::filesystem::path boost_log_path(log_path.c_str());
    if (!boost::filesystem::exists(boost_log_path))
        try
        {
            if (!boost::filesystem::create_directories(boost_log_path))
                ROS_ERROR("Cannot create directory \"%s\".", Logging::log_path.c_str());
        } catch (const boost::filesystem::filesystem_error& e)
        {

            ROS_ERROR("Cannot create path \"%s\".", Logging::log_path.c_str());
            return false;
        }
    return true;
}
//time num_cr_entries, num_unicast_entries, num_multicast_entries, size

void Logging::logRoutingTable(std::list<routing_entry>* u_entries, std::list<McTree*>* m_entries)
{
#ifndef PERFORMANCE_LOGGING
    return;
#endif
    uint32_t num_cr_entries = 0, num_unicast_entries = 0, num_multicast_entries = m_entries->size();
    std::size_t size = 0;

    for (std::list<routing_entry>::iterator i = u_entries->begin(); i != u_entries->end(); i++)
    {

        routing_entry r = *i;
        size += sizeof (r);

        if (r.cr_entry)
            num_cr_entries++;
        else
        {

            num_unicast_entries++;

        }

    }


    for (std::list<McTree*>::iterator i = m_entries->begin(); i != m_entries->end(); i++)
    {

        McTree* m = *i;

        size += sizeof (*m);

    }

    entries_r_table.push_back(getIntAsString(getMillisecondsTime()) + "," + getIntAsString(num_cr_entries) + "," + getIntAsString(num_unicast_entries) + "," + getIntAsString(num_multicast_entries) + "," + getIntAsString(size));
}

void Logging::logServiceCalls(string service_name, unsigned long time_call, unsigned long time_res, unsigned long size, bool ret_val)
{
    entries_service_calls.push_back(getIntAsString(getMillisecondsTime()) + "," + service_name.c_str() + "," + getIntAsString(time_call) + "," + getIntAsString(time_res) + "," + getIntAsString(size) + "," + getIntAsString(ret_val));
}

void Logging::logRRequestInitiater(route_request* req, routing_entry* route)
{
#ifndef PERFORMANCE_LOGGING
    return;
#endif
    /*rreq_initiater.csv (log upon every initiated route request):
    #time,rreq_id,dst,start_time,end_time,num_rrep_rcvd,num_rreq_sent,route_length,route, multicast
    time                                    : timestamp (ms)
    rreq_id                                 : The unique ID of the route request
    dst                                     : destination of the route request
    start_time                              : Time the rreq was sent (ms)
    end_time                                : Time the rrep was received (if any, otherwise empty/0)
    num_rrep_rcvd				: Number of received routes
    num_rreq_sent                           : Number of route request sent, if multiple where unsuccessful
    route_length                            : If route received, length of the route
    route                                   : The route selected in the form of the mac path 
    multicast				: true if multicast, unicast false*/

    std::string entry = "";

    /* time stamp */
    entry += getIntAsString(getMillisecondsTime()) + ",";

    /* rreq_id */
    entry += getIntAsString(req->id) + ",";

    /* dst */
    entry += req->hostname_destination + ",";

    /* time sent */
    entry += getIntAsString(req->ts) + ",";

    /* time recv */
    if (route != NULL)
        entry += getIntAsString(route->ts) + ",";
    else
        entry += ",";

    /* num_rrep_rcvd */
    entry += "1,";

    /* num_rreq_sent */
    entry += getIntAsString(req->retransmitted) + ",";

    /* route len */
    if (route != NULL)
        entry += getIntAsString(route->hobs) + ",";
    else
        entry += ",";

    /* path list */
    if (route != NULL)
    {

        entry += getPathAsStr(route->mac_path_l);
    }



    /* multicast*/
    entry += "," + getIntAsString(req->is_mc);

    entries_rreq_initi.push_back(entry);
}

void Logging::logRRequestIntermediate(RouteRequest* r)
{
#ifndef PERFORMANCE_LOGGING

    return;
#endif
    /*rreq_intermediate.csv (log upon every received route request):
    #time,rreq_id,src,rcvd_from
    time                                    : timestamp (ms)
    rreq_id                                 : ID of the route request
    src                                     : source of the route request
    rcvd_from                               : robot the route request was received from*/

    std::string entry = "";

    /* time stamp */
    entry += getIntAsString(getMillisecondsTime()) + ",";

    /* rreq_id */
    entry += getIntAsString(r->header_.id) + ",";

    /* src host */
    entry += r->hostname_source_ + ",";

    /* rcvd_from */
    entry += getMacAsStr(r->eh_h_.eh_source).c_str();



    entries_rreq_interm.push_back(entry);
}

void Logging::logRRequestReceiver(std::string h_src, uint32_t id, uint16_t counter, uint16_t hobs, bool selected, std::list<mac> path_l)
{
#ifndef PERFORMANCE_LOGGING
    return;
#endif
    /*rreq_receiver.csv (log upon every received route request):
    #time,rreq_id,counter,route_length,selected,route
    time                                    : timestamp (ms)
    rreq_id                                 : The unique ID of the route request
     * src
    counter                                 : the 'counter' received route request
    route_length                            : If route received, length of the route
    selected                                : If this route is selected and returned to the source
    route                                   : The route received in the form 'src|robot_A|robot_B|dst'
     */

    std::string entry = "";

    /* time stamp */
    entry += getIntAsString(getMillisecondsTime()) + ",";

    /* rreq_id */
    entry += getIntAsString(id) + ",";

    /* src host */
    entry += h_src + ",";

    /* counter */
    entry += getIntAsString(counter) + ",";

    /* route_length */
    entry += getIntAsString(hobs) + ",";

    /* selected */
    if (selected)
        entry += "1,";

    else
        entry += "0,";

    /* path list */
    entry += getPathAsStr(path_l);

    entries_rreq_recv.push_back(entry);
}

bool Logging::removeRouteRequestReceiver(RouteResponse* res)
{
#ifndef PERFORMANCE_LOGGING
    return false;
#endif
    for (std::list<std::string>::iterator i = Logging::entries_rreq_recv.begin(); i != Logging::entries_rreq_recv.end();)
    {
        if (Logging::entries_rreq_recv.size() > 1 && i == Logging::entries_rreq_recv.begin())
            i++; // skip header
        else
            break;

        std::vector<std::string> csv_fields;
        boost::split(csv_fields, *i, boost::is_any_of(","));

        std::vector<std::string> hostname_id;
        boost::split(hostname_id, csv_fields[1], boost::is_any_of(" "));

        std::string hostname_src = hostname_id[0];

        int req_id = 0;
        try
        {
            req_id = boost::lexical_cast<uint32_t>(std::string(hostname_id[1]));

            if (hostname_src.compare(res->hostname_source_) == 0 && res->request_id_ == req_id)
            {
                Logging::entries_rreq_recv.erase(i);
                return true;
            }
        } catch (boost::bad_lexical_cast const&)
        {
            //ROS_FATAL("%s %s %s", csv_fields[1].c_str(), hostname_id[0].c_str(), hostname_id[1].c_str());

            ROS_FATAL("In Logging.cpp: Cannot convert [%s] to integer!", hostname_id[1].c_str());
        }


    }
    return false;
}

void Logging::logMemoryConsumptionPackets(unsigned long interval_ms, boost::mutex* mtx_pack, list<Packet>* packets, boost::mutex* mtx_cached_mc, list<Packet>* cached_mc, boost::mutex* mtx_unack_link_frames, list<stc_frame>* unack_link_frames, boost::mutex* mtx_unack_cr_frames, list<ack_cr_info>* unack_cr_frames)
{
#ifndef PERFORMANCE_LOGGING
    return;
#endif


    while (ros::ok())
    {
        sleepMS(interval_ms);
        string entry = getIntAsString(getMillisecondsTime()) + ",";
        std::size_t size_uc_pack = 0, size_mc_pack = 0, size_cached_mc_packets = 0, size_unack_l_f = 0, size_unack_cr_f = 0;


        { /*  size_cached_multicast_packets complete */

            boost::unique_lock<boost::mutex> lock_g(*mtx_cached_mc);

            for (list<Packet>::iterator i = cached_mc->begin(); i != cached_mc->end(); i++)
            {

                size_cached_mc_packets += (*i).getSize();
            }

            entry += getIntAsString(size_cached_mc_packets) + ",";
        }

        { /*  size_multicast_packets_incomplete  +  size_unicast_packets_incomplete */

            boost::unique_lock<boost::mutex> lock_g(*mtx_pack);
            for (list<Packet>::iterator i = packets->begin(); i != packets->end(); i++)
            {
                if ((*i).isMcFrame())
                    size_mc_pack += (*i).getSize();
                else
                    size_uc_pack += (*i).getSize();

            }

            entry += getIntAsString(size_mc_pack) + ",";
            entry += getIntAsString(size_uc_pack) + ",";
        }


        { /*  size unack link frames */

            boost::unique_lock<boost::mutex> lock_g(*mtx_unack_link_frames);
            for (list<stc_frame>::iterator i = unack_link_frames->begin(); i != unack_link_frames->end(); i++)
                size_unack_l_f += sizeof (*i);




            entry += getIntAsString(size_unack_l_f) + ",";

        }

        { /*  size unack cr frames */

            boost::unique_lock<boost::mutex> lock_g(*mtx_unack_cr_frames);
            for (list<ack_cr_info>::iterator i = unack_cr_frames->begin(); i != unack_cr_frames->end(); i++)
                size_unack_cr_f += sizeof (*i);




            entry += getIntAsString(size_unack_cr_f) + ",";

        }


        entry += getIntAsString(getProperty("running_unicast_link_threads")) + ",";
        entry += getIntAsString(getProperty("running_multicast_threads")) + ",";
        entry += getIntAsString(getProperty("running_unicast_cr_threads")) + ",";
        entry += getIntAsString(getProperty("running_unicast_transport_threads")) + ",";



        entries_mem_consumption.push_back(entry);


    }


}

void Logging::logTransportFrame(stc_RoutedFrame* f, routing_entry* route, unsigned long start_time, unsigned long end_time, bool success)
{
   
    //  #time,id,dst,retransmissions,start_time,end_time,success,route_length,route,multicast ;
    string entry = getIntAsString(getMillisecondsTime()) + ",";

    /* id */
    entry += getIntAsString(f->frame.header_.frame_id) + ",";

    /* dst */
    entry += route->hostname_destination + ",";

    /* retransmissions */
    entry += getIntAsString(f->retransmitted)+",";

    /* start_time */
    entry += getIntAsString(start_time) + ",";

    /* end_time */
    entry += getIntAsString(end_time) + ",";

    /* success */
    entry += getBoolAsString(success) + ",";

    /*route_length*/
    entry += getIntAsString(route->hobs) + ",";

    /* route*/
    entry += getPathAsStr(route->mac_path_l) + ",";

    /* multicast */
    entry += getIntAsString(f->frame.mc_flag) + ",";


    entries_transport_frames.push_back(entry);
}

void Logging::logUcLinkTransmission(stc_frame f)
{
    //#time,src,frame_id,mac_src, mac_dst,time_sent,time_ack,retransmissions
#ifndef PERFORMANCE_LOGGING_UC_LINK_FRAMES
    return;
#endif
    string entry = getIntAsString(getMillisecondsTime()) + ",";

    /* src hostname */
    entry += f.hostname_source + ",";

    /* frame id */
    entry += getIntAsString(f.frame_id) + ",";

    /* mac src */
    entry += ",";

    /* mac dst */
    entry += getMacAsStr(f.mac) + ",";

    /* time_sent*/
    entry += getIntAsString(f.ts) + ",";

    /* time_ack*/
    entry += getIntAsString(getMillisecondsTime()) + ",";

    /*retransmissions*/
    entry += getIntAsString(f.retransmitted) + ",";


    entries_link_frames.push_back(entry);
}

void Logging::increaseProperty(std::string prop_name)
{
    increaseProperty(prop_name, 1);
}

void Logging::increaseProperty(std::string prop_name, uint32_t val)
{
    boost::unique_lock<boost::mutex> lock(Logging::mtx_logging);
    if (uint32_properties.find(prop_name) == uint32_properties.end())
        uint32_properties[prop_name] = 0;
    uint32_properties[prop_name] += val;
}

void Logging::decreaseProperty(std::string prop_name)
{
    decreaseProperty(prop_name, 1);
}

void Logging::decreaseProperty(std::string prop_name, uint32_t val)
{
    boost::unique_lock<boost::mutex> lock(Logging::mtx_logging);
    if (uint32_properties.find(prop_name) == uint32_properties.end())
        uint32_properties[prop_name] = 0;
    uint32_properties[prop_name] -= val;

}

uint32_t Logging::getProperty(std::string prop_name)
{

    boost::unique_lock<boost::mutex> lock(Logging::mtx_logging);
    if (uint32_properties.find(prop_name) == uint32_properties.end())
        uint32_properties[prop_name] = 0;

    return uint32_properties[prop_name];
}
