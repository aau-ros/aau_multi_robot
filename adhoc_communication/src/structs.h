/*
 * structs.h
 *
 *  Created on: 13.08.2013
 *      Author: Günther Cwioro
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

#include "functions.h"


bool compareMac(const unsigned char mac1[6], const unsigned char mac2[6]);
bool compareMac(char* mac1[6], char* mac2[6]);

struct routing_entry {
    std::string hostname_destination;
    std::string hostname_source;
    std::list<mac> mac_path_l;
    uint32_t id;
    unsigned long ts;
    unsigned char next_hop[6];
    unsigned char previous_hop[6]; // previous
    uint16_t hobs;
    uint16_t current_hop;
    uint8_t root_distance; //multicast
    bool cr_entry;

    routing_entry() {
        ts = getMillisecondsTime();
    }

    routing_entry(string hname_src, uint32_t id) {
        hostname_source = hname_src;
        this->id = id;
        ts = getMillisecondsTime();
    }

    bool samePath(routing_entry r) {
        if (mac_path_l.size() != r.mac_path_l.size())
            return false;

        std::list<mac>::iterator i_other = r.mac_path_l.begin();
        std::list<mac>::iterator i_this = mac_path_l.begin();
        while (i_this == mac_path_l.end() || i_other == r.mac_path_l.end()) {

            if (compareMac((*i_this).mac_adr, (*i_other).mac_adr) == false)
                return false;

            i_this++;
            i_other++;

        }

        return true;
    }

   bool operator ==(const routing_entry& st) {
        if (hostname_source.compare(st.hostname_source) == 0 && id == st.id) {
            ts = getMillisecondsTime();
            return true;
        }

        return false;
    }
};

struct route_request {
    std::string hostname_source;
    std::string hostname_destination;
    uint32_t id;
    uint8_t response_sent;
    uint8_t retransmitted;
    uint32_t hop_limit;
    uint16_t counter; // defines how much requests have been received with the same id
    unsigned char receiver_mac[6]; // only needed for mc
    uint8_t is_mc;
    //uint8_t is_join_req;

    long int ts;

    route_request() {
        counter = 0;
        ts = getMillisecondsTime();
   
        retransmitted = 0;
        response_sent = 0;
    }

    bool operator ==(const route_request& st) {
        if (hostname_source.compare(st.hostname_source) == 0 && id == st.id)// && compareMac((const char*)st.receiver_mac,(const char*) this->receiver_mac))
            return true;

        return false;
    }

};

struct stc_ack {
    std::string network_string;
    uint32_t frame_id;
    uint8_t retransmitted;
    unsigned char mac[6];

    //ecl::StopWatch timer;


    // Assignment operator.

    bool operator ==(const stc_ack& st) {

        return (frame_id == st.frame_id && compareMac(st.mac, mac));

    }

};

#include "RoutedFrame.cpp"
#include "RoutedFrame.h"
#include "defines.h"

struct stc_RoutedFrame {
    RoutedFrame frame;
    uint8_t retransmitted;
    bool frame_is_ack;
    string hostname_destination;
    unsigned char mac[6];

    unsigned long time_stamp;

    bool operator ==(const stc_RoutedFrame& st) {
        return (frame.header_.frame_id == st.frame.header_.frame_id && hostname_destination.compare(st.hostname_destination) == 0);
    }




};

struct hostname_mac {
    std::string hostname;
    unsigned char mac[6];
    //uint8_t no_hello_got;
    bool reachable;
    unsigned long ts;

    hostname_mac() {
        ts = getMillisecondsTime();
    }

    hostname_mac(unsigned char* m) {
        memcpy(mac, m, 6);
        ts = getMillisecondsTime();
    }

    void stamp() {
        ts = getMillisecondsTime();
    }

    bool operator ==(const hostname_mac& st) {
        for (int i = 0; i < 6; i++)
            if (st.mac[i] != mac[i])
                return false;

        return true;
    }
};

struct stc_packet {
    string src;
    string mc_group;
    uint32_t id;

    unsigned long ts;

    stc_packet(string source, string mc_g, uint32_t p_id) {
        src = source;
        id = p_id;
        //size = p_size;
        mc_group = mc_g;
        ts = getMillisecondsTime();
    }

    bool operator ==(const stc_packet& p) {
        return p.src.compare(src) == 0 && p.id == id && mc_group.compare(p.mc_group) == 0;


    }

};

struct ack_cr_info {
    unsigned char frame_src_mac[6];
    unsigned char frame_dst_mac[6];
    uint32_t id;
    uint32_t seq;
    unsigned long ts;
    std::string source_host;
    std::string mc_group;
    bool retransmitted;
    std::string network_string;
    uint8_t frame_type;

    ack_cr_info() {
        ts = getMillisecondsTime();
        seq = 0;
        mc_group = "";
        retransmitted = false;
    }

    bool operator ==(const ack_cr_info& p) {

        return compareMac(frame_src_mac, p.frame_src_mac) && compareMac(frame_dst_mac, p.frame_dst_mac) && id == p.id && seq == p.seq && mc_group.compare(p.mc_group) == 0 && source_host.compare(p.source_host) == 0 && frame_type == p.frame_type;
    }
};


/*
struct neighbor
{
        hostname_mac host_mac;
        nav_msgs::Odometry position;

        bool operator ==(const neighbor& st)
        {
                  return host_mac.operator ==(st.host_mac);
        }
};
 */




struct mc_tree {
    std::string group_name;
    routing_entry route_uplink;
    std::list<mac> mc_downlinks;
    bool member;
    bool active;
    bool connected;
    bool root; //true if current robot is root of the tree
    uint8_t root_distance;

    bool operator ==(const mc_tree& st) {
        if (group_name.compare(st.group_name) == 0)
            return true;

        return false;
    }

};

struct bcasts {
    uint32_t id;
    std::string src;

    bcasts(uint32_t id_, std::string src_) {
        id = id_;
        src = src_;
    }

    bool operator ==(const bcasts& st) {
        return (src.compare(st.src) == 0 && id == st.id);

    }


};


#endif /* STRUCTS_H_ */
