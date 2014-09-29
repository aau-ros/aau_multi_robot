/*
 * RouteRequest.cpp
 *
 *  Created on: 29.07.2013
 *      Author: Günther Cwioro
 *
 * Structure of a route request:
 *
 * BROADCAST MAC 	// -> ff:ff:ff:ff:ff:ff  											6Byte
 * SOURCE MAC		// MAC of the using interface										6Byte
 * ETH_TYPE_FIELD	// 0x4148 (AH as string)											2Byte
 * FRAME_TYPE		// 0x52 (R as string)												1Byte
 * REQUEST ID.		// Every node has a consecutive number for his requests				4Byte
 * MC FLAG																			1Byte
 * DEST HOST LENGTH 																	4Byte
 * DEST HOST																			VAR
 * SOURCE HOST LENGTH																	4Byte
 * SOURCE HOST																			VAR
 * MAX HOP COUNT  																		4Byte
 * HOB COUNT		// is the amount of node which are involved to process the packet	4Byte
 * PATH				//the list of macs from the hostname_source_ to destination			VAR (HOB COUNT * 6Byte)
 * CRC 																					4Byte
 *
 * Every node only accept one request with the same primary key. This is so to prevent network flooding.
 * The primary key of the request is REQUEST_ID + SOURCE_HOST
 *
 *
 * How a route request works:
 *
 * 		(Node1)		(Node3)
 * (S) 						 (D)
 * 		(Node2)		(Node4)
 * Request:
 * 1) S wants the path to D
 * 2) S sends request as broadcast
 * 3) N1 and N2 receive the request and insert their mac in the path list then they broadcast the request forward
 * 4) N3 and N3 receive the request from N1 or N2 (depends on which node is faster). They do the same like in step 2.
 * 5) The D gets the request either from N3 or from N4 and send an route response
 *
 */
#include <list>
#include <string>

#include "RouteRequest.h"
#include "defines.h"
#include "EthernetFrame.h"


RouteRequest::RouteRequest(unsigned char* buffer) {

    unsigned char* buffer_start = buffer;

    eh_header* eh = (struct eh_header *) buffer;
    buffer += sizeof (eh_header);
    rreq_header* rfh = (struct rreq_header *) buffer;
    buffer += sizeof (rreq_header);
    


    /*Source Host*/
    hostname_source_ = "";
    hostname_source_.append((const char*) buffer, rfh->hostname_source_len);
    buffer += rfh->hostname_source_len;

    /*Dest Host*/
    hostname_destination_ = "";
    hostname_destination_.append((const char*) buffer, rfh->hostname_destination_len);
    buffer += rfh->hostname_destination_len;


    /*PATH*/
    for (uint32_t i = 0; i < rfh->hop_count; i++) {
        mac current_mac;
        memcpy((unsigned char*) current_mac.mac_adr, (unsigned char*) buffer, 6);
        path_l_.push_back(current_mac);

        buffer += 6;
    }
    
        /*INIT FLAGS*/
	mc_flag_ = (rfh->flag_field / 128) % 2 == 1;
        

	/*CRC */
	uint32_t crc = 0;
	memcpy((unsigned char*) &crc, (unsigned char*) buffer, 4); // get CRC
        uint16_t dynamic_field_len = + rfh->hostname_source_len + rfh->hostname_destination_len+ path_l_.size() * 6;
	std::string crc_data_string = "";
	crc_data_string.append((const char*) buffer_start,
			this->HEADER_FIXED_LEN + dynamic_field_len) ;
	correct_crc_ = (crc == (uint32_t) GetCrc32(crc_data_string));


        buffer_str_len_ = this->HEADER_FIXED_LEN +dynamic_field_len;
                
	/* COPY HEADER FIELDS */
	memcpy(&this->eh_h_, &(*eh), sizeof(eh_header));
	memcpy(&this->header_, &(*rfh), sizeof(rreq_header));
}

/*
RouteRequest::RouteRequest(std::string sourHost,std::string destHost,uint32_t maximalHops) {

        hostname_destination_ = destHost;
        hostname_source_ = sourHost;
        this->hop_count_ = 0;
        this->hop_limit_ = maximalHops;
        id_ = this->req_count_stat++;
}
 */
RouteRequest::RouteRequest(std::string my_hostname, std::string destination, uint16_t max_hops, bool is_multicast) {
    hostname_destination_ = destination;
    hostname_source_ = my_hostname;
    header_.hop_count = 0;
    header_.hop_limit = max_hops;
    header_.id = req_count_stat++;
    header_.frame_type = FRAME_TYPE_REQUEST;

    mc_flag_ = is_multicast;
}

RouteRequest::RouteRequest(route_request req) {
    hostname_destination_ = req.hostname_destination;
    hostname_source_ = req.hostname_source;
    header_.hop_limit = req.hop_limit;
    mc_flag_ = req.is_mc;
    header_.hop_count = 0;
    header_.id = req.id;
    header_.frame_type = FRAME_TYPE_REQUEST;
}

std::string RouteRequest::getRequestAsNetworkString(unsigned char source_mac[6]) {


    /*Add the current hop mac to the routing path_l_*/
    mac currentHopMac;
    memcpy(currentHopMac.mac_adr,  source_mac, 6);
    memcpy( eh_h_.eh_source,  source_mac, 6);
    memcpy(eh_h_.eh_dest,  bcast_mac, 6);
    path_l_.push_back(currentHopMac);
    header_.hop_count++;
    
    this->header_.hop_count = this->path_l_.size();


    /*FLAG FIELD*/
    this->header_.flag_field = 0;
    if (mc_flag_)
        this->header_.flag_field += 128;


    /*LEN FIELDS */
    this->header_.hostname_source_len = hostname_source_.length();
    this->header_.hostname_destination_len = hostname_destination_.length();

    unsigned char a_buffer[ETHER_MAX_LEN];
    unsigned char* buffer_start = a_buffer;
    unsigned char* buffer = a_buffer;




    /*ETHERNET FIELDS*/
    //eh_header* eh = (struct eh_header *) buffer;
    memcpy(buffer, &this->eh_h_, sizeof (eh_header));
    buffer += sizeof (eh_header);

    /*FIXED RF HEADER FIELDS*/
    memcpy(buffer, &this->header_, sizeof (rreq_header));
    buffer += sizeof (rreq_header);

    /*SOURCE HOST */
    memcpy(buffer, this->hostname_source_.data(),
            this->hostname_source_.length());
    buffer += this->hostname_source_.length();

    /*DEST HOST */
    memcpy((unsigned char*)buffer, (unsigned char*) this->hostname_destination_.data(), this->hostname_destination_.length());
    buffer += hostname_destination_.length();


    /*PATH (list of macs from the src to dest)*/
    for (std::list<mac>::iterator it = this->path_l_.begin(); it != path_l_.end(); ++it) {
        memcpy(buffer, (unsigned char*) (*it).mac_adr, ETH_ALEN);

        buffer += ETH_ALEN;
    }


    /*CRC*/
    int dynamic_field_len = this->hostname_source_.length() + this->hostname_destination_.length() + 6 * path_l_.size();
    std::string crc_string = std::string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len);
    uint32_t crc = GetCrc32(crc_string);
    memcpy(buffer, &crc, sizeof (uint32_t));
    buffer += sizeof (uint32_t);

    

   return std::string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len + sizeof (crc));




}

RouteRequest::~RouteRequest() {
    // TODO Auto-generated destructor stub
}



