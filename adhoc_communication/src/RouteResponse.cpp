/*
 *
 * RouteResponse.h
 *
 *  Created on: 25.07.2013
 *      Author: Günther Cwioro
 *
 *
 * Structure of route response:
 *
 * BROADCAST MAC 	// -> ff:ff:ff:ff:ff:ff  											6Byte
 * SOURCE MAC		// MAC of the using interface										6Byte
 * ETH_TYPE_FIELD	// 0x4148 (AH as string)											2Byte
 * FRAME_TYPE		// 0x72 (r as string)												1Byte
 * REQUEST ID.		// Every node has a consecutive number for his requests				4Byte
 * MC FLAG																				1Byte
 * ROOT DISTANCE	// distancte to the mc root											1Byte
 * HOB COUNT		// is the amount of node which are involved to process the packet	4Byte
 * CURRENT HOP 		// hop to be next to send											4Byte
 * PATH				//the list of macs from the source to destination					HOB COUNT * 6Byte
 * SOURCE HOST LENGTH																	4Byte
 * SOURCE HOST																			VAR
 * CRC																					4Byte
 *
 *

 *
 * Every node which receives an router response, checks first if he was involved in the route request and then checks if his mac is the current hop mac and forwards the response to the source.
 * The Response takes the route backwards of the route request to the source.
 */


#include <linux/if_ether.h>

#include "RouteResponse.h"

#include "defines.h"
#include "functions.h"

RouteResponse::RouteResponse(unsigned char* buffer)
{

    unsigned char* buffer_start = buffer;
    /*ETHERNET HEADER*/
    unsigned char* mac_list_pos = buffer + 29; //points to the first mac in path_l_;
    ethhdr* eh = (struct ethhdr *) buffer;
    buffer += 15; // 14 eth_head + 1 frame_Type



    memcpy(&this->eh_, &(*eh), sizeof (ethhdr));


 

    /*REQUEST ID*/
    uint32_t num_from_network; //help var to convert the integers from bigE to littleE
    memcpy((unsigned char*) &num_from_network, (unsigned char*) buffer, 4);
    request_id_ = ntohl(num_from_network);
    buffer += 4;

    /*MC FLAG*/
    memcpy((unsigned char*) &mc_flag_, (unsigned char*) buffer, 1);
    buffer += 1;

    /*ROOT DISTANCE*/
    memcpy((unsigned char*) &root_distance, (unsigned char*) buffer, 1);
    buffer += 1;

    /*HOP COUNT*/
    memcpy((unsigned char*) &num_from_network, (unsigned char*) buffer, 4);
    hop_count_ = ntohl(num_from_network);
    buffer += 4;

    /*CURRENT HOP*/
    memcpy((unsigned char*) &num_from_network, (unsigned char*) buffer, 4);
    current_hop_ = ntohl(num_from_network);
    current_hop_++;
    buffer += 4;

    /*PATH*/
    for (uint32_t i = 0; i < hop_count_; i++)
    {
        mac currentMac;
        memcpy((unsigned char*) currentMac.mac_adr, (unsigned char*) buffer, 6);
        path_l_.push_back(currentMac);
        buffer += 6;
    }


    /*SRC HOST LEN*/
    memcpy((unsigned char*) &num_from_network, (unsigned char*) buffer, 4); // get frameId
    uint32_t hostname_source_Len = ntohl(num_from_network);
    buffer += 4;

    /*SRC HOST*/
    hostname_source_ = "";
    for (uint32_t i = 0; i < hostname_source_Len; i++)
    {
        hostname_source_.append((const char*) buffer, 1);
        buffer++;
    }

    /*CRC */
    uint32_t crc = 0;
    memcpy((unsigned char*) &num_from_network, (unsigned char*) buffer, 4); // get CRC
    crc = ntohl(num_from_network);

    std::string crc_data_string = "";
    crc_data_string.append((const char*) buffer_start, RouteResponse::HEADER_FIXED_LEN - 4 + hostname_source_.length() + 6 * hop_count_);

    if (crc == (uint32_t) GetCrc32(crc_data_string))
        correct_crc_ = true;
    else
        correct_crc_ = false;

    /*INIT mac_current_hop_ */
    unsigned char* mac_current_hop_Pos = mac_list_pos;
    uint32_t relativeMacPostion;
    if (hop_count_ >= current_hop_)
        relativeMacPostion = hop_count_ - current_hop_;
    else
        relativeMacPostion = 0;

    mac_current_hop_Pos += (relativeMacPostion * 6);

    memcpy((void*) mac_current_hop_, (unsigned char*) mac_current_hop_Pos, 6);

    /*INIT mac_previous_hop_ */
    mac_current_hop_Pos = mac_list_pos;
    if (hop_count_ - 1 >= current_hop_)
        relativeMacPostion = hop_count_ - 1 - current_hop_;
    else
        relativeMacPostion = 0;

    mac_current_hop_Pos += (relativeMacPostion * 6);

    memcpy((void*) mac_previous_hop_, (unsigned char*) mac_current_hop_Pos, 6);



    /*INIT mac_next_hop_ */
    mac_current_hop_Pos = mac_list_pos;
    if (hop_count_ + 1 >= current_hop_)
        relativeMacPostion = hop_count_ + 1 - current_hop_;
    else
        relativeMacPostion = 0;

    mac_current_hop_Pos += (relativeMacPostion * 6);

    memcpy((void*) mac_next_hop_, (unsigned char*) mac_current_hop_Pos, 6);


}

RouteResponse::RouteResponse(RouteRequest request, unsigned char macDestination[6], uint8_t root_distance)
{

    hostname_source_ = request.hostname_source_;
    this->root_distance = root_distance;
    this->hop_count_ = request.header_.hop_count + 1;
    request_id_ = request.header_.id;
    current_hop_ = 1;
    mc_flag_ = request.mc_flag_;
    //	flag_field_ = RouteRequest::createFlagField(true,false);
    this->path_l_ = request.path_l_;
    mac dMac;
    memcpy((void*) dMac.mac_adr, (void*) macDestination, 6);
    path_l_.push_back(dMac);

}

std::string RouteResponse::getResponseAsNetworkString(unsigned char source_mac[6])
{
   

    unsigned char* buffer = new unsigned char[ETHER_MAX_LEN];
    unsigned char* eth_head = buffer;
    unsigned char* eth_data = buffer + 14;
    uint32_t int_htonl;
    uint32_t buffer_offset = 0;

    /* Build Ethernet header*/
    memcpy(buffer, bcast_mac, ETH_ALEN);

    memcpy(buffer + 6, source_mac, ETH_ALEN);

    uint16_t tf = htons(ETH_TYPE);
    memcpy(eth_head + 12, &tf, 2);
    buffer_offset += 14;

    /*PACKET TYPE*/
    uint8_t ff = FRAME_TYPE_REPLY;
    memcpy(eth_data, &ff, 1);
    eth_data++;
    buffer_offset += 1;

    /*REQ ID*/
    int_htonl = htonl(request_id_);
    memcpy(eth_data, &int_htonl, sizeof (uint32_t));
    eth_data += sizeof (uint32_t);
    buffer_offset += sizeof (uint32_t);

    /*MC FLAG*/
    memcpy(eth_data, &mc_flag_, 1);
    eth_data++;
    buffer_offset += 1;

    /*ROOT DISTANCE*/
    memcpy(eth_data, &root_distance, 1);
    eth_data++;
    buffer_offset += 1;

    /*HOP COUNT*/
    int_htonl = htonl(this->hop_count_);
    memcpy(eth_data, &int_htonl, sizeof (uint32_t));
    eth_data += sizeof (uint32_t);
    buffer_offset += sizeof (uint32_t);

    /*CURRENT HOP*/
    int_htonl = htonl(current_hop_);
    memcpy(eth_data, &int_htonl, sizeof (uint32_t));
    eth_data += sizeof (uint32_t);
    buffer_offset += sizeof (uint32_t);
    /*MAC LIST*/


    for (std::list<mac>::iterator it = this->path_l_.begin(); it != path_l_.end(); ++it)
    {
        mac & tmp(*it);
        memcpy(eth_data, (unsigned char*) tmp.mac_adr, ETH_ALEN);
        eth_data += ETH_ALEN;
        buffer_offset += ETH_ALEN;
    }


    /*SOURCE HOST LENGTH*/
    int_htonl = htonl(hostname_source_.length());
    memcpy(eth_data, &int_htonl, sizeof (uint32_t));
    eth_data += sizeof (uint32_t);
    buffer_offset += sizeof (uint32_t);

    /*SOURCE HOST*/
    memcpy(eth_data, hostname_source_.data(), hostname_source_.length());
    eth_data += hostname_source_.length();
    buffer_offset += hostname_source_.length();

    /*CRC*/
    std::string crc_string = std::string((const char*) buffer, buffer_offset);
    uint32_t crc = GetCrc32(crc_string);
    int_htonl = htonl(crc);
    memcpy(eth_data, &int_htonl, sizeof (uint32_t));
    eth_data += sizeof (uint32_t);
    buffer_offset += sizeof (uint32_t);

    std::string res = std::string((const char*) buffer, buffer_offset);
    delete [] buffer;
    return res;
}

RouteResponse::~RouteResponse()
{
    // TODO Auto-generated destructor stub
}

int RouteResponse::GetCrc32(const std::string& my_string)
{
    boost::crc_32_type result;
    result.process_bytes(my_string.data(), my_string.length());
    return result.checksum();
}

