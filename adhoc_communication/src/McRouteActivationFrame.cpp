/* 
 * File:   McRouteActivationFrame.cpp
 * Author: cwioro
 * 
 * Created on August 27, 2014, 12:20 PM
 */

#include "McRouteActivationFrame.h"

McRouteActivationFrame::McRouteActivationFrame(unsigned char* next_hop, std::string mc_g, uint32_t route_id, std::string source)
{
    memcpy(header_.mac_destination, next_hop, 6);
    this->header_.frame_type = FRAME_TYPE_MC_ACTIVATION;
    this->mc_group_ = mc_g;
    this->header_.route_id = route_id;
    this->hostname_source_ = source;
    this->header_.id = McRouteActivationFrame::stat_id_count++;
}

McRouteActivationFrame::McRouteActivationFrame(unsigned char* buffer)
{


    unsigned char* buffer_start = buffer;

    eh_header* eh = (struct eh_header *) buffer;
    buffer += sizeof (eh_header);
    mc_act_header* rfh = (struct mc_act_header *) buffer;
    buffer += sizeof (mc_act_header);


       /*hostname source*/
    hostname_source_ = "";
    if (rfh->hostname_source_len > 0)
        hostname_source_.append((const char*) buffer, rfh->hostname_source_len);
    buffer += rfh->hostname_source_len;

    /*group name*/
    mc_group_ = "";
    if (rfh->mc_group_name_len > 0)
        mc_group_.append((const char*) buffer, rfh->mc_group_name_len);
    buffer += rfh->mc_group_name_len;


    /*CRC */
    uint32_t crc = 0;
    memcpy((unsigned char*) &crc, (unsigned char*) buffer, 4); // get CRC

    std::string crc_data_string = "";
    uint32_t dynamic_field_len = rfh->mc_group_name_len +rfh->hostname_source_len;
    crc_data_string.append((const char*) buffer_start,
                           this->HEADER_FIXED_LEN + dynamic_field_len);
    correct_crc_ = (crc == (uint32_t) GetCrc32(crc_data_string));


    buffer_str_len_ = this->HEADER_FIXED_LEN + dynamic_field_len + sizeof (crc);



    /* COPY HEADER FIELDS */
    memcpy(&this->eh_h_, &(*eh), sizeof (eh_header));
    memcpy(&this->header_, &(*rfh), sizeof (mc_act_header));




}

std::string McRouteActivationFrame::getFrameAsNetworkString(unsigned char source_mac[6])
{
    memcpy(this->eh_h_.eh_source, source_mac, ETH_ALEN);
    memcpy(this->eh_h_.eh_dest, bcast_mac, ETH_ALEN);


    /*LEN FIELD */
    this->header_.mc_group_name_len = mc_group_.length();
    this->header_.hostname_source_len = hostname_source_.length();

    unsigned char a_char[ETHER_MAX_LEN];
    unsigned char* buffer = a_char;
    unsigned char* buffer_start = a_char;

    /*ETHERNET FIELDS*/
    memcpy(buffer, &this->eh_h_, sizeof (eh_header));
    buffer += sizeof (eh_header);

    /*FIXED RF HEADER FIELDS*/
    memcpy(buffer, &this->header_, sizeof (mc_act_header));
    buffer += sizeof (mc_act_header);

 /*HOST SRC HOST */
    memcpy(buffer, this->hostname_source_.data(),this->mc_group_.length());
    buffer += this->hostname_source_.length();


    /*MC GROUP HOST */
    memcpy(buffer, this->mc_group_.data(),
           this->mc_group_.length());
    buffer += this->mc_group_.length();
    
    

    /*CRC*/
    int dynamic_field_len = this->mc_group_.length()+ this->hostname_source_.length();
    std::string crc_string = std::string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len);
    uint32_t crc = GetCrc32(crc_string);
    memcpy(buffer, &crc, sizeof (uint32_t));
    buffer += sizeof (uint32_t);



    return std::string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len + sizeof (crc));

}

McRouteActivationFrame::~McRouteActivationFrame()
{
}

