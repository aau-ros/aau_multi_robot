/* 
 * File:   McDisconnectFrame.cpp
 * Author: cwioro
 * 
 * Created on September 1, 2014, 1:18 PM
 */

#include "McDisconnectFrame.h"

McDisconnectFrame::McDisconnectFrame(unsigned char* next_hop, std::string mc_g)
{
    memcpy(header_.mac_destination, next_hop, 6);
    this->header_.frame_type = FRAME_TYPE_MC_DISCONNECT;
    this->header_.id = McRouteActivationFrame::stat_id_count++;
    mc_group_ = mc_g;
    disconnect_downlink = false;
    disconnect_uplink = false;
}

McDisconnectFrame::McDisconnectFrame(unsigned char* buffer)
{
    unsigned char* buffer_start = buffer;

    eh_header* eh = (struct eh_header *) buffer;
    buffer += sizeof (eh_header);
    mc_disc_header* rfh = (struct mc_disc_header *) buffer;
    buffer += sizeof (mc_disc_header);



    /*group name*/
    mc_group_ = "";
    if (rfh->mc_group_name_len > 0)
        mc_group_.append((const char*) buffer, rfh->mc_group_name_len);
    buffer += rfh->mc_group_name_len;


    /*INIT FLAGS*/
    disconnect_downlink = (rfh->flag_field / 128) % 2 == 1;
    disconnect_uplink = (rfh->flag_field / 64) % 2 == 1;



    /*CRC */
    uint32_t crc = 0;
    memcpy((unsigned char*) &crc, (unsigned char*) buffer, 4); // get CRC

    std::string crc_data_string = "";
    uint32_t dynamic_field_len = rfh->mc_group_name_len;
    crc_data_string.append((const char*) buffer_start,
                           this->HEADER_FIXED_LEN + dynamic_field_len);
    correct_crc_ = (crc == (uint32_t) GetCrc32(crc_data_string));


    buffer_str_len_ = this->HEADER_FIXED_LEN + dynamic_field_len + sizeof (crc);



    /* COPY HEADER FIELDS */
    memcpy(&this->eh_h_, &(*eh), sizeof (eh_header));
    memcpy(&this->header_, &(*rfh), sizeof (mc_disc_header));


}

McDisconnectFrame::~McDisconnectFrame()
{
}

std::string McDisconnectFrame::getFrameAsNetworkString(unsigned char* source)
{
    memcpy(this->eh_h_.eh_source, source, ETH_ALEN);
    memcpy(this->eh_h_.eh_dest, bcast_mac, ETH_ALEN);


    /*FLAG FIELD*/
    this->header_.flag_field = 0;
    if (disconnect_downlink)
        this->header_.flag_field += 128;
    if (disconnect_uplink)
        this->header_.flag_field += 64;
    
    
    if(disconnect_downlink == disconnect_uplink)
        ROS_ERROR("In McDisconnectFrame: disconnect_downlink == disconnect_uplink");
    


    /*LEN FIELD */
    this->header_.mc_group_name_len = mc_group_.length();

    unsigned char a_char[ETHER_MAX_LEN];
    unsigned char* buffer = a_char;
    unsigned char* buffer_start = a_char;

    /*ETHERNET FIELDS*/
    memcpy(buffer, &this->eh_h_, sizeof (eh_header));
    buffer += sizeof (eh_header);

    /*FIXED RF HEADER FIELDS*/
    memcpy(buffer, &this->header_, sizeof (mc_disc_header));
    buffer += sizeof (mc_disc_header);


    /*MC GROUP HOST */
    memcpy(buffer, this->mc_group_.data(),
           this->mc_group_.length());
    buffer += this->mc_group_.length();



    /*CRC*/
    int dynamic_field_len = this->mc_group_.length();
    std::string crc_string = std::string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len);
    uint32_t crc = GetCrc32(crc_string);
    memcpy(buffer, &crc, sizeof (uint32_t));
    buffer += sizeof (uint32_t);



    return std::string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len + sizeof (crc));

}
