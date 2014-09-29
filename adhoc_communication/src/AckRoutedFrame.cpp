/*
 * AckRoutedFrame.h
 *
 *  Created on: 29.07.2013
 *      Author: Günther Cwioro
 *
 */

#include <string>

#include "AckRoutedFrame.h"
#include "structs.h"
#include "EthernetFrame.h"
#include "defines.h"

AckRoutedFrame::~AckRoutedFrame()
{
    // TODO Auto-generated destructor stub
}

AckRoutedFrame::AckRoutedFrame(RoutedFrame rf)
{
    header_.frame_id = rf.header_.frame_id;
    mc_group_ = rf.mc_g_name_;
    cr_flag = false;

    mc_flag = mc_group_.compare("") != 0;


    if (mc_flag)
    {

        /* If its a multicast frame the fields packet_id and route_id will be used to store the packet_id and the packet_squence_num*/
        hostname_source_ = rf.hostname_source_;
        header_.frame_id = rf.header_.packet_id;
        header_.route_id = rf.header_.packet_sequence_num;
    }
}

AckRoutedFrame::AckRoutedFrame(unsigned char* buffer)
{
    unsigned char* buffer_start = buffer;
    buffer_str_len_ = 0;

    /*ETHERNET HEADER*/
    eh_header* eh = (struct eh_header *) buffer;
    buffer += sizeof (eh_header);
    buffer_str_len_ += sizeof (eh_header);

    /*FRAME HEADER*/
    ack_rf_header* rfh = (struct ack_rf_header *) buffer;
    buffer += sizeof (ack_rf_header);
    buffer_str_len_ += sizeof (ack_rf_header);

    /*SOURCE HOST*/
    hostname_source_ = "";
    hostname_source_.append((const char*) buffer, rfh->hostname_source_len);
    buffer += rfh->hostname_source_len;
    buffer_str_len_ += hostname_source_.length();

    /*MC GROUP HOST*/
    mc_group_ = "";
    mc_group_.append((const char*) buffer, rfh->mc_group_len);
    buffer += rfh->mc_group_len;
    buffer_str_len_ += rfh->mc_group_len;



    /*INIT FLAGS*/
    cr_flag = (rfh->flag_field / 128) % 2 == 1;
    mc_flag = (rfh->flag_field / 64) % 2 == 1;


    /*CRC */
    uint32_t crc = 0;
    memcpy((unsigned char*) &crc, (unsigned char*) buffer, 4); // get CRC

    std::string crc_data_string = "";
    uint16_t dynamic_field_len = +rfh->hostname_source_len + rfh->mc_group_len;
    crc_data_string.append((const char*) buffer_start,
                           this->HEADER_FIXED_LEN + dynamic_field_len);
    correct_crc_ = (crc == (uint32_t) GetCrc32(crc_data_string));
    buffer_str_len_++;




    /* COPY HEADER FIELDS */
    memcpy(&this->eh_h_, &(*eh), sizeof (eh_header));
    memcpy(&this->header_, &(*rfh), sizeof (ack_rf_header));



}

std::string AckRoutedFrame::getFrameAsNetworkString(uint32_t route_id, unsigned char next_hop[6], string source_host, unsigned char source[6])
{
    if (mc_flag)
    {
        memcpy(header_.mac_destination_, bcast_mac, ETH_ALEN);
    }
    else
    {
        hostname_source_ = source_host;
        header_.route_id = route_id;
        memcpy(header_.mac_destination_, next_hop, ETH_ALEN);

    }

    memcpy(eh_h_.eh_source, source, ETH_ALEN);
    header_.frame_type = FRAME_TYPE_TRANSPORT_ACK;


    /*FLAG FIELD*/
    this->header_.flag_field = 0;
    if (cr_flag)
        this->header_.flag_field += 128;
    if (mc_flag)
        this->header_.flag_field += 64;

    /*LEN FIELDS */
    this->header_.hostname_source_len = hostname_source_.length();
    this->header_.mc_group_len = mc_group_.length();

    unsigned char f_buffer[ETHER_MAX_LEN]; // = unsigned char[ETHER_MAX_LEN];
    unsigned char* buffer = f_buffer;
    unsigned char* buffer_start = f_buffer;

    /*ETHERNET FIELDS*/
    //eh_header* eh = (struct eh_header *) buffer;
    memcpy(buffer, &this->eh_h_, sizeof (eh_header));
    buffer += sizeof (eh_header);

    /*FIXED RF HEADER FIELDS*/
    memcpy(buffer, &this->header_, sizeof (ack_rf_header));
    buffer += sizeof (ack_rf_header);

    /*SOURCE HOST */
    memcpy(buffer, this->hostname_source_.data(),
           this->hostname_source_.length());
    buffer += this->hostname_source_.length();

    /*MC GROUP  */
    memcpy(buffer, this->mc_group_.data(),
           this->mc_group_.length());
    buffer += this->mc_group_.length();



    /*CRC*/
    int dynamic_field_len = this->hostname_source_.length() + this->mc_group_.length();
    std::string crc_string = std::string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len);
    uint32_t crc = GetCrc32(crc_string);
    memcpy(buffer, &crc, sizeof (uint32_t));
    buffer += sizeof (uint32_t);




    return string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len + sizeof (crc));


}

std::string AckRoutedFrame::getFrameAsNetworkString(routing_entry r, unsigned char src[6])
{
    return this->getFrameAsNetworkString(r.id,r.previous_hop,r.hostname_source,src);
}


stc_frame AckRoutedFrame::getFrameStruct()
{
    stc_frame stc;
    stc.frame_id = header_.frame_id;
    memcpy(stc.mac, header_.mac_destination_, ETH_ALEN);
    stc.mc_group = mc_group_;
    stc.hostname_source = hostname_source_;


  
    stc.retransmitted = 0;

    return stc;
}
