/*
 * AckLinkFrame.cpp
 *
 *  Created on: 29.07.2013
 *      Author: Günther Cwioro
 *
 */


#include <string>

#include "AckLinkFrame.h"
#include "EthernetFrame.h"
#include "defines.h"

AckLinkFrame::AckLinkFrame(unsigned char source[],unsigned char confirmer_mac[],unsigned char dest[],uint32_t frame_id,std::string hostname,uint8_t type)
{
    
    
    /*ETH HEADER*/
    memcpy(eh_h_.eh_source, source, 6);
    memcpy(eh_h_.eh_dest, bcast_mac, 6);

    /*FRAME HEADER*/
    memcpy(header_.mac_confirmer, confirmer_mac, 6);
    memcpy(header_.mac_destination_, dest, 6);
    this->header_.ack_frame_type = type;
    this->header_.frame_id = frame_id;
    this->hostname_source_ = hostname;
    this->header_.frame_type = FRAME_TYPE_ACK;
    this->pos_ack_flag_ = true;
    this->cr_flag_ = false;
}



AckLinkFrame::AckLinkFrame(unsigned char* buffer)
{

    unsigned char* buffer_start = buffer;
    buffer_str_len_ = 0;

    /*ETHERNET HEADER*/
    eh_header* eh = (struct eh_header *) buffer;
    buffer += sizeof (eh_header);
    buffer_str_len_ += sizeof (eh_header);

    /*FRAME HEADER*/
    ack_lf_header* rfh = (struct ack_lf_header *) buffer;
    buffer += sizeof (ack_lf_header);
    buffer_str_len_ += sizeof (ack_lf_header);

    /*SOURCE HOST*/
    hostname_source_ = "";
    hostname_source_.append((const char*) buffer, rfh->hostname_source_len);
    buffer += rfh->hostname_source_len;
    buffer_str_len_ += hostname_source_.length();


    /*INIT FLAGS*/
    pos_ack_flag_ = (rfh->flag_field / 128) % 2 == 1;
      cr_flag_ = (rfh->flag_field / 64) % 2 == 1;


    /*CRC */
    uint32_t crc = 0;
    memcpy((unsigned char*) &crc, (unsigned char*) buffer, 4); // get CRC

    std::string crc_data_string = "";
    crc_data_string.append((const char*) buffer_start, this->HEADER_FIXED_LEN + rfh->hostname_source_len );
    correct_crc_ = (crc == (uint32_t) GetCrc32(crc_data_string));
    buffer_str_len_++;



    /* COPY HEADER FIELDS */
    memcpy(&this->eh_h_, &(*eh), sizeof (eh_header));
    memcpy(&this->header_, &(*rfh), sizeof (ack_lf_header));



}

AckLinkFrame::~AckLinkFrame()
{
    // TODO Auto-generated destructor stub
}

std::string AckLinkFrame::getFrameAsNetworkString()
{


    /*FLAG FIELD*/
    this->header_.flag_field = 0;

    if (pos_ack_flag_)
        this->header_.flag_field += 128;
    if(cr_flag_)
         this->header_.flag_field += 64;

    /*LEN FIELDS */
    this->header_.hostname_source_len = hostname_source_.length();



    unsigned char f_buffer[ETHER_MAX_LEN]; // = unsigned char[ETHER_MAX_LEN];
    unsigned char* buffer = f_buffer;
    unsigned char* buffer_start = f_buffer;

    /*ETHERNET FIELDS*/
    memcpy(buffer, &this->eh_h_, sizeof (eh_header));
    buffer += sizeof (eh_header);

    /*FIXED RF HEADER FIELDS*/
    memcpy(buffer, &this->header_, sizeof (ack_lf_header));
    buffer += sizeof (ack_lf_header);

    /*SOURCE HOST */
    memcpy(buffer, this->hostname_source_.data(),this->hostname_source_.length());
    buffer += this->hostname_source_.length();


    /*CRC*/
    int dynamic_field_len = this->hostname_source_.length() ;
    std::string crc_string = std::string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len);
    uint32_t crc = GetCrc32(crc_string);
    memcpy(buffer, &crc, sizeof (uint32_t));
    buffer += sizeof (uint32_t);



    return string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len + sizeof (crc));


}



void AckLinkFrame::print_frame()
{
    ROS_ERROR("FRAME ID [%u] SRC[%s] TYPE[%u]", header_.frame_id,hostname_source_.c_str(), header_.ack_frame_type);
    ROS_ERROR("MAC_CONF[%s] SRC MAC[%s] DST MAC[%s]", getMacAsStr(header_.mac_confirmer).c_str(),getMacAsStr(eh_h_.eh_source).c_str(),getMacAsStr(eh_h_.eh_dest).c_str()  );
}