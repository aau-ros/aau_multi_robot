/* 
 * File:   MultiHopBroadcastFrame.cpp
 * Author: cwioro
 * 
 * Created on September 17, 2014, 12:00 PM
 */
#include <string>
#include "MultiHopBroadcastFrame.h"
#include "EthernetFrame.h"

MultiHopBroadcastFrame::MultiHopBroadcastFrame(std::string topic_to_publish, std::string data,  string source_host,  uint8_t payload_type_field, uint16_t hop_range)
{
    this->topic_ = topic_to_publish;
    this->hostname_source_ = source_host;
    this->payload_ = data;
    this->header_.payload_type = payload_type_field;
    this->header_.hop_limit = hop_range;
        header_.id = frame_count_stat++;

    memcpy(this->eh_h_.eh_dest, bcast_mac, 6);
    
    this->header_.current_hop = 1       ;
    
      this->header_.frame_type = FRAME_TYPE_BROADCAST;
    
}


MultiHopBroadcastFrame::MultiHopBroadcastFrame(unsigned char* buffer)
{
    unsigned char* buffer_start = buffer;
    buffer_str_len_ = 0;

    /*ETHERNET HEADER*/
    eh_header* eh = (struct eh_header *) buffer;
    buffer += sizeof (eh_header);
    buffer_str_len_ += sizeof (eh_header);

    /*FRAME HEADER*/
    mh_bcast_header* rfh = (struct mh_bcast_header *) buffer;
    buffer += sizeof (mh_bcast_header);
    buffer_str_len_ += sizeof (mh_bcast_header);

    /*SOURCE HOST*/
    hostname_source_ = "";
    hostname_source_.append((const char*) buffer, rfh->hostname_source_len);
    buffer += rfh->hostname_source_len;
    buffer_str_len_ += rfh->hostname_source_len;

    /*TOPIC*/
    topic_ = "";
    topic_.append((const char*) buffer, rfh->topic_len);
    buffer += rfh->topic_len;
    buffer_str_len_ += rfh->topic_len;


    /*PAYLOAD*/
    payload_ = "";
    payload_.append((const char*) buffer, rfh->payload_len);
    buffer += rfh->payload_len;
    buffer_str_len_ += rfh->payload_len;

    /*INIT FLAGS*/
    if(rfh->current_hop <= rfh->hop_limit)
        this->rebroadcast = true;
    else
        this->rebroadcast = false;

    /*CRC */
    uint32_t crc = 0;
    memcpy((unsigned char*) &crc, (unsigned char*) buffer, 4); // get CRC

    std::string crc_data_string = "";
    crc_data_string.append((const char*) buffer_start, this->HEADER_FIXED_LEN + rfh->hostname_source_len +  rfh->topic_len + rfh->payload_len);
    correct_crc_ = (crc == (uint32_t) GetCrc32(crc_data_string));

    
    
    /* COPY HEADER FIELDS */
    memcpy(&this->eh_h_, &(*eh), sizeof (eh_header));
    memcpy(&this->header_, &(*rfh), sizeof (mh_bcast_header));
}

MultiHopBroadcastFrame::~MultiHopBroadcastFrame()
{
}

std::string MultiHopBroadcastFrame::getFrameAsNetworkString(unsigned char* source)
{


   
    header_.current_hop++;
     memcpy(this->eh_h_.eh_source, source, 6);

    /*LEN FIELDS */
    this->header_.hostname_source_len = hostname_source_.length();
    this->header_.topic_len = topic_.length();
    this->header_.payload_len = payload_.length();




    unsigned char f_buffer[ETHER_MAX_LEN]; // = unsigned char[ETHER_MAX_LEN];
    unsigned char* buffer = f_buffer;
    unsigned char* buffer_start = f_buffer;



    /*ETHERNET FIELDS*/
    //eh_header* eh = (struct eh_header *) buffer;
    memcpy(buffer, &this->eh_h_, sizeof (eh_header));
    buffer += sizeof (eh_header);

    /*FIXED RF HEADER FIELDS*/
    memcpy(buffer, &this->header_, sizeof (mh_bcast_header));
    buffer += sizeof (mh_bcast_header);

    /*SOURCE HOST */
    memcpy(buffer, this->hostname_source_.data(),
           this->hostname_source_.length());
    buffer += this->hostname_source_.length();


    /*TOPIC*/
    memcpy(buffer, this->topic_.data(), this->topic_.length());
    buffer += topic_.length();


    /*PAYLOAD */
    memcpy(buffer, this->payload_.data(), this->payload_.length());
    buffer += payload_.length();


    /*CRC*/
    uint32_t dynamic_field_len = this->hostname_source_.length() + topic_.length() + payload_.length();
    std::string crc_string = std::string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len);
    uint32_t crc = GetCrc32(crc_string);
    memcpy(buffer, &crc, sizeof (uint32_t));
    buffer += sizeof (uint32_t);


    return string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len + sizeof (uint32_t));

}