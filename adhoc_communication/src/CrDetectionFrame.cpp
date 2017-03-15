/* 
 * File:   CrDetectionFrame.cpp
 * Author: cwioro
 * 
 * Created on August 25, 2014, 8:29 AM
 */

#include "CrDetectionFrame.h"
#include "defines.h"

CrDetectionFrame::CrDetectionFrame(unsigned char* first_mac, unsigned char* second_mac)
{
    memcpy(header_.mac1, first_mac, 6);
    memcpy(header_.mac2, second_mac, 6);
}

CrDetectionFrame::CrDetectionFrame(unsigned char* buffer)
{
    unsigned char* buffer_start = buffer;
    buffer_str_len_ = 0;

    /*ETHERNET HEADER*/
    eh_header* eh = (struct eh_header *) buffer;
    buffer += sizeof (eh_header);
    buffer_str_len_ += sizeof (eh_header);

    /*FRAME HEADER*/
    cr_detec_header* rfh = (struct cr_detec_header *) buffer;
    buffer += sizeof (cr_detec_header);
    buffer_str_len_ += sizeof (cr_detec_header);



    /*CRC */
    uint32_t crc = 0;
    memcpy((unsigned char*) &crc, (unsigned char*) buffer, 4); // get CRC

    std::string crc_data_string = "";

    crc_data_string.append((const char*) buffer_start,this->HEADER_FIXED_LEN );
    correct_crc_ = (crc == (uint32_t) GetCrc32(crc_data_string));
    buffer_str_len_++;




    /* COPY HEADER FIELDS */
    memcpy(&this->eh_h_, &(*eh), sizeof (eh_header));
    memcpy(&this->header_, &(*rfh), sizeof (cr_detec_header));

}

std::string CrDetectionFrame::getFrameAsNetworkString(unsigned char* src)
{
  

    memcpy(eh_h_.eh_source, src, ETH_ALEN);
      memcpy(eh_h_.eh_dest, bcast_mac, ETH_ALEN);
    header_.frame_type = FRAME_TYPE_CR_DETECTION;





    unsigned char f_buffer[ETHER_MAX_LEN]; // = unsigned char[ETHER_MAX_LEN];
    unsigned char* buffer = f_buffer;
    unsigned char* buffer_start = f_buffer;

    /*ETHERNET FIELDS*/
    memcpy(buffer, &this->eh_h_, sizeof (eh_header));
    buffer += sizeof (eh_header);

    /*FIXED RF HEADER FIELDS*/
    memcpy(buffer, &this->header_, sizeof (cr_detec_header));
    buffer += sizeof (cr_detec_header);


    /*CRC*/
    std::string crc_string = std::string((const char*) buffer_start, this->HEADER_FIXED_LEN);
    uint32_t crc = GetCrc32(crc_string);
    memcpy(buffer, &crc, sizeof (uint32_t));
    buffer += sizeof (uint32_t);




    return string((const char*) buffer_start, this->HEADER_FIXED_LEN  + sizeof (crc));


}

CrDetectionFrame::CrDetectionFrame(const CrDetectionFrame& orig)
{
}

CrDetectionFrame::~CrDetectionFrame()
{
}

