/* 
 * File:   CrSelectionFrame.cpp
 * Author: cwioro
 * 
 * Created on August 25, 2014, 8:26 AM
 */

#include "CrSelectionFrame.h"
#include "CrDetectionFrame.h"
#include "defines.h"

CrSelectionFrame::CrSelectionFrame(unsigned char* dst, unsigned char* relay_conn, uint8_t relay_index)
{
    memcpy(eh_h_.eh_dest, dst, ETH_ALEN);
    memcpy(header_.relay_connection, relay_conn, ETH_ALEN);
    header_.relay_index = relay_index;
    
    header_.id = CrSelectionFrame::frame_count_stat++;
}

CrSelectionFrame::CrSelectionFrame(unsigned char* buffer)
{
    unsigned char* buffer_start = buffer;
    buffer_str_len_ = 0;

    /*ETHERNET HEADER*/
    eh_header* eh = (struct eh_header *) buffer;
    buffer += sizeof (eh_header);
    buffer_str_len_ += sizeof (eh_header);

    /*FRAME HEADER*/
    cr_selection_header* rfh = (struct cr_selection_header *) buffer;
    buffer += sizeof (cr_selection_header);
    buffer_str_len_ += sizeof (cr_selection_header);



    /*CRC */
    uint32_t crc = 0;
    memcpy((unsigned char*) &crc, (unsigned char*) buffer, 4); // get CRC

    std::string crc_data_string = "";

    crc_data_string.append((const char*) buffer_start, this->HEADER_FIXED_LEN );
    correct_crc_ = (crc == (uint32_t) GetCrc32(crc_data_string));
    buffer_str_len_++;




    /* COPY HEADER FIELDS */
    memcpy(&this->eh_h_, &(*eh), sizeof (eh_header));
    memcpy(&this->header_, &(*rfh), sizeof (cr_selection_header));

}

CrSelectionFrame::CrSelectionFrame(const CrSelectionFrame& orig)
{

}

CrSelectionFrame::~CrSelectionFrame()
{

}

std::string CrSelectionFrame::getFrameAsNetworkString(unsigned char* src)
{
    memcpy(eh_h_.eh_source, src, ETH_ALEN);

    header_.frame_type = FRAME_TYPE_CR_SELECTION;





    unsigned char f_buffer[ETHER_MAX_LEN]; // = unsigned char[ETHER_MAX_LEN];
    unsigned char* buffer = f_buffer;
    unsigned char* buffer_start = f_buffer;

    /*ETHERNET FIELDS*/
    memcpy(buffer, &this->eh_h_, sizeof (eh_header));
    buffer += sizeof (eh_header);

    /*FIXED RF HEADER FIELDS*/
    memcpy(buffer, &this->header_, sizeof (cr_selection_header));
    buffer += sizeof (cr_selection_header);


    /*CRC*/
    std::string crc_string = std::string((const char*) buffer_start, this->HEADER_FIXED_LEN);
    uint32_t crc = GetCrc32(crc_string);
    memcpy(buffer, &crc, sizeof (uint32_t));
    buffer += sizeof (uint32_t);




    return string((const char*) buffer_start, this->HEADER_FIXED_LEN + sizeof (crc));

}

