/* 
 * File:   McNackFrame.cpp
 * Author: cwioro
 * 
 * Created on September 10, 2014, 11:37 AM
 */

#include <vector>

#include "McNackFrame.h"

McNackFrame::McNackFrame(unsigned char* source, unsigned char* dest, std::string hostname_src, std::string group_n, uint32_t p_id, std::vector<uint32_t> seq_n)
{

    /*ETH HEADER*/
    memcpy(eh_h_.eh_source, source, 6);
    memcpy(eh_h_.eh_dest, dest, 6);

    /*FRAME HEADER*/
    this->hostname_source_ = hostname_src;
    this->header_.packet_id = p_id;
    this->header_.frame_type = FRAME_TYPE_MC_NACK;
    this->mc_group_ = group_n;
    this->req_seq_nums_ = seq_n;
    this->header_.frame_seq_num_size = seq_n.size();
        this->header_.id = McRouteActivationFrame::stat_id_count++;
}

McNackFrame::McNackFrame(unsigned char* buffer)
{

    unsigned char* buffer_start = buffer;
    buffer_str_len_ = 0;

    /*ETHERNET HEADER*/
    eh_header* eh = (struct eh_header *) buffer;
    buffer += sizeof (eh_header);
    buffer_str_len_ += sizeof (eh_header);

    /*FRAME HEADER*/
    mc_nack_header* rfh = (struct mc_nack_header *) buffer;
    buffer += sizeof (mc_nack_header);
    buffer_str_len_ += sizeof (mc_nack_header);

    /*SOURCE HOST*/
    hostname_source_ = "";
    hostname_source_.append((const char*) buffer, rfh->hostname_source_len);
    buffer += rfh->hostname_source_len;
    buffer_str_len_ += hostname_source_.length();

    /*MC GROUP */
    mc_group_ = "";
    if (rfh->mc_group_len > 0)
    {
        mc_group_.append((const char*) buffer, rfh->mc_group_len);
        buffer += rfh->mc_group_len;
    }
    buffer_str_len_ += mc_group_.length();


    /* req frame list */
    for(uint32_t i = 0; i < rfh->frame_seq_num_size; i++)
    {
        uint32_t n;
        memcpy(&n, buffer, sizeof(uint32_t));
        buffer += sizeof(uint32_t);
        req_seq_nums_.push_back(n);
    }


    /*CRC */
    uint32_t crc = 0;
    memcpy((unsigned char*) &crc, (unsigned char*) buffer, 4); // get CRC

    std::string crc_data_string = "";
    crc_data_string.append((const char*) buffer_start, this->HEADER_FIXED_LEN + rfh->hostname_source_len + rfh->mc_group_len + rfh->frame_seq_num_size * sizeof(uint32_t));
    correct_crc_ = (crc == (uint32_t) GetCrc32(crc_data_string));
    buffer_str_len_++;



    /* COPY HEADER FIELDS */
    memcpy(&this->eh_h_, &(*eh), sizeof (eh_header));
    memcpy(&this->header_, &(*rfh), sizeof (mc_nack_header));



}

McNackFrame::~McNackFrame()
{
    // TODO Auto-generated destructor stub
}

std::string McNackFrame::getFrameAsNetworkString()
{




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
    memcpy(buffer, &this->header_, sizeof (mc_nack_header));
    buffer += sizeof (mc_nack_header);

    /*SOURCE HOST */
    memcpy(buffer, this->hostname_source_.data(),
           this->hostname_source_.length());
    buffer += this->hostname_source_.length();

    /*MC GROUP */
    memcpy(buffer, this->mc_group_.data(), this->mc_group_.length());
    buffer += mc_group_.length();

    /*req_seq_nums_*/
    for (std::vector<uint32_t>::iterator i = req_seq_nums_.begin(); i != req_seq_nums_.end(); i++)
    {
        memcpy(buffer, &(*i), sizeof(uint32_t));
        buffer += sizeof(uint32_t);
    }



    /*CRC*/
    int dynamic_field_len = this->hostname_source_.length() + this->mc_group_.length() + req_seq_nums_.size() * sizeof(uint32_t);
    std::string crc_string = std::string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len);
    uint32_t crc = GetCrc32(crc_string);
    memcpy(buffer, &crc, sizeof (uint32_t));
    buffer += sizeof (uint32_t);



    return string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len + sizeof (crc));


}

void McNackFrame::print_frame()
{
    ROS_ERROR("FRAME PACKET ID[%u] SRC[%s]  MC GROUP[%s]",  header_.packet_id, hostname_source_.c_str(), mc_group_.c_str());
    ROS_ERROR("SRC MAC[%s] DST MAC[%s]", getMacAsStr(eh_h_.eh_source).c_str(), getMacAsStr(eh_h_.eh_dest).c_str());
}

