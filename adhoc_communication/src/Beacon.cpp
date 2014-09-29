/*
 * Beacon.h
 *
 *  Created on: 19.07.2013
 *      Author: Günther Cwioro
 *
 * Hello frame structure:
 *
 * BROADCAST MAC 	// -> ff:ff:ff:ff:ff:ff  											6Byte
 * SOURCE MAC		// MAC of the using interface										6Byte
 * ETH_TYPE_FIELD	// 0x5555 (UU as string)											2Byte
 * FRAME_TYPE		-> H																1Byte
 * HOSTNAME LENGTH																		4Byte
 * HOSTNAME																				VAR
 * CRC																					4Byte
 */


#include "Beacon.h"
#include "functions.h"

Beacon::Beacon(unsigned char* source, std::string hname)
{
    memcpy(mac_source_, source, 6);


    this->hostname_ = hname;
}

Beacon::Beacon(unsigned char* buffer)
{

    
    unsigned char* buffer_start = buffer;
    /*ETHERNET HEADER*/
    eh_ = (struct ethhdr *) buffer;
    memcpy(mac_source_, eh_->h_source, 6);
    buffer += 15; // 14 eth_head + 1 frame_Type
    uint32_t num_from_network;

    /* HOSTNAME LEN */
    memcpy((unsigned char*) &num_from_network, (unsigned char*) buffer, 4); // get hostname length
    uint32_t hostnameLength = ntohl(num_from_network);
    buffer += 4;

    /*HOSTNAME*/
    hostname_ = "";  
    hostname_.append((const char*) buffer, hostnameLength);
    buffer+=hostnameLength;
    

    /*CRC */
    uint32_t crc = 0;
    memcpy((unsigned char*) &num_from_network, (unsigned char*) buffer, 4); // get CRC
    crc = ntohl(num_from_network);

    std::string crc_data_string = "";
    crc_data_string.append((const char*) buffer_start, Beacon::HEADER_FIXED_LEN - 4 + hostname_.length());

    if (crc == (uint32_t) GetCrc32(crc_data_string))
        correct_crc_ = true;
    else
        correct_crc_ = false;


}

Beacon::~Beacon()
{
    // TODO Auto-generated destructor stub
}

std::string Beacon::getFrameAsNetworkString()
{
    unsigned char* buffer = new unsigned char[ETHER_MAX_LEN];
    unsigned char* eth_head = buffer;
    unsigned char* eth_data = buffer + 14;
    uint32_t int_htonl;
    uint32_t buffer_offset = 0;

    /* Build Ethernet header*/
    eh_ = (struct ethhdr *) eth_head;
    memcpy((void *) eh_->h_dest, (void*) bcast_mac, ETH_ALEN);
    memcpy((void *) eh_->h_source, (void*) mac_source_, ETH_ALEN);
   
    uint16_t tf = htons(ETH_TYPE);
    memcpy(eth_head + 12, &tf, 2);
    buffer_offset += 14;

    /*PACKET TYPE*/
    uint8_t ff = FRAME_TYPE_BEACON;
    memcpy(eth_data, &ff, 1);
    eth_data++;
    buffer_offset += 1;
    /*HOST LENGTH*/
    int_htonl = htonl(hostname_.length());
    memcpy(eth_data, &int_htonl, sizeof (uint32_t));
    eth_data += sizeof (uint32_t);
    buffer_offset += sizeof (uint32_t);

    /*HOST*/
    memcpy(eth_data, hostname_.data(), hostname_.length());
    eth_data += hostname_.length();
    buffer_offset += hostname_.length();

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

int Beacon::GetCrc32(const std::string& my_string)
{
    boost::crc_32_type result;
    result.process_bytes(my_string.data(), my_string.length());
    return result.checksum();
}
