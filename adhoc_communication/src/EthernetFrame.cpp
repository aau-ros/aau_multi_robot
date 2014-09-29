/*
 * EthernetFrame.cpp
 *
 *  Created on: Mar 28, 2014
 *      Author: cwioro
 */

#include "EthernetFrame.h"
#include "defines.h"

EthernetFrame::EthernetFrame() {
	// TODO Auto-generated constructor stub
	this->eh_h_.eh_protocol_id = htons(ETH_TYPE);
        memcpy(this->eh_h_.eh_dest, bcast_mac,ETH_ALEN);

}

EthernetFrame::~EthernetFrame() {
	// TODO Auto-generated destructor stub
}

int EthernetFrame::GetCrc32(const std::string& s) {
    boost::crc_32_type result;
    result.process_bytes(s.data(), s.length());
    return result.checksum();
}

