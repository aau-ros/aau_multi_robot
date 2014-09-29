
/*
 * RoutedFrame.cpp
 *
 *  Created on: 29.07.2013
 *      Author: Günther Cwioro
 *
 *
 * Structure of a routed frame looks like:
 *
 * Destination MAC 	// ff:ff:ff:ff:ff:ff or "real" destination. Depends on CR			6Byte
 * SOURCE MAC		// MAC of the using interface										6Byte
 * ETH_TYPE_FIELD	// 0x4148 (AH as string)											2Byte
 * FRAME_TYPE		// 0x64 (d as string)												1Byte
 * DESTINATION		// destination mac													6Byte
 * FRAME ID			// id of frame														4Byte
 * ROUTE ID																				4Byte
 * PACKET ID																			4Byte
 * SEQUENCE NUMBER																		4Byte
 * PACKET SIZE																			4Byte
 * SOURCE HOST Length																	4Byte
 * SOURCE HOST																			VAR
 * MC GROUP LENGTH																		4Byte
 * MC GROUP NAME																		VAR
 * TOPIC_LENGTH																			4Byte
 * TOPIC			// topic that the payload will be published later					VAR
 * PAYLOAD TYPE     // this field is used to distinguish different payload types		1Byte
 * PAYLOAD_LENGTH																		4Byte
 * PAYLOAD			// payload															VAR
 * FLAG FIELD  // cr_flag, mc_flag, pos_ack, neg_ack
 * CRC CHECKSUM																			4Byte
 *
 *	How the frame is delivered:
 *
 *	NW:	(S) - (N1) - (D)
 *
 *	t(0)	(S)  data->   (N1) 	      (D)
 *	t(1)		 <-ack
 *	t(2)						data->
 *	t(3)						<-ack
 *	t(4)						<-ACK
 *	t(5)						ack->
 *	t(6)		 <-ACK
 *	t(7)		 ack->
 *
 *	To send a packet, of course its necessary that there is a route for the destination in the routing table.
 *
 *	1) Source send the frame to N1
 *	2) N1 receive the frame and sends a ack to the S. N1 looks in his routing table wich is the next host mac and send it to the D
 *	3) the D gets the frame and also sends an ack to N1.
 */

#include <string>

#include "RoutedFrame.h"
#include "EthernetFrame.h"

RoutedFrame::RoutedFrame(std::string topic_to_publish, std::string data,
                         uint8_t payload_type_field, uint32_t pack_id,
                         uint32_t pack_sequence_num, uint32_t frames_in_pack)
{

    this->payload_ = data;
    this->header_.payload_type = payload_type_field;
    this->topic_ = topic_to_publish;
    header_.frame_id = frame_count_stat;
    this->header_.packet_id = pack_id;
    header_.packet_sequence_num = pack_sequence_num;
    header_.packet_size = frames_in_pack;
    frame_count_stat++;
    cr_flag = false;
    mc_g_name_ = "";
    this->mc_flag = false;
    this->negative_ack_type = false;

    this->resend_flag = false;
    this->header_.frame_type = FRAME_TYPE_TRANSPORT_DATA;



}

RoutedFrame::RoutedFrame()
{

}

unsigned long RoutedFrame::getSize()
{
    unsigned long size = 0;
    size += this->payload_.size();
    size += RoutedFrame::HEADER_FIXED_LEN;
    return size;
}

RoutedFrame::RoutedFrame(unsigned char* buffer)
{
    unsigned char* buffer_start = buffer;
    buffer_str_len_ = 0;

    /*ETHERNET HEADER*/
    eh_header* eh = (struct eh_header *) buffer;
    buffer += sizeof (eh_header);
    buffer_str_len_ += sizeof (eh_header);

    /*FRAME HEADER*/
    rf_header* rfh = (struct rf_header *) buffer;
    buffer += sizeof (rf_header);
    buffer_str_len_ += sizeof (rf_header);

    /*SOURCE HOST*/
    hostname_source_ = "";
    hostname_source_.append((const char*) buffer, rfh->hostname_source_len);
    buffer += rfh->hostname_source_len;
    buffer_str_len_ += rfh->hostname_source_len;



    /*MC GROUP */
    mc_g_name_ = "";
    if (rfh->mc_group_len > 0)
    {
        mc_g_name_.append((const char*) buffer, rfh->mc_group_len);
        buffer += rfh->mc_group_len;
        buffer_str_len_ += rfh->mc_group_len;
    }


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
    cr_flag = (rfh->flag_field / 128) % 2 == 1;
    mc_flag = (rfh->flag_field / 64) % 2 == 1;
    negative_ack_type = (rfh->flag_field / 32) % 2 == 1;
    resend_flag = (rfh->flag_field / 16) % 2 == 1;

    /*CRC */
    uint32_t crc = 0;
    memcpy((unsigned char*) &crc, (unsigned char*) buffer, 4); // get CRC

    std::string crc_data_string = "";
    crc_data_string.append((const char*) buffer_start,
                           this->HEADER_FIXED_LEN + rfh->hostname_source_len + rfh->mc_group_len + rfh->topic_len + rfh->payload_len);
    correct_crc_ = (crc == (uint32_t) GetCrc32(crc_data_string));



    /* COPY HEADER FIELDS */
    memcpy(&this->eh_h_, &(*eh), sizeof (eh_header));
    memcpy(&this->header_, &(*rfh), sizeof (rf_header));

}

RoutedFrame::~RoutedFrame()
{
    // TODO Auto-generated destructor stub
}

std::string RoutedFrame::getFrameAsNetworkString(uint32_t route_id, unsigned char next_hop[6], string source_host, unsigned char source[6])
{
    memcpy(this->eh_h_.eh_source, source, ETH_ALEN);
    memcpy(&this->header_.mac_destination_, next_hop, ETH_ALEN);


    this->hostname_source_ = source_host;

    if (this->enable_cooperative_relaying)
        memcpy((void *) eh_h_.eh_dest, (void*) bcast_mac, ETH_ALEN);
    else
        memcpy((void *) eh_h_.eh_dest, (void*) next_hop, ETH_ALEN);
    this->header_.route_id = route_id;


    /*FLAG FIELD*/
    this->header_.flag_field = 0;
    if (cr_flag)
        this->header_.flag_field += 128;
    if (mc_flag)
        this->header_.flag_field += 64;
    if (negative_ack_type)
        this->header_.flag_field += 32;
    if (resend_flag)
        this->header_.flag_field += 16;

    /*LEN FIELDS */
    this->header_.hostname_source_len = hostname_source_.length();
    this->header_.mc_group_len = mc_g_name_.length();
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
    memcpy(buffer, &this->header_, sizeof (rf_header));
    buffer += sizeof (rf_header);

    /*SOURCE HOST */
    memcpy(buffer, this->hostname_source_.data(),
           this->hostname_source_.length());
    buffer += this->hostname_source_.length();

    /*MC GROUP */
    memcpy(buffer, this->mc_g_name_.data(), this->mc_g_name_.length());
    buffer += mc_g_name_.length();

    /*TOPIC*/
    memcpy(buffer, this->topic_.data(), this->topic_.length());
    buffer += topic_.length();


    /*PAYLOAD */
    memcpy(buffer, this->payload_.data(), this->payload_.length());
    buffer += payload_.length();


    /*CRC*/
    uint32_t dynamic_field_len = this->hostname_source_.length() + this->mc_g_name_.length() + topic_.length() + payload_.length();
    std::string crc_string = std::string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len);
    uint32_t crc = GetCrc32(crc_string);
    memcpy(buffer, &crc, sizeof (uint32_t));
    buffer += sizeof (uint32_t);


    return string((const char*) buffer_start, this->HEADER_FIXED_LEN + dynamic_field_len + sizeof (uint32_t));

}

std::string RoutedFrame::getFrameAsNetworkString(routing_entry r, unsigned char src[6])
{
    return this->getFrameAsNetworkString(r.id, r.next_hop, r.hostname_source, src);
}

stc_frame RoutedFrame::getFrameStruct()
{
    stc_frame stc;
    stc.type = header_.frame_type;
    stc.frame_id = header_.frame_id;
    memcpy(stc.mac, header_.mac_destination_, ETH_ALEN);
    stc.mc_group = mc_g_name_;
    stc.hostname_source = hostname_source_;

    stc.retransmitted = 0;

    return stc;
}


