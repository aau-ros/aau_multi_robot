/*
 * AckRoutedFrame.h
 *
 *  Created on: 29.07.2013
 *      Author: cwioro
 * 
 * Structure from acknowledgment of a routed frame looks like:
 *
 * BROADCAST MAC 	// -> ff:ff:ff:ff:ff:ff  				6Byte
 * SOURCE MAC		// MAC of the using interface				6Byte
 * ETH TYPE FIELD	// 0x4148 (AH as string) 				2Byte
 * FRAME TYPE		// 0x41   (A as string)					1Byte
 * DESTINATION MAC	// next hop to forward the frame to the root source	6Byte
 * FRAME ID	        // ID of the frame to acknowledge or packet id (if mc)	4Byte
 * ROUTE ID	        // ID of the Route or packet sequence (if mc)           4Byte
 * FLAG FIELD									1Byte
 * SOURCE HOST LENGTH								4Byte
 * SOURCE HOST									VAR
 * CRC										4Byte
 *
 * This frame type is used to acknowledge end-to-end routed frames
 */

#ifndef ACKROUTEDFRAME_H_
#define ACKROUTEDFRAME_H_

struct ack_rf_header {
    uint8_t frame_type;
    unsigned char mac_destination_[6];
    uint32_t frame_id;
    uint32_t route_id;
    uint8_t flag_field;
    uint32_t hostname_source_len;
    uint32_t mc_group_len;

} __attribute__((packed));

class AckRoutedFrame : public EthernetFrame {
public:
    AckRoutedFrame(RoutedFrame rf);
    AckRoutedFrame(unsigned char* buffer);
    virtual ~AckRoutedFrame();

    struct ack_rf_header header_;
    std::string hostname_source_;
    std::string mc_group_;

    uint32_t buffer_str_len_;

    bool cr_flag;
    bool mc_flag;


    static uint32_t HEADER_FIXED_LEN;
    stc_frame getFrameStruct();
    std::string getFrameAsNetworkString(uint32_t route_id, unsigned char next_hop[6], string source_host, unsigned char source[6]);
    std::string getFrameAsNetworkString(routing_entry r, unsigned char src[6]);

    using EthernetFrame::GetCrc32;
};


uint32_t AckRoutedFrame::HEADER_FIXED_LEN = sizeof (eh_header) + sizeof (ack_rf_header); //38;

#endif /* ACKROUTEDFRAME_H_ */
