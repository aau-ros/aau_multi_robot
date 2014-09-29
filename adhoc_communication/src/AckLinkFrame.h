/*
 * AckLinkFrame.h
 *
 *  Created on: 29.07.2013
 *      Author: Günther Cwioro
 * 
 * 
 * Structure from acknowledgment of a link frame looks like:
 *
 * BROADCAST MAC 	// -> ff:ff:ff:ff:ff:ff  						6Byte
 * SOURCE MAC		// MAC of the using interface						6Byte
 * ETH TYPE FIELD	// 0x4148 (AH as string)						2Byte
 * FRAME TYPE		// 0x61	  (a as string)							1Byte
 * DESTINATION MAC	// the source mac of the frame to acknowledge	| or bcast mac	 	6Byte
 * FRAME ID			// ID of the frame to acknowledge | or sequence num		4Byte
 * PACKET ID 		//only for neg ack (otherwise highest possible value of the type	4Byte
 * FLAG FIELD           /                                                                       1Byte
 * SOURCE HOST_LENGTH                                                                           4Byte
 * MC GROUP LENGTH										4Byte
 * SOURCE HOST											VAR		
 * MC GROUP 											VAR
 * CRC 												4Byte
 *
 * FLAG FIELD:  0       0             00 0000
 *              128     64              
 *              MC_FLAG POS_ACK_FLAG  UNDIFINED
 * 
 * This frame-type is to acknowledge incoming frames. Every time a i get a frame i should immediately send an acknowledgment, except route requests, route responses and broadcasts.
 *
*/
#ifndef AckLinkFrame_H_
#define AckLinkFrame_H_


struct ack_lf_header
{
	uint8_t frame_type;
	unsigned char mac_destination_[6];
        unsigned char mac_confirmer[6];
        uint8_t ack_frame_type;
        uint32_t frame_id;
	uint8_t  flag_field;
	uint32_t hostname_source_len;


}__attribute__((packed));


class AckLinkFrame  : public EthernetFrame {
public:
	AckLinkFrame(unsigned char* source,unsigned char* confirmer_mac,unsigned char* dest,uint32_t frame_id,std::string hostname,uint8_t type);
	
	AckLinkFrame(unsigned char* buffer);
	virtual ~AckLinkFrame();
	
        
        struct ack_lf_header header_;
        std::string hostname_source_;        
       
        bool pos_ack_flag_;
        bool cr_flag_;
        
        uint16_t buffer_str_len_;

	static uint32_t HEADER_FIXED_LEN;
        
        
        using EthernetFrame::GetCrc32;
	std::string getFrameAsNetworkString();
       
        void print_frame();

};

uint32_t AckLinkFrame::HEADER_FIXED_LEN = sizeof(eh_header) + sizeof(ack_lf_header); //41;

#endif /* AckLinkFrame_H_ */
