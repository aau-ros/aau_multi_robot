/*!
 * @file RoutedFrame.cpp
 * Network layer packet definition.
 *
 * \date 29.07.2013
 * \author Günther Cwioro, Torsten Andre
 */

#ifndef ROUTEDFRAME_H_
#define ROUTEDFRAME_H_

struct rf_header
{
	uint8_t frame_type;
	unsigned char mac_destination_[6];
	uint32_t frame_id;
	uint32_t route_id;
	uint32_t packet_id;
	uint32_t packet_sequence_num;
	uint32_t packet_size;
	uint8_t flag_field;
	uint8_t payload_type;
	uint32_t hostname_source_len;
	uint32_t mc_group_len;
	uint32_t topic_len;
	uint32_t payload_len;

}__attribute__((packed));

/*!
\class RoutedFrame
\brief The class of a routed frame

* The class contains all needed frame fields and the functions to serialize a frame or desizalied it from a buffer
*
* HEADER OF A ROUTED FRAME:
*  HEADER FIELDS:
*    - DESTINATION MAC
*         - SIZE: 6 Byte
*    - SOURCE MAC
*         - SIZE: 6 Byte
*    - ETH_TYPE_FIELD
*         - SIZE: 2 Byte
*    - FRAME_TYPE
*         - SIZE: 1 Byte
*    - DESTINATION
*         - SIZE: 6 Byte
*    - FRAME ID
*         - SIZE: 4 Byte
*    - ROUTE ID
*         - SIZE: 4 Byte
*    - PACKET ID
*         - SIZE: 4 Byte
*    - SEQUENCE NUMBER
*         - SIZE: 4 Byte
*    - PACKET SIZE
*         - SIZE: 4 Byte
*    - SOURCE HOST Length
*         - SIZE: 4 Byte
*    - SOURCE HOST
*         - SIZE: VARIABLE
*    - MC GROUP LENGTH
*         - SIZE: 4 Byte
*    - MC GROUP NAME
*         - SIZE: VARIABLE
*    - TOPIC_LENGTH
*         - SIZE: 4  Byte
*    - TOPIC
*         - SIZE: VARIABLE
*    - PAYLOAD TYPE
*         - SIZE: 1 Byte
*    - PAYLOAD_LENGTH
*         - SIZE: 4 Byte
*    - PAYLOAD
*         - SIZE: VARIABLE
*    - FLAG FIELD
*         - SIZE: 1 Byte
*    - CRC32 CHECKSUM
*         - SIZE: 4 Byte

*/
class RoutedFrame : public EthernetFrame {
public:

	RoutedFrame(std::string topic_to_publish, std::string data,uint8_t payload_type_field,uint32_t pack_id,uint32_t pack_sequence_num,uint32_t frames_in_pack);
	RoutedFrame(unsigned char* buffer);
	RoutedFrame();
	virtual ~RoutedFrame();
        unsigned long getSize();

	struct rf_header header_;
	std::string hostname_source_;
	std::string mc_g_name_;
	std::string topic_;
	std::string payload_;


//	bool correct_crc_;
	bool cr_flag;
	bool mc_flag;
	bool negative_ack_type;
	bool resend_flag;
        
        uint16_t buffer_str_len_;


	static uint32_t frame_count_stat;
	static uint32_t HEADER_FIXED_LEN;
	static bool enable_cooperative_relaying;

	using EthernetFrame::GetCrc32;
	stc_frame getFrameStruct();
	std::string getFrameAsNetworkString(routing_entry rou, unsigned char source[6]);
	std::string getFrameAsNetworkString(uint32_t route_id, unsigned char next_hop[6] ,string source_host, unsigned char source[6]);

};

uint32_t RoutedFrame::frame_count_stat = 0;
uint32_t  RoutedFrame::HEADER_FIXED_LEN = sizeof(eh_header) + sizeof(rf_header); //63
bool RoutedFrame::enable_cooperative_relaying = false;



#endif /* ROUTEDFRAME_H_ */
