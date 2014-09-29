/* 
 * File:   MultiHopBroadcastFrame.h
 * Author: cwioro
 *
 * Created on September 17, 2014, 12:00 PM
 */

#ifndef MULTIHOPBROADCASTFRAME_H
#define	MULTIHOPBROADCASTFRAME_H

struct mh_bcast_header {
    uint8_t frame_type;
    uint32_t id;
    uint8_t payload_type;
    uint16_t current_hop;
    uint16_t hop_limit;
    uint32_t hostname_source_len;
    uint32_t topic_len;
    uint32_t payload_len;

} __attribute__((packed));

class MultiHopBroadcastFrame : public EthernetFrame {
public:
    MultiHopBroadcastFrame(std::string topic_to_publish, std::string data,  string source_host, uint8_t payload_type_field, uint16_t hop_range);
    MultiHopBroadcastFrame(unsigned char* buffer);
    virtual ~MultiHopBroadcastFrame();

    std::string getFrameAsNetworkString(unsigned char* source);
    
    bool rebroadcast;

    using EthernetFrame::GetCrc32;

    static uint32_t frame_count_stat;
	static uint32_t HEADER_FIXED_LEN;

    struct mh_bcast_header header_;
    uint16_t buffer_str_len_;
    
    std::string hostname_source_;
    std::string topic_;
    std::string payload_;
private:

};

uint32_t MultiHopBroadcastFrame::frame_count_stat = 0;
uint32_t MultiHopBroadcastFrame::HEADER_FIXED_LEN = sizeof (eh_header) + sizeof (mh_bcast_header); 

#endif	/* MULTIHOPBROADCASTFRAME_H */

