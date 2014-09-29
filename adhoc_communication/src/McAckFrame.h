/* 
 * File:   McAckFrame.h
 * Author: cwioro
 *
 * Created on August 27, 2014, 10:26 AM
 */

#ifndef MCACKFRAME_H
#define	MCACKFRAME_H

struct mc_ack_header {
    uint8_t frame_type;
    unsigned char mac_destination_[6];
    uint32_t packet_id;
    uint32_t frame_seq_num;
    uint8_t flag_field;
    uint32_t hostname_source_len;
    uint32_t mc_group_len;

} __attribute__((packed));

class McAckFrame : public EthernetFrame {
public:

 
    McAckFrame(unsigned char* source, unsigned char* dest, std::string hostname_src, std::string group_n, uint32_t p_id, uint32_t seq_n);
    McAckFrame(unsigned char* buffer);
    virtual ~McAckFrame();


    struct mc_ack_header header_;
    std::string hostname_source_;
    std::string mc_group_;


    bool cr_flag_;

    uint16_t buffer_str_len_;

    static uint32_t HEADER_FIXED_LEN;


    using EthernetFrame::GetCrc32;
    std::string getFrameAsNetworkString();
   
    void print_frame();



private:

};

uint32_t McAckFrame::HEADER_FIXED_LEN = sizeof (eh_header) + sizeof (mc_ack_header);

#endif	/* MCACKFRAME_H */

