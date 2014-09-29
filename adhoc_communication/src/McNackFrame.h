/* 
 * File:   McNackFrame.h
 * Author: cwioro
 *
 * Created on September 10, 2014, 11:37 AM
 */

#ifndef MCNACKFRAME_H
#define	MCNACKFRAME_H

struct mc_nack_header {
    uint8_t frame_type;
    uint32_t id;
    uint32_t packet_id;
    uint16_t frame_seq_num_size;
    uint32_t hostname_source_len;
    uint32_t mc_group_len;

} __attribute__((packed));

class McNackFrame : public EthernetFrame {
public:

    McNackFrame(unsigned char* source, unsigned char* dest, std::string hostname_src, std::string group_n, uint32_t p_id, std::vector<uint32_t> seq_n);
    McNackFrame(unsigned char* buffer);
    virtual ~McNackFrame();


    struct mc_nack_header header_;
    std::string hostname_source_;
    std::string mc_group_;
    std::vector<uint32_t> req_seq_nums_;



    uint16_t buffer_str_len_;

    static uint32_t HEADER_FIXED_LEN;
    static uint32_t stat_id_count;
    bool correct_crc_;

    using EthernetFrame::GetCrc32;
    std::string getFrameAsNetworkString();

    void print_frame();

private:

};
uint32_t McNackFrame::HEADER_FIXED_LEN = sizeof (eh_header) + sizeof (mc_nack_header);
uint32_t McNackFrame::stat_id_count = 0;

#endif	/* MCNACKFRAME_H */

