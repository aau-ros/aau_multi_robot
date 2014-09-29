/* 
 * File:   McDisconnectFrame.h
 * Author: cwioro
 *
 * Created on September 1, 2014, 1:18 PM
 */

#ifndef MCDISCONNECTFRAME_H
#define	MCDISCONNECTFRAME_H

struct mc_disc_header {
    uint8_t frame_type;
    uint32_t id;
    unsigned char mac_destination[6];
    uint8_t flag_field;
    uint32_t mc_group_name_len;

} __attribute__((packed));

class McDisconnectFrame : public EthernetFrame {
public:

    McDisconnectFrame(unsigned char* next_hop, std::string mc_g);
    McDisconnectFrame(unsigned char* buffer);
    std::string getFrameAsNetworkString(unsigned char* source);
    using EthernetFrame::GetCrc32;
    virtual ~McDisconnectFrame();

    struct mc_disc_header header_; ///< Struct which stores all fixed length header information of the McMaintenanceFrame

    std::string mc_group_; ///< Name of the multicast group
    bool disconnect_uplink;
    bool disconnect_downlink;

    uint16_t buffer_str_len_;


    static uint32_t HEADER_FIXED_LEN; ///< The length of the beacon header (34 bytes)
    static uint32_t stat_id_count;

};

uint32_t McDisconnectFrame::HEADER_FIXED_LEN = sizeof (eh_header) + sizeof (mc_disc_header);
uint32_t McDisconnectFrame::stat_id_count = 0;

#endif	/* MCDISCONNECTFRAME_H */

