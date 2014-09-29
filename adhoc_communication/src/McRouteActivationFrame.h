/* 
 * File:   McRouteActivationFrame.h
 * Author: cwioro
 *
 * Created on August 27, 2014, 12:20 PM
 */

#ifndef MCROUTEACTIVATIONFRAME_H
#define	MCROUTEACTIVATIONFRAME_H

#include "EthernetFrame.h"

struct mc_act_header {
    uint8_t frame_type;
    uint32_t id;
    uint32_t route_id;
    unsigned char mac_destination[6];
    uint32_t mc_group_name_len;
    uint32_t hostname_source_len;


} __attribute__((packed));

class McRouteActivationFrame : public EthernetFrame {
public:

    /** Constructs the McMaintenanceFrame on the sender side.
            \param route The route the frame will take
            \param activate \todo Eventually not final const.
            \param disconnect \todo Eventually not final konnst
            \param disconnect \todo Eventually not final konnst
     */
    McRouteActivationFrame(unsigned char* next_hop, std::string mc_g, uint32_t route_id, std::string source);

    /** Constructs the McMaintenanceFrame on the receiver side.
    \param buffer The received network buffer
     */
    McRouteActivationFrame(unsigned char* buffer);


    using EthernetFrame::GetCrc32;

    /** Returns a McMaintenanceFrame as a C++ string.
    Converts the MaintenenceFrame into a C++ string to be written to
    the socket.
    \return McMaintenanceFrame as C++ string.
     */
    std::string getFrameAsNetworkString(unsigned char source[6]);

    virtual ~McRouteActivationFrame();

    struct mc_act_header header_; ///< Struct which stores all fixed length header information of the McMaintenanceFrame

    std::string mc_group_; ///< Name of the multicast group
    std::string hostname_source_; ///< Name of the multicast group

    uint16_t buffer_str_len_;

   // bool correct_crc_; ///< Indicates if the checksum of the frame is correct

    static uint32_t HEADER_FIXED_LEN; ///< The length of the beacon header (34 bytes)
    static uint32_t stat_id_count;


};
uint32_t McRouteActivationFrame::HEADER_FIXED_LEN = sizeof (eh_header) + sizeof (mc_act_header);
uint32_t McRouteActivationFrame::stat_id_count = 0;
#endif	/* MCROUTEACTIVATIONFRAME_H */

