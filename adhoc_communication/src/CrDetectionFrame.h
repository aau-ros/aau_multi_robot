/* 
 * File:   CrDetectionFrame.h
 * Author: cwioro
 *
 * Created on August 25, 2014, 8:29 AM
 */

#ifndef CRDETECTIONFRAME_H
#define	CRDETECTIONFRAME_H

#include "EthernetFrame.h"

struct cr_detec_header {
    uint8_t frame_type;
    unsigned char mac1[6];
    unsigned char mac2[6];

} __attribute__((packed));

class CrDetectionFrame : public EthernetFrame {
public:
    CrDetectionFrame(unsigned char* first_mac, unsigned char* second_mac);
    CrDetectionFrame(unsigned char* buffer);
    CrDetectionFrame(const CrDetectionFrame& orig);
    virtual ~CrDetectionFrame();
    std::string getFrameAsNetworkString(unsigned char* src);

    using EthernetFrame::GetCrc32;

    struct cr_detec_header header_;
    static uint32_t HEADER_FIXED_LEN;
    
    uint32_t buffer_str_len_;
private:

};

uint32_t CrDetectionFrame::HEADER_FIXED_LEN = sizeof (eh_header) + sizeof (cr_detec_header); //27;

#endif	/* CRDETECTIONFRAME_H */

