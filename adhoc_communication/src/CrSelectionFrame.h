/* 
 * File:   CrSelectionFrame.h
 * Author: cwioro
 *
 * Created on August 25, 2014, 8:26 AM
 */

#ifndef CRSELECTIONFRAME_H
#define	CRSELECTIONFRAME_H

struct cr_selection_header {
    uint8_t frame_type;
    uint32_t id;
    unsigned char relay_connection[6];
    uint8_t relay_index;
  

} __attribute__((packed));

class CrSelectionFrame: public EthernetFrame {
public:
    CrSelectionFrame( unsigned char* dst,unsigned char* relay_conn, uint8_t relay_index );
    CrSelectionFrame(unsigned char* buffer);
    CrSelectionFrame(const CrSelectionFrame& orig);
    virtual ~CrSelectionFrame();
    
   std::string getFrameAsNetworkString(unsigned char* src);
   
   
    using EthernetFrame::GetCrc32;

    struct cr_selection_header header_;
    static uint32_t HEADER_FIXED_LEN;
    static uint32_t frame_count_stat;
    
    uint32_t buffer_str_len_;
private:

};
uint32_t CrSelectionFrame::frame_count_stat = 0;
uint32_t CrSelectionFrame::HEADER_FIXED_LEN = sizeof (eh_header) + sizeof (cr_selection_header); //22;

#endif	/* CRSELECTIONFRAME_H */

