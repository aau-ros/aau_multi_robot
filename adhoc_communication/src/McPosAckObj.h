/* 
 * File:   McPosAckObj.h
 * Author: cwioro
 *
 * Created on August 7, 2014, 8:43 AM
 */

#ifndef MCPOSACKOBJ_H
#define	MCPOSACKOBJ_H

#include "AckRoutedFrame.h"


class McPosAckObj {
public:
    McPosAckObj(RoutedFrame* frame, McTree* group);
    McPosAckObj(const McPosAckObj& orig);

    virtual ~McPosAckObj();
    
    bool GotAck(McAckFrame* ack); //return true if all missing acks have been gotten
    std::list<unsigned char*> missing_acks_l;
    unsigned char* incoming_mac;
    
    std::string group_name;
    uint32_t sequence_num;
    uint32_t packet_id;
    
    
private:

};

#endif	/* MCPOSACKOBJ_H */

