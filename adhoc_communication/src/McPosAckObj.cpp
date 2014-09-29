/* 
 * File:   McPosAckObj.cpp
 * Author: cwioro
 * 
 * Created on August 7, 2014, 8:43 AM
 */

#include <list>

#include "McPosAckObj.h"
#include "McTree.h"
#include "structs.h"
#include "EthernetFrame.h"
#include "functions.h"
#include "header.h"

McPosAckObj::McPosAckObj(RoutedFrame* frame, McTree* group)
{

    group_name = std::string(group->group_name_);

    packet_id = frame->header_.packet_id;
    sequence_num = frame->header_.packet_sequence_num;

    //memcpy(incoming_mac,frame->eh_h_.eh_source,6);



    if (!compareMac(group->route_uplink_->next_hop, frame->eh_h_.eh_source) && !group->root)
    {
        unsigned char* mac_p = new unsigned char[6];
        memcpy(mac_p, group->route_uplink_->next_hop, 6);
        missing_acks_l.push_back(mac_p);
        //ROS_ERROR("ADD Uplink %s", getMacAsCStr(mac_p));
    }

    for (std::list<mac*>::iterator i = group->downlinks_l_.begin(); i != group->downlinks_l_.end(); i++)
    {
        mac* m = *i;


        if (!compareMac(frame->eh_h_.eh_source, m->mac_adr))
        {
            unsigned char* mac_p = new unsigned char[6];
            memcpy(mac_p, m->mac_adr, 6);
            missing_acks_l.push_back(mac_p);
            //ROS_ERROR("ADD Downlink %s", getMacAsCStr(mac_p));
        }
    }
}

McPosAckObj::McPosAckObj(const McPosAckObj& orig)
{
}

McPosAckObj::~McPosAckObj()
{
}

bool McPosAckObj::GotAck(McAckFrame* ack)
{
    if (ack->mc_group_.compare(group_name) == 0 && ack->header_.packet_id == packet_id && ack->header_.frame_seq_num == sequence_num)
    {
        for (std::list<unsigned char*>::iterator i = missing_acks_l.begin(); i != missing_acks_l.end();)
        {
            if (compareMac(*i, ack->eh_h_.eh_source))
            {
                delete (*i);
                i = missing_acks_l.erase(i);
                return missing_acks_l.empty();
            }
            else
                i++;
        }
    }
    return false;
}
