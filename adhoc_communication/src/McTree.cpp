/* 
 * File:   McTree.cpp
 * Author: cwioro
 * 
 * Created on May 21, 2014, 11:54 AM
 */

#include <list>
#include <string>

#include "McTree.h"
#include "structs.h"
#include "functions.h"
#include "RouteRequest.h"
#include "EthernetFrame.h"
#include "header.h"

#include "Logging.h"

McTree::McTree()
{
    outgoing_request_ = NULL;
    route_uplink_ = NULL;
    time_stamp_ = getMillisecondsTime();
}

McTree::McTree(const McTree& orig)
{
}

McTree::~McTree()
{

}

void McTree::safeOutgoingRequest(RouteRequest* req)
{
    if (outgoing_request_ != NULL)
        delete outgoing_request_;

    time_stamp_ = getMillisecondsTime();

    outgoing_request_ = req;
}

bool McTree::addWaitingRequest(RouteRequest* req, unsigned char* source_mac)
{

    for (list<RouteRequest* >::iterator it = waiting_requests_l_.begin(); it != waiting_requests_l_.end(); it++)
    {
        RouteRequest* r = *it;
        /* Check if same: source, host, incoming mac*/
        if (r->hostname_source_.compare(req->hostname_source_) == 0 && r->hostname_destination_.compare(req->hostname_destination_) == 0 && compareMac(r->eh_h_.eh_source, req->eh_h_.eh_source))
        {
            /* if hop count is less than the stored one, the old one will be deleted and the new one will be stored */
            if (r->header_.hop_count > req->header_.hop_count)
            {
                waiting_requests_l_.erase(it);
                delete r;
                waiting_requests_l_.push_back(req);
                return true;
            }
            else
                return false;
        }
    }

    /* check if robots source mac is not in mac path to prevent loops */
    for (list<mac>::iterator it = req->path_l_.begin(); it != req->path_l_.end(); it++)
    {
        mac m = *it;
        if (compareMac(m.mac_adr, source_mac))
            return false;
    }

    waiting_requests_l_.push_back(req);
    return true;
}

bool McTree::propagateFrame(unsigned char* sender_mac)
{
    //  ROS_DEBUG("UPLINK: %s",getHostnameFromMac( t->route_uplink.next_hop).c_str());

    /* propagate if frame is from uplink and downlinks exsists */
    if (!this->downlinks_l_.empty() && compareMac(sender_mac, route_uplink_->next_hop))
    {
        //  ROS_ERROR("propagate if frame is from uplink and downlinks exsists");
        return true;
    }





    /* Propagate frame if frame is from a downlink*/


    for (std::list<mac*>::iterator it = downlinks_l_.begin(); it != downlinks_l_.end(); ++it)
    {
        mac* m = *it;
        //            ROS_DEBUG("DOWNLINK: %s" ,getHostnameFromMac( m.mac).c_str());
        if (compareMac((unsigned char*) m->mac_adr, sender_mac))
        {
            if (!root) // propagate always if i am not the root, because there is an uplink
                return true;
            else if (root && downlinks_l_.size() > 1) // if root propagate only if there is more than one downlinks
                return true;
        }

    }


    return false;
}

bool McTree::activateBestRoute(route_request* rreq_logging)
{
    for (list<routing_entry* >::iterator it = routing_entries_l_.begin(); it != routing_entries_l_.end();)
    {


        if (route_uplink_->root_distance > (*it)->root_distance)
        {
            route_uplink_ = *it;
            it = routing_entries_l_.erase(it);

        }
        else
            it++;
    }


    Logging::logRRequestInitiater(rreq_logging, route_uplink_);


    resetTmpFields();

    if (route_uplink_->id != (0 - 1))
    { //this->route_uplink_
        this->activated = true;
        this->connected = true;

        return true;
    }
    else
        return false;


}

void McTree::resetTmpFields()
{

    for (list<routing_entry* >::iterator it = routing_entries_l_.begin(); it != routing_entries_l_.end();)
    {
        routing_entry* t = *it;
        it = routing_entries_l_.erase(it);
        delete t;

    }
    delete outgoing_request_;
    outgoing_request_ = NULL;

    /* DO not erase waitingRequests list -> is needed to answer those requests */
}

bool McTree::processFrame(unsigned char* src)
{


    return downlinkExsists(src) || compareMac(src, route_uplink_->next_hop);
}

void McTree::printTree()
{
    ROS_ERROR("GROUP NAME: %s", this->group_name_.c_str());
    std::string route_uplink = "";
    if (!this->root)
        route_uplink = std::string("Next hop:" + getMacAsStr(this->route_uplink_->next_hop) + " RD:" + getIntAsString(this->route_uplink_->root_distance) + " HOBS:" + getIntAsString(this->route_uplink_->hobs) + " CH:" + getIntAsString(this->route_uplink_-> current_hop));

    else
        ROS_ERROR("UPLINK: %s", route_uplink.c_str());

    if (!this->downlinks_l_.empty())
    {
        ROS_ERROR("DOWNLINKS:");
        int count = 0;
        for (std::list<mac*>::iterator i = downlinks_l_.begin(); i != downlinks_l_.end(); ++i)
        {
            count++;

            ROS_ERROR("%s:: %s", getIntAsString(count).c_str(), getMacAsStr((*i)->mac_adr).c_str());

        }
    }
    else ROS_ERROR("NO DOWNLINKS");
    if (!this->routing_entries_downlinks_l_.empty())
    {
        ROS_ERROR("POSSIBLE DOWNLINKS:");
        int count = 0;
        for (std::list<routing_entry*>::iterator i = routing_entries_downlinks_l_.begin(); i != routing_entries_downlinks_l_.end(); ++i)
        {

            count++;

            ROS_ERROR("%s:: %s", getIntAsString(count).c_str(), getMacAsStr((*i)->previous_hop).c_str());

        }
    }


    ROS_ERROR("CONNECTED: [%s] MEMBER[%s] ROOT[%s] ACTIVATED[%s] ", getBoolAsString(connected).c_str(), getBoolAsString(member).c_str(), getBoolAsString(root).c_str(), getBoolAsString(activated).c_str());

}

bool McTree::activateRoute(std::string* hostname_source, uint32_t* id, unsigned char* mac_adr)
{


    bool route_found = false;
    for (list<routing_entry* >::iterator it = routing_entries_l_.begin(); it != routing_entries_l_.end();)
    {

        if (*id == (*it)->id && hostname_source->compare((*it)->hostname_source) == 0 && compareMac((*it)->previous_hop, mac_adr))
        {
            route_uplink_ = *it;
            route_found = true;

            this->activated = true;
            this->connected = true;
            it = routing_entries_l_.erase(it);
        }
        else
            it++;
    }
    if (route_found)
        resetTmpFields();

    return route_found;

}

bool McTree::routeIsNew(routing_entry* r)
{

    for (list<routing_entry* >::iterator it = routing_entries_l_.begin(); it != routing_entries_l_.end(); it++)
    {

        if (r->hostname_source.compare((*it)->hostname_source) == 0 && (*it)->id == r->id && compareMac(r->next_hop, (*it)->next_hop))
            return false;

    }

    return true;
}

bool McTree::addDownlinkAsConnector(mac* m)
{
    for (list<routing_entry* >::iterator it = this->routing_entries_l_.begin(); it != routing_entries_l_.end(); it++)
    {
        if (compareMac((*it)->previous_hop, m->mac_adr) && !downlinkExsists(m->mac_adr))
        {
          //  ROS_ERROR("add downlink as connector:   %s %s", getMacAsStr(m->mac_adr).c_str(), group_name_.c_str());
            this->downlinks_l_.push_back(m);
            return true;
        }
    }
    return false;
}

bool McTree::addDownlinkAsMember(mac* m)
{
    routing_entry* entry = NULL;
    for (list<routing_entry* >::iterator it = routing_entries_downlinks_l_.begin(); it != routing_entries_downlinks_l_.end(); it++)
    {

        if (compareMac((*it)->previous_hop, m->mac_adr) && !downlinkExsists(m->mac_adr))
        {
            entry = *it;
           // ROS_ERROR("add downlink as member  %s %s", getMacAsStr(m->mac_adr).c_str(), group_name_.c_str());
            this->downlinks_l_.push_back(m);
            routing_entries_downlinks_l_.erase(it);

            //Logging::logRouteRequestReceiver(entry->hostname_source,entry->id,entry->); 
            break;


        }
      //  else
      //      ROS_ERROR("%u %u", compareMac((*it)->previous_hop, m->mac_adr), !downlinkExsists(m->mac_adr));

    }


    /* remove all other entries frome the same source*/
    if (entry != NULL)
    {
        for (list<routing_entry* >::iterator it = routing_entries_downlinks_l_.begin(); it != routing_entries_downlinks_l_.end();)
        {
            if ((*it)->hostname_source.compare(entry->hostname_source) == 0)
            {
                delete *it;
                it = routing_entries_downlinks_l_.erase(it);
            }
            else
                it++;
        }
        delete entry;
        return true;
    }
    else
        return false;

}

bool McTree::operator=(const McTree* other)
{
    return group_name_.compare(other->group_name_) == 0;
}

bool McTree::downlinkExsists(unsigned char* m)
{

    for (list<mac* >::iterator it = downlinks_l_.begin(); it != downlinks_l_.end(); it++)
    {

        if (compareMac((*it)->mac_adr, m))
        {

            return true;
        }
    }

    return false;
}

bool McTree::removeMacIfExsists(unsigned char* m)
{
    for (list<mac* >::iterator it = downlinks_l_.begin(); it != downlinks_l_.end(); it++)
    {
        if (compareMac((*it)->mac_adr, m))
        {
            delete *it;
            downlinks_l_.erase(it);
            return true;
        }
    }
    return false;
}