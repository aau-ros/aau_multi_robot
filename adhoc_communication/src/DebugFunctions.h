/* 
 * File:   DebugFunctions.h
 * Author: Guenther Cwioro
 *
 * Created on May 21, 2014, 10:04 AM
 */


#ifndef DEBUGFUNCTIONS_H
#define	DEBUGFUNCTIONS_H

#ifdef DEBUG


void printRouteRequests()
{
    for (std::list<route_request>::iterator it=route_requests_l.begin(); it != route_requests_l.end(); ++it)
    {
        route_request &req(*it);
        ROS_DEBUG("REQUEST ID[%u] SOUCRE[%s]",req.id,req.hostname_source.c_str());
    }
}

void printMcTree(mc_tree* m)
{
   
    ROS_ERROR("MC TREE GROUP[%s] ", m->group_name.c_str());
    ROS_ERROR("UPLINK: HOSTNAME[%s] MAC[%s]",getHostnameFromMac(m->route_uplink.next_hop).c_str(),getMacAsStr(m->route_uplink.next_hop).c_str());
    ROS_ERROR("ACTIVE[%u] CONNECTED[%u] MEMBER[%u] ROOT[%u] DISTANCE TO ROOT[%u]",m->active,m->connected,m->member,m->root,m->root_distance);
    ROS_ERROR("DOWNLINKS:");
    for(std::list<mac>::iterator i = m->mc_downlinks.begin(); i != m->mc_downlinks.end(); i++)
        ROS_ERROR("DOWNLINK %s",getMacAsStr((*i).mac_adr).c_str());
    
  
        
}

void printMcConnections(std::list<mc_tree> *tree)
{
    ROS_ERROR("ALL MC CONNECTIONS OF ROBOT %s:",hostname.c_str());
    std::list<mc_tree>::iterator i = tree->begin();
    for(;i != tree->end(); i++)
    {
        ROS_ERROR(" ");
        printMcTree(&(*i));
    }
    
   
}
void printRouteRequest(RouteRequest* r)
{

    ROS_ERROR("ROUTE REQUEST: ID[%u] SOURCE[%s] DEST[%s]",r->header_.id,r->hostname_source_.c_str(),r->hostname_destination_.c_str());
    ROS_ERROR("H COUN[%u] H LIMIT[%u] MC FLAG[%u]",r->header_.hop_count,r->header_.hop_limit,r->mc_flag_);
    ROS_ERROR("PATH[%s]",getPathAsStr(r->path_l_).c_str());

}
void printRouteResponse(RouteResponse* r)
{
    ROS_ERROR(" ");
    ROS_ERROR("ROUTE REQUEST: ID[%u] SOURCE[%s]",r->request_id_,r->hostname_source_.c_str());
    ROS_ERROR("ROOT DISTANCE[%u] HOP COUN[%u]  MC FLAG[%u]",r->root_distance,r->hop_count_,r->mc_flag_);
    ROS_ERROR("CURRENT HOP[%s]",getMacAsStr(r->mac_current_hop_).c_str());
    ROS_ERROR("PATH[%s]",getPathAsStr(r->path_l_).c_str());
    ROS_ERROR(" ");
}

#endif

#endif	/* DEBUGFUNCTIONS_H */

