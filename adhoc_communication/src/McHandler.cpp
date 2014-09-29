/* 
 * File:   McHandler.cpp
 * Author: cwioro
 * 
 * Created on May 21, 2014, 12:11 PM
 */



#include <string>
#include <list>

#include "McHandler.h"
#include "defines.h"

McHandler::McHandler(list<McTree*> *groups)
{
    this->groups_ = groups;
}

McHandler::McHandler(const McHandler& orig)
{
}

McHandler::~McHandler()
{
}

McTree* McHandler::getMcGroup(std::string* group_name)
{



    for (std::list<McTree*>::iterator it = groups_->begin(); it != groups_->end(); ++it)
    {


        if ((*it)->group_name_.compare(*group_name) == 0)
            return *it;
    }



    return NULL;

}

std::vector<McTree*> McHandler::lostConnectionDownlinks(unsigned char* mac_a)
{


    std::vector<McTree*> affected_trees;
    for (std::list<McTree*>::iterator it = groups_->begin(); it != groups_->end(); ++it)
    {

        McTree* tree = *it;


        if (tree->removeMacIfExsists(mac_a))
        {


            affected_trees.push_back(*it);
        }
    }

    return affected_trees;

}

std::vector<McTree*> McHandler::lostConnectionUplinks(unsigned char* mac_a)
{


    std::vector<McTree*> affected_trees;
    for (std::list<McTree*>::iterator it = groups_->begin(); it != groups_->end(); ++it)
    {
        McTree* tree = *it;

        if (compareMac(tree->route_uplink_->next_hop, mac_a))
        {

            tree->connected = false;
            affected_trees.push_back(tree);
        }
    }


    return affected_trees;

}

bool McHandler::removeGroup(std::string* group_name)
{
    McTree* t = getMcGroup(group_name);
    if (t == NULL)
        return false;
    
    ROS_ERROR("remove %s",group_name->c_str());

    groups_->remove(t);
    delete t;

}


McTree* McHandler::getMcGroup(std::string* hostname_source, uint32_t* route_id)
{



    for (std::list<McTree*>::iterator it = groups_->begin(); it != groups_->end(); ++it)
    {
        McTree* tree = *it;


        for (std::list<routing_entry*>::iterator it_r = tree->routing_entries_l_.begin(); it_r != tree->routing_entries_l_.end(); ++it_r)
        {


            if ((*it_r)->id == *route_id && (*it_r)->hostname_source.compare(*hostname_source) == 0)
                return *it;
        }
    }


    return NULL;
}

void McHandler::createGroupAsRoot(std::string* group_name)
{


    /* Create own mc group */

    this->createGroup(group_name, true, true, true, true, 0);

#ifdef MC_HANDLER_OUPUT
    ROS_ERROR("CREATE GROUP AS ROOT: %s", group_name->c_str());
#endif

}

void McHandler::addGroup(std::string* group_name)
{

    if (getMcGroup(group_name) == NULL)
    {
        this->createGroup(group_name, false, false, false, false, -1);

    }


#ifdef MC_HANDLER_OUPUT
    ROS_ERROR("CREATE GROUP AS ROOT: %s", group_name->c_str());
#endif

}

bool McHandler::addUplinkRoute(routing_entry* route)
{

    McTree* t = this->getMcGroup(&route->hostname_destination);
    if (t == NULL)
        createGroup(&route->hostname_destination, false, false, true, false, route->root_distance);

    t = this->getMcGroup(&route->hostname_destination);



    /* check if response is not from a node with a lower root distance and if the route dont exsists already*/
    if (route->root_distance <= t->route_uplink_->root_distance && t->routeIsNew(route))
    {

        t->routing_entries_l_.push_front(route);
        return true;
    }
    else
    {
        return false;
    }




}

void McHandler::addDownlinkRoute(routing_entry* route)
{

    McTree* t = this->getMcGroup(&route->hostname_destination);
    if (t == NULL)
        ROS_ERROR("Unexpected failure: want insert downlink route, but mc tree does not exists");


    t->routing_entries_downlinks_l_.push_front(route);





}

void McHandler::createGroup(std::string* group_name, bool root, bool member, bool connected, bool activated, uint16_t root_distance)
{


    McTree *my_mc_group = new McTree();
    my_mc_group->group_name_ = *group_name;
    my_mc_group->root = root;
    my_mc_group->member = member;
    my_mc_group->connected = connected;
    my_mc_group->activated = activated;

    my_mc_group->outgoing_request_ = NULL;

    /* Create routing entry for uplink*/
    my_mc_group->route_uplink_ = new routing_entry;

    my_mc_group->route_uplink_->hostname_destination = string(*group_name);
    my_mc_group->route_uplink_->hostname_source = "";
    my_mc_group->route_uplink_->id = 0 - 1;

    if (root)
    {
        my_mc_group->route_uplink_->hobs = 0;
        my_mc_group->route_uplink_->current_hop = 0;
        my_mc_group->route_uplink_->root_distance = 0;

    }
    else
    {
        my_mc_group->route_uplink_->hobs = 0 - 1;
        my_mc_group->route_uplink_->current_hop = 0 - 1;
        my_mc_group->route_uplink_->root_distance = 0 - 1;
    }


    this->groups_->push_front(my_mc_group);

}

void McHandler::setMembership(std::string* group_name, bool membership)
{
    this->getMcGroup(group_name)->member = membership;
}

#ifdef DEBUG

void McHandler::printMcGroups()
{

    for (std::list<McTree*>::iterator it = groups_->begin(); it != groups_->end(); ++it)
    {
        McTree* tree = *it;
        ROS_ERROR("NAME:[%s] ACTIVE[%u] ENTRIES: UP[%lu] DOWN[%lu] WAITING[%lu] DOWNLINKS[%lu]", tree->group_name_.c_str(), tree->activated, tree->routing_entries_l_.size(), tree->routing_entries_downlinks_l_.size(), tree->waiting_requests_l_.size(), tree->downlinks_l_.size());
    }
}
#endif 