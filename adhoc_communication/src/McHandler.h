/* 
 * File:   McHandler.h
 * Author: cwioro
 *
 * Created on May 21, 2014, 12:11 PM
 */

#ifndef MCHANDLER_H
#define	MCHANDLER_H

#include "McTree.h"
#include "McTree.cpp"


//#define MC_HANDLER_OUPUT

class McHandler {
public:
    McHandler(std::list<McTree*>* groups);
    McHandler(const McHandler& orig);
    virtual ~McHandler();
    
    McTree* getMcGroup(std::string* group_name);
    McTree* getMcGroup(std::string* hostname_source, uint32_t* route_id);
    std::vector<McTree*> lostConnectionDownlinks(unsigned char* mac);
        std::vector<McTree*> lostConnectionUplinks(unsigned char* mac);
    void createGroupAsRoot(std::string* group_name);
    void addGroup(std::string* group_name);
    bool addUplinkRoute(routing_entry* route);
    bool removeGroup(std::string* group_name);


    void addDownlinkRoute(routing_entry* route);
    void setMembership(std::string* group_name, bool membership);
#ifdef DEBUG
    void printMcGroups();
#endif    

private:
    void createGroup(std::string* group_name, bool root, bool member, bool connected, bool activated, uint16_t root_distance);
    std::list<McTree*> *groups_;

};

#endif	/* MCHANDLER_H */

