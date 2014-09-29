/* 
 * File:   McTree.h
 * Author: cwioro
 *
 * Created on May 21, 2014, 11:54 AM
 */

#ifndef MCTREE_H
#define	MCTREE_H

class McTree {
public:
    McTree();
    McTree(const McTree& orig);
    virtual ~McTree();
    bool activateBestRoute(route_request* rreq_logging);
    bool activateRoute(std::string* hostname_source, uint32_t* id, unsigned char* mac_adr);
    bool routeIsNew(routing_entry* r);
    bool processFrame(unsigned char* src);
    bool addDownlinkAsMember(mac* m);
    bool addDownlinkAsConnector(mac* m);
    bool addWaitingRequest(RouteRequest* req, unsigned char* source_mac);
    bool propagateFrame(unsigned char* sender_mac);
    void safeOutgoingRequest(RouteRequest* req);
    void resetTmpFields();
    bool downlinkExsists(unsigned char* m);
    bool removeMacIfExsists(unsigned char* m);
    void printTree();
    
    bool operator=(const McTree* other);

    std::string group_name_;
    routing_entry* route_uplink_;
    RouteRequest* outgoing_request_;
    std::list<mac*> downlinks_l_;
    unsigned long time_stamp_;
    
    std::list<RouteRequest*> waiting_requests_l_;
    std::list<routing_entry*> routing_entries_l_;
     std::list<routing_entry*> routing_entries_downlinks_l_;
    
    bool member;
    bool activated;
    bool connected; //indicates if the node is actually connected to the uplink 
    bool root; //true if current robot is root of the tree
    
    
    
  
private:
    
 

};



#endif	/* MCTREE_H */

