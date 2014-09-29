/*
 * RouteRequest.h
 *
 *  Created on: 29.07.2013
 *      Author: Günther Cwioro
 *
 *
 */

#ifndef ROUTEREQUEST_H_
#define ROUTEREQUEST_H_


struct rreq_header
{
        uint8_t frame_type;
    	uint32_t id;
        uint32_t hop_count;
	uint32_t hop_limit;
        uint8_t flag_field;
	
	uint32_t hostname_source_len;
        uint32_t hostname_destination_len;
	
}__attribute__((packed));


class RouteRequest : public EthernetFrame {
public:
	RouteRequest(std::string my_hostname, std::string destination,uint16_t max_hops, bool is_multicast);
	RouteRequest(route_request req);
	RouteRequest(unsigned char* buffer);

	virtual ~RouteRequest();
	
        struct rreq_header header_;
        std::string hostname_destination_;
	std::string hostname_source_;
	std::list<mac> path_l_;
        
        uint16_t buffer_str_len_;
        
	bool correct_crc_;
	bool mc_flag_;
	
	

	

	static uint32_t req_count_stat;
	static uint32_t HEADER_FIXED_LEN;
        
        using EthernetFrame::GetCrc32;
        std::string getRequestAsNetworkString(unsigned char source_mac[6]);
	bool isMacInPath(unsigned char mac[]) ;

};

uint32_t RouteRequest::req_count_stat = 0;
uint32_t RouteRequest::HEADER_FIXED_LEN = sizeof(eh_header) + sizeof(rreq_header);

#endif /* ROUTEREQUEST_H_ */
