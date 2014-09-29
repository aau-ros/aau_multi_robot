/*
 * RouteResponse.h
 *
 *  Created on: 25.07.2013
 *      Author: Günther Cwioro
 */

#ifndef ROUTERESPONSE_H_
#define ROUTERESPONSE_H_

class RouteResponse {
public:
	RouteResponse(RouteRequest req,unsigned char mac[6],uint8_t root_distance);
	RouteResponse(unsigned char* buffer);
	virtual ~RouteResponse();
	int GetCrc32(const std::string& my_string);
	std::string getResponseAsNetworkString(unsigned char source_mac[6]);

	struct ethhdr eh_;
	unsigned char mac_current_hop_[6];
	unsigned char mac_next_hop_[6];
	unsigned char mac_previous_hop_[6];
	uint32_t request_id_;
	std::string hostname_source_;
	uint32_t hop_count_;
	uint32_t current_hop_;
	std::list<mac> path_l_;
	bool correct_crc_;
	uint8_t mc_flag_;
	uint8_t root_distance;


	static uint32_t HEADER_FIXED_LEN;


};

uint32_t RouteResponse::HEADER_FIXED_LEN = 37;
#endif /* ROUTERESPONSE_H_ */
