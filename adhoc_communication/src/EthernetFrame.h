/*
 * EthernetFrame.h
 *
 *  Created on: Mar 28, 2014
 *      Author: cwioro
 */

#ifndef ETHERNETFRAME_H_
#define ETHERNETFRAME_H_

#include "functions.h"


struct stc_frame
{
	std::string network_string;
	uint32_t frame_id;
        
	uint8_t retransmitted;
	//bool mc_frame;// = false;;
        bool cr;
	unsigned char mac[6];
        uint8_t type;
	std::string hostname_source;
	std::string mc_group;
        unsigned long ts;
	//ecl::TimeStamp time_stamp;
	// Assignment operator.
        
        void print_frame()
        {
            ROS_ERROR("ID:[%u] SOURCE[%s] MAC[%s] TYPE[%u] GROUP[%s]",frame_id, hostname_source.c_str() , getMacAsCStr(mac), type, mc_group.c_str());
        }
        
        stc_frame()
        {
            ts = getMillisecondsTime();
        }
        
	bool operator ==(const stc_frame& st)
	{
		//ROS_DEBUG("compare");

		if (mc_group.compare("")==0 )
			return ( compareMac(st.mac,mac) && hostname_source.compare(st.hostname_source)==0&&frame_id==st.frame_id) && type == st.type;
		else
		{

			return (hostname_source.compare(st.hostname_source)==0 && frame_id==st.frame_id && mc_group.compare(st.mc_group)==0);
		}
	}
};


struct eh_header{
        unsigned char eh_dest[6];
        unsigned char eh_source[6];
	uint16_t eh_protocol_id;
}  __attribute__((packed));

class EthernetFrame {
public:
	EthernetFrame();
	virtual ~EthernetFrame();

	struct eh_header eh_h_; ///< Struct of the layer 2 Ethernet header. Includes: source mac, destination mac an protocol type
	bool correct_crc_;




/** Calculates the Crc32 of the McMaintenanceFrame
	Is needed to detect corrupt frames caused by transmission errors.
	\return the Crc32 checksum of a McMaintenanceFrame.
	*/
	int GetCrc32(const std::string& string);


};



#endif /* ETHERNETFRAME_H_ */
