/*!
 * \file defines.h
 * \brief Includes all preprocessor defines
 * \author Günther Cwioro
 */

#ifndef DEFINES_H_
#define DEFINES_H_

//#define DEBUG





#define DELAY

#define PERFORMANCE_LOGGING
#define PERFORMANCE_LOGGING_UC_LINK_SUMMARY
#define PERFORMANCE_LOGGING_MC_LINK_SUMMARY
#define PERFORMANCE_LOGGING_UC_LINK_FRAMES
#define PERFORMANCE_LOGGING_SERVICE_CALLS

unsigned char bcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};	/*Broadcast address - all 1's*/

#define ETH_TYPE 0x4148 //AH -> AD-HOC


#define FRAME_TYPE_ACK 	0x61  //a -> Acknowledgment (link frame)
#define FRAME_TYPE_BEACON 0x42 //B -> BEACON
#define FRAME_TYPE_REQUEST 0x52 //R -> Route Request
#define FRAME_TYPE_REPLY 0x72 //r -> Route Response
#define FRAME_TYPE_TRANSPORT_DATA 0x44 //D -> Data
#define FRAME_TYPE_TRANSPORT_ACK 0x41 // A -> Acknowledgment (e2e frame)
#define FRAME_TYPE_CR_DETECTION 0x64 // d -> Detection
#define FRAME_TYPE_CR_SELECTION 0x53 // s -> Selection
#define FRAME_TYPE_MC_ACK 0x6d // m -> Selection
#define FRAME_TYPE_MC_NACK 0x4e // N -> Nack
#define FRAME_TYPE_MC_ACTIVATION 0x59// Y 
#define FRAME_TYPE_MC_DISCONNECT 0x50 // P -> Prune
#define FRAME_TYPE_BROADCAST 0x62 // b

#define FRAME_DATA_TYPE_FRONTIER 0x46 // F -> Frontier
#define FRAME_DATA_TYPE_MAP 0x4d 	//M -> Map
#define FRAME_DATA_TYPE_POSITION 0x50 //P -> Position
#define FRAME_DATA_TYPE_ANY 0x41	//A -> Any (String)
#define FRAME_DATA_TYPE_POINT 0x70	//p -> Point
#define FRAME_DATA_TYPE_MAP_UPDATE 0x55	//U
#define FRAME_DATA_TYPE_CONTROLL_MSG 0x43	//C
#define FRAME_DATA_TYPE_AUCTION 0x61	//C
#define FRAME_DATA_TYPE_CLUSTER 0x63	//C
#define FRAME_DATA_TYPE_TWIST 0x54	//T -> Twist
#define FRAME_DATA_TYPE_ROBOT_UPDATE 0x52	//R 
/*Tutorial*/
#define FRAME_DATA_TYPE_QUATERNION 0x51 //0x51 in hex means Q in ASCII -> stands for QUATERNION


#define MAX_INTERN_JOIN_ATTEMPS 5  //defines how often a robot will try to join a mc group
#define MAX_JOIN_ATTEMPS 5  //defines how often a robot will try to join a mc group
#define INTERVAL_WAIT_MC_RECONN 200 //[ms]if a node gets a prune message, it will will a little to give the network time for synconization
#define INTERWAL_RECONNECT_MC_G 500 // ms

#define TIME_BEFORE_REMOVE_NEIGHBOR 5000 //[ms] 


#define MAX_TIME_CACHE_UNACK_CR_FRAMES 1500 //[ms]
#define MAX_TIME_CACHE_UNUSED_ROUTES 5 * 60 * 1000 //[ms] -> 5min
#define MAX_TIME_CACHE_PUBLISHED_PACKETS 10 * 1000 //[ms]
#define MAX_TIME_CACHE_ROUTE_REQUEST 15 * 1000// [ms] Max time a route request will be cached
//#define MAX_NO_HELLO_FROM_NEIGHBOR 15       //Max times of beacon_interval after a node will remove neighbor
#define MAX_FRAMES_CACHED 200			//Max frames that are cached (to prevent multiple incoming of the same frame)
#define INTERVAL_TO_RETRANSMIT_LINK_FRAMES 30// [ms] Interval to retransmit unacknowledged link frame
#define MAX_RELAYS 3

#define RELAY_INDEX_MULTIPLICATOR 10
//8093 

#define INTERVAL_DEBUG_OUTPUT 3000
#define INTERVAL_UPTDATE_THREAD_TO_DELETE_OBSOLETE_REQUESTS 1200 // [ms] Time that the thread to delete obsolete requests will sleep, after spin
#define INTERVAL_RESEND_ROUTED_FRAME 100 //[ms] Time to re-send a unacknowledged routed frame
#define INTERVAL_RESEND_ROUTE_REQUEST 1200 // [ms] Time to re-send a route request, if i get no response
#define INTERVAL_DELETE_OLD_PACKETS 60000 //[ms]
#define INTERVAL_WAIT_FOR_MCROUTES 2000 //[ms]

/* MULTICAST */
#define INTERVAL_BETWEEN_MC_NACK_FRAMES 3000//[µs]
#define INTERVAL_BETWEEN_RESEND_REQUESTED_FRAMES 5000 //[µs] 1000
#define INTERVAL_REQUEST_FRAMES 30 // [ms] 100
#define INTERVAL_RESEND_REQUESTED_FRAMES 1 // [ms]
#define MAX_REQUESTED_FRAMES 40 // [FRAMES] //40
#define START_RQUESTING_PACKET_FILL_LEVEL 0.7f //[%]
#define START_RQUESTING_INTERVAL_AFTER_LAST_FRAME_GOT 200// [ms]

#define MAX_REQUESTED_FRAMES_IN_NACK 40


#define DEBUG
//#define DEBUG_OUTPUT 
#define TRY_REJOIN

//#define JOIN_ALL_GROUPS
#define USE_CHANNEL_MODEL

//8020
bool burst_multicast = true;

#endif /* DEFINES_H_ */

