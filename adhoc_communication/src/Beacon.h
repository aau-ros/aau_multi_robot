/*
 * \file Beacon.h
 *
 * \date 19.07.2013
 * \author Günther Cwioro, Torsten Andre
 */

#ifndef Beacon_H_
#define Beacon_H_

/** Beacons are transmitted periodically to enable robot detection.
They allow to 
- detect new robots and
- track connectivity between robots.
*/
class Beacon {
public:
	/** Constructs the Beacon on the sender side.
	\param source The mac address of the source robot that emits the Beacon
	\param hostname The hostname of the source robot that emits the Beacon
	*/
	Beacon(unsigned char* source, std::string hostname);

	/** Constructs the Beacon on the receiver side.
	\param buffer The received network buffer
	*/
	Beacon(unsigned char* buffer);

	virtual ~Beacon();

	/** Returns a Beacon as a C++ string.
	Converts the beacon into a C++ string to be written to 
	the socket.
	\return Bacon as C++ string.
	*/
	std::string getFrameAsNetworkString();

	/** Calculates the Crc32 of a Beacon
	Is needed to detect corrupt beacons caused by transmission errors.
	\return the Crc32 checksum of a Beacon.
	*/

	int GetCrc32(const std::string& my_string);
	

	bool correct_crc_;			///< Indicates whether CRC of received beacon is correct.
	unsigned char mac_source_[6];		///< The mac address of the source robot that emits the Beacon
	struct ethhdr *eh_;			///< Defines layer 2 Ethernet header
	std::string hostname_;			///< Stores host name of the beacon sender.

	static uint32_t HEADER_FIXED_LEN;	///< The lenght of the beacon header (23 bytes)


};

uint32_t Beacon::HEADER_FIXED_LEN = 23;		

#endif /* Beacon_H_ */
