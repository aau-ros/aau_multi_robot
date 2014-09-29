/*!
 * @file Packet.h
 * Network layer packet definition.
 *
 * \date 21.08.2013
 * \author Günther Cwioro, Torsten Andre
 */

#ifndef PACKET_H_
#define PACKET_H_

/*! 
\class Packet
\brief The packet class for network layer packets.

The packet class implements all functions required to build a network layer packet.
*/
class Packet {
public:
	/*! Packet constructor.
	Constructor initializes basic members.
	\param packet_id The ID of the packet.
	\param packet_size The size of the packet in bytes.
	\param payload_data_type Defines which type the payload is
	\param source The host name of the source
	\param topic The ROS topic name on which to publish
	*/
	Packet(RoutedFrame frame);//(uint32_t packet_id,uint32_t packet_size, uint8_t payload_data_type,std::string source,std::string topic, string mc_group);
	Packet();
	virtual ~Packet();

	bool frameAlreadyExsits(RoutedFrame f);
	void refreshLists();
	void sortFrameList();
        unsigned long getSize();
    
	void requestMissingFrames();
	void refreshMissingFramesList();

	/*! Add the frame to the frame list of the packet, if the frame belongs to the packet.
	 * This method is needed to build up the packet on the receiver side.
	 * This method also checks if all frames are received and the packet is complete.
	   \param frame The frame that should be added
	   \return True, if packet is complete and ready to publish. False, in every other case

	*/
	bool addFrame(RoutedFrame frame);
	bool isMcFrame();
	bool isNack();

	/*! Returns the packet's payload.

	\return True if \todo what?, false otherwise.
	*/
	std::string getPayload();

	uint32_t id_;				///< Packet ID 
	uint32_t highest_seq;
	uint32_t size_;				///< Size of packet in bytes 
	uint8_t data_type_;			///< \todo What is this?
       
	bool wrong_sequence_;
	std::string hostname_source_;		///< hostname of the source
	std::string mc_group_;		///< name of the mc group
	std::string topic_;			///< ROS topic on which to publish 
	std::list<RoutedFrame> frames_l_;	///< \todo What is this?
	std::list<uint32_t> missed_sequences_l_; ///< A list with all lost pending frames
	//ecl::TimeStamp time_stamp_;		///< Time stamp when \todo What does the time stamp mean? 
        unsigned long ts_;
        unsigned long ts_last_frame_request;
       
        
        static int NACK_THRESHOLD;
};
int Packet::NACK_THRESHOLD = 10;
#endif /* PACKET_H_ */
