/*!
 * @file PositionSubscriber.h
 *
 * \date 11.11.2013
 * \author: Günther Cwioro, Torsten Andre
 */

#ifndef POSITIONSUBSCRIBER_H_
#define POSITIONSUBSCRIBER_H_


/*!
\class PositionSubscriber
\brief Subscripes to the position of the stage simulation

The PositionSubscripe class implements all function to build up a channel model in the stage simulation
*/
class PositionSubscriber {
public:
	/*! Packet constructor.
	Constructor without parametes.
	*/
	PositionSubscriber() ;
	virtual ~PositionSubscriber();

	/*!
	 *
	   \return the Y coordinate of the specific robot in stage
	*/
	double getYPos();

	/*!
	 *
	   \return the X coordinate of the specific robot in stage
	*/
	double getXPos();

	/*! Calculates the distance between a other robot and this instance in stage
	 *
	   \param other PositionSubscribe of the other robot
	   \return the calculated distance
	*/
	double calcDistance(PositionSubscriber* other);

	/*! callback method of the subscribed topic. Refreshes the robot position of the instance.
	 *
	   \param position Current robot position
	*/
	void Subscribe(const nav_msgs::Odometry::ConstPtr& position);

        bool initialized; ///< Defines if the robot position has been initialized 
	std::string robot_name_; ///< Name of the robot in stage. e.g: "robot_0"
	uint32_t robot_number_; ///< Number of the robot in stage. e.g: number of "robot_0" would be "0"

	nav_msgs::Odometry position; ///< Latest position of the specific robot

private:
	double x_pos_, y_pos_;
	uint16_t callback_count, callback_refresh;


};

#endif /* POSITIONSUBSCRIBER_H_ */
