/*
 * JointState.h
 *
 *  Created on: Jan 18, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_ROS_CONVERSION_SENSOR_MSGS_JOINTSTATEWRAPPER_H_
#define INCLUDE_CT_ROS_CONVERSION_SENSOR_MSGS_JOINTSTATEWRAPPER_H_

#include <sensor_msgs/JointState.h>

#include <ct/rbd/state/JointState.h>

namespace ct {
namespace ros {

class JointStateWrapper : public sensor_msgs::JointState
{
public:

	typedef sensor_msgs::JointState Base;

	JointStateWrapper() : Base()
	{
	}

	template <size_t NJOINTS>
	JointStateWrapper(const ct::rbd::JointState<NJOINTS>& jointState, const ::ros::Time& time = ::ros::Time::now()) :
		Base()
	{
		this->header.stamp = time;
		this->position.resize(NJOINTS);
		this->velocity.resize(NJOINTS);
		this->effort.resize(NJOINTS);
		this->name.resize(NJOINTS);

		for (size_t i=0; i<NJOINTS; i++)
		{
			this->position[i] = jointState.getPosition(i);
			this->velocity[i] = jointState.getVelocity(i);
		}
	}

	sensor_msgs::JointState& toROSMsg() { return *this; }

	/**
	 * Constructor with state and name vector.
	 * @param jointState
	 * @param names Can be any vector of length NJOINTS that has a bracket operator
	 */
	template <size_t NJOINTS, class NameVector>
	JointStateWrapper(const ct::rbd::JointState<NJOINTS>& jointState, const NameVector& names, const ::ros::Time& time = ::ros::Time::now()) :
		Base()
	{
		this->header.stamp = time;
		this->position.resize(NJOINTS);
		this->velocity.resize(NJOINTS);
		this->effort.resize(NJOINTS);
		this->name.resize(NJOINTS);

		for (size_t i=0; i<NJOINTS; i++)
		{
			this->position[i] = jointState.getPosition(i);
			this->velocity[i] = jointState.getVelocity(i);
			this->name[i] = names[i];
		}
	}

};

}
}




#endif /* INCLUDE_CT_ROS_CONVERSION_SENSOR_MSGS_JOINTSTATEWRAPPER_H_ */
