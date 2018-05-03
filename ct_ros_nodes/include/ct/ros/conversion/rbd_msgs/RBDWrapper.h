/*
 * TransformWrapper.h
 *
 *  Created on: Jan 18, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_ROS_CONVERSION_RBD_MSGS_RBDWRAPPER_H_
#define INCLUDE_CT_ROS_CONVERSION_RBD_MSGS_RBDWRAPPER_H_

#include <ct_ros_msgs/rbdStateMsg.h>
#include <ct/ros/conversion/sensor_msgs/JointStateWrapper.h>

#include <ct/rbd/state/RBDState.h>

namespace ct {
namespace ros {

class RBDWrapper : public ct_ros_msgs::rbdStateMsg
{
public:

	typedef ct_ros_msgs::rbdStateMsg Base;

	RBDWrapper() : Base()
	{
	}

	template <size_t NJOINTS>
	RBDWrapper(const ct::rbd::RBDState<NJOINTS>& state, const ::ros::Time& time = ::ros::Time::now()) 
	: Base()
	{
		this->jointState = JointStateWrapper(state.joints(), time);

		this->basePose.position.x = state.basePose().position().x();
		this->basePose.position.y = state.basePose().position().y();
		this->basePose.position.z = state.basePose().position().z();

		this->basePose.orientation.x = state.basePose().getRotationQuaternion().x();
		this->basePose.orientation.y = state.basePose().getRotationQuaternion().y();
		this->basePose.orientation.z = state.basePose().getRotationQuaternion().z();
		this->basePose.orientation.w = state.basePose().getRotationQuaternion().w();

		this->baseVelocity.linear.x = state.baseLinearVelocity().x();
		this->baseVelocity.linear.y = state.baseLinearVelocity().y();
		this->baseVelocity.linear.z = state.baseLinearVelocity().z();
		this->baseVelocity.angular.x = state.baseLocalAngularVelocity().x();
		this->baseVelocity.angular.y = state.baseLocalAngularVelocity().y();
		this->baseVelocity.angular.z = state.baseLocalAngularVelocity().z();
	}

	ct_ros_msgs::rbdStateMsg& toROSMsg() { return *this; }

	template <size_t NJOINTS, class NameVector>
	RBDWrapper(const ct::rbd::RBDState<NJOINTS>& state, const NameVector& names, const ::ros::Time& time = ::ros::Time::now())
	: Base()
	{
		this->jointState = JointStateWrapper(state.joints(), names, time);

		this->basePose.position.x = state.basePose().position().x();
		this->basePose.position.y = state.basePose().position().y();
		this->basePose.position.z = state.basePose().position().z();

		this->basePose.orientation.x = state.basePose().getRotationQuaternion().x();
		this->basePose.orientation.y = state.basePose().getRotationQuaternion().y();
		this->basePose.orientation.z = state.basePose().getRotationQuaternion().z();
		this->basePose.orientation.w = state.basePose().getRotationQuaternion().w();

		this->baseVelocity.linear.x = state.baseLinearVelocity().x();
		this->baseVelocity.linear.y = state.baseLinearVelocity().y();
		this->baseVelocity.linear.z = state.baseLinearVelocity().z();
		this->baseVelocity.angular.x = state.baseLocalAngularVelocity().x();
		this->baseVelocity.angular.y = state.baseLocalAngularVelocity().y();
		this->baseVelocity.angular.z = state.baseLocalAngularVelocity().z();		
	}

};

}
}




#endif /* INCLUDE_CT_ROS_CONVERSION_RBD_MSGS_RBDWRAPPER_H_ */
