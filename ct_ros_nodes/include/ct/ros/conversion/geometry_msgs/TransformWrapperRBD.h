/*
 * TransformWrapper.h
 *
 *  Created on: Jan 18, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_ROS_CONVERSION_GEOMETRY_MSGS_TRANSFORMWRAPPERRBD_H_
#define INCLUDE_CT_ROS_CONVERSION_GEOMETRY_MSGS_TRANSFORMWRAPPERRBD_H_

#include <geometry_msgs/Transform.h>

#include <ct/rbd/state/RigidBodyPose.h>

namespace ct {
namespace ros {

class TransformWrapperRBD : public ct::rbd::RigidBodyPose
{
public:

	typedef ct::rbd::RigidBodyPose Base;

	TransformWrapperRBD() : Base()
	{
	}

	TransformWrapperRBD(const geometry_msgs::Transform& msg) : Base()
	{
		this->position().x() = msg.translation.x;
		this->position().y() = msg.translation.y;
		this->position().z() = msg.translation.z;

		this->getRotationQuaternion().x() = msg.rotation.x;
		this->getRotationQuaternion().y() = msg.rotation.y;
		this->getRotationQuaternion().z() = msg.rotation.z;
		this->getRotationQuaternion().w() = msg.rotation.w;
	}

};

}
}




#endif /* INCLUDE_CT_ROS_CONVERSION_GEOMETRY_MSGS_TRANSFORMWRAPPER_H_ */
