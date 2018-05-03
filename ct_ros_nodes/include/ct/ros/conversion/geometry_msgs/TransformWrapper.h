/*
 * TransformWrapper.h
 *
 *  Created on: Jan 18, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_ROS_CONVERSION_GEOMETRY_MSGS_TRANSFORMWRAPPER_H_
#define INCLUDE_CT_ROS_CONVERSION_GEOMETRY_MSGS_TRANSFORMWRAPPER_H_

#include <geometry_msgs/Transform.h>

#include <ct/rbd/state/RigidBodyPose.h>

namespace ct {
namespace ros {

class TransformWrapper : public geometry_msgs::Transform
{
public:

	typedef geometry_msgs::Transform Base;

	TransformWrapper() : Base()
	{
	}

	TransformWrapper(const ct::rbd::RigidBodyPose& pose) : Base()
	{
		this->translation.x = pose.position().x();
		this->translation.y = pose.position().y();
		this->translation.z = pose.position().z();

		this->rotation.x = pose.getRotationQuaternion().x();
		this->rotation.y = pose.getRotationQuaternion().y();
		this->rotation.z = pose.getRotationQuaternion().z();
		this->rotation.w = pose.getRotationQuaternion().w();
	}

};

}
}




#endif /* INCLUDE_CT_ROS_CONVERSION_GEOMETRY_MSGS_TRANSFORMWRAPPER_H_ */
