/*
 * RBDStatePublisher.h
 *
 *  Created on: Jan 18, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_ROS_PUBLISHERS_RBDSTATEPUBLISHERCUSTOM_H_
#define INCLUDE_CT_ROS_PUBLISHERS_RBDSTATEPUBLISHERCUSTOM_H_

#include <ros/ros.h>
#include <ct/ros/conversion/rbd_msgs/RBDWrapper.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <ct/rbd/state/RBDState.h>

namespace ct {
namespace ros {

class RBDStatePublisherCustom
{
public:

	RBDStatePublisherCustom(
			const std::vector<std::string>& jointNames = std::vector<std::string>(),
			const std::string& baseFrame = std::string("/base"),
			const std::string& worldFrame = std::string("/world")
	) :
		jointNames_(jointNames),
		baseFrame_(baseFrame),
		worldFrame_(worldFrame)
	{}

	void advertise(::ros::NodeHandle& nh, const char* jointStateTopicName, const int& bufferLength)
	{
		rbdStatePublisher_ = nh.advertise<RBDWrapper::Base>(jointStateTopicName, bufferLength);
	}

	void setJointNames(const std::vector<std::string>& jointNames)
	{
		jointNames_ = jointNames;
	}

	template <size_t NJOINTS>
	void publishState(const ct::rbd::RBDState<NJOINTS>& state, const ::ros::Time& time = ::ros::Time::now())
	{
		if (jointNames_.size() < NJOINTS)
		{
			std::cout << "RBDStatePublisherCustom: Warning, joint names does not have enough entries, resizing." << std::endl;
			jointNames_.resize(NJOINTS);
		}
		RBDWrapper rbdStateMsg(state, jointNames_, time);

		rbdStatePublisher_.publish(rbdStateMsg.toROSMsg());
	}


private:
	std::vector<std::string> jointNames_;

	std::string baseFrame_;
	std::string worldFrame_;

	::ros::Publisher rbdStatePublisher_;

	::tf2_ros::StaticTransformBroadcaster poseBroadcaster_;
};



}
}



#endif /* INCLUDE_CT_ROS_PUBLISHERS_RBDStatePublisherCustom_H_ */
