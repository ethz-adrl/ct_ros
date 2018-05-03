/*
 * RBDStatePublisher.h
 *
 *  Created on: Jan 18, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_ROS_PUBLISHERS_RBDSTATEPUBLISHER_H_
#define INCLUDE_CT_ROS_PUBLISHERS_RBDSTATEPUBLISHER_H_

#include <ros/ros.h>
#include <ct/ros/conversion/sensor_msgs/JointStateWrapper.h>
#include <ct/ros/conversion/geometry_msgs/TransformWrapper.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <ct/rbd/state/RBDState.h>

namespace ct {
namespace ros {

class RBDStatePublisher
{
public:

	RBDStatePublisher(
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
		jointStatePublisher_ = nh.advertise<JointStateWrapper::Base>(jointStateTopicName, bufferLength);
	}

	void setJointNames(const std::vector<std::string>& jointNames)
	{
		jointNames_ = jointNames;
	}

	template <size_t NJOINTS>
	void publishState(const ct::rbd::RBDState<NJOINTS>& state, const ::ros::Time& time = ::ros::Time::now())
	{
		publishJointState(state.joints(), time);
		publishBasePoseTF(state.basePose(), time);
	}

	template <size_t NJOINTS>
	void publishJointState(const ct::rbd::JointState<NJOINTS>& jointState, const ::ros::Time& time = ::ros::Time::now())
	{
		if (jointNames_.size() < NJOINTS)
		{
			std::cout << "RBDStatePublisher: Warning, joint names does not have enough entries, resizing." << std::endl;
			jointNames_.resize(NJOINTS);
		}
		JointStateWrapper jointStateMsg(jointState, jointNames_, time);

		jointStatePublisher_.publish(jointStateMsg.toROSMsg());
	}

	void publishBasePoseTF(const ct::rbd::RigidBodyPose& pose, const ::ros::Time& time = ::ros::Time::now())
	{
		geometry_msgs::TransformStamped transformStamped;

		transformStamped.header.stamp = time;
		transformStamped.header.frame_id = worldFrame_;
		transformStamped.child_frame_id = baseFrame_;
		transformStamped.transform = TransformWrapper(pose);
		poseBroadcaster_.sendTransform(transformStamped);
	}



private:
	std::vector<std::string> jointNames_;

	std::string baseFrame_;
	std::string worldFrame_;

	::ros::Publisher jointStatePublisher_;

	::tf2_ros::StaticTransformBroadcaster poseBroadcaster_;
};



}
}



#endif /* INCLUDE_CT_ROS_PUBLISHERS_RBDSTATEPUBLISHER_H_ */
