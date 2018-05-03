/*
 * PoseVisualizer.h
 *
 *  Created on: 20.08.2016
 *      Author: markus giftthaler
 */


#include <ct/ros/vis/visualizer/PoseVisualizer.h>
#include <ct/ros/vis/visualizer/colorMapping.h>

namespace ct {
namespace ros {

void PoseVisualizer::generateMessages(std::vector<geometry_msgs::PoseStamped>& messages)
{

	assert( points_.size() == quaternions_.size() && "Pose Visualizer: number of arrow positions and orientations is not the same" );

	for(size_t i = 0; i< points_.size(); i++){

		geometry_msgs::PoseStamped newPose;
		newPose.pose.position.x = points_[i](0);
		newPose.pose.position.y = points_[i](1);
		newPose.pose.position.z = points_[i](2);
		newPose.pose.orientation.w = quaternions_[i].w();
		newPose.pose.orientation.x = quaternions_[i].x();
		newPose.pose.orientation.y = quaternions_[i].y();
		newPose.pose.orientation.z = quaternions_[i].z();

		newPose.header.stamp = ::ros::Time::now();
		newPose.header.frame_id = this->frameId_;

		messages.push_back(newPose);
	}
}

} //namespace ros
} //namespace ct
