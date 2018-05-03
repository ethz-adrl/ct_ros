/*
 * TrajectoryVisualizer.cpp
 *
 *  Created on: 14.10.2013
 *      Author: neunertm
 */

#include <ct/ros/vis/visualizer/TrajectoryVisualizer.h>

namespace ct {
namespace ros {


void TrajectoryVisualizer::generateMessages(std::vector<visualization_msgs::Marker>& messages)
{
	size_t size = _trajectory->size();

	visualization_msgs::Marker marker;
	marker.id = markerId_;
	marker.header.frame_id = _frameId;
	marker.header.stamp = ::ros::Time();
	marker.ns = _rvizNamespace;

	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::MODIFY;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 0.005;

	std_msgs::ColorRGBA color;
	color.r = 0.0;
	color.g = 1.0;
	color.b = 0.0;
	color.a = 1.0;

	marker.color = color;

	for (size_t i=0; i<size; i++)
	{
		geometry_msgs::Point point;
		point.x = (*_trajectory)[i].x();
		point.y = (*_trajectory)[i].y();
		point.z = (*_trajectory)[i].z();
		marker.points.push_back(point);
	}

	messages.push_back(marker);
}

} //namespace ros
} //namespace ct
