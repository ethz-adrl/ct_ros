/*
 * SphereVisualizer.h
 *
 *  Created on: 20.08.2016
 *      Author: mgiftthaler
 */

#include <ct/ros/vis/visualizer/SphereVisualizer.h>
#include <ct/ros/vis/visualizer/colorMapping.h>

namespace ct {
namespace ros {

void SphereVisualizer::generateMessages(std::vector<visualization_msgs::Marker>& messages)
{
	visualization_msgs::Marker marker;
	marker.id = this->markerId_;
	marker.header.frame_id = frameId_;
	marker.header.stamp = ::ros::Time();
	marker.ns = rvizNamespace_;

	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = pointSize_;
	marker.scale.y = pointSize_;


	for (size_t i=0; i<points_.size(); i++)
	{
		geometry_msgs::Point point;
		point.x = points_[i](0);
		point.y = points_[i](1);
		point.z = points_[i](2);
		marker.points.push_back(point);

		std_msgs::ColorRGBA color = getColor(i, 0, points_.size());
		marker.colors.push_back(color);
	}

	messages.push_back(marker);
}

} //namespace rbd
} //namespace ct
