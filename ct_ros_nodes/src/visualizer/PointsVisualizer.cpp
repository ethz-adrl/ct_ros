#include <ct/ros/vis/visualizer/PointsVisualizer.h>
#include <ct/ros/vis/visualizer/colorMapping.h>

namespace ct {
namespace ros {

void PointsVisualizer::generateMessages(std::vector<visualization_msgs::Marker>& messages)
{
	visualization_msgs::Marker marker;
	marker.id = 1;
	marker.header.frame_id = frameId_;
	marker.header.stamp = ::ros::Time();
	marker.ns = rvizNamespace_;

	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::MODIFY;

	marker.scale.x = pointSize_;
	marker.scale.y = pointSize_;


	for (size_t i=0; i<points_.size(); i++)
	{
		geometry_msgs::Point point;
		point.x = points_[i](0);
		point.y = points_[i](1);
		point.z = points_[i](2);
		marker.points.push_back(point);

		std_msgs::ColorRGBA color = getColor(i, 0, points_.size()-1);
		marker.colors.push_back(color);
	}

	marker.lifetime = lifetime_;

	messages.push_back(marker);
}

} //namespace ros
} //namespace ct
