#include <ct/ros/vis/visualizer/PointsBinVisualizer.h>
#include <ct/ros/vis/visualizer/colorMapping.h>

namespace ct {
namespace ros {

void PointsBinVisualizer::generateMessages(std::vector<visualization_msgs::Marker>& messages)
{
  int n_markers = markers_.size();


  // TODO: quite slow for large points set. Only plot last few points to increase speed.
	for (size_t i=points_.size()-2; i<points_.size(); i++)
	{
		geometry_msgs::Point point;
		point.x = points_[i](0);
		point.y = points_[i](1);
		point.z = points_[i](2);

		int marker_id = calcMarker(point);
		std_msgs::ColorRGBA color = getColor(marker_id, 0, n_markers);

		markers_[marker_id].points.push_back(point);
		markers_[marker_id].colors.push_back(color);
	}

  for (visualization_msgs::Marker m : markers_)
    messages.push_back(m);
}


void PointsBinVisualizer::createMarkers()
{

  markers_.clear();

  int n_markers = (z_max_ - z_min_) / dz_;

  for (int i = 0; i<n_markers; i++)
  {
    visualization_msgs::Marker marker;
    marker.id = i+1;
    marker.header.frame_id = frameId_;
    marker.header.stamp = ::ros::Time();
    marker.ns = rvizNamespace_ + std::to_string(i);

    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::MODIFY;

    marker.scale.x = pointSize_;
    marker.scale.y = pointSize_;
    marker.scale.z = pointSize_;

    markers_.push_back(marker);
  };
}


double PointsBinVisualizer::calcMarker(const geometry_msgs::Point& point)
{
  int marker = std::round((point.z - z_min_) / dz_);

  // bounds checking
  if (marker < 0) marker = 0;
  if (marker >= static_cast<int>(markers_.size())) marker = markers_.size()-1;

  return marker;
}


void PointsBinVisualizer::clearAllPoints()
{
  points_.clear();
  createMarkers();
}

} //namespace ros
} //namespace ct
