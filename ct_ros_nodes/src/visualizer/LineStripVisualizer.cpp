/**
 * LineStripVisualizer.cpp
 *
 *  Created on: 8.3.2016
 *      Author: mgiftthaler@ethz.ch
 */

#include <ct/ros/vis/visualizer/LineStripVisualizer.h>
#include <ct/ros/vis/visualizer/colorMapping.h>


namespace ct  {
namespace ros {

void LineStripVisualizer::generateMessages(
    std::vector<visualization_msgs::Marker>& messages) {
  visualization_msgs::Marker marker;
  marker.id = 1;
  marker.header.frame_id = frameId_;
  marker.header.stamp = ::ros::Time();
  marker.ns = rvizNamespace_;

  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = pointSize_;
  marker.scale.y = pointSize_;

  for (size_t i = 0u; i < points_.size(); ++i) {
    geometry_msgs::Point point;
    point.x = points_[i](0);
    point.y = points_[i](1);
    point.z = points_[i](2);
    marker.points.push_back(point);
  }

  marker.color = color_;

  marker.lifetime = lifetime_;

  messages.push_back(marker);
}

}  // namespace ros
}  // namespace ct
