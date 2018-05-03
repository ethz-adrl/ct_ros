#include <ct/ros/vis/visualizer/ArrowVisualizer.h>
#include <ct/ros/vis/visualizer/colorMapping.h>

namespace ct {
namespace ros {

void ArrowVisualizer::generateMessages(std::vector<visualization_msgs::Marker>& messages)
{
	if(pose_mode_ == true)
	{
		assert( positions_.size() == orientations_.size() && "Arrow Visualizer: number of arrow positions and orientations is not the same" );
		assert( magnitudes_.size() == positions_.size() && "Arrow Visualizer: number of arrow positions and magnitudes is not the same" );

		for(size_t i = 0; i< positions_.size(); i++)
		{
			visualization_msgs::Marker marker;
			marker.id = this->markerId_ + i;
			marker.header.frame_id = frameId_;
			marker.header.stamp = ::ros::Time();
			marker.ns = rvizNamespace_;

			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::MODIFY;

			marker.pose.position.x = positions_[i](0);
			marker.pose.position.y = positions_[i](1);
			marker.pose.position.z = positions_[i](2);
			marker.pose.orientation.w = orientations_[i].w();
			marker.pose.orientation.x = orientations_[i].x();
			marker.pose.orientation.y = orientations_[i].y();
			marker.pose.orientation.z = orientations_[i].z();

			marker.scale.x = scaling_* magnitudes_[i];
			marker.scale.y = diameter_;

			std_msgs::ColorRGBA color = getColor(i, 0, magnitudes_.size()-1);
			marker.colors.push_back(color);

			messages.push_back(marker);

		}
	}
	else{	// running in position and velocity mode

		assert( positions_.size() == velocities_.size() && "Arrow Visualizer: number of arrow positions and velocities is not the same" );
		for(size_t i = 0; i< positions_.size(); i++)
		{
			visualization_msgs::Marker marker;
			marker.id = this->markerId_ + i;
			marker.header.frame_id = frameId_;
			marker.header.stamp = ::ros::Time();
			marker.ns = rvizNamespace_;

			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::MODIFY;

			marker.pose.position.x = 0.0;
			marker.pose.position.y = 0.0;
			marker.pose.position.z = 0.0;
			marker.pose.orientation.w = 0.0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;

			marker.scale.x =  diameter_;
			marker.scale.y = 2*diameter_;

			geometry_msgs::Point start_point;
			start_point.x = positions_[i](0);
			start_point.y = positions_[i](1);
			start_point.z = positions_[i](2);
			marker.points.push_back(start_point);

			geometry_msgs::Point end_point;
			end_point.x = positions_[i](0) + scaling_ * velocities_[i](0);
			end_point.y = positions_[i](1) + scaling_ * velocities_[i](1);
			end_point.z = positions_[i](2) + scaling_ * velocities_[i](2);
			marker.points.push_back(end_point);


			marker.color = color_;

			messages.push_back(marker);
		}
	}


}

} //namespace ros
} //namespace ct
