/*
 * Map3DVisualizer.h
 *
 *  Created on: 14.10.2013
 *      Author: neunertm
 */


#include "../colorMapping.h"

namespace ct {
namespace ros {


template <typename T>
void Map3DVisualizer<T>::generateMessages(std::vector<visualization_msgs::Marker>& messages)
{
	ct::map_size_t size = _map->getSize();
	float resolution = _map->getGridResolution();
	point_t origin = _map->getOrigin();
	T max = _map->getMax();
	T min = _map->getMin();

	visualization_msgs::Marker marker;
	marker.id = markerId_;
	marker.header.frame_id = frameId_;
	marker.header.stamp = ros::Time();
	marker.ns = rvizNamespace_;

	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.action = visualization_msgs::Marker::MODIFY;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = resolution;
	marker.scale.y = resolution;
	marker.scale.z = resolution;

	for (size_t x=0; x<size.x; x++)
	{
		for (size_t y=0; y<size.y; y++)
		{
			for (size_t z=0; z<size.z; z++)
			{
				if ((*_map)[index_t(x,y,z)] == 0)
					continue;

				std_msgs::ColorRGBA color = getColor((*_map)[index_t(x,y,z)], min, max);
				marker.colors.push_back(color);

				geometry_msgs::Point point;
				point.x = origin.x() + x*resolution;
				point.y = origin.y() + y*resolution;
				point.z = origin.z() + z*resolution;

				marker.points.push_back(point);
			}
		}
	}

	messages.push_back(marker);
}

// some briefs (not required but makes code look nicer)
typedef std::shared_ptr<Map3DVisualizer<float> > Map3DVis_f_ptr;
typedef std::shared_ptr<Map3DVisualizer<int> > Map3DVis_i_ptr;
typedef std::shared_ptr<Map3DVisualizer<bool> > Map3DVis_b_ptr;

} // namespace ros
} // namespace ct
