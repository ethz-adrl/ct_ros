/*
 * LineStripVisualizer.h
 *
 *  Created on: 8.3.2016
 *      Author: Hamza Merzic
 */

#ifndef LINESTRIPVISUALIZER_H_
#define LINESTRIPVISUALIZER_H_

#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include <visualization_msgs/MarkerArray.h>
#include <ct/ros/vis/VisualizerBase.h>

#include <Eigen/Dense>

namespace ct {
namespace ros {


class LineStripVisualizer : public VisualizerBase{
public:
	typedef std::vector<Eigen::Vector3d,
			Eigen::aligned_allocator<Eigen::Vector3d> > point_list_t;

	LineStripVisualizer(
			uint32_t markerId,
			const std::string& frameId,
			const std::string& rvizNamespace,
			double pointSize = 0.2)
	: VisualizerBase(markerId, frameId, rvizNamespace),
	  pointSize_(pointSize),
	  lifetime_(999999)
	{
		color_.a = 1;
		color_.r = 1;
		color_.g = 0;
		color_.b = 0;
	}

	~LineStripVisualizer() {}

	void generateMessages(std::vector<visualization_msgs::Marker>& messages);

	void setPoints(const point_list_t& points) { points_ = points; }
	void addPoint(const Eigen::Vector3d& point) { points_.push_back(point); }
	void changeColor(std_msgs::ColorRGBA color) { color_ = color; }
	void clear() { points_.clear(); }
	void setLifetime(double lifet){lifetime_ = (::ros::Duration)lifet;}

private:
	double pointSize_;
	point_list_t points_;
	std_msgs::ColorRGBA color_;
	std::vector<visualization_msgs::Marker> markers_;
	::ros::Duration lifetime_;
};

}  // namespace ros
}  // namespace ct

#endif  // LINESTRIPVISUALIZER_H_
