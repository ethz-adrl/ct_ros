/*
 * SphereVisualizer.h
 *
 *  Created on: 20.08.2016
 *      Author: mgiftthaler
 */

#ifndef CT_SPHERE_VISUALIZER_H_
#define CT_SPHERE_VISUALIZER_H_

#include <ct/ros/vis/VisualizerBase.h>

#include <memory>

#include <Eigen/Dense>

namespace ct{
namespace ros{

class SphereVisualizer : public VisualizerBase
{
public:

	typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > point_list_t;

	SphereVisualizer (uint32_t markerId, const std::string& frameId, const std::string& rvizNamespace, double pointSize = 0.2) :
		VisualizerBase(markerId, frameId, rvizNamespace),
		pointSize_(pointSize)
	{
	}

	void setPoints(const point_list_t& points) { points_ = points; }
	void addPoint(const Eigen::Vector3d& point) { points_.push_back(point); }

	~SphereVisualizer() {};

	void generateMessages(std::vector<visualization_msgs::Marker>& messages);

private:
	point_list_t points_;
	double pointSize_;
};

} // namespace ros
} // namespace ct


#endif /* POINTSVISUALIZER_H_ */

