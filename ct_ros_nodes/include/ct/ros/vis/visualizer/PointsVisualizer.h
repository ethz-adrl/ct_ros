#ifndef CT_ROS_POINTSVISUALIZER_H_
#define CT_ROS_POINTSVISUALIZER_H_

#include <Eigen/Dense>
#include <ct/ros/vis/VisualizerBase.h>

#include <memory>

namespace ct{
namespace ros{

class PointsVisualizer : public VisualizerBase
{
public:

	typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > point_list_t;

	PointsVisualizer (uint32_t markerId, const std::string& frameId, const std::string& rvizNamespace, double pointSize = 0.2) :
		VisualizerBase(markerId, frameId, rvizNamespace),
		pointSize_(pointSize),
		lifetime_(999999)
	{
	}

	void setPoints(const point_list_t& points) { points_ = points; }
	void addPoint(Eigen::Vector3d point) { points_.push_back(point); }

	~PointsVisualizer() {};
	void clear() { points_.clear(); }

	void generateMessages(std::vector<visualization_msgs::Marker>& messages);
	void setLifetime(double lifet){lifetime_ = (::ros::Duration)lifet;}


private:
	point_list_t points_;
	double pointSize_;
	::ros::Duration lifetime_;
};

} // namespace ros
} // namespace ct


#endif /* POINTSVISUALIZER_H_ */

