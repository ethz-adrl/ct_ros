/*
 * PointsBinVisualizer.h
 *
 *  Created on: 11.10.2014
 *      Author: winklera
 */

#ifndef CT_ROS_POINTSBINVISUALIZER_H_
#define CT_ROS_POINTSBINVISUALIZER_H_

#include <memory>
#include <Eigen/Dense>

#include <ct/ros/vis/VisualizerBase.h>
#include <visualization_msgs/MarkerArray.h>


namespace ct{
namespace ros{

/**
 * @brief Like PointsVisualizer, except that different height ranges can be
 *        selectively hidden since they are in different namespaces.
 */
class PointsBinVisualizer : public VisualizerBase
{
public:

	typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > point_list_t;

	PointsBinVisualizer (uint32_t markerId, const std::string& frameId,
	                  const std::string& rvizNamespace, double pointSize = 0.2,
	                  double z_min=0, double z_max=1, double dz=0.1) :
		VisualizerBase(markerId, frameId, rvizNamespace),
		pointSize_(pointSize)
	{
	  z_min_ = z_min;
	  z_max_ = z_max;
	  dz_    = dz;
	  createMarkers();

	  color_.a = 1;
	  color_.r = 1;
	  color_.g = 0;
	  color_.b = 0;
	}
	~PointsBinVisualizer() {};


	void generateMessages(std::vector<visualization_msgs::Marker>& messages);

	void setPoints(const point_list_t& points) { points_ = points; }
	void addPoint(Eigen::Vector3d point) { points_.push_back(point); }
	void changeColor(std_msgs::ColorRGBA color) { color_ = color; }
	void createMarkers();
	double calcMarker(const geometry_msgs::Point& point);
	void clearAllPoints();


private:
	point_list_t points_;
	double pointSize_;
	std_msgs::ColorRGBA color_;
	std::vector<visualization_msgs::Marker> markers_;
	double z_min_, z_max_; ///< height range between which colors vary
	double dz_; ///< discretization height for different colors
};

} // namespace rbd
} // namespace ct


#endif /* POINTSBINVISUALIZER_H_ */
