/*
 * ArrowVisualizer.h
 *
 *  Created on: 05.07.2016
 *      Author: mgiftthaler
 *
 *      Visualizer to plot an array of vectors of same style
 */

#ifndef CT_ARROW_VISUALIZER_H_
#define CT_ARROW_VISUALIZER_H_

#include <ct/ros/vis/VisualizerBase.h>

#include <memory>

#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>


namespace ct{
namespace ros{

class ArrowVisualizer : public VisualizerBase
{
public:

	typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > position_list_t;
	typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > velocity_list_t;
	typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > orientation_list_t;
	typedef std::vector<double> magnitude_list_t;


	ArrowVisualizer (uint32_t markerId, const std::string& frameId, const std::string& rvizNamespace, double diameter = 0.05, double scaling = 1.0) :
		VisualizerBase(markerId, frameId, rvizNamespace),
		diameter_(diameter),
		scaling_(scaling),
		pose_mode_(false)
	{
	    color_.a = 1;
	    color_.r = 1;
	    color_.g = 1;
	    color_.b = 1;
	}

	~ArrowVisualizer() {};

	void setArrows(const position_list_t& positions, const orientation_list_t orientations, const magnitude_list_t magnitudes)
	{
		positions_ = positions;
		orientations_ = orientations;
		magnitudes_ = magnitudes;

		pose_mode_ = true;
	}

	void setArrows(const position_list_t& positions, const velocity_list_t& velocities)
	{
		positions_ = positions;
		velocities_ = velocities;

		pose_mode_ = false;
	}

	void setColor(const std_msgs::ColorRGBA color) {color_ = color;}

	void generateMessages(std::vector<visualization_msgs::Marker>& messages);

private:
	double diameter_;
	double scaling_;

	position_list_t 	positions_;
	velocity_list_t 	velocities_;
	orientation_list_t 	orientations_;
	magnitude_list_t 	magnitudes_;

	bool pose_mode_; 	// specify pose and magnitude or position and velocity vector?
	std_msgs::ColorRGBA color_;

};

} // namespace rbd
} // namespace ct



#endif
