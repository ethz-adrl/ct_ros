/*
 * PoseVisualizer.h
 *
 *  Created on: 20.08.2016
 *      Author: mgiftthaler
 */

#pragma once

#include <ct/ros/vis/VisualizerBase.h>
#include <geometry_msgs/PoseStamped.h>
#include <ct/rbd/rbd.h>

#include <memory>


namespace ct {
namespace ros {

class PoseVisualizer : public VisBase<geometry_msgs::PoseStamped>
{
public:
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> point_list_t;
    typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> quaternion_list_t;


    PoseVisualizer(const std::string& frameId, const std::string& rvizNamespace, const size_t markerId = 0)
        : VisBase<geometry_msgs::PoseStamped>(markerId, frameId, rvizNamespace)
    {
    }


    void setPose(const Eigen::Vector3d& point, const Eigen::Quaterniond& quat)
    {
        points_.clear();
        quaternions_.clear();
        addPose(point, quat);
    }

    void setPose(const ct::rbd::RigidBodyPose& pose)
    {
        points_.clear();
        quaternions_.clear();
        addPose(pose.position().toImplementation(), pose.getRotationQuaternion().toImplementation());
    }

    void setPoses(const point_list_t& points, const quaternion_list_t& quats)
    {
        points_ = points;
        quaternions_ = quats;
    }

    void addPose(const Eigen::Vector3d& point, const Eigen::Quaterniond& quat)
    {
        points_.push_back(point);
        quaternions_.push_back(quat);
    }

    ~PoseVisualizer(){};

    void generateMessages(std::vector<geometry_msgs::PoseStamped>& messages);

private:
    point_list_t points_;
    quaternion_list_t quaternions_;
};

}  // namespace ros
}  // namespace ct
