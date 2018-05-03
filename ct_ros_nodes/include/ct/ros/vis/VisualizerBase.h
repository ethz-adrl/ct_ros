#ifndef CT_VISUALIZERBASE_HPP_
#define CT_VISUALIZERBASE_HPP_

#include <visualization_msgs/Marker.h>
#include "ros/ros.h"


namespace ct {
namespace ros {


template<class MSG_TYPE = visualization_msgs::Marker>
class VisBase
{
public:
	VisBase(uint32_t markerId, const std::string& frameId, const std::string& rvizNamespace) :
		frameId_(frameId),
		rvizNamespace_(rvizNamespace),
		markerId_(markerId)
	{}

	virtual ~VisBase() {};

	// to be implemented by base class
	virtual void generateMessages(std::vector<MSG_TYPE>& messages) = 0;

protected:
	std::string frameId_;
	std::string rvizNamespace_;
	uint32_t markerId_;
};

using VisualizerBase = VisBase<visualization_msgs::Marker>;


} // namespace rbd
} // namespace ct

#endif /* CT_VISUALIZERBASE_HPP_ */
