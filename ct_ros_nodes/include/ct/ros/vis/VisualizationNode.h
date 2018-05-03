/*
 * VisualizationNode.h
 *
 *  Created on: 14.10.2013
 *      Author: neunertm
 */

#ifndef CT_VISUALIZATIONNODE_H_
#define CT_VISUALIZATIONNODE_H_

#include "VisualizerBase.h"

namespace ct{
namespace ros{


template<class MSG_TYPE = visualization_msgs::Marker>
class VisNode
{
public:
  static const int kVisualizeAll = -1;

	VisNode(::ros::NodeHandle& nodeHandle, const std::string& topicName)
	{
		_publisher = nodeHandle.advertise<MSG_TYPE>( topicName, 0 );
	}

	/**
	 * @param visualizer to be added to the node.
	 * @return visualizer id for later access.
	 */
	int addVisualizer(std::shared_ptr<VisBase<MSG_TYPE>> visualizer);

	/**
	 * Get reference to visualizer wished to be modified
	 */
	template<class VISUALIZER_TYPE>
	VISUALIZER_TYPE& getVisualizer(size_t pos)
	{
	  return *(std::dynamic_pointer_cast<VISUALIZER_TYPE>(_visualizers.at(pos)));
	}

	void visualize(int visId = kVisualizeAll);

private:
	::ros::Publisher _publisher;
	std::vector<std::shared_ptr<VisBase<MSG_TYPE>> > _visualizers;
};


using VisualizationNode = VisNode<>;

} //namespace ros
} //namespace ct

#include <ct/ros/vis/visualizer/implementation/VisualizationNode.h>

#endif /* CT_VISUALIZATIONNODE_H_ */
