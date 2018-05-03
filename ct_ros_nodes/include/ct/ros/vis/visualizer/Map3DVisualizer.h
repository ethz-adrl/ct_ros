/*
 * map3DVisualization.h
 *
 *  Created on: 11.10.2013
 *      Author: neunertm
 */

#ifndef MAP3DVISUALIZER_H_
#define MAP3DVISUALIZER_H_

#include <memory>

#include <maps/Map3D.h>
#include <visualization_msgs/MarkerArray.h>
#include "../../vis/VisualizerBase.h"

namespace ct{
namespace ros{

template <typename T>
class Map3DVisualizer : public VisualizerBase
{
public:
	Map3DVisualizer (std::shared_ptr< ct::Map3D<T> > map, uint32_t markerId, const std::string& frameId, const std::string& rvizNamespace) :
		VisualizerBase(markerId, frameId, rvizNamespace),
		_map(map)
	{
	}

	~Map3DVisualizer() {};

	void generateMessages(std::vector<visualization_msgs::Marker>& messages);

private:
	std::shared_ptr< ct::Map3D<T> > _map;
};

// some briefs (not required but makes code look nicer)
typedef std::shared_ptr<Map3DVisualizer<float> > Map3DVis_f_ptr;
typedef std::shared_ptr<Map3DVisualizer<int> > Map3DVis_i_ptr;
typedef std::shared_ptr<Map3DVisualizer<bool> > Map3DVis_b_ptr;

} // namespace rbd
} // namespace ct


#include "../../vis/visualizer/implementation/Map3DVisualizer.h"

#endif /* MAP3DVISUALIZER_H_ */
