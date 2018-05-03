/*
 * VisualizationNodeRT.h
 *
 *  Created on: 20.08.2016
 *      Author: mgiftthaler
 */

namespace ct{
namespace ros{


template<class MSG_TYPE>
int VisNodeRT<MSG_TYPE>::addVisualizer(std::shared_ptr<VisBase<MSG_TYPE>> visualizer)
{
	_visualizers.push_back(visualizer);
	return _visualizers.size()-1;
}


template<class MSG_TYPE>
void VisNodeRT<MSG_TYPE>::visualize(int vis_id)
{
  std::vector<MSG_TYPE> markers;

  // only visualize the specified visualizers
  std::vector<std::shared_ptr<VisBase<MSG_TYPE>> > to_visualize;
  if (vis_id == kVisualizeAll)
    to_visualize =  _visualizers;
  else
    to_visualize.push_back(_visualizers[vis_id]);

  for (std::shared_ptr<VisBase<MSG_TYPE>> vis_ptr : to_visualize)
  {
    std::vector<MSG_TYPE> submarkers;
    vis_ptr->generateMessages(submarkers);

    for (size_t i=0; i<submarkers.size(); i++)
      markers.push_back(submarkers[i]);
  }

  for (int i=0; i<markers.size(); i++)
  {
	  if(_publisher->trylock()){
		  _publisher->msg_ = markers[i];
		  _publisher->unlockAndPublish();
	  }
  }
}


} //namespace ros
} //namespace ct

