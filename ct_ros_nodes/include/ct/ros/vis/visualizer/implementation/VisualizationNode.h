
namespace ct {
namespace ros {


template <class MSG_TYPE>
int VisNode<MSG_TYPE>::addVisualizer(std::shared_ptr<VisBase<MSG_TYPE>> visualizer)
{
    _visualizers.push_back(visualizer);
    return _visualizers.size() - 1;
}


template <class MSG_TYPE>
void VisNode<MSG_TYPE>::visualize(int vis_id)
{
    std::vector<MSG_TYPE> markers;

    // only visualize the specified visualizers
    std::vector<std::shared_ptr<VisBase<MSG_TYPE>>> to_visualize;
    if (vis_id == kVisualizeAll)
        to_visualize = _visualizers;
    else
        to_visualize.push_back(_visualizers[vis_id]);

    for (size_t i = 0; i < to_visualize.size(); i++)
    {
        std::vector<MSG_TYPE> submarkers;
        to_visualize[i]->generateMessages(submarkers);

        for (size_t j = 0; j < submarkers.size(); j++)
            _publisher.publish(submarkers[j]);
    }
}


}  //namespace ros
}  //namespace ct
