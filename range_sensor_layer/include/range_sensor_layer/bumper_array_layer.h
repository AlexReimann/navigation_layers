#ifndef range_sensor_layer_SRC_BUMPER_ARRAY_LAYER_H_
#define range_sensor_layer_SRC_BUMPER_ARRAY_LAYER_H_

#include <gopher_std_msgs/Bumpers.h>

#include "range_array_layer.h"

namespace range_sensor_layer
{

class BumperArrayLayer : public RangeArrayLayer
{
public:
  void onInitialize();

protected:
  virtual void registerSubscriber(const std::string& topic_name, ros::NodeHandle& nh);
  virtual void incomingBumperArray(const gopher_std_msgs::BumpersConstPtr& bumpers_array);

  double frame_to_obstacle_distance_;
};

} /* end namespace */

#endif
