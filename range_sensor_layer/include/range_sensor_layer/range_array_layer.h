#ifndef range_sensor_layer_RANGE_ARRAY_LAYER_H_
#define range_sensor_layer_RANGE_ARRAY_LAYER_H_

#include "extended_range_sensor_layer.h"

#include <gopher_std_msgs/Ranges.h>

namespace range_sensor_layer
{

class RangeArrayLayer : public ExtendedRangeSensorLayer
{
public:
  void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);

protected:
  virtual void registerSubscriber(const std::string& topic_name, ros::NodeHandle& nh);
  virtual void incomingRangeArray(const gopher_std_msgs::RangesConstPtr& range_array);

  bool reset_every_cycle_;
};

} /* end namespace */

#endif
