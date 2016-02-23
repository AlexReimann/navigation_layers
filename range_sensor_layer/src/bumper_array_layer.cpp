#include <limits>
#include <pluginlib/class_list_macros.h>

#include "range_sensor_layer/bumper_array_layer.h"

PLUGINLIB_EXPORT_CLASS(range_sensor_layer::BumperArrayLayer, costmap_2d::Layer)

namespace range_sensor_layer
{

void BumperArrayLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);

  nh.param("frame_to_obstacle_distance", frame_to_obstacle_distance_, 0.1);

  RangeArrayLayer::onInitialize();
}

void BumperArrayLayer::registerSubscriber(const std::string& topic_name, ros::NodeHandle& nh)
{
  range_subs_.push_back(nh.subscribe(topic_name, 100, &BumperArrayLayer::incomingBumperArray, this));
  ROS_INFO("BumperArrayLayer: subscribed to topic %s", range_subs_.back().getTopic().c_str());
}

void BumperArrayLayer::incomingBumperArray(const gopher_std_msgs::BumpersConstPtr& bumpers_array)
{
  boost::mutex::scoped_lock lock(range_message_mutex_);

  sensor_msgs::Range converted_to_range_msgs;
  converted_to_range_msgs.min_range = frame_to_obstacle_distance_;
  converted_to_range_msgs.max_range = frame_to_obstacle_distance_;
  converted_to_range_msgs.radiation_type = sensor_msgs::Range::INFRARED;

  gopher_std_msgs::Bumper bumper;
  for (int i = 0; i < bumpers_array->bumpers.size(); ++i)
  {
    bumper = bumpers_array->bumpers.at(i);

    converted_to_range_msgs.header.stamp = bumper.header.stamp;
    converted_to_range_msgs.header.frame_id = bumper.header.frame_id;

    if (bumper.is_pressed)
    {
      converted_to_range_msgs.range = -std::numeric_limits<float>::infinity();
    }
    else
    {
      converted_to_range_msgs.range = std::numeric_limits<float>::infinity();
    }

    range_msgs_buffer_.push_back(converted_to_range_msgs);
  }
}

} /* end namespace */
