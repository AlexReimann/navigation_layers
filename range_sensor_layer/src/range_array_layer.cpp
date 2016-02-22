#include <pluginlib/class_list_macros.h>
#include <boost/algorithm/string.hpp>

#include "range_sensor_layer/range_array_layer.h"

PLUGINLIB_EXPORT_CLASS(range_sensor_layer::RangeArrayLayer, costmap_2d::Layer)

namespace range_sensor_layer
{

//redo everything of the original and extended, but change to use an array of ranges
void RangeArrayLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  buffered_readings_ = 0;
  last_reading_time_ = ros::Time::now();
  default_value_ = to_cost(0.5);
  phi_v_ = 1.2;
  max_angle_ = 12.5 * M_PI / 180;

  matchSize();
  min_x_ = min_y_ = -std::numeric_limits<double>::max();
  max_x_ = max_y_ = std::numeric_limits<double>::max();

  // Default topic names list contains a single topic: /sonar
  // We use the XmlRpcValue constructor that takes a XML string and reading start offset
  const char* xml = "<value><array><data><value>/sonar</value></data></array></value>";
  int zero_offset = 0;
  std::string topics_ns;
  XmlRpc::XmlRpcValue topic_names(xml, &zero_offset);

  nh.param("ns", topics_ns, std::string());
  nh.param("topics", topic_names, topic_names);

  nh.param("no_readings_timeout", no_readings_timeout_, .0);

  nh.param("clear_threshold", clear_threshold_, .2);
  nh.param("mark_threshold", mark_threshold_, .8);

  nh.param("clear_on_max_reading", clear_on_max_reading_, false);
  nh.param("reset_every_cycle", reset_every_cycle_, false);

  InputSensorType input_sensor_type = ALL;
  std::string sensor_type_name;
  nh.param("input_sensor_type", sensor_type_name, std::string("ALL"));

  boost::to_upper(sensor_type_name);
  ROS_INFO("%s: %s as input_sensor_type given", name_.c_str(), sensor_type_name.c_str());

  if (sensor_type_name == "VARIABLE")
    input_sensor_type = VARIABLE;
  else if (sensor_type_name == "FIXED")
    input_sensor_type = FIXED;
  else if (sensor_type_name == "ALL")
    input_sensor_type = ALL;
  else
  {
    ROS_ERROR("%s: Invalid input sensor type: %s", name_.c_str(), sensor_type_name.c_str());
  }

  // Validate topic names list: it must be a (normally non-empty) list of strings
  if ((topic_names.valid() == false) || (topic_names.getType() != XmlRpc::XmlRpcValue::TypeArray))
  {
    ROS_ERROR("Invalid topic names list: it must be a non-empty list of strings");
    return;
  }

  if (topic_names.size() < 1)
  {
    // This could be an error, but I keep it as it can be useful for debug
    ROS_WARN("Empty topic names list: range sensor layer will have no effect on costmap");
  }

  // Traverse the topic names list subscribing to all of them with the same callback method
  for (unsigned int i = 0; i < topic_names.size(); i++)
  {
    if (topic_names[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_WARN("Invalid topic names list: element %d is not a string, so it will be ignored", i);
    }
    else
    {
      std::string topic_name(topics_ns);
      if ((topic_name.size() > 0) && (topic_name.at(topic_name.size() - 1) != '/'))
        topic_name += "/";
      topic_name += static_cast<std::string>(topic_names[i]);

      if (input_sensor_type == VARIABLE)
        processRangeMessageFunc_ = boost::bind(&RangeArrayLayer::processVariableRangeMsg, this, _1);
      else if (input_sensor_type == FIXED)
        processRangeMessageFunc_ = boost::bind(&RangeArrayLayer::processFixedRangeMsg, this, _1);
      else if (input_sensor_type == ALL)
        processRangeMessageFunc_ = boost::bind(&RangeArrayLayer::processRangeMsg, this, _1);
      else
      {
        ROS_ERROR(
            "%s: Invalid input sensor type: %s. Did you make a new type and forgot to choose the subscriber for it?",
            name_.c_str(), sensor_type_name.c_str());
      }

      registerSubscriber(topic_name, nh);
    }
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &RangeArrayLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  global_frame_ = layered_costmap_->getGlobalFrameID();

  //Extension part
  XmlRpc::XmlRpcValue layer_update_topics;
  nh.param("layer_update_topics", layer_update_topics, layer_update_topics);
  parseExternalSubscribers(layer_update_topics, nh);
}

void RangeArrayLayer::registerSubscriber(const std::string& topic_name, ros::NodeHandle& nh)
{
  range_subs_.push_back(nh.subscribe(topic_name, 100, &RangeArrayLayer::incomingRangeArray, this));
  ROS_INFO("RangeArrayLayer: subscribed to topic %s", range_subs_.back().getTopic().c_str());
}

void RangeArrayLayer::incomingRangeArray(const gopher_std_msgs::RangesConstPtr& range_array)
{
  boost::mutex::scoped_lock lock(range_message_mutex_);

  for (int i = 0; i < range_array->ranges.size(); ++i)
  {
    range_msgs_buffer_.push_back(range_array->ranges.at(i));
  }
}

void RangeArrayLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                   double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  if (reset_every_cycle_)
  {
    this->costmap_2d::Costmap2D::resetMaps();
  }

  RangeSensorLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

} /* end namespace */
