#pragma once

#include "lvs_driver/msg/lvs_profile.hpp"
#include "lvs_driver/oxapi_backend.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>
#include <string>

namespace lvs_driver
{

class LvsNode : public rclcpp::Node
{
public:
  explicit LvsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onTimer();
  bool connectSensor();
  void publishProfile(const ProfileData & profile);

private:
  std::string sensor_ip_;
  std::string frame_id_;
  double publish_rate_hz_ {30.0};

  std::unique_ptr<OxApiBackend> backend_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr points_pub_;
  rclcpp::Publisher<lvs_driver::msg::LvsProfile>::SharedPtr profile_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

}  // namespace lvs_driver