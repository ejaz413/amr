#include "lvs_driver/lvs_node.hpp"

#include "geometry_msgs/msg/point32.hpp"

#include <algorithm>
#include <chrono>
#include <string>

namespace lvs_driver
{

LvsNode::LvsNode(const rclcpp::NodeOptions & options)
: Node("lvs_node", options)
{
  sensor_ip_ = this->declare_parameter<std::string>("sensor_ip", "192.168.0.250");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "lvs_frame");
  publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 30.0);

  points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("lvs/points", 10);
  profile_pub_ = this->create_publisher<lvs_driver::msg::LvsProfile>("lvs/profile", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>("lvs/status", 10);

  backend_ = std::make_unique<OxApiBackend>(sensor_ip_);
  connectSensor();

  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)));

  timer_ = this->create_wall_timer(period, std::bind(&LvsNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(), "lvs_node started");
  RCLCPP_INFO(this->get_logger(), "sensor_ip=%s", sensor_ip_.c_str());
  RCLCPP_INFO(this->get_logger(), "frame_id=%s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "publish_rate_hz=%.3f", publish_rate_hz_);
}

bool LvsNode::connectSensor()
{
  std_msgs::msg::String status;

  if (!backend_->connect()) {
    status.data = "connect_failed";
    status_pub_->publish(status);
    RCLCPP_ERROR(this->get_logger(), "Failed to connect OxApi sensor");
    return false;
  }

  status.data = "connected";
  status_pub_->publish(status);
  RCLCPP_INFO(this->get_logger(), "OxApi sensor connected");
  return true;
}

void LvsNode::onTimer()
{
  if (!backend_ || !backend_->isConnected()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Sensor not connected");
    return;
  }

  ProfileData profile;
  if (!backend_->getProfile(profile)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to read profile");
    return;
  }

  publishProfile(profile);
}

void LvsNode::publishProfile(const ProfileData & profile)
{
  sensor_msgs::msg::PointCloud cloud_msg;
  cloud_msg.header.stamp = this->now();
  cloud_msg.header.frame_id = frame_id_;
  cloud_msg.points.reserve(profile.points.size());

  lvs_driver::msg::LvsProfile profile_msg;
  profile_msg.stamp = cloud_msg.header.stamp;
  profile_msg.frame_id = frame_id_;
  profile_msg.sensor_timestamp = profile.sensor_timestamp;
  profile_msg.precision = profile.precision;
  profile_msg.valid = profile.valid;

  profile_msg.x_mm.reserve(profile.points.size());
  profile_msg.z_mm.reserve(profile.points.size());
  profile_msg.intensity.reserve(profile.points.size());

  for (const auto & pt : profile.points) {
    geometry_msgs::msg::Point32 p;
    p.x = pt.x_mm / 1000.0f;
    p.y = 0.0f;
    p.z = pt.z_mm / 1000.0f;
    cloud_msg.points.push_back(p);

    profile_msg.x_mm.push_back(pt.x_mm);
    profile_msg.z_mm.push_back(pt.z_mm);
    profile_msg.intensity.push_back(pt.intensity);
  }

  points_pub_->publish(cloud_msg);
  profile_pub_->publish(profile_msg);
}

}  // namespace lvs_driver