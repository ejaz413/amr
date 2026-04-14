#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <mutex>
#include <string>
#include <vector>
#include <limits>

namespace fastech_hardware
{

class FastechSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FastechSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // helpers
  bool connect_locked();
  void close_locked();
  int meters_to_pulse(double meters) const;
  double pulse_to_meters(int pulse) const;

  // thread safety
  std::mutex mtx_;

  // connection/config
  bool connected_{false};

  std::string port_name_{"ttyUSB1"};
  int port_id_{0};                // FASTECH port ID (often 1)
  uint32_t baudrate_{115200};
  int slave_id_{2};

  // kinematics/conversion
  double pulses_per_rev_{4000.0};
  double travel_mm_per_motor_rev_{9.42};
  double pulses_per_meter_{1.0};

  // motion params
  uint32_t velocity_pps_{5000};

  // state/command
  double hw_pos_m_{0.0};
  double hw_vel_mps_{0.0};
  double cmd_pos_m_{0.0};

  int last_cmd_pulse_{std::numeric_limits<int>::min()};

  // lifecycle flags
  bool need_servo_enable_{false};
  bool servo_enabled_{false};
};

}  // namespace fastech_hardware
