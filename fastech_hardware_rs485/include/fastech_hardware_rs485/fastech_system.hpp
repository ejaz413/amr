#pragma once

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace fastech_hardware_rs485
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
  struct JointCfg
  {
    std::string name;
    int slave_id{2};

    // If linear: pos unit = meters, pulses_per_unit = pulses/m
    // If rotary:  pos unit = radians, pulses_per_unit = pulses/rad
    bool is_linear{true};

    double pulses_per_rev{4000.0};

    // Linear axis
    double travel_mm_per_motor_rev{1.0};

    // Rotary axis
    double travel_deg_per_motor_rev{0.0};

    double pulses_per_unit{0.0};   // pulses/m or pulses/rad
    uint32_t velocity_pps{5000};   // pulses/sec for MoveSingleAxisAbsPosEx

    // state/command storage
    double hw_pos{0.0};
    double hw_vel{0.0};
    double cmd_pos{0.0};

    int last_cmd_pulse{std::numeric_limits<int>::min()};
    bool need_servo_enable{true};
    bool servo_enabled{false};

    // log throttling (ms since steady epoch)
    uint64_t last_warn_servo_ms{0};
    uint64_t last_warn_read_ms{0};
    uint64_t last_warn_write_ms{0};
  };

  // connection
  std::string port_name_{"fastech_rs485"};
  int port_id_{0};
  uint32_t baudrate_{115200};
  bool connected_{false};

  std::mutex mtx_;
  std::vector<JointCfg> joints_;

  // helpers
  bool connect_locked();
  void close_locked();

  int pos_to_pulse(const JointCfg & j, double pos_si) const;
  double pulse_to_pos(const JointCfg & j, int pulse) const;
  double pps_to_vel(const JointCfg & j, int pps) const;

  bool try_enable_servo(JointCfg & j);

  static uint64_t steady_ms();
  static bool should_log(uint64_t & last_ms, uint64_t period_ms);
};

}  // namespace fastech_hardware_rs485
