#pragma once

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

// ===== Linux compatibility for FASTECH Windows-style types =====
#ifndef _WIN32
  #ifndef BYTE
    using BYTE  = uint8_t;
  #endif
  #ifndef BOOL
    using BOOL  = int;
  #endif
  #ifndef TRUE
    #define TRUE 1
  #endif
  #ifndef FALSE
    #define FALSE 0
  #endif
#endif
// ===============================================================

// FASTECH Plus-E SDK (Ethernet)
#include "FAS_EziMOTIONPlusE.h"

namespace fastech_hardware
{

class FastechPlusESystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FastechPlusESystem)

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
  enum class CmdMode { POSITION, VELOCITY };

  struct JointCfg
  {
    std::string name;

    // Ethernet connection per joint
    std::string ip{"0.0.0.0"};
    int bd_id{0};                  // unique handle you assign per motor
    std::string comm_type{"tcp"};  // "tcp" or "udp"
    bool connected{false};

    // Scaling
    bool is_linear{false};         // wheels: rotary
    double pulses_per_rev{4000.0};
    double travel_mm_per_motor_rev{0.0};   // optional linear
    double travel_deg_per_motor_rev{360.0};
    double pulses_per_unit{0.0};   // pulses/rad (rotary) or pulses/m (linear)

    // Command mode from URDF interface
    CmdMode cmd_mode{CmdMode::VELOCITY};

    // Motion parameters (Plus-E header uses int for pps/pulse)
    int abspos_velocity_pps{5000}; // used if POSITION mode
    int dir_sign{+1};              // flip direction if needed

    // Deadbands (SDK units)
    int deadband_pulse{0};
    int deadband_pps{0};

    // state (SI)
    double hw_pos{0.0};            // rad (or m)
    double hw_vel{0.0};            // rad/s (or m/s)

    // commands (SI)
    double cmd_pos{0.0};
    double cmd_vel{0.0};

    // internal (SDK units)
    int last_cmd_pulse{std::numeric_limits<int>::min()};
    int last_cmd_pps{0};
    int last_cmd_dir{1};           // CW=1, CCW=0
    bool last_was_stopped{true};

    bool need_servo_enable{true};
    bool servo_enabled{false};

    // throttling
    uint64_t last_warn_ms{0};
  };

  std::mutex mtx_;
  std::vector<JointCfg> joints_;

  // helpers
  static uint64_t steady_ms();
  static bool should_log(uint64_t & last_ms, uint64_t period_ms);
  static bool parse_ip4(const std::string & ip, BYTE out[4]);

  bool connect_all_locked();
  void close_all_locked();

  bool ensure_servo_on_locked(JointCfg & j);
  bool clear_alarm_if_any_locked(JointCfg & j);

  int pos_to_pulse(const JointCfg & j, double pos_si) const;
  double pulse_to_pos(const JointCfg & j, int pulse) const;

  int vel_to_pps(const JointCfg & j, double vel_si) const;

  bool cmd_position_locked(JointCfg & j, int target_pulse);
  bool cmd_velocity_locked(JointCfg & j, int pps, int dir /*CW=1 CCW=0*/);
};

}  // namespace fastech_hardware