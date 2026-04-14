#include "fastech_hardware_rs485/fastech_system.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <cmath>
#include <chrono>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

// ===== Linux compatibility for FASTECH Windows-style types =====
#ifndef _WIN32
  #include <cstdint>
  #ifndef BYTE
    using BYTE  = uint8_t;
  #endif
  #ifndef WORD
    using WORD  = uint16_t;
  #endif
  #ifndef DWORD
    using DWORD = uint32_t;
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

// FASTECH SDK
#include "FAS_EziMOTIONPlusR.h"

namespace fastech_hardware_rs485
{

static std::wstring to_wstring_ascii(const std::string & s)
{
  return std::wstring(s.begin(), s.end());
}

static bool is_connect_ok(int ret)
{
  return (ret == FMM_OK) || (ret == TRUE) || (ret == 1);
}

uint64_t FastechSystem::steady_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

bool FastechSystem::should_log(uint64_t & last_ms, uint64_t period_ms)
{
  const uint64_t now = steady_ms();
  if (last_ms == 0 || (now - last_ms) >= period_ms) {
    last_ms = now;
    return true;
  }
  return false;
}

int FastechSystem::pos_to_pulse(const JointCfg & j, double pos_si) const
{
  long long p = std::llround(pos_si * j.pulses_per_unit);
  if (p > std::numeric_limits<int>::max()) p = std::numeric_limits<int>::max();
  if (p < std::numeric_limits<int>::min()) p = std::numeric_limits<int>::min();
  return static_cast<int>(p);
}

double FastechSystem::pulse_to_pos(const JointCfg & j, int pulse) const
{
  return static_cast<double>(pulse) / j.pulses_per_unit;
}

double FastechSystem::pps_to_vel(const JointCfg & j, int pps) const
{
  return static_cast<double>(pps) / j.pulses_per_unit;
}

bool FastechSystem::connect_locked()
{
  if (connected_) return true;

  const unsigned int baud = static_cast<unsigned int>(baudrate_);
  const unsigned char pid = static_cast<unsigned char>(port_id_);

  auto try_connect = [&](const std::string & port_str) -> int {
    const std::wstring wport = to_wstring_ascii(port_str);
    return FAS_Connect(wport.c_str(), baud, pid);
  };

  int ret = try_connect(port_name_);
  if (is_connect_ok(ret)) {
    connected_ = true;
    return true;
  }

  if (port_name_.rfind("/dev/", 0) == 0) {
    const std::string base = port_name_.substr(5);
    ret = try_connect(base);
    if (is_connect_ok(ret)) {
      RCLCPP_WARN(rclcpp::get_logger("fastech_hardware_rs485"),
                  "FAS_Connect succeeded using '%s' instead of '%s'",
                  base.c_str(), port_name_.c_str());
      connected_ = true;
      return true;
    }
  } else {
    const std::string dev = "/dev/" + port_name_;
    ret = try_connect(dev);
    if (is_connect_ok(ret)) {
      RCLCPP_WARN(rclcpp::get_logger("fastech_hardware_rs485"),
                  "FAS_Connect succeeded using '%s' instead of '%s'",
                  dev.c_str(), port_name_.c_str());
      connected_ = true;
      return true;
    }
  }

  connected_ = false;
  RCLCPP_ERROR(rclcpp::get_logger("fastech_hardware_rs485"),
               "FAS_Connect failed (port_name=%s baud=%u port_id=%d). Last ret=%d (FMM_OK=%d)",
               port_name_.c_str(), baudrate_, port_id_, ret, FMM_OK);
  return false;
}

void FastechSystem::close_locked()
{
  if (!connected_) return;
  FAS_Close(static_cast<unsigned char>(port_id_));
  connected_ = false;
}

bool FastechSystem::try_enable_servo(JointCfg & j)
{
  const unsigned char p = static_cast<unsigned char>(port_id_);
  const unsigned char s = static_cast<unsigned char>(j.slave_id);

  (void)FAS_ServoAlarmReset(p, s);

  const int ret = FAS_ServoEnable(p, s, TRUE);
  if (ret == FMM_OK) {
    j.servo_enabled = true;
    j.need_servo_enable = false;
    return true;
  }

  if (should_log(j.last_warn_servo_ms, 2000)) {
    RCLCPP_WARN(rclcpp::get_logger("fastech_hardware_rs485"),
                "ServoEnable failed for joint=%s slave=%d (ret=%d). Will retry.",
                j.name.c_str(), j.slave_id, ret);
  }
  return false;
}

hardware_interface::CallbackReturn FastechSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  auto get_hw = [&](const std::string & key, const std::string & def) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    return (it == info_.hardware_parameters.end()) ? def : it->second;
  };

  port_name_ = get_hw("port_name", "ttyUSB0");
  port_id_   = std::stoi(get_hw("port_id", get_hw("port_no", "1")));
  baudrate_  = static_cast<uint32_t>(std::stoul(get_hw("baudrate", "115200")));

  joints_.clear();
  joints_.reserve(info_.joints.size());

  for (const auto & ji : info_.joints) {
    JointCfg j;
    j.name = ji.name;

    auto get_j = [&](const std::string & key, const std::string & def) -> std::string {
      auto it = ji.parameters.find(key);
      return (it == ji.parameters.end()) ? def : it->second;
    };

    j.slave_id       = std::stoi(get_j("slave_id", "2"));
    j.pulses_per_rev = std::stod(get_j("pulses_per_rev", "4000"));
    j.velocity_pps   = static_cast<uint32_t>(std::stoul(get_j("velocity_pps", "5000")));

    // rotary if travel_deg_per_motor_rev is provided
    const std::string deg_str = get_j("travel_deg_per_motor_rev", "");
    if (!deg_str.empty()) {
      j.is_linear = false;
      j.travel_deg_per_motor_rev = std::stod(deg_str);

      if (j.pulses_per_rev <= 0.0 || j.travel_deg_per_motor_rev <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("fastech_hardware_rs485"),
                     "Joint %s: pulses_per_rev and travel_deg_per_motor_rev must be > 0",
                     j.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      const double rad_per_motor_rev = j.travel_deg_per_motor_rev * M_PI / 180.0;
      j.pulses_per_unit = j.pulses_per_rev / rad_per_motor_rev; // pulses/rad
    } else {
      j.is_linear = true;
      j.travel_mm_per_motor_rev = std::stod(get_j("travel_mm_per_motor_rev", "1.0"));

      if (j.pulses_per_rev <= 0.0 || j.travel_mm_per_motor_rev <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("fastech_hardware_rs485"),
                     "Joint %s: pulses_per_rev and travel_mm_per_motor_rev must be > 0",
                     j.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      j.pulses_per_unit = j.pulses_per_rev * 1000.0 / j.travel_mm_per_motor_rev; // pulses/m
    }

    joints_.push_back(j);
  }

  RCLCPP_INFO(rclcpp::get_logger("fastech_hardware_rs485"),
              "Configured: port_name=%s port_id=%d baud=%u joints=%zu",
              port_name_.c_str(), port_id_, baudrate_, joints_.size());

  for (const auto & j : joints_) {
    RCLCPP_INFO(rclcpp::get_logger("fastech_hardware_rs485"),
                "  joint=%s slave=%d type=%s pulses_per_unit=%.3f (%s)",
                j.name.c_str(), j.slave_id,
                j.is_linear ? "linear(m)" : "rotary(rad)",
                j.pulses_per_unit,
                j.is_linear ? "pulses/m" : "pulses/rad");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FastechSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(joints_.size() * 2);

  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.hw_pos);
    out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.hw_vel);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> FastechSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(joints_.size());

  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.cmd_pos);
  }
  return out;
}

hardware_interface::CallbackReturn FastechSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::scoped_lock lk(mtx_);
  if (!connect_locked()) return hardware_interface::CallbackReturn::ERROR;

  for (auto & j : joints_) {
    int p = 0;
    const int ret = FAS_GetActualPos(
      static_cast<unsigned char>(port_id_),
      static_cast<unsigned char>(j.slave_id),
      &p);

    if (ret != FMM_OK) {
      RCLCPP_ERROR(rclcpp::get_logger("fastech_hardware_rs485"),
                   "Slave %d not responding for joint %s (GetActualPos ret=%d)",
                   j.slave_id, j.name.c_str(), ret);
      return hardware_interface::CallbackReturn::ERROR;
    }

    j.hw_pos = pulse_to_pos(j, p);
    j.cmd_pos = j.hw_pos;
    j.last_cmd_pulse = p;
    j.need_servo_enable = true;
    j.servo_enabled = false;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FastechSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::scoped_lock lk(mtx_);
  if (!connect_locked()) return hardware_interface::CallbackReturn::ERROR;

  for (auto & j : joints_) {
    j.need_servo_enable = true;
    j.servo_enabled = false;
    j.last_cmd_pulse = pos_to_pulse(j, j.cmd_pos);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FastechSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::scoped_lock lk(mtx_);

  const unsigned char p = static_cast<unsigned char>(port_id_);
  for (auto & j : joints_) {
    const unsigned char s = static_cast<unsigned char>(j.slave_id);
    (void)FAS_MoveStop(p, s);
    (void)FAS_ServoEnable(p, s, FALSE);
    j.servo_enabled = false;
    j.need_servo_enable = true;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FastechSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::scoped_lock lk(mtx_);
  close_locked();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FastechSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::scoped_lock lk(mtx_);
  if (!connected_) return hardware_interface::return_type::ERROR;

  const unsigned char p = static_cast<unsigned char>(port_id_);

  for (auto & j : joints_) {
    if (j.need_servo_enable && !j.servo_enabled) {
      (void)try_enable_servo(j);
    }

    int pos_pulse = 0;
    int vel_pps = 0;

    const int r1 = FAS_GetActualPos(p, static_cast<unsigned char>(j.slave_id), &pos_pulse);
    const int r2 = FAS_GetActualVel(p, static_cast<unsigned char>(j.slave_id), &vel_pps);

    if (r1 == FMM_OK) j.hw_pos = pulse_to_pos(j, pos_pulse);
    if (r2 == FMM_OK) j.hw_vel = pps_to_vel(j, vel_pps);

    if (r1 != FMM_OK || r2 != FMM_OK) {
      if (should_log(j.last_warn_read_ms, 2000)) {
        RCLCPP_WARN(rclcpp::get_logger("fastech_hardware_rs485"),
                    "Read warning joint=%s slave=%d (pos_ret=%d vel_ret=%d)",
                    j.name.c_str(), j.slave_id, r1, r2);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FastechSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::scoped_lock lk(mtx_);
  if (!connected_) return hardware_interface::return_type::ERROR;

  const unsigned char p = static_cast<unsigned char>(port_id_);

  for (auto & j : joints_) {
    if (j.need_servo_enable && !j.servo_enabled) {
      if (!try_enable_servo(j)) {
        continue; // don't command until servo is enabled
      }
    }

    const int target = pos_to_pulse(j, j.cmd_pos);
    if (j.last_cmd_pulse != std::numeric_limits<int>::min() && target == j.last_cmd_pulse) {
      continue;
    }

    const unsigned char s = static_cast<unsigned char>(j.slave_id);

    int ret = FAS_PositionAbsOverride(p, s, target);
    if (ret != FMM_OK) {
      MOTION_OPTION_EX opt{};
      ret = FAS_MoveSingleAxisAbsPosEx(p, s, target, static_cast<DWORD>(j.velocity_pps), &opt);
      if (ret != FMM_OK) {
        if (should_log(j.last_warn_write_ms, 2000)) {
          RCLCPP_WARN(rclcpp::get_logger("fastech_hardware_rs485"),
                      "Write failed joint=%s slave=%d ret=%d",
                      j.name.c_str(), j.slave_id, ret);
        }
        continue;
      }
    }

    j.last_cmd_pulse = target;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace fastech_hardware_rs485

PLUGINLIB_EXPORT_CLASS(fastech_hardware_rs485::FastechSystem, hardware_interface::SystemInterface)
