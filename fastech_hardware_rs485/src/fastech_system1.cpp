#include "fastech_hardware/fastech_system.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

// ===== Linux compatibility for FASTECH Windows-style types =====
#ifndef _WIN32
  #include <cstdint>
  typedef uint8_t  BYTE;
  typedef uint16_t WORD;
  typedef uint32_t DWORD;
  typedef int      BOOL;
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

namespace fastech_hardware
{

static std::wstring to_wstring_ascii(const std::string & s)
{
  return std::wstring(s.begin(), s.end());  // port strings are ASCII
}

static bool is_connect_ok(int ret)
{
  // Some SDKs return FMM_OK(0) for success, others return TRUE(1)
  return (ret == FMM_OK) || (ret == TRUE) || (ret == 1);
}

int FastechSystem::meters_to_pulse(double meters) const
{
  long long p = std::llround(meters * pulses_per_meter_);
  if (p > std::numeric_limits<int>::max()) p = std::numeric_limits<int>::max();
  if (p < std::numeric_limits<int>::min()) p = std::numeric_limits<int>::min();
  return static_cast<int>(p);
}

double FastechSystem::pulse_to_meters(int pulse) const
{
  return static_cast<double>(pulse) / pulses_per_meter_;
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

  // 1) try exactly what user configured
  int ret = try_connect(port_name_);
  if (is_connect_ok(ret)) {
    connected_ = true;
    return true;
  }

  // 2) fallback: if "/dev/ttyUSB0" -> try "ttyUSB0"
  if (port_name_.rfind("/dev/", 0) == 0) {
    const std::string base = port_name_.substr(5); // remove "/dev/"
    ret = try_connect(base);
    if (is_connect_ok(ret)) {
      RCLCPP_WARN(rclcpp::get_logger("fastech_hardware"),
                  "FAS_Connect succeeded using '%s' instead of '%s'",
                  base.c_str(), port_name_.c_str());
      connected_ = true;
      return true;
    }
  } else {
    // 3) fallback: if "ttyUSB0" -> try "/dev/ttyUSB0"
    const std::string dev = "/dev/" + port_name_;
    ret = try_connect(dev);
    if (is_connect_ok(ret)) {
      RCLCPP_WARN(rclcpp::get_logger("fastech_hardware"),
                  "FAS_Connect succeeded using '%s' instead of '%s'",
                  dev.c_str(), port_name_.c_str());
      connected_ = true;
      return true;
    }
  }

  connected_ = false;
  RCLCPP_ERROR(rclcpp::get_logger("fastech_hardware"),
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

hardware_interface::CallbackReturn FastechSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("fastech_hardware"),
                 "This simple package expects exactly 1 joint, but got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto getp = [&](const std::string & key, const std::string & def) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    return (it == info_.hardware_parameters.end()) ? def : it->second;
  };

  port_name_ = getp("port_name", "ttyUSB0");
  port_id_   = std::stoi(getp("port_id", getp("port_no", "1")));
  baudrate_  = static_cast<uint32_t>(std::stoul(getp("baudrate", "115200")));
  slave_id_  = std::stoi(getp("slave_id", "2"));

  pulses_per_rev_          = std::stod(getp("pulses_per_rev", "4000"));
  travel_mm_per_motor_rev_ = std::stod(getp("travel_mm_per_motor_rev", "9.42"));
  velocity_pps_            = static_cast<uint32_t>(std::stoul(getp("velocity_pps", "5000")));

  if (pulses_per_rev_ <= 0.0 || travel_mm_per_motor_rev_ <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("fastech_hardware"),
                 "pulses_per_rev and travel_mm_per_motor_rev must be > 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // pulses/m = (pulses/rev) * (1000 mm/m) / (mm/rev)
  pulses_per_meter_ = pulses_per_rev_ * 1000.0 / travel_mm_per_motor_rev_;

  RCLCPP_INFO(rclcpp::get_logger("fastech_hardware"),
              "Configured: port_name=%s port_id=%d baud=%u slave=%d, ppm=%.3f pulses/m",
              port_name_.c_str(), port_id_, baudrate_, slave_id_, pulses_per_meter_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FastechSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_pos_m_);
  out.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_mps_);
  return out;
}

std::vector<hardware_interface::CommandInterface> FastechSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &cmd_pos_m_);
  return out;
}

hardware_interface::CallbackReturn FastechSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::scoped_lock lk(mtx_);

  if (!connect_locked()) return hardware_interface::CallbackReturn::ERROR;

  int p = 0;
  const int ret = FAS_GetActualPos(
    static_cast<unsigned char>(port_id_),
    static_cast<unsigned char>(slave_id_),
    &p);

  if (ret != FMM_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("fastech_hardware"),
                 "Slave %d not responding (FAS_GetActualPos ret=%d)", slave_id_, ret);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ✅ initialize states/command to current position (prevents sudden motion)
  hw_pos_m_ = pulse_to_meters(p);
  cmd_pos_m_ = hw_pos_m_;
  last_cmd_pulse_ = p;

  // reset lifecycle flags
  need_servo_enable_ = false;
  servo_enabled_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FastechSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::scoped_lock lk(mtx_);
  if (!connect_locked()) return hardware_interface::CallbackReturn::ERROR;

  // ✅ Do NOT call vendor SDK here (can block controller_manager executor)
  need_servo_enable_ = true;
  servo_enabled_ = false;

  // Prevent first-cycle command spam
  last_cmd_pulse_ = meters_to_pulse(cmd_pos_m_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FastechSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::scoped_lock lk(mtx_);
  (void)FAS_MoveStop(static_cast<unsigned char>(port_id_), static_cast<unsigned char>(slave_id_));
  (void)FAS_ServoEnable(static_cast<unsigned char>(port_id_), static_cast<unsigned char>(slave_id_), FALSE);

  need_servo_enable_ = false;
  servo_enabled_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FastechSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::scoped_lock lk(mtx_);
  close_locked();
  need_servo_enable_ = false;
  servo_enabled_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FastechSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::scoped_lock lk(mtx_);
  if (!connected_) return hardware_interface::return_type::ERROR;

  // ✅ enable servo lazily (NOT in on_activate)
  if (need_servo_enable_ && !servo_enabled_) {
    const unsigned char p_id = static_cast<unsigned char>(port_id_);
    const unsigned char s_id = static_cast<unsigned char>(slave_id_);

    (void)FAS_ServoAlarmReset(p_id, s_id);
    const int r = FAS_ServoEnable(p_id, s_id, TRUE);

    if (r == FMM_OK) {
      servo_enabled_ = true;
      need_servo_enable_ = false;
      RCLCPP_INFO(rclcpp::get_logger("fastech_hardware"),
                  "Servo enabled (slave=%d)", slave_id_);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("fastech_hardware"),
                   "FAS_ServoEnable failed ret=%d (slave=%d)", r, slave_id_);
      return hardware_interface::return_type::ERROR;
    }
  }

  int p = 0;
  int v_pps = 0;

  if (FAS_GetActualPos(static_cast<unsigned char>(port_id_), static_cast<unsigned char>(slave_id_), &p) != FMM_OK)
    return hardware_interface::return_type::ERROR;

  if (FAS_GetActualVel(static_cast<unsigned char>(port_id_), static_cast<unsigned char>(slave_id_), &v_pps) != FMM_OK)
    return hardware_interface::return_type::ERROR;

  hw_pos_m_ = pulse_to_meters(p);
  hw_vel_mps_ = pulse_to_meters(v_pps);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FastechSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::scoped_lock lk(mtx_);
  if (!connected_) return hardware_interface::return_type::ERROR;

  // ✅ Don't command motion until servo is enabled
  if (!servo_enabled_) {
    return hardware_interface::return_type::OK;
  }

  const int target = meters_to_pulse(cmd_pos_m_);
  if (target == last_cmd_pulse_)
    return hardware_interface::return_type::OK;

  const unsigned char p = static_cast<unsigned char>(port_id_);
  const unsigned char s = static_cast<unsigned char>(slave_id_);

  int ret = FAS_PositionAbsOverride(p, s, target);
  if (ret != FMM_OK) {
    MOTION_OPTION_EX opt{};
    ret = FAS_MoveSingleAxisAbsPosEx(p, s, target, static_cast<DWORD>(velocity_pps_), &opt);
    if (ret != FMM_OK) return hardware_interface::return_type::ERROR;
  }

  last_cmd_pulse_ = target;
  return hardware_interface::return_type::OK;
}

}  // namespace fastech_hardware

PLUGINLIB_EXPORT_CLASS(fastech_hardware::FastechSystem, hardware_interface::SystemInterface)
