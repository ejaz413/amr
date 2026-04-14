#include "fastech_hardware/fastech_plus_e_system.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <chrono>
#include <cstring>
#include <sstream>
#include <unistd.h> // usleep

using namespace PE;

namespace fastech_hardware
{

static constexpr int DIR_CW  = 1;
static constexpr int DIR_CCW = 0;

static void log_axis_bits(const std::string & joint, int bd_id, uint32_t status_dw)
{
  // NOTE: Exact bit meanings depend on the vendor header.
  // We print the raw hex plus common named bitfields (if available in your header).
  EZISERVO2_AXISSTATUS st{};
  st.dwValue = status_dw;

  RCLCPP_WARN(rclcpp::get_logger("fastech_plus_e"),
              "AxisStatus joint=%s bd_id=%d dw=0x%08X "
              "(SERVOON=%d ERRORALL=%d MOTIONING=%d)",
              joint.c_str(), bd_id, status_dw,
              (int)st.FFLAG_SERVOON,
              (int)st.FFLAG_ERRORALL,
              (int)st.FFLAG_MOTIONING);
}

uint64_t FastechPlusESystem::steady_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

bool FastechPlusESystem::should_log(uint64_t & last_ms, uint64_t period_ms)
{
  const uint64_t now = steady_ms();
  if (last_ms == 0 || (now - last_ms) >= period_ms) { last_ms = now; return true; }
  return false;
}

bool FastechPlusESystem::parse_ip4(const std::string & ip, BYTE out[4])
{
  int a=0,b=0,c=0,d=0; char dot=0;
  std::stringstream ss(ip);
  if (!(ss >> a >> dot) || dot!='.') return false;
  if (!(ss >> b >> dot) || dot!='.') return false;
  if (!(ss >> c >> dot) || dot!='.') return false;
  if (!(ss >> d)) return false;
  if (a<0||a>255||b<0||b>255||c<0||c>255||d<0||d>255) return false;
  out[0]=static_cast<BYTE>(a);
  out[1]=static_cast<BYTE>(b);
  out[2]=static_cast<BYTE>(c);
  out[3]=static_cast<BYTE>(d);
  return true;
}

hardware_interface::CallbackReturn FastechPlusESystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  joints_.clear();
  joints_.reserve(info_.joints.size());

  for (const auto & ji : info_.joints) {
    JointCfg j;
    j.name = ji.name;

    auto get_j = [&](const std::string & key, const std::string & def) -> std::string {
      auto it = ji.parameters.find(key);
      return (it == ji.parameters.end()) ? def : it->second;
    };

    j.ip        = get_j("ip", "");
    j.bd_id     = std::stoi(get_j("bd_id", "0"));
    j.comm_type = get_j("comm_type", "tcp");
    std::transform(j.comm_type.begin(), j.comm_type.end(), j.comm_type.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c)); });

    j.abspos_velocity_pps = std::stoi(get_j("abspos_velocity_pps", get_j("velocity_pps", "5000")));
    j.dir_sign  = std::stoi(get_j("dir_sign", "1"));

    j.pulses_per_rev = std::stod(get_j("pulses_per_rev", "4000"));

    const std::string deg_str = get_j("travel_deg_per_motor_rev", "");
    const std::string mm_str  = get_j("travel_mm_per_motor_rev", "");

    if (!mm_str.empty()) {
      j.is_linear = true;
      j.travel_mm_per_motor_rev = std::stod(mm_str);
      if (j.travel_mm_per_motor_rev <= 0.0) return hardware_interface::CallbackReturn::ERROR;
      j.pulses_per_unit = j.pulses_per_rev * 1000.0 / j.travel_mm_per_motor_rev;
    } else {
      j.is_linear = false;
      j.travel_deg_per_motor_rev = deg_str.empty() ? 360.0 : std::stod(deg_str);
      if (j.travel_deg_per_motor_rev <= 0.0) return hardware_interface::CallbackReturn::ERROR;
      const double rad_per_rev = j.travel_deg_per_motor_rev * M_PI / 180.0;
      j.pulses_per_unit = j.pulses_per_rev / rad_per_rev;
    }

    bool has_vel = false;
    bool has_pos = false;
    for (const auto & ci : ji.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_VELOCITY) has_vel = true;
      if (ci.name == hardware_interface::HW_IF_POSITION) has_pos = true;
    }
    j.cmd_mode = has_vel ? CmdMode::VELOCITY : CmdMode::POSITION;
    if (!has_vel && !has_pos) return hardware_interface::CallbackReturn::ERROR;

    j.deadband_pulse = std::stoi(get_j("deadband_pulse", "0"));
    j.deadband_pps   = std::stoi(get_j("deadband_pps", "0"));

    if (j.ip.empty()) return hardware_interface::CallbackReturn::ERROR;

    joints_.push_back(j);
  }

  RCLCPP_INFO(rclcpp::get_logger("fastech_plus_e"), "Configured Plus-E joints=%zu", joints_.size());
  for (const auto & j : joints_) {
    RCLCPP_INFO(rclcpp::get_logger("fastech_plus_e"),
                "  joint=%s ip=%s bd_id=%d comm=%s cmd_mode=%s pulses_per_unit=%.3f",
                j.name.c_str(), j.ip.c_str(), j.bd_id, j.comm_type.c_str(),
                (j.cmd_mode == CmdMode::VELOCITY ? "velocity" : "position"),
                j.pulses_per_unit);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FastechPlusESystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(joints_.size() * 2);
  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.hw_pos);
    out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.hw_vel);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> FastechPlusESystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(joints_.size());
  for (auto & j : joints_) {
    if (j.cmd_mode == CmdMode::VELOCITY) {
      out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.cmd_vel);
    } else {
      out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.cmd_pos);
    }
  }
  return out;
}

bool FastechPlusESystem::connect_all_locked()
{
  for (auto & j : joints_) {
    if (j.connected) continue;

    BYTE ipb[4]{0,0,0,0};
    if (!parse_ip4(j.ip, ipb)) return false;

    int ok = 0;
    if (j.comm_type == "udp") {
      ok = FAS_Connect(ipb[0], ipb[1], ipb[2], ipb[3], j.bd_id);
    } else {
      ok = FAS_ConnectTCP(ipb[0], ipb[1], ipb[2], ipb[3], j.bd_id);
    }

    if (ok == 0) {
      RCLCPP_ERROR(rclcpp::get_logger("fastech_plus_e"),
                   "Connect failed joint=%s ip=%s bd_id=%d comm=%s",
                   j.name.c_str(), j.ip.c_str(), j.bd_id, j.comm_type.c_str());
      return false;
    }

    j.connected = true;
    j.need_servo_enable = true;
    j.servo_enabled = false;
    j.last_was_stopped = true;
    j.last_cmd_pps = 0;
    j.last_cmd_dir = DIR_CW;
  }
  return true;
}

void FastechPlusESystem::close_all_locked()
{
  for (auto & j : joints_) {
    if (j.connected) {
      FAS_Close(j.bd_id);
      j.connected = false;
    }
  }
}

bool FastechPlusESystem::clear_alarm_if_any_locked(JointCfg & j)
{
  EZISERVO2_AXISSTATUS st{};
  const int rs = FAS_GetAxisStatus(j.bd_id, &st.dwValue);
  if (rs != FMM_OK) return false;

  if (st.FFLAG_ERRORALL) {
    const int rr = FAS_ServoAlarmReset(j.bd_id);
    if (rr != FMM_OK) {
      if (should_log(j.last_warn_ms, 2000)) {
        RCLCPP_WARN(rclcpp::get_logger("fastech_plus_e"),
                    "ServoAlarmReset failed joint=%s bd_id=%d ret=%d status=0x%08X",
                    j.name.c_str(), j.bd_id, rr, (unsigned)st.dwValue);
      }
      return false;
    }
    // After reset, give drive a moment
    usleep(50 * 1000);
  }

  return true;
}

bool FastechPlusESystem::ensure_servo_on_locked(JointCfg & j)
{
  if (!j.need_servo_enable && j.servo_enabled) return true;

  (void)clear_alarm_if_any_locked(j);

  EZISERVO2_AXISSTATUS st{};
  int rs = FAS_GetAxisStatus(j.bd_id, &st.dwValue);
  if (rs != FMM_OK) return false;

  // Try enabling servo repeatedly for up to 2 seconds
  if (!st.FFLAG_SERVOON) {
    const int ret = FAS_ServoEnable(j.bd_id, TRUE);
    if (ret != FMM_OK && should_log(j.last_warn_ms, 2000)) {
      RCLCPP_WARN(rclcpp::get_logger("fastech_plus_e"),
                  "ServoEnable call failed joint=%s bd_id=%d ret=%d status=0x%08X",
                  j.name.c_str(), j.bd_id, ret, (unsigned)st.dwValue);
    }

    for (int i = 0; i < 2000; ++i) { // 2000ms
      usleep(1000);
      rs = FAS_GetAxisStatus(j.bd_id, &st.dwValue);
      if (rs == FMM_OK && st.FFLAG_SERVOON) break;
    }
  }

  if (st.FFLAG_SERVOON) {
    j.servo_enabled = true;
    j.need_servo_enable = false;
    return true;
  }

  // Still not ON -> print raw + common flags
  if (should_log(j.last_warn_ms, 2000)) {
    RCLCPP_WARN(rclcpp::get_logger("fastech_plus_e"),
                "Servo did not turn ON joint=%s bd_id=%d status=0x%08X",
                j.name.c_str(), j.bd_id, (unsigned)st.dwValue);
    log_axis_bits(j.name, j.bd_id, st.dwValue);
  }

  return false;
}

int FastechPlusESystem::pos_to_pulse(const JointCfg & j, double pos_si) const
{
  long long p = llround(pos_si * j.pulses_per_unit);
  if (p > std::numeric_limits<int>::max()) p = std::numeric_limits<int>::max();
  if (p < std::numeric_limits<int>::min()) p = std::numeric_limits<int>::min();
  return static_cast<int>(p);
}

double FastechPlusESystem::pulse_to_pos(const JointCfg & j, int pulse) const
{
  return static_cast<double>(pulse) / j.pulses_per_unit;
}

int FastechPlusESystem::vel_to_pps(const JointCfg & j, double vel_si) const
{
  const double pps_f = std::abs(vel_si) * j.pulses_per_unit;
  const double clamped = std::min(pps_f, static_cast<double>(std::numeric_limits<int>::max()));
  return static_cast<int>(llround(clamped));
}

bool FastechPlusESystem::cmd_position_locked(JointCfg & j, int target_pulse)
{
  int ret = FAS_PositionAbsOverride(j.bd_id, target_pulse);
  if (ret == FMM_OK) return true;

  MOTION_OPTION_EX opt{};
  ret = FAS_MoveSingleAxisAbsPosEx(j.bd_id, target_pulse, j.abspos_velocity_pps, &opt);
  return (ret == FMM_OK);
}

bool FastechPlusESystem::cmd_velocity_locked(JointCfg & j, int pps, int dir)
{
  auto log_status = [&](const char* where, int ret) {
    if (!should_log(j.last_warn_ms, 2000)) return;
    EZISERVO2_AXISSTATUS st{};
    const int rs = FAS_GetAxisStatus(j.bd_id, &st.dwValue);
    RCLCPP_WARN(rclcpp::get_logger("fastech_plus_e"),
      "%s failed joint=%s bd_id=%d pps=%d dir=%d ret=%d axis_ret=%d status=0x%08X",
      where, j.name.c_str(), j.bd_id, pps, dir, ret, rs, (unsigned)st.dwValue);
  };

  if (pps <= 0) {
    const int ret = FAS_MoveStop(j.bd_id);
    if (ret != FMM_OK) log_status("MoveStop", ret);
    j.last_was_stopped = true;
    j.last_cmd_pps = 0;
    return (ret == FMM_OK);
  }

  if (pps < 1) pps = 1;

  if (j.last_was_stopped || dir != j.last_cmd_dir) {
    const int ret = FAS_MoveVelocity(j.bd_id, pps, dir);
    if (ret != FMM_OK) {
      log_status("MoveVelocity", ret);
      return false;
    }
    j.last_was_stopped = false;
    j.last_cmd_pps = pps;
    j.last_cmd_dir = dir;
    return true;
  }

  int ret = FAS_VelocityOverride(j.bd_id, pps);
  if (ret == FMM_OK) {
    j.last_cmd_pps = pps;
    return true;
  }

  ret = FAS_MoveVelocity(j.bd_id, pps, dir);
  if (ret != FMM_OK) {
    log_status("VelOverride+MoveVelocity", ret);
    return false;
  }

  j.last_cmd_pps = pps;
  return true;
}

hardware_interface::CallbackReturn FastechPlusESystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::scoped_lock lk(mtx_);
  if (!connect_all_locked()) return hardware_interface::CallbackReturn::ERROR;

  for (auto & j : joints_) {
    int p = 0;
    int v = 0;

    (void)FAS_GetActualPos(j.bd_id, &p);
    (void)FAS_GetActualVel(j.bd_id, &v);

    j.hw_pos = pulse_to_pos(j, p);
    j.hw_vel = static_cast<double>(v) / j.pulses_per_unit;

    j.last_cmd_pulse = p;
    if (j.cmd_mode == CmdMode::POSITION) j.cmd_pos = j.hw_pos;
    if (j.cmd_mode == CmdMode::VELOCITY) j.cmd_vel = 0.0;

    j.need_servo_enable = true;
    j.servo_enabled = false;
    j.last_was_stopped = true;
    j.last_cmd_pps = 0;
    j.last_cmd_dir = DIR_CW;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FastechPlusESystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::scoped_lock lk(mtx_);
  if (!connect_all_locked()) return hardware_interface::CallbackReturn::ERROR;
  for (auto & j : joints_) {
    j.need_servo_enable = true;
    j.servo_enabled = false;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FastechPlusESystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  std::scoped_lock lk(mtx_);
  for (auto & j : joints_) {
    if (j.connected) {
      (void)FAS_MoveStop(j.bd_id);
      (void)FAS_ServoEnable(j.bd_id, FALSE);
      j.servo_enabled = false;
      j.need_servo_enable = true;
    }
  }
  close_all_locked();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FastechPlusESystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::scoped_lock lk(mtx_);
  close_all_locked();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FastechPlusESystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::scoped_lock lk(mtx_);
  for (auto & j : joints_) {
    if (!j.connected) continue;

    int p = 0;
    int v = 0;

    const int r1 = FAS_GetActualPos(j.bd_id, &p);
    const int r2 = FAS_GetActualVel(j.bd_id, &v);

    if (r1 == FMM_OK) j.hw_pos = pulse_to_pos(j, p);
    if (r2 == FMM_OK) j.hw_vel = static_cast<double>(v) / j.pulses_per_unit;

    if ((r1 != FMM_OK || r2 != FMM_OK) && should_log(j.last_warn_ms, 2000)) {
      RCLCPP_WARN(rclcpp::get_logger("fastech_plus_e"),
                  "Read warning joint=%s bd_id=%d (pos_ret=%d vel_ret=%d)",
                  j.name.c_str(), j.bd_id, r1, r2);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FastechPlusESystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::scoped_lock lk(mtx_);

  for (auto & j : joints_) {
    if (!j.connected) continue;

    if (!ensure_servo_on_locked(j)) {
      continue;
    }

    if (j.cmd_mode == CmdMode::POSITION) {
      const int target = pos_to_pulse(j, j.cmd_pos);
      if (j.last_cmd_pulse != std::numeric_limits<int>::min()) {
        const int delta = std::abs(target - j.last_cmd_pulse);
        if ((j.deadband_pulse > 0 && delta < j.deadband_pulse) ||
            (j.deadband_pulse == 0 && target == j.last_cmd_pulse)) {
          continue;
        }
      }
      if (cmd_position_locked(j, target)) j.last_cmd_pulse = target;

    } else {
      const double signed_vel = j.cmd_vel * static_cast<double>(j.dir_sign);
      const int pps = vel_to_pps(j, signed_vel);
      const int dir = (signed_vel >= 0.0) ? DIR_CW : DIR_CCW;

      const int dpps = std::abs(pps - j.last_cmd_pps);
      if (j.deadband_pps > 0 && dpps < j.deadband_pps && dir == j.last_cmd_dir) {
        continue;
      }

      (void)cmd_velocity_locked(j, pps, dir);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace fastech_hardware

PLUGINLIB_EXPORT_CLASS(fastech_hardware::FastechPlusESystem, hardware_interface::SystemInterface)