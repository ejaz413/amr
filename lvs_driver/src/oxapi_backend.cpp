#include "lvs_driver/oxapi_backend.hpp"

#include <algorithm>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#ifdef LVS_USE_OXAPI
#include <OXApi/Ox.h>
#endif

namespace lvs_driver
{

#ifdef LVS_USE_OXAPI

class OxApiBackend::Impl
{
public:
  Impl() = default;

  ~Impl()
  {
    disconnect();
  }

  bool connect(const std::string & ip)
  {
    disconnect();

    try {
      ox_ = Baumer::OXApi::Ox::Create(ip);
      if (!ox_) {
        return false;
      }

      ox_->Connect();
      connected_ = true;
      return true;
    } catch (const std::exception &) {
      disconnect();
      return false;
    }
  }

  void disconnect()
  {
    if (ox_) {
      try {
        ox_->Disconnect();
      } catch (...) {
      }
      ox_.reset();
    }
    connected_ = false;
  }

  bool isConnected() const
  {
    return connected_ && static_cast<bool>(ox_);
  }

  bool getProfile(ProfileData & profile)
  {
    if (!isConnected()) {
      return false;
    }

    if (readIntensityProfile(profile)) {
      return true;
    }

    return readPlainProfile(profile);
  }

private:
  bool readPlainProfile(ProfileData & profile)
  {
    try {
      auto p = ox_->GetProfile();

      profile.points.clear();
      profile.points.reserve(static_cast<std::size_t>(p.Length));
      profile.sensor_timestamp = static_cast<uint64_t>(p.TimeStamp);
      profile.precision = static_cast<uint32_t>(p.Precision);
      profile.valid = true;

      const std::size_t n = std::min(
        static_cast<std::size_t>(p.Length),
        std::min(p.X.size(), p.Z.size()));

      for (std::size_t i = 0; i < n; ++i) {
        ProfilePoint pt;
        pt.x_mm = static_cast<float>(p.X.at(i)) / static_cast<float>(profile.precision);
        pt.z_mm = static_cast<float>(p.Z.at(i)) / static_cast<float>(profile.precision);
        pt.intensity = 0;
        profile.points.push_back(pt);
      }

      return !profile.points.empty();
    } catch (const std::exception &) {
      return false;
    }
  }

  bool readIntensityProfile(ProfileData & profile)
  {
    try {
      auto p = ox_->GetIntensityProfile();

      profile.points.clear();
      profile.points.reserve(static_cast<std::size_t>(p.Length));
      profile.sensor_timestamp = static_cast<uint64_t>(p.TimeStamp);
      profile.precision = static_cast<uint32_t>(p.Precision);
      profile.valid = true;

      const std::size_t n = std::min(
        static_cast<std::size_t>(p.Length),
        std::min(std::min(p.X.size(), p.Z.size()), p.I.size()));

      for (std::size_t i = 0; i < n; ++i) {
        ProfilePoint pt;
        pt.x_mm = static_cast<float>(p.X.at(i)) / static_cast<float>(profile.precision);
        pt.z_mm = static_cast<float>(p.Z.at(i)) / static_cast<float>(profile.precision);
        pt.intensity = static_cast<uint32_t>(p.I.at(i));
        profile.points.push_back(pt);
      }

      return !profile.points.empty();
    } catch (const std::exception &) {
      return false;
    }
  }

private:
  using OxHandle = decltype(Baumer::OXApi::Ox::Create(std::declval<std::string>()));
  OxHandle ox_ {};
  bool connected_ {false};
};

#endif

OxApiBackend::OxApiBackend(const std::string & sensor_ip)
: sensor_ip_(sensor_ip)
#ifdef LVS_USE_OXAPI
, impl_(std::make_unique<Impl>())
#endif
{
}

OxApiBackend::~OxApiBackend()
{
  disconnect();
}

bool OxApiBackend::connect()
{
#ifdef LVS_USE_OXAPI
  if (!impl_) {
    connected_ = false;
    return false;
  }

  connected_ = impl_->connect(sensor_ip_);
  return connected_;
#else
  connected_ = false;
  return false;
#endif
}

void OxApiBackend::disconnect()
{
#ifdef LVS_USE_OXAPI
  if (impl_) {
    impl_->disconnect();
  }
#endif
  connected_ = false;
}

bool OxApiBackend::isConnected() const
{
#ifdef LVS_USE_OXAPI
  return impl_ && impl_->isConnected();
#else
  return false;
#endif
}

bool OxApiBackend::getProfile(ProfileData & profile)
{
#ifdef LVS_USE_OXAPI
  if (!impl_) {
    return false;
  }
  return impl_->getProfile(profile);
#else
  (void)profile;
  return false;
#endif
}

}  // namespace lvs_driver