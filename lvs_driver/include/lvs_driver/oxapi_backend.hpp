#pragma once

#include "lvs_driver/types.hpp"

#include <memory>
#include <string>

namespace lvs_driver
{

class OxApiBackend : public ILvsBackend
{
public:
  explicit OxApiBackend(const std::string & sensor_ip);
  ~OxApiBackend() override;

  bool connect() override;
  void disconnect() override;
  bool isConnected() const override;
  bool getProfile(ProfileData & profile) override;

private:
  std::string sensor_ip_;
  bool connected_ {false};

#ifdef LVS_USE_OXAPI
  class Impl;
  std::unique_ptr<Impl> impl_;
#endif
};

}  // namespace lvs_driver