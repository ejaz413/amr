#pragma once

#include <cstdint>
#include <vector>

namespace lvs_driver
{

struct ProfilePoint
{
  float x_mm {0.0f};
  float z_mm {0.0f};
  uint32_t intensity {0};
};

struct ProfileData
{
  uint64_t sensor_timestamp {0};
  uint32_t precision {100};
  bool valid {false};
  std::vector<ProfilePoint> points;
};

class ILvsBackend
{
public:
  virtual ~ILvsBackend() = default;

  virtual bool connect() = 0;
  virtual void disconnect() = 0;
  virtual bool isConnected() const = 0;
  virtual bool getProfile(ProfileData & profile) = 0;
};

}  // namespace lvs_driver