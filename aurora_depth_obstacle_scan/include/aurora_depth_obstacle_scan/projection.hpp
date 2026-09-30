#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace aurora_depth_obstacle_scan
{
struct Config
{
  double min_height{0.05}, max_height{0.90};
  double min_range{0.20}, max_range{3.00};
  // Provisional diagnostic bounds, NOT an experimentally accepted sensor range.
  double min_depth{0.20}, max_depth{3.00};
  double angle_min{-1.5707963267948966}, angle_max{1.5707963267948966};
  double angle_increment{0.017453292519943295};
  int pixel_stride_x{2}, pixel_stride_y{2};
};

struct Transform
{
  std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::array<double, 3> translation{0, 0, 0};
};

struct Counts
{
  std::size_t sampled{0}, valid_depth{0}, height_pass{0}, binned{0};
};

class Projection
{
public:
  explicit Projection(const Config & config);
  void camera(std::uint32_t width, std::uint32_t height, const std::array<double, 9> & k);
  Counts project(const std::uint8_t * data, std::size_t size, std::size_t step,
    bool big_endian, const Transform & transform, std::vector<float> & ranges) const;
  std::size_t bins() const {return bins_;}
  double angle_max() const {return config_.angle_min + (bins_ - 1) * config_.angle_increment;}
  std::size_t ray_updates() const {return ray_updates_;}

private:
  Config config_;
  std::size_t bins_, ray_updates_{0};
  std::uint32_t width_{0}, height_{0};
  std::array<double, 9> k_{};
  std::vector<double> ray_x_, ray_y_;
};
}  // namespace aurora_depth_obstacle_scan
