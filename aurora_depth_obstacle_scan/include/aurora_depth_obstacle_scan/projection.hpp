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
  std::size_t sampled{0}, valid_depth{0}, height_pass{0}, binned{0}, free_evidence{0};
};

class Projection
{
public:
  explicit Projection(const Config & config);
  void camera(std::uint32_t width, std::uint32_t height, const std::array<double, 9> & k);
  // ranges: nearest in-band return per bin, +Inf without one.
  // free_ranges (optional): how far each bin was seen free, for costmap
  // clearing. The nearest in-band return if there is one; otherwise the
  // farthest floor return (below min_height) or max_range for in-band returns
  // beyond it, capped just under max_range; NaN when the bin gave no evidence
  // (no valid pixel, or only returns above max_height).
  Counts project(const std::uint8_t * data, std::size_t size, std::size_t step,
    bool big_endian, const Transform & transform, std::vector<float> & ranges,
    std::vector<float> * free_ranges = nullptr) const;
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
