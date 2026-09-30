#include "aurora_depth_obstacle_scan/projection.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace aurora_depth_obstacle_scan
{
Projection::Projection(const Config & c) : config_(c)
{
  for (double v : {c.min_height, c.max_height, c.min_range, c.max_range,
      c.min_depth, c.max_depth, c.angle_min, c.angle_max, c.angle_increment})
  {
    if (!std::isfinite(v)) {throw std::invalid_argument("Nonfinite projection parameter");}
  }
  constexpr double pi = 3.14159265358979323846;
  if (c.min_height > c.max_height || c.min_range <= 0 || c.max_range <= c.min_range ||
    c.min_depth <= 0 || c.max_depth <= c.min_depth || c.angle_min < -pi || c.angle_max > pi ||
    c.angle_max <= c.angle_min || c.angle_increment <= 0 ||
    c.pixel_stride_x < 1 || c.pixel_stride_y < 1)
  {
    throw std::invalid_argument("Invalid projection limits/stride");
  }
  const double intervals = (c.angle_max - c.angle_min) / c.angle_increment;
  if (intervals < 1 || intervals > 65535) {
    throw std::invalid_argument("Angular grid must have 2..65536 bins");
  }
  bins_ = static_cast<std::size_t>(std::floor(intervals + 1e-10)) + 1;
}

void Projection::camera(std::uint32_t width, std::uint32_t height,
  const std::array<double, 9> & k)
{
  if (!width || !height || width > 8192 || height > 8192 ||
    !std::all_of(k.begin(), k.end(), [](double v) {return std::isfinite(v);}) ||
    k[0] <= 0 || k[4] <= 0 || k[1] != 0 || k[3] != 0 || k[6] != 0 || k[7] != 0 || k[8] != 1)
  {
    throw std::invalid_argument("Invalid pinhole CameraInfo K/dimensions");
  }
  if (width == width_ && height == height_ && k == k_) {return;}
  ray_x_.resize(width);
  ray_y_.resize(height);
  for (std::size_t u = 0; u < width; ++u) {ray_x_[u] = (u - k[2]) / k[0];}
  for (std::size_t v = 0; v < height; ++v) {ray_y_[v] = (v - k[5]) / k[4];}
  width_ = width;
  height_ = height;
  k_ = k;
  ++ray_updates_;
}

Counts Projection::project(const std::uint8_t * data, std::size_t size, std::size_t step,
  bool big_endian, const Transform & t, std::vector<float> & ranges,
  std::vector<float> * free_ranges) const
{
  if (!data || !width_ || !height_ || step < std::size_t(width_) * 4 ||
    step > size / height_ || ranges.size() != bins_ ||
    (free_ranges && free_ranges->size() != bins_))
  {
    throw std::invalid_argument("Invalid image buffer/stride or scan buffer");
  }
  if (!std::all_of(t.rotation.begin(), t.rotation.end(), [](double v) {return std::isfinite(v);}) ||
    !std::all_of(t.translation.begin(), t.translation.end(), [](double v) {return std::isfinite(v);}))
  {
    throw std::invalid_argument("Nonfinite transform");
  }
  std::fill(ranges.begin(), ranges.end(), std::numeric_limits<float>::infinity());
  // laser_geometry drops ranges >= range_max, so free space reaching it is
  // published just under it.
  const float free_cap = static_cast<float>(config_.max_range - 1e-3);
  if (free_ranges) {
    std::fill(free_ranges->begin(), free_ranges->end(), 0.0F);
  }
  const std::uint16_t endian = 1;
  const bool swap = big_endian != (*reinterpret_cast<const std::uint8_t *>(&endian) == 0);
  const auto & r = t.rotation;
  Counts counts;
  for (std::size_t v = 0; v < height_; v += config_.pixel_stride_y) {
    for (std::size_t u = 0; u < width_; u += config_.pixel_stride_x) {
      ++counts.sampled;
      std::uint32_t bits;
      std::memcpy(&bits, data + v * step + u * 4, 4);
      if (swap) {
        bits = ((bits & 0xff) << 24) | ((bits & 0xff00) << 8) |
          ((bits & 0xff0000) >> 8) | ((bits & 0xff000000) >> 24);
      }
      float depth;
      std::memcpy(&depth, &bits, 4);
      if (!std::isfinite(depth) || depth <= 0 || depth < config_.min_depth ||
        depth > config_.max_depth) {continue;}
      ++counts.valid_depth;
      const double cx = ray_x_[u] * depth, cy = ray_y_[v] * depth, cz = depth;
      const double z = r[6] * cx + r[7] * cy + r[8] * cz + t.translation[2];
      if (!std::isfinite(z) || z > config_.max_height) {continue;}
      // Below the band is the floor: no obstacle, but the ray crossed the band.
      const bool floor = z < config_.min_height;
      if (floor && !free_ranges) {continue;}
      if (!floor) {++counts.height_pass;}
      const double x = r[0] * cx + r[1] * cy + r[2] * cz + t.translation[0];
      const double y = r[3] * cx + r[4] * cy + r[5] * cz + t.translation[1];
      const double rr = x * x + y * y;
      if (!std::isfinite(rr) || rr < config_.min_range * config_.min_range) {continue;}
      const bool beyond = rr > config_.max_range * config_.max_range;
      if (beyond && !free_ranges) {continue;}
      const double angle = std::atan2(y, x);
      if (angle < config_.angle_min || angle > angle_max()) {continue;}
      const auto bin = static_cast<std::size_t>(
        std::llround((angle - config_.angle_min) / config_.angle_increment));
      if (bin >= ranges.size()) {continue;}
      if (!floor && !beyond) {
        ranges[bin] = std::min(ranges[bin], static_cast<float>(std::sqrt(rr)));
        ++counts.binned;
      } else {
        auto & seen = (*free_ranges)[bin];
        seen = std::max(seen, std::min(static_cast<float>(std::sqrt(rr)), free_cap));
        ++counts.free_evidence;
      }
    }
  }
  if (free_ranges) {
    for (std::size_t bin = 0; bin < bins_; ++bin) {
      auto & seen = (*free_ranges)[bin];
      if (std::isfinite(ranges[bin])) {
        seen = ranges[bin];
      } else if (seen <= 0.0F) {
        seen = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
  return counts;
}
}  // namespace aurora_depth_obstacle_scan
