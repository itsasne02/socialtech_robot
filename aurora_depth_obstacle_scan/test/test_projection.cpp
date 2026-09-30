#include "aurora_depth_obstacle_scan/projection.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

using namespace aurora_depth_obstacle_scan;

namespace
{
Transform mount(double height = 0.6)
{
  return {{0, 0, 1, -1, 0, 0, 0, -1, 0}, {0.2, 0, height}};
}
std::array<double, 9> k(double fx = 100, double fy = 100, double cy = 0)
{
  return {fx, 0, 0, 0, fy, cy, 0, 0, 1};
}
bool host_big_endian()
{
  const uint16_t one = 1;
  return *reinterpret_cast<const uint8_t *>(&one) == 0;
}
Counts run(Projection & p, const std::vector<float> & d, size_t width,
  const Transform & t, std::vector<float> & out)
{
  return p.project(reinterpret_cast<const uint8_t *>(d.data()), d.size() * 4,
    width * 4, host_big_endian(), t, out);
}
Config full_sampling()
{
  Config c;
  c.pixel_stride_x = c.pixel_stride_y = 1;
  return c;
}
}

TEST(Projection, ClosestPointAndNoHistoryOrBufferGrowth)
{
  Projection p(full_sampling());
  p.camera(2, 1, k(1000000));
  std::vector<float> out(p.bins());
  auto * buffer = out.data();
  const auto counts = run(p, {2.0F, 1.0F}, 2, mount(), out);
  EXPECT_EQ(counts.binned, 2U);
  EXPECT_NEAR(out[90], 1.2, 1e-6);
  run(p, {INFINITY, NAN}, 2, mount(), out);
  EXPECT_TRUE(std::all_of(out.begin(), out.end(), [](float v) {return std::isinf(v) && v > 0;}));
  EXPECT_EQ(out.data(), buffer);
}

TEST(Projection, RejectsInvalidAndOutOfOpticalDepthBounds)
{
  Projection p(full_sampling());
  p.camera(7, 1, k());
  std::vector<float> out(p.bins());
  auto c = run(p, {NAN, INFINITY, -INFINITY, 0, -1, 0.1F, 3.1F}, 7, mount(), out);
  EXPECT_EQ(c.valid_depth, 0U);
  EXPECT_EQ(c.binned, 0U);
}

TEST(Projection, HeightIsAfterFullRobotTransform)
{
  Projection p(full_sampling());
  // Middle ray projects exactly onto the floor: z=0.6-(v-cy)*depth/fy=0.
  p.camera(1, 3, k(100, 1, 0.4));
  std::vector<float> out(p.bins());
  auto c = run(p, {1, 1, 1}, 1, mount(), out);
  EXPECT_EQ(c.height_pass, 0U);  // z=1.0, 0.0, -1.0, all outside 0.05..0.90
  p.camera(1, 1, k());
  c = run(p, {1}, 1, mount(0.75), out);
  EXPECT_EQ(c.height_pass, 1U);  // table height
  c = run(p, {1}, 1, mount(0.04), out);
  EXPECT_EQ(c.height_pass, 0U);  // no base_footprint offset substituted
}

TEST(Projection, CameraPitchAffectsHeight)
{
  Projection p(full_sampling());
  p.camera(1, 1, k());
  std::vector<float> out(p.bins());
  const double cs = std::sqrt(3.0) / 2;
  Transform pitched{{0, 0.5, cs, -1, 0, 0, 0, -cs, 0.5}, {0.2, 0, 0.6}};
  EXPECT_EQ(run(p, {1}, 1, mount(), out).binned, 1U);
  EXPECT_EQ(run(p, {1}, 1, pitched, out).binned, 0U);  // z=1.1
}

TEST(Projection, HorizontalRangeAndAngularFilter)
{
  auto config = full_sampling();
  config.min_depth = 0.01;
  config.max_depth = 10;
  Projection p(config);
  p.camera(1, 1, k());
  std::vector<float> out(p.bins());
  auto t = mount();
  t.translation[0] = 0;
  EXPECT_EQ(run(p, {0.1F}, 1, t, out).binned, 0U);
  EXPECT_EQ(run(p, {3.1F}, 1, t, out).binned, 0U);
  EXPECT_EQ(run(p, {0.5F}, 1, t, out).binned, 1U);
  t.rotation = {0, 0, -1, 1, 0, 0, 0, -1, 0};
  EXPECT_EQ(run(p, {1}, 1, t, out).binned, 0U);  // behind robot
}

TEST(Projection, StrideIsRespected)
{
  Config c;
  Projection p(c);
  p.camera(4, 4, k());
  std::vector<float> out(p.bins());
  EXPECT_EQ(run(p, std::vector<float>(16, 1), 4, mount(), out).sampled, 4U);
}

TEST(Projection, EndianAndPaddedRows)
{
  Projection p(full_sampling());
  p.camera(1, 2, k());
  std::vector<float> out(p.bins());
  const std::vector<uint8_t> big{0x3f, 0x80, 0, 0, 0xde, 0xad, 0xbe, 0xef,
    0x40, 0, 0, 0, 0xde, 0xad, 0xbe, 0xef};
  EXPECT_EQ(p.project(big.data(), big.size(), 8, true, mount(), out).binned, 2U);
  EXPECT_NEAR(out[90], 1.2, 1e-6);
  const std::vector<uint8_t> little{0, 0, 0x80, 0x3f, 0xde, 0xad, 0xbe, 0xef,
    0, 0, 0, 0x40, 0xde, 0xad, 0xbe, 0xef};
  EXPECT_EQ(p.project(little.data(), little.size(), 8, false, mount(), out).binned, 2U);
  EXPECT_NEAR(out[90], 1.2, 1e-6);
  EXPECT_THROW(p.project(big.data(), 11, 8, true, mount(), out), std::invalid_argument);
}

TEST(Projection, RayCacheChangesOnlyWithCalibration)
{
  Projection p(full_sampling());
  p.camera(2, 1, k());
  p.camera(2, 1, k());
  EXPECT_EQ(p.ray_updates(), 1U);
  p.camera(2, 1, k(90));
  EXPECT_EQ(p.ray_updates(), 2U);
  p.camera(3, 1, k(90));
  EXPECT_EQ(p.ray_updates(), 3U);
  EXPECT_THROW(p.camera(3, 1, k(0)), std::invalid_argument);
}

TEST(Projection, RejectsInvalidConfiguration)
{
  Config c;
  c.angle_increment = 0;
  EXPECT_THROW(Projection{c}, std::invalid_argument);
  c = Config{};
  c.min_height = NAN;
  EXPECT_THROW(Projection{c}, std::invalid_argument);
  c = Config{};
  c.pixel_stride_y = 0;
  EXPECT_THROW(Projection{c}, std::invalid_argument);
}

TEST(Projection, AngularEndpointUsesLastBin)
{
  Projection p(full_sampling());
  p.camera(1, 1, k());
  std::vector<float> out(p.bins());
  const Transform t{{1, 0, 0, 0, 0, 1, 0, -1, 0}, {0, 0, 0.6}};
  EXPECT_EQ(run(p, {1}, 1, t, out).binned, 1U);
  EXPECT_NEAR(out.back(), 1, 1e-6);
}
