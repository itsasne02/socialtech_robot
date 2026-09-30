#include "aurora_depth_obstacle_scan/projection.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace aurora_depth_obstacle_scan
{
class DepthScan : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;
  using Info = sensor_msgs::msg::CameraInfo;
  using Clock = std::chrono::steady_clock;

public:
  DepthScan() : Node("aurora_depth_obstacle_scan")
  {
    Config c;
    c.min_height = param("min_height", c.min_height);
    c.max_height = param("max_height", c.max_height);
    c.min_range = param("min_range", c.min_range);
    c.max_range = param("max_range", c.max_range);
    c.min_depth = param("min_depth", c.min_depth);
    c.max_depth = param("max_depth", c.max_depth);
    c.angle_min = param("angle_min", c.angle_min);
    c.angle_max = param("angle_max", c.angle_max);
    c.angle_increment = param("angle_increment", c.angle_increment);
    const auto sx = param<int64_t>("pixel_stride_x", c.pixel_stride_x);
    const auto sy = param<int64_t>("pixel_stride_y", c.pixel_stride_y);
    if (sx < 1 || sy < 1 || sx > 8192 || sy > 8192) {
      throw std::invalid_argument("pixel strides must be 1..8192");
    }
    c.pixel_stride_x = static_cast<int>(sx);
    c.pixel_stride_y = static_cast<int>(sy);
    rate_ = param("publish_rate", 5.0);
    tolerance_ = param("transform_tolerance", 0.05);
    max_age_ = param("max_input_age", 1.0);
    if (!std::isfinite(rate_) || rate_ <= 0 || rate_ > 60 ||
      !std::isfinite(tolerance_) || tolerance_ < 0 || tolerance_ > 1 ||
      !std::isfinite(max_age_) || max_age_ <= 0)
    {
      throw std::invalid_argument("Invalid rate, transform_tolerance or max_input_age");
    }
    projection_ = std::make_unique<Projection>(c);
    scan_.header.frame_id = "base_link";
    scan_.angle_min = static_cast<float>(c.angle_min);
    scan_.angle_max = static_cast<float>(projection_->angle_max());
    scan_.angle_increment = static_cast<float>(c.angle_increment);
    scan_.range_min = static_cast<float>(c.min_range);
    scan_.range_max = static_cast<float>(c.max_range);
    // Every bin derives from one image; no per-ray acquisition interval.
    scan_.time_increment = 0;
    scan_.ranges.resize(projection_->bins());
    const auto depth_topic = param<std::string>("depth_topic",
      "/slamware_ros_sdk_server_node/depth_image_raw");
    const auto info_topic = param<std::string>("camera_info_topic",
      "/slamware_ros_sdk_server_node/camera_info");
    const auto output_topic = param<std::string>("scan_topic", "/aurora/depth_obstacle_scan");
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic, rclcpp::SensorDataQoS().keep_last(1));
    image_sub_ = create_subscription<Image>(depth_topic, rclcpp::SensorDataQoS().keep_last(8),
      [this](Image::ConstSharedPtr image) {
        images_[image_index_++ % images_.size()] = std::move(image);
        match();
      });
    info_sub_ = create_subscription<Info>(info_topic, rclcpp::SensorDataQoS().keep_last(8),
      [this](Info::ConstSharedPtr info) {
        infos_[info_index_++ % infos_.size()] = std::move(info);
        match();
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_), [this]() {
      if (pending_image_ && pending_info_) {
        auto d = std::move(pending_image_);
        auto i = std::move(pending_info_);
        process(*d, *i);
      }
    });
    RCLCPP_WARN(get_logger(),
      "Depth accuracy and Robot 2 camera translation not metrically validated. "
      "Heights use base_link; depth bounds provisional. "
      "Parameters are startup-only; restart to apply changes.");
  }

private:
  template<class T> T param(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }

  void match()
  {
    for (auto & image : images_) {
      if (!image) {continue;}
      for (auto & info : infos_) {
        if (!info || image->header.stamp != info->header.stamp) {continue;}
        // Keep only the newest exact pair until the next processing tick.
        pending_image_ = std::move(image);
        pending_info_ = std::move(info);
        break;
      }
    }
  }

  void process(const Image & d, const Info & i)
  {
    const auto started = Clock::now();
    try {
      if (d.encoding != "32FC1" || d.header.frame_id.empty() ||
        d.header.frame_id != i.header.frame_id || d.width != i.width || d.height != i.height ||
        (d.header.stamp.sec == 0 && d.header.stamp.nanosec == 0))
      {
        throw std::invalid_argument("Invalid image encoding/frame/dimensions/stamp");
      }
      if (i.binning_x > 1 || i.binning_y > 1 || i.roi.x_offset || i.roi.y_offset ||
        (i.roi.width && i.roi.width != i.width) || (i.roi.height && i.roi.height != i.height))
      {
        throw std::invalid_argument("Only full native depth CameraInfo is supported");
      }
      for (double distortion : i.d) {
        if (!std::isfinite(distortion) || std::abs(distortion) > 1e-9) {
          throw std::invalid_argument("Depth CameraInfo must describe zero-distortion native depth");
        }
      }
      const rclcpp::Time stamp(d.header.stamp, get_clock()->get_clock_type());
      const double age = (now() - stamp).seconds();
      if (age > max_age_ || age < -tolerance_) {
        throw std::invalid_argument("Stale or future ROS input stamp");
      }
      projection_->camera(d.width, d.height, i.k);
      const auto tf = buffer_->lookupTransform("base_link", d.header.frame_id, stamp,
        rclcpp::Duration::from_seconds(tolerance_));
      const auto & q = tf.transform.rotation;
      const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (!std::isfinite(norm2) || std::abs(norm2 - 1) > 1e-3) {
        throw std::invalid_argument("Invalid TF quaternion");
      }
      const tf2::Matrix3x3 matrix(tf2::Quaternion(q.x, q.y, q.z, q.w));
      Transform transform;
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {transform.rotation[row * 3 + col] = matrix[row][col];}
      }
      transform.translation = {tf.transform.translation.x, tf.transform.translation.y,
        tf.transform.translation.z};
      const auto counts = projection_->project(d.data.data(), d.data.size(), d.step,
        d.is_bigendian != 0, transform, scan_.ranges);
      scan_.header.stamp = d.header.stamp;
      scan_.scan_time = last_published_ == Clock::time_point{} ? static_cast<float>(1 / rate_) :
        static_cast<float>(std::chrono::duration<double>(started - last_published_).count());
      publisher_->publish(scan_);
      last_published_ = started;
      const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "frame process+publish=%.3fms ROS_stamp_age=%.1fms samples=%zu valid=%zu height=%zu "
        "binned=%zu ray_updates=%zu (ROS stamp age is NOT acquisition latency)",
        elapsed, age * 1000, counts.sampled, counts.valid_depth, counts.height_pass,
        counts.binned, projection_->ray_updates());
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Skipping depth frame: %s", error.what());
    }
  }

  std::unique_ptr<Projection> projection_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Subscription<Info>::SharedPtr info_sub_;
  std::array<Image::ConstSharedPtr, 8> images_{};
  std::array<Info::ConstSharedPtr, 8> infos_{};
  std::size_t image_index_{0}, info_index_{0};
  sensor_msgs::msg::LaserScan scan_;
  Image::ConstSharedPtr pending_image_;
  Info::ConstSharedPtr pending_info_;
  rclcpp::TimerBase::SharedPtr timer_;
  Clock::time_point last_published_{};
  double rate_{5}, tolerance_{0.05}, max_age_{1};
};
}  // namespace aurora_depth_obstacle_scan

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<aurora_depth_obstacle_scan::DepthScan>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("aurora_depth_obstacle_scan"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
