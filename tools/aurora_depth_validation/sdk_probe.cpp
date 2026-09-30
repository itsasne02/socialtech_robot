// Diagnostics only: queries and session-local depth subscription, no ROS,
// map-mode changes, robot commands, OpenCV, or PointCloud2 publication.
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <filesystem>
#include <fstream>
#include <limits>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
// SDK 2.1.1 includes its time-sync wrapper inside its namespace.
// Include standard headers first to avoid creating a nested std namespace.
#include <aurora_pubsdk_inc.h>

using namespace rp::standalone::aurora;
using Clock = std::chrono::steady_clock;

static std::string json_string(const std::string& value) {
    std::ostringstream s;
    s << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') s << '\\' << c;
        else if (c < 32) s << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c) << std::dec;
        else s << c;
    }
    return s.str() + '"';
}

static int fail(const char* stage, int code) {
    std::cout << "{\"event\":\"failure\",\"stage\":" << json_string(stage)
              << ",\"sdk_error\":" << code
              << ",\"result\":\"incomplete; do not infer unsupported depth\"}\n";
    return 2;
}

static bool layout_valid(const RemoteEnhancedImagingFrame& f, unsigned format, unsigned channels) {
    const auto& d = f.desc.image_desc;
    return f.image._data && d.width && d.height && d.format == format &&
        uint64_t(d.stride) >= uint64_t(d.width) * channels * sizeof(float) &&
        uint64_t(d.data_size) >= uint64_t(d.stride) * d.height;
}

struct Stream {
    uint64_t last = 0;
    size_t count = 0;
    Clock::time_point first{}, latest{};
    bool observe(const RemoteEnhancedImagingFrame& f, const char* name) {
        if (f.desc.timestamp_ns == last) return false;
        if (f.desc.timestamp_ns < last) {
            std::cout << "{\"event\":\"timestamp_regression\",\"stream\":" << json_string(name) << "}\n";
        }
        last = f.desc.timestamp_ns;
        latest = Clock::now();
        if (!count++) first = latest;
        const auto& d = f.desc.image_desc;
        std::cout << "{\"event\":\"frame\",\"stream\":" << json_string(name)
                  << ",\"device_timestamp_ns\":" << last << ",\"width\":" << d.width
                  << ",\"height\":" << d.height << ",\"stride\":" << d.stride
                  << ",\"format\":" << d.format << ",\"bytes\":" << d.data_size << "}\n";
        return true;
    }
    double hz() const {
        const double dt = std::chrono::duration<double>(latest - first).count();
        return count > 1 && dt > 0 ? (count - 1) / dt : 0;
    }
};

int main(int argc, char** argv) {
    if (argc < 2 || argc > 4 || std::string(argv[1]) == "--help") {
        std::cout << "Usage: sdk_probe DEVICE_IP [SECONDS=15] [NEW_CAPTURE_DIR]; wrap in timeout 45.\n";
        return argc == 2 ? 0 : 2;
    }
    double seconds = 15;
    try { if (argc >= 3) seconds = std::stod(argv[2]); }
    catch (const std::exception&) { return fail("invalid duration", -1); }
    if (!std::isfinite(seconds) || seconds < 1 || seconds > 60) return fail("duration outside [1,60]", -1);
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    std::filesystem::path capture_dir;
    if (argc == 4) {
        capture_dir = argv[3];
        if (std::filesystem::exists(capture_dir)) return fail("capture directory already exists", -1);
        std::filesystem::create_directories(capture_dir);
    }
    slamtec_aurora_sdk_version_info_t version{};
    slamtec_aurora_sdk_errorcode_t error = 0;
    if (!RemoteSDK::GetSDKInfo(version, &error)) return fail("SDK version", error);
    std::cout << std::boolalpha << std::unitbuf;
    std::cout << "{\"event\":\"sdk\",\"version\":" << json_string(version.sdk_version_string) << "}\n";
    SDKConfig config;
    config.creation_flags = SLAMTEC_AURORA_SDK_SESSION_FLAG_NO_PREVIEW_IMAGE_SUBSCRIPTION;
    std::unique_ptr<RemoteSDK, decltype(&RemoteSDK::DestroySession)> sdk(
        RemoteSDK::CreateSession(nullptr, config, &error), RemoteSDK::DestroySession);
    if (!sdk) return fail("create session", error);
    if (!sdk->connect(SDKServerConnectionDesc(argv[1]), &error)) return fail("connect", error);
    RemoteDeviceBasicInfo info;
    uint64_t info_stamp = 0;
    bool info_ok = false;
    const auto info_deadline = Clock::now() + std::chrono::seconds(10);
    do {
        info_ok = sdk->dataProvider.getLastDeviceBasicInfo(info, info_stamp, &error);
        if (!info_ok) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (!info_ok && Clock::now() < info_deadline);
    if (!info_ok) return fail("DeviceBasicInfo", error);
    // Interpret capabilities only after the explicit successful query above.
    const bool depth_supported = info.isSupportDepthCamera();
    std::cout << "{\"event\":\"device_basic_info\",\"query_success\":true,\"sdk_error\":" << error
              << ",\"timestamp_ns\":" << info_stamp << ",\"model\":" << json_string(info.getDeviceModelString())
              << ",\"firmware\":" << json_string(std::string(info.firmware_version_string, strnlen(info.firmware_version_string, sizeof(info.firmware_version_string))))
              << ",\"sensing_feature_bits\":" << info.sensing_feature_bitmaps
              << ",\"hardware_feature_bits\":" << info.hwfeature_bitmaps
              << ",\"software_feature_bits\":" << info.swfeature_bitmaps
              << ",\"isSupportDepthCamera\":" << depth_supported
              << ",\"isDepthCameraSupported\":" << sdk->enhancedImaging.isDepthCameraSupported() << "}\n";
    if (!depth_supported) {
        std::cout << "{\"event\":\"result\",\"status\":\"depth unsupported after successful DeviceBasicInfo\"}\n";
        return 3;
    }
    if (!sdk->controller.setEnhancedImagingSubscription(SLAMTEC_AURORA_SDK_ENHANCED_IMAGE_TYPE_DEPTH, true))
        return fail("depth subscription", -1);
    slamtec_aurora_sdk_depthcam_config_info_t depth_config{};
    bool config_ok = false;
    const auto config_deadline = Clock::now() + std::chrono::seconds(10);
    do {
        config_ok = sdk->enhancedImaging.getDepthCameraConfig(depth_config);
        if (!config_ok) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (!config_ok && Clock::now() < config_deadline);
    if (!config_ok) return fail("depth config", -1);
    std::cout << "{\"event\":\"depth_config\",\"query_success\":true,\"width\":" << depth_config.image_width
              << ",\"height\":" << depth_config.image_height << ",\"fps\":" << depth_config.fps
              << ",\"frame_skip\":" << depth_config.frame_skip << ",\"bound_camera_id\":" << depth_config.binded_cam_id << "}\n";
    Stream depth, xyz;
    size_t matching = 0, invalid_layout = 0, captured = 0;
    auto last_capture = Clock::time_point{};
    const auto started = Clock::now();
    while (std::chrono::duration<double>(Clock::now() - started).count() < seconds) {
        RemoteEnhancedImagingFrame d, p;
        const bool have_d = sdk->enhancedImaging.peekDepthCameraFrame(d, SLAMTEC_AURORA_SDK_DEPTHCAM_FRAME_TYPE_DEPTH_MAP);
        const bool have_p = sdk->enhancedImaging.peekDepthCameraFrame(p, SLAMTEC_AURORA_SDK_DEPTHCAM_FRAME_TYPE_POINT3D);
        bool new_depth = false;
        if (have_d) {
            if (layout_valid(d, 3, 1)) new_depth = depth.observe(d, "DEPTH_MAP");
            else ++invalid_layout;
        }
        if (have_p) {
            if (layout_valid(p, 4, 3)) xyz.observe(p, "POINT3D");
            else ++invalid_layout;
        }
        if (new_depth && have_p && layout_valid(p, 4, 3) && d.desc.timestamp_ns == p.desc.timestamp_ns &&
            d.desc.image_desc.width == p.desc.image_desc.width && d.desc.image_desc.height == p.desc.image_desc.height) {
            ++matching;
            if (!capture_dir.empty() && captured < 20 && Clock::now() - last_capture >= std::chrono::milliseconds(500)) {
                const auto prefix = capture_dir / std::to_string(d.desc.timestamp_ns);
                auto write_frame = [&](const RemoteEnhancedImagingFrame& f, const char* suffix) {
                    std::ofstream out(prefix.string() + suffix, std::ios::binary | std::ios::trunc);
                    out.write(static_cast<const char*>(f.image._data), f.desc.image_desc.data_size);
                    out.close();
                    return bool(out);
                };
                if (!write_frame(d, "_depth.bin") || !write_frame(p, "_xyz.bin")) return fail("write capture", -1);
                const uint32_t endian_check = 1;
                std::ofstream meta(prefix.string() + ".json");
                meta << std::boolalpha << "{\"timestamp_ns\":" << d.desc.timestamp_ns
                     << ",\"point3d_timestamp_ns\":" << p.desc.timestamp_ns
                     << ",\"width\":" << d.desc.image_desc.width << ",\"height\":" << d.desc.image_desc.height
                     << ",\"depth_stride\":" << d.desc.image_desc.stride << ",\"xyz_stride\":" << p.desc.image_desc.stride
                     << ",\"is_bigendian\":" << (*reinterpret_cast<const uint8_t*>(&endian_check) == 0)
                     << ",\"depth_file\":" << json_string(prefix.filename().string() + "_depth.bin")
                     << ",\"xyz_file\":" << json_string(prefix.filename().string() + "_xyz.bin") << "}\n";
                meta.close();
                if (!meta) return fail("write capture metadata", -1);
                last_capture = Clock::now();
                ++captured;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const bool pass = depth.count >= 5 && xyz.count >= 5 && matching >= 5 && invalid_layout == 0;
    std::cout << "{\"event\":\"summary\",\"sdk_presence_pass\":" << pass
              << ",\"depth_frames\":" << depth.count << ",\"point3d_frames\":" << xyz.count
              << ",\"equal_timestamp_pairs\":" << matching << ",\"invalid_layout\":" << invalid_layout
              << ",\"depth_observed_hz\":" << depth.hz() << ",\"point3d_observed_hz\":" << xyz.hz()
              << ",\"captured_pairs\":" << captured
              << ",\"clock\":\"steady_clock receive time; not device/ROS time\"}\n";
    sdk->disconnect();
    return pass ? 0 : 4;
}
