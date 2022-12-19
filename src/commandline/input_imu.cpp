#include <cassert>

#include "../api/types.hpp"
#include "input.hpp"
#include "parameters.hpp"

InputImu::InputImu()
{
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8); // Not used, but is required by librealsense
    m_pipeline.start(cfg);
}

InputImu::~InputImu()
{
    m_pipeline.stop();
}

IMUFrame InputImu::next_imu_frame()
{
    auto frames = m_pipeline.wait_for_frames();
    return { .gyro = frames.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F),
        .acc = frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F),
        .video = frames.get_fisheye_frame(1) };
}

void InputImu::set_parameters(CommandLineParameters& cmdParameters)
{
    auto set_camera_parameters = [&](int index, auto& fx, auto& fy, auto& ppx, auto& ppy, auto& coeffs, auto& imu_to_camera) {
        auto video_profile = m_pipeline.get_active_profile().get_stream(RS2_STREAM_FISHEYE, index).as<rs2::video_stream_profile>();
        auto video_intrin = video_profile.get_intrinsics();
        auto extrin = m_pipeline.get_active_profile().get_stream(RS2_STREAM_GYRO).get_extrinsics_to(video_profile); // Is the gyro always equal to the acc?

        if (video_intrin.model != RS2_DISTORTION_KANNALA_BRANDT4)
            log_error("Unsupported distortion model %s", rs2_distortion_to_string(video_intrin.model));

        fx = video_intrin.fx;
        fy = video_intrin.fy;
        ppx = video_intrin.ppx;
        ppy = video_intrin.ppy;
        log_debug("fx %10f fy %10f ppx %10f ppy %10f", fx, fy, ppx, ppy);

        coeffs.reserve(4);
        coeffs = { std::cbegin(video_intrin.coeffs), std::cend(video_intrin.coeffs) - 1 };
        log_debug("Distortion coeffs: %10f %10f %10f %10f", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);

        auto r = extrin.rotation;
        auto t = extrin.translation;
        imu_to_camera.reserve(16);
        // Column-major
        imu_to_camera = {
            r[0], r[1], r[2], 0,
            r[3], r[4], r[5], 0,
            r[6], r[7], r[8], 0,
            t[0], t[1], t[2], 1
        };
        log_debug("Imu to camera:");
        for (size_t i = 0; i < 4; ++i)
            log_debug("%10f %10f %10f %10f", imu_to_camera[i], imu_to_camera[i + 4], imu_to_camera[i + 8], imu_to_camera[i + 12]);
    };

    auto& param = cmdParameters.parameters;
    param.tracker.fisheyeCamera = true;
    param.odometry.rot = 0;
    log_debug("Camera 1:");
    set_camera_parameters(1, param.tracker.focalLengthX, param.tracker.focalLengthY, param.tracker.principalPointX, param.tracker.principalPointY, param.tracker.distortionCoeffs, param.odometry.imuToCameraMatrix);
    // log_debug("Camera 2:");
    // set_camera_parameters(2, param.tracker.secondFocalLengthX, param.tracker.secondFocalLengthY, param.tracker.secondPrincipalPointX, param.tracker.secondPrincipalPointY, param.tracker.secondDistortionCoeffs, param.odometry.secondImuToCameraMatrix);
}

double InputImu::get_fps() const
{
    return m_pipeline.get_active_profile().get_stream(RS2_STREAM_FISHEYE, 1).as<rs2::video_stream_profile>().fps();
}

void InputImu::get_resolution(int& width, int& height) const
{
    auto intrin = m_pipeline.get_active_profile().get_stream(RS2_STREAM_FISHEYE, 1).as<rs2::video_stream_profile>().get_intrinsics();
    width = intrin.width;
    height = intrin.height;
}

api::CameraParameters InputImu::get_frame_intrin() const
{
    auto intrin = m_pipeline.get_active_profile().get_stream(RS2_STREAM_FISHEYE, 1).as<rs2::video_stream_profile>().get_intrinsics();
    api::CameraParameters params;
    params.focalLengthX = intrin.fx;
    params.focalLengthY = intrin.fy;
    params.principalPointX = intrin.ppx;
    params.principalPointY = intrin.ppy;
    return params;
}
