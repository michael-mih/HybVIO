#include <cassert>

#include "../api/types.hpp"
#include "input.hpp"
#include "parameters.hpp"

static constexpr auto WIDTH = 1280,
                      HEIGHT = 720,
                      FPS = 30;

InputImu::InputImu()
{
    {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        m_imu_pipeline.start(cfg);
    }
    {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);
        m_video_pipeline.start(cfg);
    }
}

InputImu::~InputImu()
{
    m_imu_pipeline.stop();
    m_video_pipeline.stop();
}

rs2::video_frame InputImu::next_video_frame()
{
    return *m_video_pipeline.wait_for_frames().begin();
}

IMUFrame InputImu::next_imu_frame()
{
    auto frames = m_imu_pipeline.wait_for_frames();
    return { .gyro = frames.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F), .acc = frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F) };
}

void InputImu::set_parameters(CommandLineParameters& cmdParameters)
{
    auto intrin = m_video_pipeline.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();

    auto& param = cmdParameters.parameters;
    param.tracker.focalLengthX = intrin.fx;
    param.tracker.focalLengthY = intrin.fy;
    param.tracker.principalPointX = intrin.ppx;
    param.tracker.principalPointY = intrin.ppy;
    // Distortion coeffs
}

double InputImu::get_fps() const
{
    return FPS;
}

void InputImu::get_resolution(int& width, int& height) const
{
    auto intrin = m_video_pipeline.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    width = intrin.width;
    height = intrin.height;
}

api::CameraParameters InputImu::get_frame_intrin() const
{
    auto intrin = m_video_pipeline.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    if (intrin.model != RS2_DISTORTION_NONE) {
        printf("has distortion model %s\n", rs2_distortion_to_string(intrin.model));
        // assert(!"Cannot handle distortion model that is not none");
    }
    api::CameraParameters params;
    params.focalLengthX = intrin.fx;
    params.focalLengthY = intrin.fy;
    params.principalPointX = intrin.ppx;
    params.principalPointY = intrin.ppy;
    return params;
}
