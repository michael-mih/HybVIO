#include <cassert>

#include "../api/types.hpp"
#include "input.hpp"

static constexpr auto WIDTH = 1280,
                      HEIGHT = 720,
                      FPS = 30;

InputImu::InputImu()
{
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);
    m_pipeline.start(cfg);
}

InputImu::~InputImu() { m_pipeline.stop(); }

std::pair<InputType, rs2::frame> InputImu::next_frame();
{
    auto frame = *m_pipeline.wait_for_frames().begin();
    InputType type;
    if (auto motion = frame.as<rs2::motion_frame>()) {
        if (motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            type = InputType::Gyroscope;

        else if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            type = InputType::Accelerometer;
        else
            assert(!"Unreachable");
    } else if (auto color = frame.as<rs2::video_frame>()) {
        assert(color.get_profile().stream_type() == RS2_STREAM_COLOR && color.get_profile().format() == RS2_FORMAT_BGR8);
        type = InputType::Video;
    } else {
        assert(!"Unreachable");
    }
    return { type, frame };
}

void InputImu::set_parameters(CommandLineParameters& cmdParameters)
{
    auto intrin = get_frame_intrin();
    auto& param = cmdParameters.parameters;
    param.tracker.focalLengthX = intrin.focalLengthX;
    param.tracker.focalLengthY = intrin.focalLengthY;
    param.tracker.principalPointX = intrin.principalPointX;
    param.tracker.principalPointY = intrin.principalPointY;
    // Distortion coeffs
}

double InputImu::get_fps() const
{
    return FPS;
}

void InputImu::get_resolution(int& width, int& height) const
{
    auto intrin = m_pipeline.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    width = intrin.width;
    height = intrin.height;
}

CameraParameters InputImu::get_frame_intrin() const
{
    auto intrin = m_pipeline.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    if (intrin.model != RS2_DISTORTION_NONE)
        assert(!"Cannot handle distortion model that is not none");
    return {
        .focalLengthX = intrin.fx,
        .focalLengthY = intrin.fy,
        .principalPointX = intrin.ppx,
        .principalPointY = intrin.ppy
    };
}
