#include <cassert>

#include "../api/types.hpp"
#include "input.hpp"

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

std::string InputImu::getInputVideoPath(int cameraInd) const
{
    assert(!"Doesn't work");
}

void InputImu::set_parameters(CommandLineParameters& cmdParameters)
{
    assert(!"Doesn't work");
}

bool InputImu::getParametersAvailable() const
{

    assert(!"Doesn't work");
}

std::map<api::PoseHistory, std::vector<api::Pose>> InputImu::getPoseHistories()
{

    assert(!"Doesn't work");
}

std::string InputImu::getLastJSONL() const
{

    assert(!"Doesn't work");
}

bool InputImu::canEcho() const
{
    return false;
}
