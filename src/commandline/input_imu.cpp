#include "../../../librealsense/include/librealsense2/rs.hpp"
#include "input.hpp"
#include "src/api/types.hpp"
#include <cassert>
#include <memory>

rs2::frame g_last_video_frame;

class InputImu : public odometry::InputI {
public:
    InputImu()
    {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_Y8);
        m_pipeline.start(cfg);
    }

    ~InputImu() { m_pipeline.stop(); }

    virtual odometry::InputSample nextType() override
    {
        auto frames = m_pipeline.wait_for_frames();
        for (auto frame : frames) {
            m_last_frame = frame;
            if (auto motion = frame.as<rs2::motion_frame>()) {
                if (motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
                    return odometry::InputSample::GYROSCOPE;

                if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
                    return odometry::InputSample::ACCELEROMETER;

                assert(!"Unreachable");
            }
            if (auto color = frame.as<rs2::video_frame>()) {
                assert(color.get_profile().stream_type() == RS2_STREAM_COLOR && color.get_profile().format() == RS2_FORMAT_Y8);
                g_last_video_frame = frame;
                return odometry::InputSample::FRAME;
            }
            assert(!"Unreachable");
        }
    }

    virtual void getGyroscope(double& t, api::Vector3d& p) override
    {
        assert(m_last_frame.get_profile().stream_type() == RS2_STREAM_GYRO && m_last_frame.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F);
        t = m_last_frame.get_timestamp();
        auto motion = m_last_frame.as<rs2::motion_frame>();
        assert(motion);
        p.x = motion.get_motion_data().x;
        p.y = motion.get_motion_data().y;
        p.z = motion.get_motion_data().z;
    }

    virtual void getAccelerometer(double& t, api::Vector3d& p) override
    {
        assert(m_last_frame.get_profile().stream_type() == RS2_STREAM_ACCEL && m_last_frame.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F);
        t = m_last_frame.get_timestamp();
        auto motion = m_last_frame.as<rs2::motion_frame>();
        assert(motion);
        p.x = motion.get_motion_data().x;
        p.y = motion.get_motion_data().y;
        p.z = motion.get_motion_data().z;
    }

    virtual void getFrames(double& t, int& framesInd, std::vector<odometry::InputFrame>& frames) override
    {
        assert(m_last_frame.get_profile().stream_type() == RS2_STREAM_COLOR && m_last_frame.get_profile().format() == RS2_FORMAT_Y8);
        t = m_last_frame.get_timestamp();
        framesInd = m_last_frame.get_frame_number();
        frames.push_back({ {}, t });
    }

    virtual std::string getInputVideoPath(int cameraInd) const override
    {
        assert(!"Doesn't work");
    }

    virtual void setAlgorithmParametersFromData(odometry::Parameters& parameters) override
    {
        assert(!"Doesn't work");
    }

    virtual void setParameters(CommandLineParameters& cmdParameters) override
    {
        assert(!"Doesn't work");
    }

    virtual std::string getParametersString() const override
    {
        assert(!"Doesn't work");
    }

    virtual bool getParametersAvailable() const override
    {

        assert(!"Doesn't work");
    }

    virtual std::map<api::PoseHistory, std::vector<api::Pose>> getPoseHistories()
    {

        assert(!"Doesn't work");
    }

    virtual std::string getLastJSONL() const override
    {

        assert(!"Doesn't work");
    }

    virtual bool canEcho() const override { return false; }

private:
    rs2::pipeline m_pipeline;
    rs2::frame m_last_frame;
};

namespace odometry {
std::unique_ptr<InputI> buildInputImu()
{
    return std::make_unique<InputImu>();
}
}
