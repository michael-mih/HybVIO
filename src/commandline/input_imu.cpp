#include "../../../librealsense/include/librealsense2/rs.hpp"
#include "input.hpp"
#include "src/api/types.hpp"

class InputImu : public odometry::InputI {
public:
  InputImu() {
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    m_pipeline.start(cfg);
  }

  ~InputImu() { m_pipeline.stop(); }

  virtual odometry::InputSample nextType() override {
    auto frames = m_pipeline.wait_for_frames();
    for (auto frame : frames) {
      if (auto motion = frame.as<rs2::motion_frame>()) {
        if (motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
            motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
          m_last_frame = frame;
          return odometry::InputSample::GYROSCOPE;
        }
        if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
            motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
          m_last_frame = frame;
          return odometry::InputSample::ACCELEROMETER;
        }
      }
    }
  }

  virtual void getGyroscope(double &t, api::Vector3d &p) override {
    assert(m_last_frame.get_profile().stream_type() == RS2_STREAM_GYRO &&
           m_last_frame.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F);
    t = m_last_frame.get_timestamp();
    auto motion = m_last_frame.as<rs2::motion_frame>();
    assert(motion);
    p.x = motion.get_motion_data().x;
    p.y = motion.get_motion_data().y;
    p.z = motion.get_motion_data().z;
  }

  virtual void getAccelerometer(double &t, api::Vector3d &p) override {
    assert(m_last_frame.get_profile().stream_type() == RS2_STREAM_ACCEL &&
           m_last_frame.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F);
    t = m_last_frame.get_timestamp();
    auto motion = m_last_frame.as<rs2::motion_frame>();
    assert(motion);
    p.x = motion.get_motion_data().x;
    p.y = motion.get_motion_data().y;
    p.z = motion.get_motion_data().z;
  }

  virtual void getFrames(double &t, int &framesInd,
                         std::vector<odometry::InputFrame> &frames) override {}

  virtual std::string getInputVideoPath(int cameraInd) const override {
    assert(!"Doesn't work");
  }

  virtual void
  setAlgorithmParametersFromData(odometry::Parameters &parameters) override {
    assert(!"Doesn't work");
  }

private:
  rs2::pipeline m_pipeline;
  rs2::frame m_last_frame;
};
