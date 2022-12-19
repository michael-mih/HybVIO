#pragma once

#include "../../../librealsense/include/librealsense2/rs.hpp"
#include "../api/internal.hpp"
#include "../api/vio.hpp"
#include "../tracker/camera.hpp"

struct CommandLineParameters;

struct IMUFrame {
    rs2::motion_frame gyro;
    rs2::motion_frame acc;
    rs2::video_frame video;
};

class InputImu {
public:
    InputImu();
    ~InputImu();

    IMUFrame next_imu_frame();

    /**
     * Read algorithm parameters.
     * @param cmdParameters Struct to which parameters will be placed.
     */
    void set_parameters(CommandLineParameters& cmdParameters);

    double get_fps() const;
    void get_resolution(int& width, int& height) const;
    api::CameraParameters get_frame_intrin() const;

private:
    rs2::pipeline m_pipeline;
};
