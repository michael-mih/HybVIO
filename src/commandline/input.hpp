#pragma once

#include "../../../librealsense/include/librealsense2/rs.hpp"
#include "../api/internal.hpp"
#include "../api/vio.hpp"
#include "../tracker/camera.hpp"

struct CommandLineParameters;

enum class InputType {
    Gyroscope,
    Accelerometer,
    Video
};

class InputImu {
public:
    InputImu();
    ~InputImu();

    std::pair<InputType, rs2::frame> next_frame();

    /**
     * Read algorithm parameters.
     * @param cmdParameters Struct to which parameters will be placed.
     */
    void set_parameters(CommandLineParameters& cmdParameters);

    double get_fps() const;
    void get_resolution(int& width, int& height) const;
    CameraParameters get_frame_intrin() const;

private:
    rs2::pipeline m_pipeline;
};
