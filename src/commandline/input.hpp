#pragma once

#include "../../../librealsense/include/librealsense2/rs.hpp"
#include "../api/internal.hpp"
#include "../api/vio.hpp"
#include "../tracker/camera.hpp"

namespace odometry {
struct CommandLineParameters;
}

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
    std::string getInputVideoPath(int cameraInd) const;

    /**
     * Read algorithm parameters.
     * @param cmdParameters Struct to which parameters will be placed.
     */
    void set_parameters(odometry::CommandLineParameters& cmdParameters);
    bool getParametersAvailable() const;
    std::map<api::PoseHistory, std::vector<api::Pose>> getPoseHistories();
    std::string getLastJSONL() const;
    /**
     * True for JSONL input, false otherwise.
     */
    bool canEcho() const;

private:
    rs2::pipeline m_pipeline;
};
