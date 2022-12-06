#include <deque>
#include <future>
#include <opencv2/opencv.hpp>
#include <thread>

#include "../api/output_buffer.hpp"
#include "../api/type_convert.hpp"
#include "../api/vio.hpp"
#include "../commandline/command_queue.hpp"
#include "../commandline/parameters.hpp"
#include "../odometry/debug.hpp"
#include "../odometry/util.hpp"
#include "../tracker/util.hpp"
#include "../util/timer.hpp"
#include "../util/util.hpp"
#include "../views/views.hpp"
#include "../views/visualization_internals.hpp"
#include "imu_visualization.hpp"
#include "input.hpp"
#include "video_input.hpp"

#include <accelerated-arrays/future.hpp>
#include <accelerated-arrays/opencv_adapter.hpp>
#ifdef DAZZLING_GPU_ENABLED
    #include <accelerated-arrays/opengl/image.hpp>
    #include <accelerated-arrays/opengl/operations.hpp>
#endif

#ifdef USE_SLAM
    #include "../slam/slam_implementation.hpp"
    #ifdef BUILD_VISUALIZATIONS
        #include "../slam/slam_viewer.hpp"
    #endif
#endif

#ifdef BUILD_VISUALIZATIONS
    #include "../api/slam_map_point_record.hpp"
    #include "visual_update_viewer.hpp"
#endif

namespace {

using PoseHistoryMap = std::map<api::PoseHistory, std::vector<api::Pose>>;

class Visualizer {
public:
    Visualizer(const cmd::Parameters& parameters, bool hasMapViewer, bool hasVuViewer)
        :
#ifdef BUILD_VISUALIZATIONS
    #ifdef USE_SLAM
        mapViewer(parameters, commands)
        ,
    #endif
        vuViewer(odometry::viewer::VisualUpdateViewer::create(parameters, commands))
        ,
#endif
        hasMapViewer(hasMapViewer)
        , hasVuViewer(hasVuViewer)
    {
        // Silence warnings in BUILD_VISUALIZATIONS=OFF build.
        (void)this->hasMapViewer;
        (void)this->hasVuViewer;

        if (parameters.main.gpu) {
#ifdef DAZZLING_GPU_ENABLED
            if (hasMapViewer || hasVuViewer) {
                auto queue = accelerated::Processor::createQueue();
                gpuProcessingQueue = queue.get();
                visualProcessor = std::move(queue);
            } else {
    #ifdef DAZZLING_GLFW_ENABLED
        // if not OpenGL window & context, create a hidden one
        #ifdef __APPLE__
                // On Mac we must run OpenGL stuff in main thread, so use sync mode.
                // TOOD: Process OpenGL stuff from queue on main thread and run
                // algorithm on separate thread
                visualProcessor = accelerated::opengl::createGLFWProcessor(
                    accelerated::opengl::GLFWProcessorMode::SYNC);
        #else
                visualProcessor = accelerated::opengl::createGLFWProcessor(
                    accelerated::opengl::GLFWProcessorMode::ASYNC);
        #endif
    #else
                assert(false && "No GLFW available!");
    #endif
            }
#else
            assert(false && "compiled without GPU support");
#endif
        } else {
            visualProcessor = accelerated::Processor::createInstant();
        }
    }

    void addVisualizationMat(const std::string& title, const cv::Mat& frame)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (visualizations.find(title) == visualizations.end()) {
            visualizations[title] = frame;
        }
    }

    // Call from the main thread. OS X cannot do graphics from outside main
    // thread.
    bool visualizeMats()
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (shouldQuit)
            return false;
#if defined(BUILD_VISUALIZATIONS) && defined(USE_SLAM)
        for (const auto& v : mapViewer.get_data_publisher().pollVisualizations()) {
            visualizations[v.first] = v.second;
        }
#endif
        for (const auto& v : visualizations) {
            assert(!v.second.empty());
            cv::imshow(v.first, v.second);
        }
        // Note! This doesn't conflict with Pangolin, only one of them can detect a
        // single keypress, there is no duplication
        int key = cv::waitKey(1);
        if (key >= 0) {
            commands.keyboardInput(key);
        }
        visualizations.clear();
        return true;
    }

    void terminate()
    {
        log_debug("Visualizer::terminate");
        std::lock_guard<std::mutex> lock(mutex);
        shouldQuit = true;
    }

    void cleanup()
    {
        log_debug("Visualizer::cleanup start");
        if (gpuProcessingQueue) {
            gpuProcessingQueue->processAll();
        } else {
            visualProcessor->enqueue([]() {}).wait();
        }
        log_debug("Visualizer::cleanup completed");
    }

    void setFrameRotation(int rotation)
    {
#if defined(BUILD_VISUALIZATIONS) && defined(USE_SLAM)
        std::lock_guard<std::mutex> lock(mutex);
        mapViewer.get_data_publisher().setFrameRotation(rotation);
#else
        (void)rotation;
#endif
    }

    CommandQueue& getCommands() { return commands; }

    accelerated::Future processMaybeOnGpu(const std::function<void()>& op)
    {
        assert(visualProcessor);
        return visualProcessor->enqueue(op);
    }

#ifdef BUILD_VISUALIZATIONS
    void blockWhilePaused()
    {
    #ifdef USE_SLAM
        while (mapViewer.is_paused()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    #endif
    }

    void setupViewers()
    {
        std::lock_guard<std::mutex> lock(mutex);
    #ifdef USE_SLAM
        if (hasMapViewer)
            mapViewer.setup();
    #endif
        if (hasVuViewer)
            vuViewer->setup();
    }

    void setFixedData(const PoseHistoryMap& poseHistories,
        const Eigen::Matrix4d& imuToCamera,
        const Eigen::Matrix4d& secondImuToCamera)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (hasVuViewer)
            vuViewer->setFixedData(poseHistories, imuToCamera, secondImuToCamera);
    }

    #ifdef USE_SLAM
    slam::viewer::Viewer mapViewer;
    #endif
    std::unique_ptr<odometry::viewer::VisualUpdateViewer> vuViewer;
#else
    void blockWhilePaused()
    {
    }
    void setupViewers() { }
    void drawViewer() { }
#endif

    void draw()
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (shouldQuit)
            return;
#ifdef BUILD_VISUALIZATIONS
    #ifdef USE_SLAM
        if (hasMapViewer)
            mapViewer.draw();
    #endif
        if (hasVuViewer)
            vuViewer->draw();
#endif
        if (gpuProcessingQueue)
            gpuProcessingQueue->processAll();
    }

    accelerated::Processor& getProcessor()
    {
        assert(visualProcessor);
        return *visualProcessor;
    }

private:
    const bool hasMapViewer;
    const bool hasVuViewer;
    CommandQueue commands;
    std::mutex mutex;
    std::map<std::string, cv::Mat> visualizations;
    bool shouldQuit = false;
    std::unique_ptr<accelerated::Processor> visualProcessor;
    accelerated::Queue* gpuProcessingQueue = nullptr;
};

class FpsMeter {
private:
    std::chrono::steady_clock::time_point t0;
    size_t frameCount = 0;
    static constexpr size_t UPDATE_INTERVAL_FRAMES = 10;
    float curFps = 0;

public:
    FpsMeter() { t0 = std::chrono::steady_clock::now(); }

    float update()
    {
        if (++frameCount % UPDATE_INTERVAL_FRAMES == 0) {
            auto now = std::chrono::steady_clock::now();
            auto tNanos = (now - t0).count();
            curFps = UPDATE_INTERVAL_FRAMES / (1e-9 * tNanos);
            t0 = now;
        }
        return curFps;
    }
};

struct VideoConfig {
    int algorithmWidth, algorithmHeight;
    int inputWidth, inputHeight;
    int frameSub = 1;
    double displayScale = 1.0;
    double algorithmScale = 1.0;
};

InputImu setupInputAndOutput(CommandLineParameters& cmd, VideoConfig& videoConfig, int argc, char* argv[])
{
    const cmd::ParametersMain& main = cmd.cmd.main;

    int inputWidth, inputHeight;

    InputImu input;
    input.set_parameters(cmd);

    // parse again to allow overriding automatically set parameters.
    cmd.parse_argv(argc, argv);

    auto fps = input.get_fps();

    videoConfig.frameSub = static_cast<int>(round(fps / cmd.parameters.tracker.targetFps));
    if (videoConfig.frameSub <= 0)
        videoConfig.frameSub = 1;
    log_debug("The framerate is %.1f/%d = %.1f fps.", fps, videoConfig.frameSub, fps / static_cast<double>(videoConfig.frameSub));

    input.get_resolution(inputWidth, inputHeight);

    // Scale of image to feed odometry (tracker). Upsample only explicitly.
    if (main.targetFrameWidthUpsample > 0) {
        videoConfig.algorithmScale = double(main.targetFrameWidthUpsample) / inputWidth;
        if (videoConfig.algorithmScale > 1.0)
            log_warn("Upsampling algorithm frame input.");
    } else {
        videoConfig.algorithmScale = std::min(double(main.targetFrameWidth) / inputWidth, 1.0);
    }
    // Use a dummy resize to predict resized frame dimensions because
    // neither round() nor floor() produce correct results for very small
    // scales.
    cv::Mat resizeTest(inputHeight, inputWidth, CV_8UC1);
    cv::resize(resizeTest, resizeTest, cv::Size(), videoConfig.algorithmScale, videoConfig.algorithmScale, cv::INTER_CUBIC);
    videoConfig.algorithmWidth = resizeTest.cols;
    videoConfig.algorithmHeight = resizeTest.rows;

    // Size of image to display on screen.
    double displayLongerSide = main.windowResolution;
    const int algoLongerSide = std::max(videoConfig.algorithmWidth, videoConfig.algorithmHeight);
    if (displayLongerSide <= 0)
        displayLongerSide = algoLongerSide;

    videoConfig.inputWidth = inputWidth;
    videoConfig.inputHeight = inputHeight;

    // To avoid hassle of scaling pixel coordinates of drawn tracks etc, for
    // display window, scale the already scaled algorithm input image.
    videoConfig.displayScale = displayLongerSide / algoLongerSide;

    if (videoConfig.algorithmScale != 1.0)
        assert(!"Should resize to algorithm{Width,Height}");

    return input;
}

}

int run_algorithm(int argc, char* argv[], Visualizer& visualizer, CommandLineParameters& cmd)
{
    const cmd::ParametersMain& main = cmd.cmd.main;
    if (main.logLevel >= odometry::Parameters::VERBOSITY_DEBUG)
        cmd.parameters.verbosity = odometry::Parameters::VERBOSITY_DEBUG;
    if (main.logLevel >= odometry::Parameters::VERBOSITY_EXTRA)
        cmd.parameters.verbosity = odometry::Parameters::VERBOSITY_EXTRA;

    std::unique_ptr<util::TimeStats> mainTimeStats;
    if (cmd.cmd.main.timer) {
        odometry::TIME_STATS = std::make_unique<util::TimeStats>();
        slam::TIME_STATS = std::make_unique<util::TimeStats>();
        mainTimeStats = std::make_unique<util::TimeStats>();
    }

    VideoConfig videoConfig;

    auto input = setupInputAndOutput(cmd, videoConfig, argc, argv);
    auto frame_intrinsic = input.get_frame_intrin();

    api::InternalAPI::DebugParameters debugParameters;
    debugParameters.recordingPath = main.recordingPath;
    debugParameters.videoRecordingPath = main.videoRecordingPath;
    debugParameters.api.inputFrameWidth = videoConfig.algorithmWidth;
    debugParameters.api.inputFrameHeight = videoConfig.algorithmHeight;
    debugParameters.api.parameters = cmd.parameters;
    debugParameters.nVisualBuffers = 5;
    debugParameters.visualizePointCloud = main.displayPointCloud;
    debugParameters.visualizePoseWindow = main.displayPose;
    debugParameters.videoAlgorithmScale = videoConfig.algorithmScale;
    if (!main.displayVideo) {
        debugParameters.visualization = api::InternalAPI::VisualizationMode::NONE;
    } else if (main.displayTracks) {
        debugParameters.visualization = api::InternalAPI::VisualizationMode::TRACKS;
    } else if (main.displayTracksAll) {
        debugParameters.visualization = api::InternalAPI::VisualizationMode::TRACKS_ALL;
    } else if (main.displayOpticalFlow != odometry::OpticalFlowVisualization::NONE) {
        debugParameters.api.parameters.tracker.saveOpticalFlow = main.displayOpticalFlow;
        if (main.displayOpticalFlow == odometry::OpticalFlowVisualization::FAILURES)
            debugParameters.visualization = api::InternalAPI::VisualizationMode::OPTICAL_FLOW_FAILURES;
        else
            debugParameters.visualization = api::InternalAPI::VisualizationMode::OPTICAL_FLOW;
    } else if (main.displayStereoMatching) {
        debugParameters.visualization = api::InternalAPI::VisualizationMode::STEREO_MATCHING;
    } else if (main.displayCornerMeasure) {
        debugParameters.visualization = api::InternalAPI::VisualizationMode::CORNER_MEASURE;
    } else if (main.displayStereoDisparity) {
        debugParameters.visualization = api::InternalAPI::VisualizationMode::STEREO_DISPARITY;
    } else if (main.displayStereoDepth) {
        debugParameters.visualization = api::InternalAPI::VisualizationMode::STEREO_DEPTH;
    } else if (main.displayStereoEpipolarCurves != odometry::StereoEpipolarVisualization::NONE) {
        debugParameters.api.parameters.tracker.saveStereoEpipolar = main.displayStereoEpipolarCurves;
        debugParameters.visualization = api::InternalAPI::VisualizationMode::STEREO_EPIPOLAR;
    } else if (main.displayPlainVideo) {
        debugParameters.visualization = api::InternalAPI::VisualizationMode::PLAIN_VIDEO;
    }

    if (main.stepMode) {
        visualizer.getCommands().keyboardInput('a'); // TODO. hacky
    }

    auto api = api::buildVio(debugParameters);

    api::OutputBuffer outputBuffer;
    outputBuffer.targetDelaySeconds = debugParameters.api.parameters.odometry.targetOutputDelaySeconds;

    {
        const auto& pt = cmd.parameters.tracker;
        log_debug("Focal length: %g, %g, PP: %g, %g, Input size: %dx%d, Algorithm "
                  "frame size: %d, scale %.2f",
            pt.focalLengthX, pt.focalLengthY, pt.principalPointX,
            pt.principalPointY, videoConfig.inputWidth,
            videoConfig.inputHeight, videoConfig.algorithmWidth,
            videoConfig.algorithmScale);
        if (pt.useStereo) {
            log_debug("Second camera: Focal length: %g, %g, PP: %g, %g",
                pt.secondFocalLengthX, pt.secondFocalLengthY,
                pt.secondPrincipalPointX, pt.secondPrincipalPointY);
        }
    }

    ImuVisualization gyroVisu(5.0), accVisu(30.0), uncertaintyVisuPos(.02), uncertaintyVisuRot(.2);
#ifdef BUILD_VISUALIZATIONS
    std::vector<slam::MapPointRecord> pointCloud;
    odometry::DebugAPI odometryDebug;
    odometryDebug.endDebugCallback = [&pointCloud](const std::map<int, slam::MapPointRecord>& pointCloudExtend) {
        // Convert to vector because we don't need the track ids.
        pointCloud.reserve(pointCloud.size() + pointCloudExtend.size());
        for (const auto& it : pointCloudExtend) {
            pointCloud.push_back(it.second);
        }
    };

    if (main.visualUpdateViewer) {
        odometryDebug.publisher = &visualizer.vuViewer->getPublisher();
    }
    #ifdef USE_SLAM
    if (cmd.parameters.slam.useSlam) {
        odometryDebug.slamDebug = std::make_unique<slam::DebugAPI>();
        odometryDebug.slamDebug->dataPublisher = &visualizer.mapViewer.get_data_publisher();
        odometryDebug.slamDebug->commandQueue = &visualizer.getCommands();
        odometryDebug.slamDebug->mapSavePath = main.slamMapPosesPath;
        odometryDebug.slamDebug->endDebugCallback = [&pointCloud](const std::vector<slam::MapPointRecord>& pointCloudExtend) {
            pointCloud.reserve(pointCloud.size() + pointCloudExtend.size());
            pointCloud.insert(pointCloud.end(), pointCloudExtend.begin(), pointCloudExtend.end());
        };
    }
    #endif
    api->connectDebugApi(odometryDebug);
#endif

    // PoseHistoryMap poseHistories = input.getPoseHistories();
    // for (auto it = poseHistories.begin(); it != poseHistories.end(); ++it) {
    //     // TODO: move to new api
    //     api->setPoseHistory(it->first, it->second);
    // }

#ifdef BUILD_VISUALIZATIONS
    // Duplication from automaticCameraParametersWhereUnset().
    cmd.parameters.imuToCamera = odometry::util::vec2matrix(cmd.parameters.odometry.imuToCameraMatrix);
    if (cmd.parameters.odometry.secondImuToCameraMatrix.size() > 1) {
        cmd.parameters.secondImuToCamera = odometry::util::vec2matrix(cmd.parameters.odometry.secondImuToCameraMatrix);
    } else {
        cmd.parameters.secondImuToCamera = cmd.parameters.imuToCamera;
    }
    // visualizer.setFixedData(poseHistories, cmd.parameters.imuToCamera, cmd.parameters.secondImuToCamera);
#endif

    api->onOutput = [&](std::shared_ptr<const api::VioApi::VioOutput> output) {
        outputBuffer.addProcessedFrame(std::move(output));
    };

    int rotation = cmd.parameters.odometry.rot;
    if (rotation != 0) {
        log_debug("Rotating display video %d degrees clockwise.", 90 * rotation);
    }
    visualizer.setFrameRotation(rotation);

    double lastVisuTime = 0.0;
    bool firstFrame = true;
    bool shouldQuit = false;
    int outputCounter = 0;
    int lastFramesInd = 0;
    bool useStereo = cmd.parameters.tracker.useStereo;
    std::shared_ptr<const api::VioApi::VioOutput> apiOutput = nullptr;

    std::shared_ptr<accelerated::Image> frameVisualizationImage;
    std::shared_ptr<api::Visualization> visualizationVideo;
    if (main.displayVideo)
        visualizationVideo = api->createVisualization("VIDEO");
    std::shared_ptr<api::Visualization> visualizationKfCorrelation;
    std::shared_ptr<api::Visualization> visualizationPose;
    std::shared_ptr<api::Visualization> visualizationCovarianceMagnitude;
    cv::Mat cpuVisuFrame, resizedColorFrame, rotatedColorFrame, corrFrame, poseFrame, magnitudeFrame, prevGrayFrame;
    FpsMeter fpsMeter;

#ifdef DAZZLING_GPU_ENABLED
    std::unique_ptr<accelerated::Image::Factory> gpuImageFactory;
    std::unique_ptr<accelerated::Image> gpuInputImage;
#endif

    // only needed in certain experimental combinations with pyrLKUseGpu
    api->onOpenGlWork = [&visualizer, apiWeak = std::weak_ptr { api }] {
        visualizer.processMaybeOnGpu([apiWeak] {
            if (auto api = apiWeak.lock()) {
                api->processOpenGl();
            }
        });
    };

    for (;;) {
        auto [type, frame] = input.next_frame();

        bool shouldVisualize = false;
        constexpr double INS_ONLY_VISU_INTERVAL = 0.1; // seconds
        bool didOutput = false;

        switch (type) {
        case InputType::Gyroscope: {
            timer(mainTimeStats, "gyroscope");
            auto motion = frame.as<rs2::motion_frame>();
            auto t = motion.get_timestamp();
            auto motion_data = motion.get_motion_data();
            api::Vector3d p { motion_data.x, motion_data.y, motion_data.z };
            api->addGyro(t, p);
            if (main.displayImuSamples)
                gyroVisu.addSample(t, p);

            if (!cmd.parameters.odometry.visualUpdateEnabled && t > lastVisuTime + INS_ONLY_VISU_INTERVAL) {
                lastVisuTime = t;
                shouldVisualize = true;
            }
            break;
        }
        case InputType::Accelerometer: {
            timer(mainTimeStats, "accelerometer");
            auto motion = frame.as<rs2::motion_frame>();
            auto t = motion.get_timestamp();
            auto motion_data = motion.get_motion_data();
            api::Vector3d p { motion_data.x, motion_data.y, motion_data.z };
            api->addAcc(t, p);
            if (main.displayImuSamples)
                accVisu.addSample(t, p);
            break;
        }
        case InputType::Video: {
            if (mainTimeStats)
                mainTimeStats->startFrame();
            timer(mainTimeStats, "frame");

            auto video = frame.as<rs2::video_frame>();
            auto t = video.get_timestamp();

            // If resized, then sizes are not equal
            assert(video.get_width() == videoConfig.inputWidth && video.get_height() == videoConfig.inputHeight);
            assert(video.get_data_size() == video.get_width() * video.get_height() * video.get_bytes_per_pixel());

            // Lifetime of frame data?
            cv::Mat input_frame { video.get_height(), video.get_width(), CV_8UC3, const_cast<void*>(video.get_data()) }; // const_cast is a hack

            // Check frame indices are consecutive?

            if (videoConfig.algorithmScale != 1.0) {
                assert(!"Cannot handle scale that is not 1");
                // odometry::InputFrame& meta = frames.at(cameraInd);
                // meta.intrinsic.focalLengthX *= videoConfig.algorithmScale;
                // meta.intrinsic.focalLengthY *= videoConfig.algorithmScale;
            }

            if (cmd.parameters.verbosity >= odometry::Parameters::VERBOSITY_DEBUG && apiOutput) {
                // TODO: Relies on internal API
                auto& debug = reinterpret_cast<const api::InternalAPI::Output&>(*apiOutput);
                std::cout << debug.stateAsString << std::endl;
                // Velocity pseudo update only considers the x and y components, so
                // this print can give some idea how it's doing.
                auto v = apiOutput->velocity;
                double speed = std::sqrt(v.x * v.x + v.y * v.y);
                std::cout << "xy speed: " << (speed * 3.6) << "km/h"
                          << " FPS " << fpsMeter.update() << std::endl;
            }
            if (cmd.parameters.verbosity >= odometry::Parameters::VERBOSITY_EXTRA) {
                std::cout << "t in microseconds " << static_cast<long>(std::round(t * 1e6)) << std::endl;
            }

            shouldVisualize = true;

            if (cmd.parameters.tracker.matchSuccessiveIntensities > 0.0) {
                if (input_frame.type() != CV_8UC1)
                    cvtColor(input_frame, input_frame, cv::COLOR_BGR2GRAY);
                if (prevGrayFrame.empty())
                    prevGrayFrame = input_frame.clone();
                tracker::util::matchIntensities(prevGrayFrame, input_frame, cmd.parameters.tracker.matchSuccessiveIntensities);
                input_frame.copyTo(prevGrayFrame);
            }
            // if (cmd.parameters.tracker.useStereo && cmd.parameters.tracker.matchStereoIntensities) {
            //     assert(frames.size() == 2);
            //     cv::Mat& im0 = *inputFrames[0];
            //     cv::Mat& im1 = *inputFrames[1];
            //     if (im0.type() != CV_8UC1)
            //         cvtColor(im0, im0, cv::COLOR_BGR2GRAY);
            //     if (im1.type() != CV_8UC1)
            //         cvtColor(im1, im1, cv::COLOR_BGR2GRAY);
            //     tracker::util::matchIntensities(im0, im1);
            // }

            accelerated::Image* frameImage;
            auto openCvImg = accelerated::opencv::ref(input_frame, true); // prefer fixed point
            if (main.gpu) {
#ifdef DAZZLING_GPU_ENABLED
                if (!gpuImageFactory)
                    gpuImageFactory = accelerated::opengl::Image::createFactory(visualizer.getProcessor());
                if (!gpuInputImage)
                    gpuInputImage = gpuImageFactory->createLike(*openCvImg);
                // async, but no need to wait here.
                // NOTE: there is a risk that the cv::Mat is freed / overwritten
                // unless there is sufficient buffering in VideoInput
                accelerated::opencv::copy(input_frame, *gpuInputImage);
                frameImage = gpuInputImage.get();
#else
                assert(false);
#endif
            } else {
                frameImage = openCvImg.get();
            }

            auto& firstImage = *frameImage;

            auto out = outputBuffer.pollOutput(t);
            didOutput = !!out;
            if (out) {
                apiOutput = out->output;
                outputCounter++;
            }

            auto colorFormat = firstImage.channels == 1 ? api::VioApi::ColorFormat::GRAY : api::VioApi::ColorFormat::RGB;

            visualizer.processMaybeOnGpu([&] {
                          if (firstImage.storageType == accelerated::Image::StorageType::CPU) {
                              api->addFrameMonoVarying(
                                  t, frame_intrinsic, firstImage.width,
                                  firstImage.height,
                                  accelerated::cpu::Image::castFrom(firstImage).getDataRaw(),
                                  colorFormat, 0);
                          } else {
#ifdef DAZZLING_GPU_ENABLED
                              api->addFrameMonoOpenGl(
                                  t, frame_intrinsic, firstImage.width,
                                  firstImage.height,
                                  accelerated::opengl::Image::castFrom(firstImage).getTextureId(),
                                  colorFormat, 0);
#endif
                          }
                      })
                .wait();

            break;
        }
        }

        if (outputCounter % main.visuUpdateInterval != 0)
            shouldVisualize = false;

        if (shouldVisualize) {
            timer(mainTimeStats, "visualizations");
            if (main.displayVideo && didOutput) {
                assert(apiOutput);
                if (!frameVisualizationImage)
                    frameVisualizationImage = visualizationVideo->createDefaultRenderTarget();
                visualizationVideo->update(apiOutput);
                visualizer.processMaybeOnGpu([&] { visualizationVideo->render(*frameVisualizationImage); }).wait();
                if (frameVisualizationImage->storageType == accelerated::Image::StorageType::CPU) {
                    // The visualization produced output in CPU format (as a cv::Mat)
                    cpuVisuFrame = accelerated::opencv::ref(*frameVisualizationImage);
                } else {
                    // ... in GPU format
                    accelerated::opencv::copy(*frameVisualizationImage, cpuVisuFrame).wait();
                    // note: it would also be relatively simple to GPU-rescale here
                }
                // Scale and rotate for display on screen.
                if (videoConfig.displayScale == 1.0) {
                    resizedColorFrame = cpuVisuFrame;
                } else {
                    cv::resize(cpuVisuFrame, resizedColorFrame, cv::Size(), videoConfig.displayScale, videoConfig.displayScale, cv::INTER_CUBIC);
                }
                tracker::util::rotateMatrixCW90(resizedColorFrame, rotatedColorFrame, rotation);
                visualizer.addVisualizationMat("odometry", rotatedColorFrame);
            }
            if (main.displayCorrelation) {
                if (!visualizationKfCorrelation)
                    visualizationKfCorrelation = api->createVisualization("KF_CORRELATION");
                visualizer.processMaybeOnGpu([&]() { visualizationKfCorrelation->render(corrFrame); }).wait();
                visualizer.addVisualizationMat("correlation", corrFrame);
            }
            if (main.displayPose && apiOutput) {
                constexpr int gray = 150;
                poseFrame = cv::Scalar(gray, gray, gray);
                if (!visualizationPose)
                    visualizationPose = api->createVisualization("POSE");
                visualizationPose->update(apiOutput);
                if (visualizationPose->ready()) {
                    visualizer.processMaybeOnGpu([&]() { visualizationPose->render(poseFrame); }).wait();
                    visualizer.addVisualizationMat("pose", poseFrame);
                }
            }
            if (main.displayCovarianceMagnitude) {
                if (!visualizationCovarianceMagnitude)
                    visualizationCovarianceMagnitude = api->createVisualization("COVARIANCE_MAGNITUDES");
                visualizer.processMaybeOnGpu([&]() {
                              visualizationCovarianceMagnitude->render(magnitudeFrame);
                          })
                    .wait();
                visualizer.addVisualizationMat("covariance magnitude, absolute, log scale", magnitudeFrame);
            }
            if (main.displayImuSamples && apiOutput) {
                const double t = apiOutput->pose.time;
                visualizer.addVisualizationMat("gyroscope", gyroVisu.draw(t));
                visualizer.addVisualizationMat("accelerometer", accVisu.draw(t));
            }

            // Handle user input.
            visualizer.blockWhilePaused();
            if (visualizer.getCommands().getStepMode() == CommandQueue::StepMode::ODOMETRY) {
                visualizer.getCommands().waitForAnyKey();
            }
            while (!visualizer.getCommands().empty()) {
                auto command = visualizer.getCommands().dequeue();
                if (command.type == CommandQueue::Type::QUIT) {
                    shouldQuit = true;
                } else if (command.type == CommandQueue::Type::POSE) {
                    auto kind = api::PoseHistory::OUR;
                    bool valid = true;
                    switch (command.value) {
                    case 1:
                        kind = api::PoseHistory::OUR;
                        break;
                    case 2:
                        kind = api::PoseHistory::GROUND_TRUTH;
                        break;
                    case 3:
                        kind = api::PoseHistory::EXTERNAL;
                        break;
                    case 4:
                        kind = api::PoseHistory::OUR_PREVIOUS;
                        break;
                    case 5:
                        kind = api::PoseHistory::GPS;
                        break;
                    case 6:
                        kind = api::PoseHistory::RTK_GPS;
                        break;
                    default:
                        valid = false;
                        break;
                    }
                    if (valid && main.displayPose) {
                        bool value = api->getPoseOverlayHistoryShown(kind);
                        api->setPoseOverlayHistoryShown(kind, !value);
                    }
                } else if (command.type == CommandQueue::Type::LOCK_BIASES) {
                    api->lockBiases();
                } else if (command.type == CommandQueue::Type::ROTATE) {
                    rotation++;
                    visualizer.setFrameRotation(rotation);
                } else if (command.type == CommandQueue::Type::CONDITION_ON_LAST_POSE) {
                    api->conditionOnLastPose();
                } else if (command.type == CommandQueue::Type::NONE) {
                    // This shouldn't happen, because we check that Queue isn't empty
                    break;
                } else if (command.type == CommandQueue::Type::ANY_KEY) {
                    // Ignore other keypresses, just remove them from queue
                } else {
                    log_warn("Unknown command type %u", command.type);
                }
            }
        }
        if (shouldQuit)
            break;
    }

    if (!main.skipOpenGlCleanup) {
        visualizer.processMaybeOnGpu([&] {
                      log_debug("api->cleanupOpenGl()");
                      api->destroyOpenGl();
                  })
            .wait();
    }

    visualizer.terminate();

    // Blocks until the SLAM worker threads have quit
    log_debug("api.reset()");
    api.reset();

    if (odometry::TIME_STATS) {
        log_info("Odometry %s", odometry::TIME_STATS->perFrameTimings().c_str());
    }
    if (cmd.parameters.slam.useSlam && slam::TIME_STATS) {
        log_info("Slam %s", slam::TIME_STATS->perFrameTimings().c_str());
    }
    if (mainTimeStats) {
        log_info("Main %s", mainTimeStats->perFrameTimings().c_str());
    }

    return 0;
}

int run_visualizer(bool anyVisualProcessing, Visualizer& visualizer)
{
    if (!anyVisualProcessing)
        return 0;

    visualizer.setupViewers();
    while (visualizer.visualizeMats()) {
        visualizer.draw();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    visualizer.cleanup();
    log_debug("run_visualizer end");
    return 0;
}

int main(int argc, char* argv[])
{
    CommandLineParameters cmd(argc, argv);
    util::setup_logging(cmd.cmd.main.logLevel);

    const bool anyVisualizations = cmd.cmd.main.displayPose || cmd.cmd.main.visualUpdateViewer || cmd.cmd.main.displayVideo || cmd.cmd.main.displayCorrelation || cmd.cmd.main.displayCovarianceMagnitude || cmd.cmd.main.displayImuSamples || cmd.cmd.main.displayStereoMatching || cmd.cmd.slam.displayViewer || cmd.cmd.slam.displayKeyframe || cmd.cmd.slam.visualizeMapPointSearch || cmd.cmd.slam.visualizeOrbMatching || cmd.cmd.slam.visualizeLoopOrbMatching;

    const bool needVisualProcessing = anyVisualizations || cmd.cmd.main.gpu;

    const bool mapViewer = cmd.parameters.slam.useSlam && cmd.cmd.slam.displayViewer;
    const bool vuViewer = cmd.cmd.main.visualUpdateViewer;
    Visualizer visualizer(cmd.cmd, mapViewer, vuViewer);
    auto ret = std::async(run_visualizer, needVisualProcessing, std::ref(visualizer));

    run_algorithm(argc, argv, visualizer, cmd);

    log_debug("waiting for ret.get() at the end of main");
    return ret.get();
}
