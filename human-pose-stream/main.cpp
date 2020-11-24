// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

/**
* \brief The entry point for the Inference Engine Human Pose Estimation demo application
* \file human_pose_estimation_demo/main.cpp
* \example human_pose_estimation_demo/main.cpp
*/

#include <vector>
#include <string>

#include <inference_engine.hpp>

#include <samples/ocv_common.hpp>

#include "human_pose_estimation_demo.hpp"
#include "human_pose_estimator.hpp"
#include "render_human_pose.hpp"

#include "tnyosc.hpp"
#include <boost/asio.hpp>

#include <librealsense2/rs.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_frame.h>

// osc settings
#define HOST ("127.0.0.1")
#define PORT ("7400")

#define INPUT_IR

#if defined(INPUT_COLOR)
    #define STREAM          RS2_STREAM_COLOR
    #define FORMAT          RS2_FORMAT_BGR8
    #define WIDTH           640
    #define HEIGHT          480
    #define FPS             30
    #define STREAM_INDEX    0
#elif defined(INPUT_IR_RGB)
    #define STREAM          RS2_STREAM_INFRARED
    #define FORMAT          RS2_FORMAT_BGR8
    #define WIDTH           640
    #define HEIGHT          480
    #define FPS             30
    #define STREAM_INDEX    0
#else
    #define STREAM          RS2_STREAM_INFRARED
    #define FORMAT          RS2_FORMAT_Y8
    #define WIDTH           640
    #define HEIGHT          480
    #define FPS             30
    #define STREAM_INDEX    1
#endif

using namespace InferenceEngine;
using namespace human_pose_estimation;
using boost::asio::ip::udp;

// osc setup
boost::asio::io_service io_service;
udp::socket oscSocket(io_service, udp::endpoint(udp::v4(), 0));
udp::resolver resolver(io_service);
udp::resolver::query query(udp::v4(), HOST, PORT);
udp::resolver::iterator iterator = resolver.resolve(query);

// realsense
bool isRealSenseMode = false;

void check_error(rs2_error* e)
{
    if (e)
    {
        printf("rs_error was raised when calling %s(%s):\n", rs2_get_failed_function(e), rs2_get_failed_args(e));
        printf("    %s\n", rs2_get_error_message(e));
        exit(EXIT_FAILURE);
    }
}

void print_device_info(rs2_device* dev)
{
    rs2_error* e = 0;
    printf("\nUsing device 0, an %s\n", rs2_get_device_info(dev, RS2_CAMERA_INFO_NAME, &e));
    check_error(e);
    printf("    Serial number: %s\n", rs2_get_device_info(dev, RS2_CAMERA_INFO_SERIAL_NUMBER, &e));
    check_error(e);
    printf("    Firmware version: %s\n\n", rs2_get_device_info(dev, RS2_CAMERA_INFO_FIRMWARE_VERSION, &e));
    check_error(e);
}

bool ParseAndCheckCommandLine(int argc, char *argv[]) {
    // ---------------------------Parsing and validation of input args--------------------------------------

    gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
    if (FLAGS_h) {
        showUsage();
        showAvailableDevices();
        return false;
    }

    std::cout << "Parsing input parameters" << std::endl;

    if (FLAGS_i.empty()) {
        throw std::logic_error("Parameter -i is not set");
    }

    if (FLAGS_m.empty()) {
        throw std::logic_error("Parameter -m is not set");
    }

    return true;
}

void sendToOsc(const std::vector<HumanPose>& poses, float width, float height) {
    int count = 0;
    tnyosc::Bundle bundle;

    tnyosc::Message posesMsg("/poses");
    posesMsg.append(static_cast<int>(poses.size()));
    bundle.append(posesMsg);

    for (HumanPose const &pose : poses) {
        // std::cout << "Score [" << count << "]: " << pose.score << std::endl;
        tnyosc::Message msg("/pose");

        msg.append(count);
        msg.append(pose.score / 100.0f);

        for (auto const &keypoint : pose.keypoints) {
            msg.append(keypoint.x / width);
            msg.append(keypoint.y / height);
        }

        bundle.append(msg);
    }

    oscSocket.send_to(boost::asio::buffer(bundle.data(), bundle.size()), *iterator);
}

int main(int argc, char *argv[]) {
    try {
        std::cout << "InferenceEngine: " << GetInferenceEngineVersion() << std::endl;

        // ------------------------------ Parsing and validation of input args ---------------------------------
        if (!ParseAndCheckCommandLine(argc, argv)) {
            return EXIT_SUCCESS;
        }

        HumanPoseEstimator estimator(FLAGS_m, FLAGS_d, FLAGS_pc);
        cv::VideoCapture cap;
        rs2_error* e = 0;

        rs2_context* ctx = rs2_create_context(RS2_API_VERSION, &e);
        check_error(e);

        /* Get a list of all the connected devices. */
        // The returned object should be released with rs2_delete_device_list(...)
        rs2_device_list* device_list = rs2_query_devices(ctx, &e);
        check_error(e);

        int dev_count = rs2_get_device_count(device_list, &e);
        check_error(e);
        printf("There are %d connected RealSense devices.\n", dev_count);
        if (0 == dev_count)
            return EXIT_FAILURE;

        // Get the first connected device
        // The returned object should be released with rs2_delete_device(...)
        rs2_device* dev = rs2_create_device(device_list, 0, &e);
        check_error(e);

        print_device_info(dev);

        // Create a pipeline to configure, start and stop camera streaming
        // The returned object should be released with rs2_delete_pipeline(...)
        rs2_pipeline* pipeline =  rs2_create_pipeline(ctx, &e);
        check_error(e);

        // Create a config instance, used to specify hardware configuration
        // The retunred object should be released with rs2_delete_config(...)
        rs2_config* config = rs2_create_config(&e);
        check_error(e);

        // Request a specific configuration
        rs2_config_enable_stream(config, STREAM, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS, &e);
        check_error(e);

        rs2_pipeline_profile* pipeline_profile;

        if (FLAGS_i == "cam") {
            int index = std::stoi(FLAGS_index, nullptr, 0);
            if (!cap.open(index))
                throw std::logic_error("Cannot open input camera: " + FLAGS_i + " index: " + FLAGS_index);
        } else if (FLAGS_i == "rs") {
            printf("setting sensor options...");
            rs2_sensor_list* sensors = rs2_query_sensors(dev, &e);
            check_error(e);
            // 0 => Depth sensor
            // 1 => Color Sensor (hopefully)
            rs2_sensor* sensor = rs2_create_sensor(sensors, 0, &e);
            check_error(e);
            rs2_options* options =  (rs2_options*)sensor;
            check_error(e);
            // settings
            rs2_set_option(options, RS2_OPTION_EMITTER_ENABLED, 0, &e);
            //rs2_set_option(options, RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0, &e);
            //rs2_set_option(options, RS2_OPTION_EXPOSURE, 20 * 1000, &e);

            // more options:
            // https://intelrealsense.github.io/librealsense/doxygen/rs__option_8h.html#a8b9c011f705cfab20c7eaaa7a26040e2

            check_error(e);
            printf("done!");

            // Start the pipeline streaming
            // The retunred object should be released with rs2_delete_pipeline_profile(...)
            pipeline_profile = rs2_pipeline_start_with_config(pipeline, config, &e);
            if (e)
            {
                printf("The connected device doesn't support color streaming!\n");
                exit(EXIT_FAILURE);
            }
            std::cout << "librealsense running!" << std::endl;

            isRealSenseMode = true;
        } else {
            if (!cap.open(FLAGS_i))
                throw std::logic_error("Cannot open input file: " + FLAGS_i);
        }

        std::cout << "Started OSC client on port [" << PORT << "]" << std::endl;

        int delay = 33;
        double inferenceTime = 0.0;
        cv::Mat image;
        cv::Mat irImage;

        rs2_frame* frame;
        rs2_frame* frames;

        if(isRealSenseMode) {
            frames = rs2_pipeline_wait_for_frames(pipeline, RS2_DEFAULT_TIMEOUT, &e);
            check_error(e);

            // just take first frame
            frame = rs2_extract_frame(frames, 0, &e);
            check_error(e);

            unsigned long long frame_number = rs2_get_frame_number(frame, &e);
            check_error(e);

            std::cout << "Frame Number: " << frame_number << std::endl;

            // copy to image
            if(FORMAT == RS2_FORMAT_Y8) {
                const unsigned char* ir_frame_data = (const unsigned char*)(rs2_get_frame_data(frame, &e));
                check_error(e);
                irImage = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC1, (void*)ir_frame_data, cv::Mat::AUTO_STEP);
                cvtColor(irImage, image, cv::COLOR_GRAY2RGB);
            } else {
                const uint8_t* rgb_frame_data = (const uint8_t*)(rs2_get_frame_data(frame, &e));
                image = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)rgb_frame_data, cv::Mat::AUTO_STEP);
            }

            //  maybe not necessary
            rs2_release_frame(frame);
            rs2_release_frame(frames);
        } else {
            if (!cap.read(image)) {
                throw std::logic_error("Failed to get frame from cv::VideoCapture");
            }
        }
        estimator.estimate(image);  // Do not measure network reshape, if it happened

        std::cout << "To close the application, press 'CTRL+C' here";
        if (!FLAGS_no_show) {
            std::cout << " or switch to the output window and press ESC key" << std::endl;
            std::cout << "To pause execution, switch to the output window and press 'p' key" << std::endl;
        }
        std::cout << std::endl;

        do {
            double t1 = static_cast<double>(cv::getTickCount());
            if(FORMAT == RS2_FORMAT_Y8) {
                cvtColor(irImage, image, cv::COLOR_GRAY2RGB);
            }
            std::vector <HumanPose> poses = estimator.estimate(image);
            double t2 = static_cast<double>(cv::getTickCount());
            if (inferenceTime == 0) {
                inferenceTime = (t2 - t1) / cv::getTickFrequency() * 1000;
            } else {
                inferenceTime = inferenceTime * 0.95 + 0.05 * (t2 - t1) / cv::getTickFrequency() * 1000;
            }

            if (FLAGS_r) {
                for (HumanPose const &pose : poses) {
                    std::stringstream rawPose;
                    rawPose << std::fixed << std::setprecision(0);
                    for (auto const &keypoint : pose.keypoints) {
                        rawPose << keypoint.x << "," << keypoint.y << " ";
                    }
                    rawPose << pose.score;
                    std::cout << rawPose.str() << std::endl;
                }
            }

            // send to osc
            sendToOsc(poses, static_cast<float>(image.cols), static_cast<float>(image.rows));

            if (FLAGS_no_show) {
                continue;
            }

            renderHumanPose(poses, image);

            cv::Mat fpsPane(35, 155, CV_8UC3);
            fpsPane.setTo(cv::Scalar(153, 119, 76));
            cv::Mat srcRegion = image(cv::Rect(8, 8, fpsPane.cols, fpsPane.rows));
            cv::addWeighted(srcRegion, 0.4, fpsPane, 0.6, 0, srcRegion);
            std::stringstream fpsSs;
            fpsSs << "FPS: " << int(1000.0f / inferenceTime * 100) / 100.0f;
            cv::putText(image, fpsSs.str(), cv::Point(16, 32),
                        cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0, 0, 255));
            cv::imshow("Human Pose Stream", image);

            int key = cv::waitKey(delay) & 255;
            if (key == 'p') {
                delay = (delay == 0) ? 33 : 0;
            } else if (key == 27) {
                break;
            }

            // read image
            if(isRealSenseMode) {
                // todo: read realsense frame
            } else {
                cap.read(image);
            }
        } while (true);

        // Stop the pipeline streaming
        rs2_pipeline_stop(pipeline, &e);
        check_error(e);

        // Release resources
        rs2_delete_pipeline_profile(pipeline_profile);
        rs2_delete_config(config);
        rs2_delete_pipeline(pipeline);
        rs2_delete_device(dev);
        rs2_delete_device_list(device_list);
        rs2_delete_context(ctx);

    }
    catch (const std::exception &error) {
        std::cerr << "[ ERROR ] " << error.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (...) {
        std::cerr << "[ ERROR ] Unknown/internal exception happened." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Execution successful" << std::endl;
    return EXIT_SUCCESS;
}
