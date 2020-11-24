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

#include <librealsense2/rs.hpp>

// osc settings
#define HOST ("127.0.0.1")
#define PORT ("7400")

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
rs2::pipeline pipeline;
bool isRealSenseMode = false;

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

void setupRealSense() {
    std::cout << "setting up realsense..." << std::endl;

    // setting up realsense
    pipeline.start();

    rs2::frameset frames = pipeline.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();

    float width = depth.get_width();
    float height = depth.get_height();

    float dist_to_center = depth.get_distance(width / 2, height / 2);

    std::cout << "The camera is facing an object " << dist_to_center << " meters away" << std::endl;
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

        if (FLAGS_i == "cam") {
            int index = std::stoi(FLAGS_index, nullptr, 0);
            if (!cap.open(index))
                throw std::logic_error("Cannot open input camera: " + FLAGS_i + " index: " + FLAGS_index);
        } else if (FLAGS_i == "rs") {
            // realsense
            setupRealSense();
            isRealSenseMode = true;
        } else {
            if (!cap.open(FLAGS_i))
                throw std::logic_error("Cannot open input file: " + FLAGS_i);
        }

        std::cout << "Started OSC client on port [" << PORT << "]" << std::endl;

        int delay = 33;
        double inferenceTime = 0.0;
        cv::Mat image;
        if(isRealSenseMode) {
            //todo: read realsense frame
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
