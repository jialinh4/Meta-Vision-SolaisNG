//
// Created by liuzikai on 3/11/21.
//

#include "Camera.h"
#include <iostream>

namespace meta {

bool Camera::open(const package::ParamSet &params) {
    capInfoSS.str(std::string());

    // Open the camera_
    cap.open(params.camera_id(), cv::CAP_ANY);
    if (!cap.isOpened()) {
        capInfoSS << "Failed to open camera_ " << params.camera_id() << "\n";
        std::cerr << capInfoSS.rdbuf();
        return false;
    }

    // Set parameters
    if (!cap.set(cv::CAP_PROP_FRAME_WIDTH, params.image_width())) {
        capInfoSS << "Failed to set width.\n";
    }
    if (!cap.set(cv::CAP_PROP_FRAME_HEIGHT, params.image_height())) {
        capInfoSS << "Failed to set height.\n";
    }
    if (!cap.set(cv::CAP_PROP_FPS, params.fps())) {
        capInfoSS << "Failed to set fps.\n";
    }
    if (params.gamma().enabled()) {
        if (!cap.set(cv::CAP_PROP_GAMMA, params.gamma().val())) {
            capInfoSS << "Failed to set gamma.\n";
        }
    }

    // Get a test frame
    cap.read(buffer[0]);
    if (buffer->empty()) {
        capInfoSS << "Failed to fetch test image from camera_ " << params.camera_id() << "\n";
        std::cerr << capInfoSS.rdbuf();
        return false;
    }
    if (buffer[0].cols != params.image_width() || buffer[0].rows != params.image_height()) {
        capInfoSS << "Invalid frame size. "
                  << "Expected: " << params.image_width() << "x" << params.image_height() << ", "
                  << "Actual: " << buffer[0].cols << "x" << buffer[0].rows << "\n";
        std::cerr << capInfoSS.rdbuf();
        return false;
    }

    // Report actual parameters
    capInfoSS << "Camera " << params.camera_id() << ", "
              << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT)
              << " @ " << cap.get(cv::CAP_PROP_FPS) << " fps\n"
              << "Gamma: " << cap.get(cv::CAP_PROP_GAMMA) << "\n";
    std::cout << capInfoSS.rdbuf();

    // Start the fetching thread
    if (th) {
        release();
    }
    threadShouldExit = false;
    th = new std::thread(&Camera::readFrameFromCamera, this);

    return true;
}

Camera::~Camera() {
    release();
}

void Camera::registerNewFrameCallBack(Camera::NewFrameCallBack callBack, void *param) {
    callbacks.emplace_back(callBack, param);
}

void Camera::readFrameFromCamera() {

    while(!threadShouldExit) {

        int workingBuffer = 1 - lastBuffer;

        if (!cap.isOpened()) {
            break;
        }

        if (!cap.read(buffer[workingBuffer])) {
            continue;
        }

        bufferFrameID[workingBuffer] = bufferFrameID[lastBuffer] + 1;
        lastBuffer = workingBuffer;

        for (const auto &item : callbacks) {
            item.first(item.second);
        }

    }

    cap.release();
    std::cout << "Camera closed\n";
}

void Camera::release() {
    if (th) {
        threadShouldExit = true;
        th->join();
        delete th;
        th = nullptr;
    }
}

}