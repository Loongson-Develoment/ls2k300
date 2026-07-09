#include "Camera.h"


Camera::Camera()
    : width_(CAMERA_WIDTH)
    , height_(CAMERA_HEIGHT)
    , fps_(CAMERA_FPS)
    , buffers_(CAMERA_BUFFERS)
{
}


bool Camera::Open()
{
    cap_.open("/dev/video0", cv::CAP_V4L2);
    if (!cap_.isOpened()) 
    {
        std::cerr << "Error: Could not open camera" << std::endl;
        return false;
    }

    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap_.set(cv::CAP_PROP_CONVERT_RGB, 0);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, buffers_);
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap_.set(cv::CAP_PROP_EXPOSURE, 300);

    return true;
}


void Camera::Close()
{
    if (cap_.isOpened()) {
        cap_.release();
        std::cout << "[INFO] deinit camera" << std::endl;
    }

}

bool Camera::ReadFrame(cv::Mat& frame)
{
    if (!cap_.isOpened()) {
        std::cerr << "Error: Camera not opened" << std::endl;
        return false;
    }

    if (!cap_.read(frame)) {
        std::cerr << "Error: Could not read frame from camera" << std::endl;
        return false;
    }
    cv::cvtColor(frame, frame, cv::COLOR_YUV2BGR_YUYV);

    return true;
}

Camera::~Camera()
{
    Close();
}





