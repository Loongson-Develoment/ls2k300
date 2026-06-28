#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "opencv2/opencv.hpp"

#define CAMERA_WIDTH   640
#define CAMERA_HEIGHT  480
#define CAMERA_FPS     120
#define CAMERA_BUFFERS 1


class Camera{

public:
    Camera();
    ~Camera();

    bool Open();
    void Close();
    bool ReadFrame(cv::Mat& frame);

private:
    cv::VideoCapture cap_;
    int width_;
    int height_;
    int fps_;
    int buffers_;



};


#endif /* _CAMERA_H_ */
