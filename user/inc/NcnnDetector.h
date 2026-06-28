#ifndef USER_NCNN_DETECTOR_H
#define USER_NCNN_DETECTOR_H

#include <string>
#include <vector>

#include <net.h>
#include "opencv2/opencv.hpp"

struct DetectResult {
    float center_x;
    float center_y;
    float width;
    float height;
    float score;
};

class NcnnDetector {
public:
    NcnnDetector();

    bool Load(const std::string& param_path, const std::string& bin_path);
    bool IsLoaded() const;
    const std::string& LastError() const;
    ncnn::Mat Reasoning(const cv::Mat& img);
    bool Detect(const ncnn::Mat& out, int img_width, int img_height, std::vector<DetectResult>& results);
    DetectResult Max_score(std::vector<DetectResult>& results);


private:
    ncnn::Mat Transform(const cv::Mat& img);
    static bool modelfile_exists(const char *path);
    ncnn::Net net_;
    bool loaded_;
    std::string last_error_;
    float conf_threshold_;
    float nms_threshold_;
    const char* ncnn_model_param_path = "./model.ncnn.param";
    const char* ncnn_model_bin_path = "./model.ncnn.bin";
};

#endif
