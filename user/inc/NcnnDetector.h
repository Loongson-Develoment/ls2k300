#ifndef USER_NCNN_DETECTOR_H
#define USER_NCNN_DETECTOR_H

#include <string>
#include <vector>

#include <net.h>
#include "opencv2/opencv.hpp"

struct NcnnDetection {
    cv::Rect box;
    float score;
};

struct NcnnDetectTiming {
    double preprocess_ms;
    double inference_ms;
    double postprocess_ms;
    double total_ms;

    NcnnDetectTiming()
        : preprocess_ms(0.0)
        , inference_ms(0.0)
        , postprocess_ms(0.0)
        , total_ms(0.0)
    {
    }
};

class NcnnDetector {
public:
    NcnnDetector();

    bool Load(const std::string& param_path, const std::string& bin_path);
    bool IsLoaded() const;
    const std::string& LastError() const;
    const NcnnDetectTiming& LastTiming() const;
    bool Detect(const cv::Mat& img, std::vector<NcnnDetection>* detections, NcnnDetectTiming* timing = 0);

private:
    ncnn::Mat Transform(const cv::Mat& img);
    ncnn::Mat Reasoning(const cv::Mat& img, NcnnDetectTiming* timing);

    ncnn::Net net_;
    bool loaded_;
    std::string last_error_;
    float conf_threshold_;
    float nms_threshold_;
    NcnnDetectTiming last_timing_;
};

#endif
