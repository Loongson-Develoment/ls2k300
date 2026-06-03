#include "NcnnDetector.h"

#include <algorithm>
#include <chrono>

namespace {

double DurationMs(const std::chrono::steady_clock::time_point& start,
                  const std::chrono::steady_clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

}  // namespace

NcnnDetector::NcnnDetector()
    : loaded_(false)
    , conf_threshold_(0.5f)
    , nms_threshold_(0.45f)
{
    net_.opt.use_vulkan_compute = false;
    net_.opt.num_threads = 1;
}

bool NcnnDetector::Load(const std::string& param_path, const std::string& bin_path)
{
    loaded_ = false;
    last_error_.clear();
    net_.clear();

    if (param_path.empty() || bin_path.empty()) {
        last_error_ = "model path is empty";
        return false;
    }

    if (net_.load_param(param_path.c_str()) != 0) {
        last_error_ = "load_param failed: " + param_path;
        return false;
    }

    if (net_.load_model(bin_path.c_str()) != 0) {
        last_error_ = "load_model failed: " + bin_path;
        net_.clear();
        return false;
    }

    loaded_ = true;
    return true;
}

bool NcnnDetector::IsLoaded() const
{
    return loaded_;
}

const std::string& NcnnDetector::LastError() const
{
    return last_error_;
}

const NcnnDetectTiming& NcnnDetector::LastTiming() const
{
    return last_timing_;
}

ncnn::Mat NcnnDetector::Transform(const cv::Mat& img)
{
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(img.data, ncnn::Mat::PIXEL_BGR2RGB, img.cols, img.rows, 128, 128);
    float mean_vals[3] = {0.0f, 0.0f, 0.0f};
    float norm_vals[3] = {1.0/255.0f, 1.0/255.0f, 1.0/255.0f};
    in.substract_mean_normalize(mean_vals, norm_vals);
    return in;
}

ncnn::Mat NcnnDetector::Reasoning(const cv::Mat& img, NcnnDetectTiming* timing)
{
    ncnn::Mat out;

    if (!loaded_) {
        last_error_ = "model is not loaded";
        return out;
    }

    const std::vector<const char*>& input_names = net_.input_names();
    const std::vector<const char*>& output_names = net_.output_names();

    if (input_names.empty() || output_names.empty()) {
        last_error_ = "input or output names are empty";
        return out;
    }

    ncnn::Extractor ex = net_.create_extractor();
    const std::chrono::steady_clock::time_point preprocess_start = std::chrono::steady_clock::now();
    ncnn::Mat in = Transform(img);
    const std::chrono::steady_clock::time_point preprocess_end = std::chrono::steady_clock::now();
    if (timing != 0) {
        timing->preprocess_ms = DurationMs(preprocess_start, preprocess_end);
    }

    if (ex.input(input_names[0], in) != 0) {
        last_error_ = "extractor input failed";
        return out;
    }

    const std::chrono::steady_clock::time_point inference_start = std::chrono::steady_clock::now();
    if (ex.extract(output_names[0], out) != 0) {
        last_error_ = "extractor extract failed";
        return out;
    }
    const std::chrono::steady_clock::time_point inference_end = std::chrono::steady_clock::now();
    if (timing != 0) {
        timing->inference_ms = DurationMs(inference_start, inference_end);
    }

    return out;
}

bool NcnnDetector::Detect(const cv::Mat& img, std::vector<NcnnDetection>* detections, NcnnDetectTiming* timing)
{
    last_timing_ = NcnnDetectTiming();

    if (detections == 0) {
        last_error_ = "detections pointer is null";
        return false;
    }

    detections->clear();

    if (img.empty()) {
        last_error_ = "input image is empty";
        return false;
    }

    const std::chrono::steady_clock::time_point total_start = std::chrono::steady_clock::now();
    ncnn::Mat out = Reasoning(img, &last_timing_);
    if (out.empty()) {
        if (last_error_.empty()) {
            last_error_ = "reasoning output is empty";
        }
        return false;
    }

    if (out.dims != 2 || out.h < 5) {
        last_error_ = "unexpected output shape";
        return false;
    }

    std::vector<cv::Rect> candidate_boxes;
    std::vector<float> candidate_scores;
    candidate_boxes.reserve(out.w);
    candidate_scores.reserve(out.w);

    const float scale_x = static_cast<float>(img.cols) / 128.0f;
    const float scale_y = static_cast<float>(img.rows) / 128.0f;

    const float* px = out.row(0);
    const float* py = out.row(1);
    const float* pw = out.row(2);
    const float* ph = out.row(3);
    const float* ps = out.row(4);

    const std::chrono::steady_clock::time_point postprocess_start = std::chrono::steady_clock::now();
    for (int i = 0; i < out.w; ++i) {
        const float score = ps[i];
        if (score < conf_threshold_) {
            continue;
        }

        const float cx = px[i] * scale_x;
        const float cy = py[i] * scale_y;
        const float w = pw[i] * scale_x;
        const float h = ph[i] * scale_y;

        int x1 = static_cast<int>(cx - w * 0.5f);
        int y1 = static_cast<int>(cy - h * 0.5f);
        int box_w = static_cast<int>(w);
        int box_h = static_cast<int>(h);

        x1 = std::max(0, x1);
        y1 = std::max(0, y1);
        if (x1 >= img.cols || y1 >= img.rows) {
            continue;
        }

        box_w = std::min(box_w, img.cols - x1);
        box_h = std::min(box_h, img.rows - y1);
        if (box_w <= 0 || box_h <= 0) {
            continue;
        }

        candidate_boxes.push_back(cv::Rect(x1, y1, box_w, box_h));
        candidate_scores.push_back(score);
    }

    std::vector<int> keep_indices;
    cv::dnn::NMSBoxes(candidate_boxes, candidate_scores, conf_threshold_, nms_threshold_, keep_indices);

    for (size_t i = 0; i < keep_indices.size(); ++i) {
        const int idx = keep_indices[i];
        NcnnDetection det;
        det.box = candidate_boxes[idx];
        det.score = candidate_scores[idx];
        detections->push_back(det);
    }

    const std::chrono::steady_clock::time_point total_end = std::chrono::steady_clock::now();
    last_timing_.postprocess_ms = DurationMs(postprocess_start, total_end);
    last_timing_.total_ms = DurationMs(total_start, total_end);
    if (timing != 0) {
        *timing = last_timing_;
    }

    return true;
}
