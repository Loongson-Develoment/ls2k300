#include "NcnnDetector.h"

#include <iostream>
#include <unistd.h>

NcnnDetector::NcnnDetector()
    : loaded_(false)
    , conf_threshold_(0.5f)
    , nms_threshold_(0.2f)
    , ncnn_model_param_path("./model.ncnn.param")
    , ncnn_model_bin_path("./model.ncnn.bin")   
{
    net_.opt.use_vulkan_compute = false;
    net_.opt.num_threads = 1;

    if (modelfile_exists(ncnn_model_param_path) && modelfile_exists(ncnn_model_bin_path)) 
    {        
        if (Load(ncnn_model_param_path, ncnn_model_bin_path)) 
        {
            std::cout << "[INFO] ncnn model loaded: "
                      << ncnn_model_param_path
                      << " / "
                      << ncnn_model_bin_path
                      << std::endl;
        } 
        else 
        {
            std::cerr << "[WARN] ncnn model load failed: "
                      << LastError()
                      << std::endl;
        }
    } 
    else {
        std::cout << "[INFO] ncnn model files not found, skip detector init"
                    << std::endl;
    }
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

ncnn::Mat NcnnDetector::Transform(const cv::Mat& img)
{
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(
        img.data,
        ncnn::Mat::PIXEL_BGR2RGB,
        img.cols,
        img.rows,
        128,
        128
    );

    float mean_vals[3] = {0.0f, 0.0f, 0.0f};
    float norm_vals[3] = {1.0f / 255.0f, 1.0f / 255.0f, 1.0f / 255.0f};
    in.substract_mean_normalize(mean_vals, norm_vals);

    return in;
}

ncnn::Mat NcnnDetector::Reasoning(const cv::Mat& img)
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
    ncnn::Mat in = Transform(img);

    if (ex.input(input_names[0], in) != 0) {
        last_error_ = "extractor input failed";
        return out;
    }

    if (ex.extract(output_names[0], out) != 0) {
        last_error_ = "extractor extract failed";
        return out;
    }

    return out;
}

bool NcnnDetector::Detect(const ncnn::Mat& out, int img_width, int img_height, std::vector<DetectResult>& results)
{
    results.clear();

    if (out.empty()) {
        last_error_ = "reasoning output is empty";
        return false;
    }

    if (out.dims != 2 || out.h < 5) {
        last_error_ = "unexpected output shape";
        return false;
    }

    const float scale_x = static_cast<float>(img_width) / 128.0f;
    const float scale_y = static_cast<float>(img_height) / 128.0f;

    const float* cx = out.row(0);
    const float* cy = out.row(1);
    const float* w = out.row(2);
    const float* h = out.row(3);
    const float* score = out.row(4);

    for (int i = 0; i < out.w; ++i) {
        if (score[i] < conf_threshold_) {
            continue;
        }

        DetectResult result;
        result.center_x = cx[i] * scale_x;
        result.center_y = cy[i] * scale_y;
        result.width = w[i] * scale_x;
        result.height = h[i] * scale_y;
        result.score = score[i];
        results.push_back(result);
    }

    last_error_.clear();
    return true;
}

bool NcnnDetector::modelfile_exists(const char *path)
{
    return path != 0 && access(path, F_OK) == 0;
}

DetectResult NcnnDetector::Max_score(std::vector<DetectResult>& results)
{
    if (results.empty()) {
        return DetectResult();
    }

    size_t max_index = 0;
    float max_score = results[0].score;
    for (size_t i = 1; i < results.size(); ++i) {
        if (results[i].score > max_score) {
            max_score = results[i].score;
            max_index = i;
        }
    }

    return results[max_index];
}
