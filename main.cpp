#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
  std::cout << "OpenCV Build Test for ls2k0300" << std::endl;
  std::cout << "OpenCV Version: " << CV_VERSION << std::endl;

  // 创建一个简单的矩阵来验证 OpenCV 是否正常工作
  cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC3);
  cv::circle(image, cv::Point(50, 50), 40, cv::Scalar(0, 255, 0), 2);

  if (image.empty()) {
    std::cerr << "OpenCV Mat creation failed!" << std::endl;
    return -1;
  }

  std::cout << "Successfully initialized OpenCV Mat and performed a drawing "
               "operation."
            << std::endl;
  return 0;
}
