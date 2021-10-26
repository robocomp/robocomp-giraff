//
// Created by benjamin on 26.03.20.
//

#include <sstream>
#include <vector>
#include <string>
#include <BodyDetector.h>
#include <opencv4/opencv2/opencv.hpp>

BodyDetector::BodyDetector()
{
    network_ = cv::dnn::readNetFromCaffe("assets/mobilenet.prototxt", "assets/mobilenet.caffemodel");

    if (network_.empty())
    {
        std::ostringstream ss;
        ss << "Failed to load network with the following settings:\n"
           << "Configuration: " + std::string("assets/mobile.prototxt") + "\n"
           << "Binary: " + std::string("assets/mobile.caffemodel") + "\n";
        throw std::invalid_argument(ss.str());
    }
}

std::vector<cv::Rect> BodyDetector::detector(const cv::Mat &frame)
{
    cv::Mat input_blob = cv::dnn::blobFromImage(frame, scale_factor_, cv::Size(input_image_width_, input_image_height_), mean_values_, false, false);
    network_.setInput(input_blob);
    cv::Mat detection = network_.forward();
    cv::Mat detection_matrix(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < detection_matrix.rows; i++)
    {
        float confidence = detection_matrix.at<float>(i, 2);
        if (confidence < confidence_threshold_)
            continue;

        int x_left_bottom = static_cast<int>(detection_matrix.at<float>(i, 3) * frame.cols);
        int y_left_bottom = static_cast<int>(detection_matrix.at<float>(i, 4) * frame.rows);
        int x_right_top = static_cast<int>(detection_matrix.at<float>(i, 5) * frame.cols);
        int y_right_top = static_cast<int>(detection_matrix.at<float>(i, 6) * frame.rows);

        boxes.emplace_back(x_left_bottom, y_left_bottom, (x_right_top - x_left_bottom), (y_right_top - y_left_bottom));
    }
    return boxes;
}
