//
// Created by benjamin on 26.03.20.
//
#ifndef VISUALS_BODYDETECTOR_H
#define VISUALS_BODDETECTOR_H
#include <opencv4/opencv2/dnn.hpp>

class BodyDetector
{
    public:
        explicit BodyDetector();
        std::vector<cv::Rect> detector(const cv::Mat &frame);

    private:
        /// Face detection network
        cv::dnn::Net network_;
        /// Input image width
        const int input_image_width_ = 300;
        /// Input image height
        const int input_image_height_ = 300;
        /// Scale factor when creating image blob
        const double scale_factor_ = 1;
        /// Mean normalization values network was trained with
        const cv::Scalar mean_values_ = {104., 177.0, 123.0};
        /// Face detection confidence threshold
        const float confidence_threshold_ = 0.5;
};


#endif //VISUALS_BODYDETECTOR_H
