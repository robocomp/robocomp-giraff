//
// Created by benjamin on 26.03.20.
//
#ifndef VISUALS_FACEDETECTOR_H
#define VISUALS_FACEDETECTOR_H
#include <opencv4/opencv2/dnn.hpp>

class FaceDetector
{
    public:
        explicit FaceDetector();

    /// Detect faces in an image frame
    /// \param frame Image to detect faces in
    /// \return Vector of detected faces
        std::vector<cv::Rect> detect_face_rectangles(const cv::Mat &frame);

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


#endif //VISUALS_FACEDETECTOR_H
