/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "FaceDetector.h"
#include "BodyDetector.h"
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/face.hpp>
#include <sstream>
#include <iostream>
#include <istream>
#include <fstream>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
    void compute_L1();
    void compute_L2();
    void compute_L3();
    int startup_check();
	void initialize(int period);

private:
    enum class L1_State { SEARCHING, BODY_DETECTED, FACE_DETECTED, EYES_DETECTED, HANDS_DETECTED };
    L1_State l1_state = L1_State::SEARCHING;
    std::map<L1_State, QString> l1_map{{L1_State::SEARCHING, "SEARCHING"},
                                       {L1_State::BODY_DETECTED, "BODY_DETECTED"},
                                       {L1_State::FACE_DETECTED, "FACE_DETECTED"}};

    enum class L2_State { EXPECTING, PERSON };
    L2_State l2_state = L2_State::EXPECTING;

    enum class L3_State { WAITING, READY_TO_INTERACT, INTERACTING, START_FOLLOWING, FOLLOWING, STOP };
    L3_State l3_state = L3_State::WAITING;

    bool startup_check_flag;
    cv::VideoCapture cap;
    FaceDetector face_detector;
    //BodyDetector body_detector;

    //using DetectRes = std::tuple<std::optional<std::tuple<QRect, int>>, std::optional<std::tuple<QRect, int>>>;
    using DetectRes = std::tuple<std::optional<std::tuple<int,int,int>>, std::optional<std::tuple<int,int,int>>>;
    DetectRes read_image();

    // YOLO
    // Initialize the parameters
    cv::dnn::Net net;
    float confThreshold = 0.5; // Confidence threshold
    float nmsThreshold = 0.5;  // Non-maximum suppression threshold
    int inpWidth = 320;  // Width of network's input image
    int inpHeight = 320; // Height of network's input image
    std::vector<std::string> classes;
    vector<cv::String> get_outputs_names(const cv::dnn::Net &net);
    vector<cv::Rect> yolo_detector(cv::Mat &frame);

    // OPenCV Face
    cv::CascadeClassifier faceDetector;
    cv::Ptr<cv::face::Facemark> facemark;

    // Level 1
    void move_tablet(std::optional<std::tuple<int,int,int>> body_o, std::optional<std::tuple<int,int,int>> face_o);
    void move_base(std::optional<std::tuple<int,int,int>> body_o, std::optional<std::tuple<int,int,int>> face_o);
    double dyn_state = 0;
    // Level 2
    struct Person
    {

    };
    Person person;

    void move_eyes(optional<tuple<int, int, int>> face_o);
};

#endif
