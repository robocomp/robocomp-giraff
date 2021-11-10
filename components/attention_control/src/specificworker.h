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
#include <ranges>
#include <chrono>
#include <Eigen/Dense>
#include <QPolygonF>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>


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
    void compute_bill();
    int startup_check();
	void initialize(int period);

private:
    bool startup_check_flag;
    cv::VideoCapture cap;
    FaceDetector face_detector;
    //BodyDetector body_detector;
    AbstractGraphicViewer *viewer;

    //robot
    struct Robot
    {
        float current_rot_speed = 0;
        float current_adv_speed = 0;
        float length = 400;
    };
    Robot robot;
    RoboCompLaser::TLaserData ldata;
    bool person_outside_laser(const QPointF &body, float body_dist);
    bool clear_path_to_point(const QPointF &p, const QPolygonF &laser_poly);
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    void draw_laser(const RoboCompLaser::TLaserData &ldata);
    RoboCompFullPoseEstimation::FullPoseEuler r_state;

    // camera
    //using DetectRes = std::tuple<std::optional<std::tuple<QRect, int>>, std::optional<std::tuple<QRect, int>>>;
    using DetectRes = std::tuple<std::optional<std::tuple<int,int,int, QPointF>>, std::optional<std::tuple<int,int,int,QPointF>>>;
    DetectRes read_image();
    struct Camera
    {
        float focal_x;
        int cols, rows;
    };
    Camera camera;

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

    enum class Parts {NONE, FACE, BODY};
    enum class Actions { FOLLOW, WAIT };

    // Level 1
    QTimer timer_l1;
    enum class L1_State { SEARCHING, BODY_DETECTED, FACE_DETECTED, EYES_DETECTED, HANDS_DETECTED };
    L1_State l1_state = L1_State::SEARCHING;
    std::map<L1_State, QString> l1_map{{L1_State::SEARCHING, "SEARCHING"},
                                       {L1_State::BODY_DETECTED, "BODY_DETECTED"},
                                       {L1_State::FACE_DETECTED, "FACE_DETECTED"}};
    struct L1_Person  // a light representation of an instantaneous person
    {
        Eigen::Vector2f pos;
        float orientation;
        float height;
        double dyn_state = 0;
        Parts looking_at = Parts::NONE;
        Actions current_action = Actions::WAIT;
        void print()
        {
            std::cout << "-------- L1 ----------" << std::endl;
            std::cout << "pos: " << pos.x() << ", " <<  pos.y() << std::endl;
            if(looking_at == Parts::FACE) std::cout << "looking_at: FACE" << std::endl;
            if(looking_at == Parts::BODY) std::cout << "looking_at: BODY" << std::endl;
            if(looking_at == Parts::NONE) std::cout << "looking_at: NONE" << std::endl;
            std::cout << "dyn: " << dyn_state << std::endl;
        }
    };
    L1_Person l1_person;
    void move_tablet(const DetectRes &detected);
    void move_base(const DetectRes &detected);
    enum class State_Base{FORWARD, TURN, BORDER};

    // Level 2
    QTimer timer_l2;
    enum class L2_State { EXPECTING, PERSON };
    L2_State l2_state = L2_State::EXPECTING;
    struct L2_Person // a stable representation of a multimodal person
    {
        std::chrono::time_point<std::chrono::system_clock> creation_time;
        Eigen::Vector2f pos;
        float orientation;
        Eigen::Vector2f vel;
        float angular_spped;
        float height;
        std::string name;
        Parts looking_at = Parts::NONE;
        void print()
        {
            std::cout << "-------- L2 -----------" << std::endl;
            std::cout << "pos: " << pos.x() << ", " << pos.y() << std::endl;
            if(looking_at == Parts::FACE)
            {  std::cout << "looking_at: FACE" << std::endl;}
            if(looking_at == Parts::BODY)
            {  std::cout << "looking_at: BODY" << std::endl;}
            std::cout << "duration: " << std::chrono::duration_cast<chrono::seconds>(std::chrono::system_clock::now()-creation_time).count() << std::endl;
        }
        Actions current_action = Actions::WAIT;
    };
    std::vector<L2_Person> l2_people;

    // Level 3
    QTimer timer_l3;
    enum class L3_State { WAITING, READY_TO_INTERACT, START_FOLLOWING, FOLLOWING, STOP, SEARCHING };
    L3_State l3_state = L3_State::WAITING;
    struct L3_Person
    {
        void print(L3_State state)
        {
            std::cout << "-------- L3 ---------" << std::endl;
            if(state == L3_State::WAITING) std::cout << "state: WAITING" << std::endl;
            if(state == L3_State::READY_TO_INTERACT) std::cout << "state: READY_TO_INTERACT" << std::endl;
            if(state == L3_State::START_FOLLOWING) std::cout << "state: START_FOLLOWING" << std::endl;
            if(state == L3_State::FOLLOWING) std::cout << "state: FOLLOWING" << std::endl;
            if(state == L3_State::SEARCHING) std::cout << "state: SEARCHING" << std::endl;
            if(state == L3_State::STOP) std::cout << "state: STOP" << std::endl;
        }
    };
    L3_Person l3_person;
    void move_eyes(optional<tuple<int, int, int>> face_o);

    // Bill
    QTimer timer_bill;
    enum class Bill_State {TO_POINT, NEW_POINT};
    Bill_State bill_state = Bill_State::NEW_POINT;

    // tilt
    int inverted_tilt = 1;

    // aux
    Eigen::Vector2f from_robot_to_world(const Eigen::Vector2f &p);
    Eigen::Vector2f from_world_to_robot(const Eigen::Vector2f &p);
    inline QPointF toQPointF(const Eigen::Vector2f &p);
    inline Eigen::Vector2f toEigen2f(const QPointF &p);

    float min_laser_distance(float min, float max);
};

#endif
