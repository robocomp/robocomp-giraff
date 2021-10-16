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
#include "specificworker.h"
#include <cppitertools/enumerate.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    try{ jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{0.0001, 1 });}  // radians. 0 vertical
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    // get image, body and face
    float face_x_error = 0, face_y_error = 0, body_x_error = 0, body_y_error = 0, face_dist = 0, body_dist = 0;
    const auto &[body_o, face_o] = read_image();
    if (face_o.has_value())         // MOVE THIS INSIDE read_image
    {
        auto [face, f_dist] = face_o.value();
        face_x_error = 150 - face.center().x();
        face_y_error = 150 - face.center().y();
        face_dist = f_dist;
    }
    if (body_o.has_value())
    {
        auto [body, b_dist] = body_o.value();
        body_x_error = 150 - body.center().x();
        body_y_error = 150 - body.center().y();
        body_dist = b_dist;
    }

    // state machin
//    switch(state)
//    {
//        case State::WAITING:
//        case State::BODY_DETECTED:
//        case State::FACE_DETECTED:
//        case State::READY_TO_INTERACT:
//        case State::INTERACTING:
//    }

    //move_eyes();
    move_tablet(body_y_error, face_y_error);
    move_base(body_x_error, face_x_error, body_dist, face_dist);
}

SpecificWorker::DetectRes SpecificWorker::read_image()
{
    try
    {
        //RoboCompCameraRGBDSimple::TImage top_img = camerargbdsimple_proxy->getImage("camera_tablet");
        RoboCompCameraRGBDSimple::TRGBD rgbd = camerargbdsimple_proxy->getAll("camera_tablet");
        const auto &top_img = rgbd.image;
        const auto &top_depth = rgbd.depth;
        if (top_img.width != 0 and top_img.height != 0)
        {
            cv::Mat img(top_img.height, top_img.width, CV_8UC3, rgbd.image.image.data());
            cv::Mat depth(top_depth.height, top_depth.width, CV_32FC1, rgbd.depth.depth.data());
            cv::Mat n_img;
            cv::resize(img, n_img, cv::Size(300, 300));
            DetectRes ret{{},{}};

            // body
            vector<cv::Rect> found;
            vector<double> weights;
            cv::Mat n_img_bw;
            cv::cvtColor(n_img, n_img_bw, cv::COLOR_BGR2GRAY);
            hog.detectMultiScale(n_img_bw, found, weights);
            if (not found.empty())
            {
                auto r = found.front();
                cv::rectangle(n_img, r, cv::Scalar(0, 0, 255), 3);
                QRect rect = QRect(r.x, r.y, r.width, r.height);
                float k = depth.at<float>(rect.center().x(), rect.center().y()) * 1000;  //mmm
                std::get<0>(ret) = {rect, k};
            }
            else
            {
                // face
                auto rectangles = face_detector.detect_face_rectangles(n_img);
                if (not rectangles.empty())
                {
                    auto r = rectangles.front();
                    cv::rectangle(n_img, r, cv::Scalar(0, 105, 205), 3);
                    QRect rect = QRect(r.x, r.y, r.width, r.height);
                    float k = depth.at<float>(rect.center().x(), rect.center().y()) * 1000;  //mmm
                    std::get<1>(ret) = {rect, k};
                }
            }

            // ehow image
            cv::cvtColor(n_img, n_img, cv::COLOR_BGR2RGB);
            cv::imshow("Camera tablet", n_img);
            cv::waitKey(1);

            return ret;
        }
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    qWarning() << "Empty image or comms problem";
    return {};
}
void SpecificWorker::move_tablet(float body_y_error, float face_y_error)
{
    if(body_y_error != 0)
    {
        const float delta = 0.1;
        float tilt = (delta / 100) * body_y_error;  // map from -100,100 to -0.1,0.1 rads
        auto pos = jointmotorsimple_proxy->getMotorState("tablet_joint").pos;
        //qInfo() << __FUNCTION__ << "BODY: pos" << pos << "error" << body_y_error << "tilt" << tilt << pos - tilt;
        try { jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{pos - tilt, 1}); }  // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    }
    else if( face_y_error != 0)
    {
        const float delta = 0.1;
        const float tilt = (delta / 100) * face_y_error;  // map from -100,100 to -0.1,0.1 rads
        const float pos = jointmotorsimple_proxy->getMotorState("tablet_joint").pos;
        //qInfo() << __FUNCTION__ << "FACE: pos" << pos << "error" << face_y_error << "tilt" << tilt << pos - tilt;
        try { jointmotorsimple_proxy->setPosition("tablet_joint", RoboCompJointMotorSimple::MotorGoalPosition{pos - tilt, 1}); }  // radians. 0 vertical
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    }
}
void SpecificWorker::move_base(float body_x_error, float face_x_error, float body_dist, float face_dist)
{
    // rotate base
    const float gain = 0.5;
    float rot = 0.0;
    if(body_x_error != 0)
        rot = -(2.f / 100) * body_x_error;
    else if(face_x_error != 0)
        rot = -(2.f / 100) * face_x_error;
    float advance = 0;
    if(body_dist > 0 and body_dist < 800)
        advance = -(400.0 / 800.0) * body_dist;
    if(face_dist > 0 and face_dist < 800)
        advance = -(400.0 / 800.0) * face_dist;
    try { differentialrobot_proxy->setSpeedBase(advance, gain * rot); }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
}
////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompEmotionalMotor you can call this methods:
// this->emotionalmotor_proxy->expressAnger(...)
// this->emotionalmotor_proxy->expressDisgust(...)
// this->emotionalmotor_proxy->expressFear(...)
// this->emotionalmotor_proxy->expressJoy(...)
// this->emotionalmotor_proxy->expressSadness(...)
// this->emotionalmotor_proxy->expressSurprise(...)

