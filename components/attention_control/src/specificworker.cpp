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

    cap.open(0, cv::CAP_ANY);
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
    if(auto  face_o = read_image(); face_o.has_value())
    {
        // get center and deltas
        auto face = face_o.value();
        QPoint center = face.center();
        float x_error = 150-center.x();
        float y_error = 150-center.y();
        qInfo() << center.x() << center.y() << 150-center.x() << 150-center.y();

        // move eyes
        // move tablet
        // move base
        try
        {
            float rot = -(2.f / 100) * x_error;
            float adv = (500.f / 100.f) * y_error;
            differentialrobot_proxy->setSpeedBase(0, rot);
        }
        catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    }
}

std::optional<QRect> SpecificWorker::read_image()
{
    try
    {
        RoboCompCameraRGBDSimple::TImage top_img = camerargbdsimple_proxy->getImage("camera_tablet");
        if (top_img.width != 0 and top_img.height != 0)
        {
            cv::Mat img(top_img.height, top_img.width, CV_8UC3, top_img.image.data());
            cv::Mat n_img;
            cv::resize(img, n_img, cv::Size(300, 300));
            auto rectangles = face_detector.detect_face_rectangles(n_img);
            cv::Scalar color(0, 105, 205);
            int frame_thickness = 4;
            //qInfo() << __FUNCTION__ << rectangles.size();
            for (const auto &r: rectangles)
                cv::rectangle(n_img, r, color, frame_thickness);

            cv::imshow("Camera tablet", n_img);
            cv::waitKey(5);
            if(not rectangles.empty())
            {
                auto r = rectangles.front();
                return QRect(r.x, r.y, r.width, r.height);
            }
            else
                return {};
        }
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    return {};
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

