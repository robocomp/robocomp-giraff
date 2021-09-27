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
#include "cppitertools/sliding_window.hpp"
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
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    int stop_threshold = 650, slow_threshold = 1250, min_speed = 0, max_speed = 700, residue = 200, trim = 3; //COPPELIA VALUES
//    float stop_threshold = 650.0, slow_threshold = 1250.0;
//    float min_speed = 0.0, max_speed = 0.6;
//    float residue = 0.1;
//    float trim = 3.0;   // GIRAFF  VALUES
    float adv = 0.8;
    float rot = 0;
    float m = (max_speed - min_speed) * 1. / (slow_threshold - stop_threshold);
    float n = min_speed - m * stop_threshold + residue;

    if( auto ldata = laser_proxy->getLaserData(); !ldata.empty())
    {
        //Using only distance values
        std::vector<float> distances;
        for(auto &point : ldata)
            distances.push_back(point.dist);

        //Filter, try to fix laser measure errors
        if(distances[0] < 200)
            distances[0] = 200;
        for(auto &&window : iter::sliding_window(distances, 2))
        {
            if(window[1] < 200)
                window[1] = window[0];
        }

        //Sort and take the lower distance value
        int limit = distances.size()/trim;
        std::sort(distances.begin() + limit, distances.end() - limit, [](float a, float b){return a < b;});
        float minValue = distances[limit];

        //Set the speeds depending on minValue, x1, x2, y1, y2
        if(minValue < stop_threshold) {
            rot = 0.8;
            adv = min_speed;
        }else if(minValue < slow_threshold)
            adv = m * minValue + n;
        else
            adv = max_speed;

        try
        {
            cout << "adv: " << adv << "     rot: " << rot << endl;
            differentialrobot_proxy->setSpeedBase(adv, rot);
        }
        catch(const Ice::Exception &e)
            { std::cout << e.what() << std::endl;}
    }
}

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
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

