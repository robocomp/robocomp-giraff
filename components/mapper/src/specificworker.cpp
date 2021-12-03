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
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/combinations_with_replacement.hpp>
#include <cppitertools/zip.hpp>
#include <cppitertools/zip_longest.hpp>
#include <numeric>

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

    this->dimensions = QRectF(-5100, -2600, 10200, 5200);
    viewer = new AbstractGraphicViewer(this, this->dimensions);
    this->resize(900,450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract
    try
    {
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

    // grid
    grid.initialize(dimensions, TILE_SIZE, &viewer->scene, false);
    qInfo() << __FUNCTION__ << "Grid initialized to " << this->dimensions;

            ////////////////////////////////////////////////////
	this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<double> noise(-0.01, 0.01);
    //r_state.rz += noise(mt);    // add  noise
    //robot
    try
    {
        r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz*180/M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    //laser
    RoboCompLaser::TLaserData ldata;
    try
    {
        ldata = laser_proxy->getLaserData();
        draw_laser(ldata);
        update_map(ldata);
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    static float initial_angle;
    static int current_room = 0, last_room = -1;
    static Eigen::Vector2f center_room_w;

    switch (state)
    {
        case State::IDLE:
            qInfo() << __FUNCTION__ << "IDLE";
            state = State::INIT_TURN;
            break;

        case State::INIT_TURN:
            initial_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            move_robot(0, 0.5);
            state = State::TURN;
            break;

        case State::TURN:
        {
            qInfo() << "TURN";
            float current = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            if (fabs(current - initial_angle) < (M_PI + 0.1) and fabs(current - initial_angle) > (M_PI - 0.1))
            {
                move_robot(0,0);
                state = State::ESTIMATE;
            }
            move_robot(0, 0.5);
            // search for corners
            // compute derivative wrt distance
            std::vector<float> derivatives(ldata.size());
            derivatives[0] = 0;
            for (const auto &&[k, l]: iter::sliding_window(ldata, 2) | iter::enumerate)
                derivatives[k + 1] = l[1].dist - l[0].dist;

            // filter derivatives greater than a threshold
            std::vector<Eigen::Vector2f> peaks;
            for (const auto &&[k, der]: iter::enumerate(derivatives))
            {
                RoboCompLaser::TData l;
                if (der > 800)
                {
                    l = ldata.at(k - 1);
                    peaks.push_back(from_robot_to_world(Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle))));
                }
                else if (der < -800)
                {
                    l = ldata.at(k);
                    peaks.push_back(from_robot_to_world(Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle))));
                }
            }
            qInfo() << "  peaks " << peaks.size();

            // pairwise comparison of peaks to filter in doors
            for (auto &&c: iter::combinations_with_replacement(peaks, 2))
            {
                if ((c[0] - c[1]).norm() < 1100 and
                    (c[0] - c[1]).norm() > 600)
                {
                    Door d{c[0], c[1], doors.size()};
                    d.to_rooms.insert(current_room);
                    if (auto r = std::find_if(doors.begin(), doors.end(), [d](auto a) { return d == a; }); r == doors.end())
                        doors.emplace_back(d);
                }
            }
            qInfo() <<  "   doors" << doors.size();
            draw_doors(doors, &viewer->scene);
            break;
        }
        case State::ESTIMATE:
        {
            qInfo() << "ESTIMATE: room ->" << current_room << last_room;
            // create list of occuppied points
            RoboCompRoomDetection::ListOfPoints points;
            for (const auto &[key, val]: std::ranges::filter_view(grid, [](auto a) { return not a.second.free; }))
                points.emplace_back(RoboCompRoomDetection::Corner{key.x, key.z});

            RoboCompRoomDetection::ListOfPoints sampled_points;
            auto gen = std::mt19937{std::random_device{}()};
            std::ranges::sample(points, std::back_inserter(sampled_points), std::clamp((double)points.size(), points.size() / 4.0, 300.0), gen);

            qInfo() << __FUNCTION__ << "    proxy working...";
            RoboCompRoomDetection::Rooms detected_rooms;
            try
            { detected_rooms = roomdetection_proxy->detectRoom(sampled_points); }
            catch (const Ice::Exception &e)
            { std::cout << e.what() << std::endl; }
            qInfo() << __FUNCTION__ << "    number of detected rooms: " << detected_rooms.size();

            // pick the largest one (biggest area)
            std::vector<IOU::Quad> rects;
            for (const auto &r: detected_rooms)   // build a IOU::Quad vector
                rects.emplace_back(IOU::Quad(IOU::Point(r[0].x, r[0].y), IOU::Point(r[1].x, r[1].y),
                                             IOU::Point(r[2].x, r[2].y), IOU::Point(r[3].x, r[3].y)));
            auto max = std::ranges::max_element(rects, [](auto a, auto b) { return a.area() < b.area(); });
            if(max == rects.end()) break;  // no rooms found

            // additional checks to make sure the rect explains most of the points around the robot.
            Room room(*max, current_room);
            if (auto r = std::find_if(rooms.begin(), rooms.end(), [room](auto a) { return room == a; }); r == rooms.end())
                rooms.push_back(room);

            // complete door info about room connection. We need the list of doors visible from here
            // adjust door to room's wall

            // draw
            QPolygonF poly;
            for (auto p: room.points)
                poly << QPointF(p.x, p.y);
            auto v = viewer->scene.addPolygon(poly, QPen(QColor("Blue"), 90));
            v->setZValue(100);

            // Choose door
            current_door = -1;
            for(const auto &[k, d] : doors | iter::enumerate)
                if(d.to_rooms.find(current_room) != d.to_rooms.end() and d.to_rooms.find(last_room) == d.to_rooms.end())
                {
                    current_door = k;
                    qInfo() << "   Current door " << doors[k].id;
                    break;
                }
            if( current_door<0 )
            {
                qWarning() << __FUNCTION__ << " No available door to goto. Returning to IDLE";
                std::terminate();
            }

            state = State::GOTO_DOOR;
            break;
        }
        case State::GOTO_DOOR:
        {
            qInfo() << __FUNCTION__ << "GOTO_DOOR" << " room:" << current_room  << last_room;
            // pick a point 1 meter ahead of center of door position
            auto tr = from_world_to_robot(doors[current_door].get_external_midpoint());
            float dist = tr.norm();
            if(dist < 150)  // at target
            {
                try
                { differentialrobot_proxy->setSpeedBase(0, 0); }
                catch (const Ice::Exception &e)
                { std::cout << e.what() << std::endl; }
                if( auto to_room_o = this->doors[current_door].connecting_room(current_room); to_room_o.has_value())
                {
                    // getting into a known room
                    qInfo() << __FUNCTION__ << "GETTING INTO A KNOWN ROOM";
                    last_room = current_room;
                    current_room = to_room_o.value();
                    std::terminate();
                }
                else  //new room
                {
                    last_room = current_room;
                    current_room = rooms.size();
                    doors[current_door].to_rooms.insert(current_room);
                    // clean all cells
                    grid.set_all_to_free();
                }
                qInfo() << __FUNCTION__ << "Leaving GOTO_DOOR, into room " << current_room;
                // center point for new room
                float x = ldata.size()/2;
                float d = x/4;
                float laser_center = ldata[x].dist;
                float izq = ldata[d].dist;
                float der = ldata[ldata.size()-d].dist;
                Eigen::Vector2f center_r ( (der-izq)/2, (-800 + laser_center)/2);
                center_room_w = from_robot_to_world( center_r);
                // draw
                //auto dest = viewer->scene.addEllipse(center_room_w.x()-100, center_room_w.y()-100, 200, 200, QPen(QColor("Magenta"), 50));
                //dest->setZValue(200);

                qInfo() << __FUNCTION__ << "Center point " << center_room_w.x() << center_room_w.y();
                //state = State::GOTO_ROOM_CENTER;
                state = State::INIT_TURN;
                break;
            }
            // call dynamic window
            QPolygonF laser_poly;
            for(auto &&l : ldata)
                laser_poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
            auto [_, __, adv, rot, ___] = dw.compute(tr, laser_poly,
                                                       Eigen::Vector3f(r_state.x, r_state.y, r_state.rz),
                                                       Eigen::Vector3f(r_state.vx, r_state.vy, r_state.vrz),
                                                       &viewer->scene);
            const float rgain = 0.8;
            float rotation = rgain*rot;
            float dist_break = std::clamp(from_world_to_robot(target.to_eigen()).norm() / 1000.0, 0.0, 1.0);
            float advance = constants.max_advance_speed * dist_break * gaussian(rotation);
            try
            { differentialrobot_proxy->setSpeedBase(advance, rotation); }
            catch (const Ice::Exception &e)
            { std::cout << e.what() << std::endl; }
            break;
        }
        case State::GOTO_ROOM_CENTER:
        {
            qInfo() << __FUNCTION__ << "GOTO_ROOM_CENTER. Room:" << current_room << last_room;
            // if at room center goto INIT_TURN
            auto tr = from_world_to_robot(center_room_w);
            float dist = tr.norm();
            if(dist < 80)
            {
                try
                { differentialrobot_proxy->setSpeedBase(0, 0); }
                catch (const Ice::Exception &e)
                { std::cout << e.what() << std::endl; }
                state = State::INIT_TURN;
                qInfo() << "LLEGO -------------------------";
                break;
            }
            float beta = 0.0;
            if(dist>80)
                beta = atan2(tr.x(), tr.y());
            float adv_break = std::clamp(constants.max_advance_speed/1000.0, 0.0, 1.0);
            float adv = constants.max_advance_speed * adv_break * gaussian(beta);
            try
            { differentialrobot_proxy->setSpeedBase(adv, beta); }
            catch (const Ice::Exception &e)
            { std::cout << e.what() << std::endl; }
            break;
        }
    }
}

////////////////////////////////// AUX /////////////////////////////////////
void SpecificWorker::draw_doors(const std::vector<Door> &local_doors, QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem *> door_lines;  // to remove graph objects
    for (auto dp: door_lines) scene->removeItem(dp);
    door_lines.clear();
    for (const auto r: local_doors)
    {
        door_lines.push_back(scene->addLine(r.p1.x(), r.p1.y(), r.p2.x(), r.p2.y(), QPen(QColor("Magenta"), 50)));
        door_lines.back()->setZValue(200);
    }
}
void SpecificWorker::move_robot(float adv, float rot)
{
    try
    { differentialrobot_proxy->setSpeedBase(adv, rot); }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
}
float SpecificWorker::gaussian(float x)
{
    const double xset = 0.5;
    const double yset = 0.4;
    const double s = -xset*xset/log(yset);
    return exp(-x*x/s);
}
void SpecificWorker::update_map(const RoboCompLaser::TLaserData &ldata)
{
    // grid
    Eigen::Vector2f lw;
    for(const auto &l : ldata)
    {
        if(l.dist > constants.robot_semi_length)
        {
            Eigen::Vector2f tip(l.dist*sin(l.angle), l.dist*cos(l.angle));
            Eigen::Vector2f p = from_robot_to_world(tip);
            int target_kx = (p.x() - grid.dim.left()) / grid.TILE_SIZE;
            int target_kz = (p.y() - grid.dim.bottom()) / grid.TILE_SIZE;
            int last_kx = -1000000;
            int last_kz = -1000000;

            int num_steps = ceil(l.dist/(constants.tile_size/2.0));
            for(const auto &&step : iter::range(0.0, 1.0-(1.0/num_steps), 1.0/num_steps))
            {
                Eigen::Vector2f p = from_robot_to_world(tip*step);
                int kx = (p.x() - grid.dim.left()) / grid.TILE_SIZE;
                int kz = (p.y() - grid.dim.bottom()) / grid.TILE_SIZE;
                if(kx != last_kx and kx != target_kx and kz != last_kz and kz != target_kz)
                    grid.add_miss(from_robot_to_world(tip * step));
                last_kx = kx;
                last_kz = kz;
            }
            if(l.dist <= constants.max_laser_range)
                grid.add_hit(from_robot_to_world(tip));
            // else
            //     grid.add_miss(from_robot_to_world(tip));
        }
    }
}
Eigen::Vector2f SpecificWorker::from_robot_to_world(const Eigen::Vector2f &p)
{
    Eigen::Matrix2f matrix;
    matrix << cos(r_state.rz) , -sin(r_state.rz) , sin(r_state.rz) , cos(r_state.rz);
    return (matrix * p) + Eigen::Vector2f(r_state.x, r_state.y);
}
Eigen::Vector2f SpecificWorker::from_world_to_robot(const Eigen::Vector2f &p)
{
    Eigen::Matrix2f matrix;
    matrix << cos(r_state.rz) , -sin(r_state.rz) , sin(r_state.rz) , cos(r_state.rz);
    return (matrix.transpose() * (p - Eigen::Vector2f(r_state.x, r_state.y)));
}
void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    if (laser_polygon != nullptr)
        viewer->scene.removeItem(laser_polygon);

    QPolygonF poly;
    poly << QPointF(0,0);
    for(auto &&l : ldata)
        poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
    poly.pop_back();

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}
void SpecificWorker::new_target_slot(QPointF t)
{
    qInfo() << __FUNCTION__ << " Received new target at " << t;
    target.pos = t;
    target.active = true;
}

/////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

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

