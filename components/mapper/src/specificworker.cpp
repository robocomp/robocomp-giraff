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
#include <cppitertools/filter.hpp>
#include <cppitertools/filterfalse.hpp>
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
/**-
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
    viewer_robot = new AbstractGraphicViewer(this->frame_robot, this->dimensions);
    const auto &[rp, re] = viewer_robot->add_robot(ROBOT_LENGTH, ROBOT_LENGTH);
    robot_polygon = rp;
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract

    viewer_graph = new AbstractGraphicViewer(this->frame_graph, this->dimensions);
    this->resize(1200,450);

    connect(viewer_robot, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

    // grid
    //grid.initialize(dimensions, TILE_SIZE, &viewer_robot->scene, false);
    //qInfo() << __FUNCTION__ << "Grid initialized to " << this->dimensions;

    ////////////////////////////////////////////////////
	this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}
void SpecificWorker::compute()zZZZZZZZ
{
    read_base();
    read_laser();

    if(G.current_room().is_unknown) // explore room
    {
        static std::future<bool> future_explore;
        if(not future_explore.valid())
            future_explore = std::async(std::launch::async, &SpecificWorker::explore, this );
        else
            if(future_explore.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                auto res = future_explore.get();
                G.current_room().is_unknown = false;
                qInfo() << __FUNCTION__ << "Future EXPLORE ended with result:" << res;
                room_detector.minimize_door_distances(G);
                G.draw_all(&viewer_robot->scene, &viewer_graph->scene);
            }
    }
    else  // change room
    {
        static std::future<bool> future_visit;
        if(not future_visit.valid())
            future_visit = std::async(std::launch::async, &SpecificWorker::change_room, this);
        else
            if(future_visit.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                auto res = future_visit.get();
                qInfo() << __FUNCTION__ << "Future VISIT ended with " << res;
                G.current_room().print();
                room_detector.minimize_door_distances(G);
                G.project_doors_on_room_side(G.current_room(), &viewer_robot->scene);
                G.draw_all(&viewer_robot->scene, &viewer_graph->scene);
                // if known room check if it matches the prediction. If not, set it as unknown so it is explored again
            }
    }
}
////////////////////////////////////////////////////////////////////////////
bool SpecificWorker::explore()
{
    qInfo() << __FUNCTION__ << "Starting explore...";
    float initial_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
    move_robot(0, 0.4);
    float current = initial_angle;
    while (not (fabs(current - initial_angle) < (M_PI + 0.1) and fabs(current - initial_angle) > (M_PI - 0.1)))
    {
        detect_doors();
        current = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    move_robot(0, 0);
    return  estimate_rooms();
}
bool SpecificWorker::change_room()
{
    qInfo() << __FUNCTION__ << "Entering change in room " << G.current_room().id;
    // Choose an un-explored destination room
    Graph_Rooms::Door new_door;
    int new_room_id = -1;
    // door to unknown room
    if( auto hit = std::ranges::find_if(G.current_room().doors, [](auto d){ return d.to_room == -1;}); hit != G.current_room().doors.end())
    {
        new_door = (*hit);
        new_room_id = -1;
    }
    else //  door to known room
    {
        std::vector<Graph_Rooms::Door> selected_doors;
        auto gen = std::mt19937{std::random_device{}()};
        std::ranges::sample(G.current_room().doors, std::back_inserter(selected_doors), 1, gen);
        if(not selected_doors.empty())
        {
            new_door = selected_doors.front();
            new_room_id = new_door.to_room;
        }
        else
        {
            qInfo() << "WARNING, no door to choose";
            return false;
        }
    }
    // move to the new room
    // pick a point 1 meter ahead of center of door position and in the other room
    auto mid_point = new_door.get_external_midpoint(Eigen::Vector2f(r_state.x, r_state.y));
    float dist = from_world_to_robot(mid_point).norm();

    while( dist > constants.final_distance_to_target) // until target is reached
    {
        auto tr = from_world_to_robot(mid_point);
        dist = tr.norm();

        // call dynamic window
        QPolygonF laser_poly;
        for(auto &&l : ldata)
            laser_poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
        auto [_, __, adv, rot, ___] = dw.compute(tr, laser_poly,
                                                 Eigen::Vector3f(r_state.x, r_state.y, r_state.rz),
                                                 Eigen::Vector3f(r_state.vx, r_state.vy, r_state.vrz),
                                                 nullptr /*&viewer_robot->scene*/);
        const float rgain = 0.8;
        float rotation = rgain*rot;
        float dist_break = std::clamp(from_world_to_robot(target.to_eigen()).norm() / 1000.0, 0.0, 1.0);
        float advance = constants.max_advance_speed * dist_break * gaussian(rotation);
        move_robot(advance, rotation);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if(new_room_id == -1)
    {
        G.rooms.emplace_back(Graph_Rooms::Room(G.rooms.size()));
        G.current_room_local = G.rooms.back().id;
    }
    else
        G.current_room_local = new_room_id;

    qInfo() << __FUNCTION__ << "    Robot reached target room" << G.current_room_local;
    move_robot(0,0);
    grid.set_all_to_free();
    // if known room, initialize the grid with the room geometry
    return true;
}
void SpecificWorker::detect_doors()
{
    move_robot(0, 0.5);
    // search for corners. Compute derivative wrt distance
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
    //qInfo() << __FUNCTION__  << "  peaks " << peaks.size();

    // pairwise comparison of peaks to filter in doors
    for (auto &&c: iter::combinations_with_replacement(peaks, 2))
        if ((c[0] - c[1]).norm() < 1100 and (c[0] - c[1]).norm() > 550)
            G.add_door_to_current_room(c[0], c[1]);
}
bool SpecificWorker::estimate_rooms()
{
    // create list of occuppied points
    RoboCompRoomDetection::ListOfPoints points;
    for (const auto &[key, val]: std::ranges::filter_view(grid, [](auto a) { return not a.second.free; }))
        points.emplace_back(RoboCompRoomDetection::Corner{(float)key.x, (float)key.z});

    // filter out points beyond room doors
    RoboCompRoomDetection::ListOfPoints inside_points;
    bool skip=false;
    for (const auto &c : points)
    {
        for (const auto &d: G.current_room().doors)
        {
            // all in robot's reference system
            QPointF robot_rq(0, 0);
            auto point_r = from_world_to_robot(Eigen::Vector2f(c.x, c.y));
            auto point_rq = to_qpointf(point_r);
            auto point_r_distance = point_r.norm();
            auto mid_door_distance = from_world_to_robot(d.get_midpoint()).norm();
            // line from point to robot
            auto l_point = QLineF(point_rq, robot_rq);
            // door line
            auto l_door = QLineF(to_qpointf(from_world_to_robot(d.p1)), to_qpointf(from_world_to_robot(d.p2)));
            // compute intersection
            // NoIntersection	Indicates that the lines do not intersect; i.e. they are parallel.
            // UnboundedIntersection	The two lines intersect, but not within the range defined by their lengths.
            //                          This will be the case if the lines are not parallel. intersect() will also return
            //                          this value if the intersect point is within the start and end point of only one of the lines.
            // BoundedIntersection	The two lines intersect with each other within the start and end points of each line.
            QPointF inter_point;
            auto res = l_door.intersect(l_point, &inter_point);
            if (res == QLineF::BoundedIntersection and point_r_distance > mid_door_distance) // outside the door
            {
                grid.set_free(c.x, c.y);
                skip = true;
                continue;
            }
        }
        if(not skip) inside_points.push_back(c);
        skip = false;
    }
    qInfo() << __FUNCTION__ << "Points size: " << points.size() << "Points left outside:" << points.size() - inside_points.size();

    // sample size()/4 points from detected corners
    //   RoboCompRoomDetection::ListOfPoints sampled_points = points;
    //    auto gen = std::mt19937{std::random_device{}()};
    //    std::ranges::sample(inside_points, std::back_inserter(sampled_points), std::clamp((double)inside_points.size(), inside_points.size() / 4.0, 400.0), gen);

    if(inside_points.empty())
        return false;

    Eigen::MatrixX3d my_points;
    my_points.resize(inside_points.size(), 3);
    for(auto &&[i, p] : inside_points | iter::enumerate)
    {
        my_points(i, 0) = p.x; my_points(i, 1) = p.y; my_points(i, 2) = 1.0;
    }

    // call optimizer
    QRectF ro = room_detector.compute_room(my_points);
    IOU::Quad max(IOU::Point(ro.left(), ro.top()), IOU::Point(ro.right(), ro.top()), IOU::Point(ro.right(), ro.bottom()), IOU::Point(ro.left(), ro.bottom()));

    // update current room
    Graph_Rooms::Room &room = G.current_room();
    room.quad = max;
    room.room_rect = ro;
    G.project_doors_on_room_side(G.current_room(), &viewer_robot->scene);
    return true;
}
////////////////////////////////// AUX /////////////////////////////////////
void SpecificWorker::read_base()
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<double> noise(-0.01, 0.01);
    //r_state.rz += noise(mt);    // add  noise

    try
    {
        r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz * 180 / M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
}
void SpecificWorker::read_laser()
{
    //laser
    try
    {
        ldata = laser_proxy->getLaserData();
        draw_laser(ldata);
        update_map(ldata);
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
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
        viewer_robot->scene.removeItem(laser_polygon);

    QPolygonF poly;
    poly << QPointF(0,0);
    for(auto &&l : ldata)
        poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
    poly.pop_back();

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer_robot->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
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

//case State::GOTO_ROOM_CENTER:
//{
//qInfo() << __FUNCTION__ << "GOTO_ROOM_CENTER. Room:" << current_room << last_room;
//// if at room center goto INIT_TURN
//auto tr = from_world_to_robot(center_room_w);
//float dist = tr.norm();
//if(dist < 80)
//{
//try
//{ differentialrobot_proxy->setSpeedBase(0, 0); }
//catch (const Ice::Exception &e)
//{ std::cout << e.what() << std::endl; }
//state = State::INIT_TURN;
//qInfo() << "LLEGO -------------------------";
//break;
//}
//float beta = 0.0;
//if(dist>80)
//beta = atan2(tr.x(), tr.y());
//float adv_break = std::clamp(constants.max_advance_speed/1000.0, 0.0, 1.0);
//float adv = constants.max_advance_speed * adv_break * gaussian(beta);
//try
//{ differentialrobot_proxy->setSpeedBase(adv, beta); }
//catch (const Ice::Exception &e)
//{ std::cout << e.what() << std::endl; }
//break;
//}

// draw
//auto dest = viewer->scene.addEllipse(center_room_w.x()-100, center_room_w.y()-100, 200, 200, QPen(QColor("Magenta"), 50));
//dest->setZValue(200);

// center point for new room
//float x = ldata.size()/2;
//float d = x/4;
//float laser_center = ldata[x].dist;
//float izq = ldata[d].dist;
//float der = ldata[ldata.size()-d].dist;
//Eigen::Vector2f center_r ( (der-izq)/2, (-800 + laser_center)/2);
//center_room_w = from_robot_to_world( center_r);


// pick the one with the largest area)
//    std::vector<IOU::Quad> rects;
//    for (const auto &r: detected_rooms)   // build a IOU::Quad vector
//        rects.emplace_back(IOU::Quad(IOU::Point(r[0].x, r[0].y), IOU::Point(r[1].x, r[1].y),
//                                     IOU::Point(r[2].x, r[2].y), IOU::Point(r[3].x, r[3].y)));
//    auto max = std::ranges::max_element(rects, [](auto a, auto b) { return a.area() < b.area(); });
//    if(max == rects.end())
//    {
//        qInfo() << __FUNCTION__ << "No rooms found. Stopping";
//        new_data_state.room_detected = false;
//        return new_data_state;
//    }
//    else


//    qInfo() << __FUNCTION__ << "    proxy working...";
//    RoboCompRoomDetection::Rooms detected_rooms;
//    try
//    { detected_rooms = roomdetection_proxy->detectRoom(sampled_points); }
//    catch (const Ice::Exception &e)
//    { std::cout << e.what() << std::endl; }
//    qInfo() << __FUNCTION__ << "    number of detected rooms by proxy: " << detected_rooms.size();
//


//// draw target point
//static QGraphicsItem* dest= nullptr;
//if(dest != nullptr) viewer_robot->scene.removeItem(dest);
//dest = viewer_robot->scene.addEllipse(0, 0, 200, 200, QPen(QColor("Magenta"), 50));
//dest->setPos(mid_point.x(), mid_point.y()); dest->setZValue(200);