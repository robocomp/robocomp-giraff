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
    robot_polygon = viewer_robot->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract

    viewer_graph = new AbstractGraphicViewer(this->frame_graph, this->dimensions);
    this->resize(1200,450);

    connect(viewer_robot, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

    // grid
    grid.initialize(dimensions, TILE_SIZE, &viewer_robot->scene, false);
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
    try
    {
        ldata = laser_proxy->getLaserData();
        draw_laser(ldata);
        update_map(ldata);
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    switch (state)
    {
        case State::INIT:
            qInfo() << __FUNCTION__ << "INIT";
            data_state.current_room = 0; // first initial room
            state = State::EXPLORING;
            break;
        case State::EXPLORING:
            //qInfo() << __FUNCTION__ << "EXPLORING";
            data_state = exploring(data_state);
            if(data_state.no_rooms_found)
            {
                qInfo() << __FUNCTION__ << "No rooms found. going to IDLE";
                state = State::IDLE;
                break;
            }
            if (data_state.room_detected)
            {
                // draw
                try{ G.rooms.at(data_state.current_room).draw(&viewer_robot->scene);} catch(const std::exception &e){ std::cout << e.what() << std::endl;};
                G.draw_node(data_state.current_room, &viewer_graph->scene);
                G.draw_edge(data_state.current_door, &viewer_graph->scene);

                auto[next_door, next_room] = choose_exit_door(data_state);
                qInfo() << __FUNCTION__ << "    choose exit door" << next_door << " and room" << next_room;
                if (next_door != -1)
                {
                    data_state.current_door = next_door;
                    data_state.next_room = next_room;
                    data_state.last_room = data_state.current_room;
                    state = State::CHANGING_ROOM;
                } else
                {
                    qInfo() << __FUNCTION__ << "No doors found";
                    state = State::IDLE;
                }
            }
            break;
        case State::VISITING:
        {
            qInfo() << __FUNCTION__ << "VISITING";
            auto[next_door, next_room] = choose_exit_door(data_state);
            qInfo() << __FUNCTION__ << "    choose exit door" << next_door << " and room" << next_room;
            if (next_door != -1)
            {
                data_state.current_door = next_door;
                data_state.next_room = next_room;
                data_state.last_room = data_state.current_room;
                state = State::CHANGING_ROOM;
            } else
            {
                qInfo() << __FUNCTION__ << "No doors found";
                state = State::IDLE;
            }
            break;
        }
        case State::CHANGING_ROOM:
            qInfo() << __FUNCTION__ << "CHANGING_ROOM";
            data_state = changing_room(data_state);
            if(data_state.at_target_room)
            {
                data_state.at_target_room = false;
                data_state.room_detected = false;
                grid.set_all_to_free();
                if(data_state.next_room == -1)  //new room
                {
                    data_state.current_room = (int)G.rooms.size(); // new room number
                    G.doors.at(data_state.current_door).to_rooms.insert(data_state.current_room);
                    state = State::EXPLORING;
                }
                else
                {
                    // grid.compute_occupied_from_rectangle();
                    state = State::VISITING;
                    data_state.current_room = data_state.next_room; // new room number
                }
            }
            break;
        case State::IDLE:
            qInfo() << __FUNCTION__ << "IDLE";
            move_robot(0.0,0.0);
            break;
    }
    for(auto &r : G.rooms)
        r.print();
    for(auto &d : G.doors)
        d.print();
    data_state.print();
    qInfo() << __FUNCTION__ << "---------------------------";

}

////////////////////////////////////////////////////////////////////////////
SpecificWorker::Data_State SpecificWorker::exploring(const Data_State &data_state)
{
    static float initial_angle;
    static int count = 0;
    Data_State new_data_state = data_state;
    static ExploreState explore_state = ExploreState::INIT_TURN;
    switch (explore_state)
    {
        case ExploreState::INIT_TURN:
            initial_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            move_robot(0, 0.5);
            explore_state = ExploreState::TURN;
            break;

        case ExploreState::TURN:
        {
            //qInfo() << "TURN";
            // check if turn is finished
            float current = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            if (fabs(current - initial_angle) < (M_PI + 0.1) and fabs(current - initial_angle) > (M_PI - 0.1))
            {
                move_robot(0, 0);
                explore_state = ExploreState::ESTIMATE;
                count = 0;
                break;
            }
            else  // keep turning
            {
                detect_doors(data_state);
                G.draw_doors(G.doors, &viewer_robot->scene);
            }
            break;
        }
        case ExploreState::ESTIMATE:
        {
            //qInfo() << "ESTIMATE: room ->" << current_room << last_room;
            auto res_data = estimate_rooms(data_state);
            if(not res_data.room_detected )
            {
                qInfo() << __FUNCTION__ << count << "th try";
                if (count++ > 5)
                {
                    new_data_state.no_rooms_found = true;
                    explore_state = ExploreState::INIT_TURN;    // so next time we start by turning
                }
            }
            else
            {
                    new_data_state.room_detected = true;        // change state in higher state machine
                    explore_state = ExploreState::INIT_TURN;    // so next time we start by turning
            }
            break;
        }
    }
    return new_data_state;
}
void SpecificWorker::detect_doors(const Data_State &data_state)
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
    qInfo() << __FUNCTION__  << "  peaks " << peaks.size();

    // pairwise comparison of peaks to filter in doors
    for (auto &&c: iter::combinations_with_replacement(peaks, 2))
        if ((c[0] - c[1]).norm() < 1100 and (c[0] - c[1]).norm() > 600)
        {
            Graph_Rooms::Door d{c[0], c[1], (int)G.doors.size()};
            d.to_rooms.insert(data_state.current_room);
            std::vector<Graph_Rooms::Door>::iterator hit;
            if (hit = std::find_if(G.doors.begin(), G.doors.end(), [d](auto a) { return d == a; }); hit == G.doors.end())  // new door
                G.doors.emplace_back(d);
            else // there is a door equal to the detected door. Check if it connects to other room and does not contain this room
                if((*hit).to_rooms.size()==1 and not (*hit).connects_to_room(data_state.current_room))
                {
                    (*hit).to_rooms.insert(data_state.current_door);
                    qInfo() << __FUNCTION__ << "Insert new room " << data_state.current_room << "in door " << (*hit).id;
                }
        }
    qInfo() << "  doors computed" << G.doors.size();
}
SpecificWorker::Data_State SpecificWorker::estimate_rooms(const Data_State &data_state)
{
    Data_State new_data_state = data_state;
    // create list of occuppied points
    RoboCompRoomDetection::ListOfPoints points;
    for (const auto &[key, val]: std::ranges::filter_view(grid, [](auto a) { return not a.second.free; }))
        points.emplace_back(RoboCompRoomDetection::Corner{(float)key.x, (float)key.z});

    // filter out points beyond doors
    RoboCompRoomDetection::ListOfPoints inside_points;
    for (const auto &c : points)
        for(const auto &d : G.doors | iter::filter([r = data_state.current_room](auto d){return d.connects_to_room(r);}))
        {
            auto p_r = from_world_to_robot(Eigen::Vector2f(c.x, c.y));
            auto mid_r = from_robot_to_world(d.get_midpoint());
            auto focal = mid_r.norm();
            if( not( (p_r.norm() > focal*1.3)) )// and fabs(focal * p_r.x() / p_r.y()) < d.width()))
                inside_points.push_back(c);
//            else
//                qInfo() << __FUNCTION__ << "discarded " << c.x << c.y;
        }

    // sample size()/4 points from detected corners
    RoboCompRoomDetection::ListOfPoints sampled_points;
    auto gen = std::mt19937{std::random_device{}()};
    std::ranges::sample(inside_points, std::back_inserter(sampled_points), std::clamp((double)inside_points.size(), inside_points.size() / 4.0, 300.0), gen);

//    qInfo() << __FUNCTION__ << "    proxy working...";
//    RoboCompRoomDetection::Rooms detected_rooms;
//    try
//    { detected_rooms = roomdetection_proxy->detectRoom(sampled_points); }
//    catch (const Ice::Exception &e)
//    { std::cout << e.what() << std::endl; }
//    qInfo() << __FUNCTION__ << "    number of detected rooms by proxy: " << detected_rooms.size();
//

    Eigen::MatrixX3f my_points;
    my_points.resize(sampled_points.size(), 3);
    for(auto &&[i, p] : sampled_points | iter::enumerate)
    {
        my_points(i, 0) = p.x; my_points(i, 1) = p.y; my_points(i, 2) = 1.0;
    }
    QRectF ro = room_detector.compute_room(my_points);
    IOU::Quad max(IOU::Point(ro.left(), ro.top()), IOU::Point(ro.right(), ro.top()), IOU::Point(ro.right(), ro.bottom()), IOU::Point(ro.left(), ro.bottom()));

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
    new_data_state.room_detected = true;

    // insert the new room only if it does not exist yet
    Graph_Rooms::Room room(max, data_state.current_room);
    if (auto r = std::find_if(G.rooms.begin(), G.rooms.end(), [room](auto a) { return room == a; }); r == G.rooms.end())
        G.rooms.push_back(room);

    // complete door info about room connection. We need the list of doors visible from here
    // adjust door to room's wall

    return new_data_state;
}
SpecificWorker::State SpecificWorker::visiting(State &state){ return state;}
SpecificWorker::Data_State SpecificWorker::changing_room(const Data_State &data_state)
{
    Data_State new_data_state = data_state;
    qInfo() << __FUNCTION__ << " going to room " << data_state.next_room << " from: "
            << data_state.current_room << "through: " << data_state.current_door;

    // pick a point 1 meter ahead of center of door position and in the other room
    Graph_Rooms::Door door;
    try{ door = G.doors.at(data_state.current_door); } catch(std::exception &e){ std::cout << e.what() << std::endl;};
    const Graph_Rooms::Room &room = G.rooms.at(data_state.current_room);
    auto rx = room.quad.p1 + (room.quad.p3-room.quad.p1)/2;
    auto mid_point = door.get_external_midpoint(data_state.current_room, Eigen::Vector2f(rx.x, rx.y));

    // draw target point
    static QGraphicsItem* dest= nullptr;
    if(dest != nullptr) viewer_robot->scene.removeItem(dest);
    dest = viewer_robot->scene.addEllipse(0, 0, 200, 200, QPen(QColor("Magenta"), 50));
    dest->setPos(mid_point.x(), mid_point.y()); dest->setZValue(200);

    auto tr = from_world_to_robot(mid_point);
    float dist = tr.norm();
    if(dist < 150)  // at target
    {
        qInfo() << __FUNCTION__ << "    Robot reached target room" << data_state.next_room;
        move_robot(0,0);
        new_data_state.at_target_room = true;
    }
    else  // continue to room
    {
        // call dynamic window
        QPolygonF laser_poly;
        for(auto &&l : ldata)
            laser_poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
        auto [_, __, adv, rot, ___] = dw.compute(tr, laser_poly,
                                                 Eigen::Vector3f(r_state.x, r_state.y, r_state.rz),
                                                 Eigen::Vector3f(r_state.vx, r_state.vy, r_state.vrz),
                                                 &viewer_robot->scene);
        const float rgain = 0.8;
        float rotation = rgain*rot;
        float dist_break = std::clamp(from_world_to_robot(target.to_eigen()).norm() / 1000.0, 0.0, 1.0);
        float advance = constants.max_advance_speed * dist_break * gaussian(rotation);
        move_robot(advance, rotation);
    }
    return new_data_state;
}
std::tuple<int, int> SpecificWorker::choose_exit_door(const Data_State &data_state)
{
    // Choose an un-explored destination room
    int new_door = -1;
    int new_room = -1;
    for(const auto &[k, d] : G.doors | iter::filter([r = data_state.current_room](auto d)
                    { return d.connects_to_room(r) and d.to_rooms.size()==1;}) | iter::enumerate)
        {
            new_door = d.id;
            new_room = -1;  //unkown
            qInfo() << "   Door to NEW room selected" << d.id;
            d.print();
            break;
        }
    if(new_door == -1)  // no door to un-explored room. Pick a random one
    {
        std::vector<int> potential_doors;
        for (const auto &[k, d]: G.doors | iter::filter([r = data_state.current_room](auto d) { return d.connects_to_room(r); }) | iter::enumerate)
            potential_doors.push_back(d.id);
        std::vector<int> selected_doors;
        auto gen = std::mt19937{std::random_device{}()};
        std::ranges::sample(potential_doors, std::back_inserter(selected_doors), 1, gen);
        new_door = selected_doors.front();
        auto room_res = std::ranges::find_if(G.doors.at(new_door).to_rooms, [cr = data_state.current_room](auto id){ return id != cr;});
        new_room = *room_res;
        qInfo() << "   Door" << new_door << "to KNOWN room selected" << new_room;
    }
    return std::make_tuple(new_door, new_room );
}

////////////////////////////////// AUX /////////////////////////////////////
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