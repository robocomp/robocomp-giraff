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
#include <ranges>

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
    const auto &[rp, ep] = viewer_robot->add_robot(constants.robot_length, constants.robot_length);
    robot_polygon = rp;
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract

    viewer_graph = new AbstractGraphicViewer(this->frame_graph, this->dimensions);
    this->resize(1200,450);

    //connect(viewer_robot, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
    // grid
    // grid.initialize(dimensions, constants.tile_size, &viewer_robot->scene, false);
    // qInfo() << __FUNCTION__ << "Grid initialized to " << this->dimensions;



    ////////////////////////////////////////////////////
    this->Period = period;
    if(this->startup_check_flag)
        this->startup_check();
    else
        timer.start(Period);
}
void SpecificWorker::compute()
{
    read_base();
    read_laser();
    RoboCompCameraRGBDSimple::TImage img = read_camera();
    if (img.image.empty())
    {
        qWarning() << __FUNCTION__ << "Image empty. Returning;";
        return;
    };
    tags.clear();
    tags = read_apriltags(img);

    switch(state)
    {
        case States::IDLE:
            break;
        case States::INIT_EXPLORING:
            state = init_exploring();
            break;
        case States::EXPLORING:
            state = exploring();    // if a tag is found that corresponds to an existing room, quit exploring
            break;
        case States::AFTER_EXPLORING:
            state = after_exploring();
            break;
        case States::INIT_CHANGING_ROOM:
            state = init_changing_room();
            break;
        case States::CHANGING_ROOM:
            state = changing_room();
            break;
        case States::AFTER_CHANGING_ROOM:
            state = after_changing_room();
            break;
    };
}

////////////////////////////////////////////////////////////////////////////
SpecificWorker::States SpecificWorker::init_exploring()
{
    qInfo() << __FUNCTION__ << ": Explore first time. Starting explore with new local_grid...";
    QRectF dim(-2000, -2000, 4000, 4000);  //compute from laser max
    grid_world_pose = {.ang=robot_pose.ang, .pos=robot_pose.pos};
    local_grid.initialize(dim, constants.tile_size, &viewer_robot->scene, false,
                          std::string(), grid_world_pose.toQpointF(), grid_world_pose.ang);
    local_grid_is_active = true;  // to synchronize with updatemap
    move_robot(0, 0.4);
    initial_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
    peaks.clear();
    qInfo() << __FUNCTION__ << "Exploring now...";
    return States::EXPLORING;
}
SpecificWorker::States SpecificWorker::exploring()
{
    float current = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
    static std::vector<std::tuple<float, int>> tags_set;
    if(not (fabs(current - initial_angle) < (M_PI + 0.1) and fabs(current - initial_angle) > (M_PI - 0.1)))
    {
        for(const auto &t : this->tags)
            tags_set.push_back(std::make_tuple(t.tz, t.id));

        update_map(ldata);
        auto new_peaks = detect_doors();
        peaks.insert(std::end(peaks), std::begin(new_peaks), std::end(new_peaks));
    }
    else
    {
        if(not tags_set.empty())
        {
            auto [dist, id] = std::ranges::min(tags_set, [](auto a, auto b) { return std::get<0>(a) < std::get<0>(b); });
            tags_set.clear();
            qInfo() << __FUNCTION__ << "Finished exploring. Current detected room" << id;
            current_tag = id;
            return States::AFTER_EXPLORING;
        }
        else qWarning() << __FUNCTION__ << "Attention: not tags encountered during exploration";
    }
    return States::EXPLORING;
}
SpecificWorker::States SpecificWorker::after_exploring()
{
    // do after end exploring once
    qInfo() << __FUNCTION__ << "After exploring";
    // pairwise comparison of peaks to filter in doors. Peaks are in grid RF
    for (auto &&c: iter::combinations_with_replacement(peaks, 2))
        if ((c[0] - c[1]).norm() < 1100 and (c[0] - c[1]).norm() > 550)
            G.add_door_to_current_room(c[0], c[1]);

    move_robot(0, 0);
    estimate_rooms();

    qInfo() << __FUNCTION__ << "Changing key";
    // current_room is -1 for unknown rooms
    G.change_current_room_key_to(current_tag);
    G.current_room().is_unknown = false;
    G.current_room().id = current_tag;
    G.current_room_local = current_tag;
    G.current_room().print();

    // room_detector.minimize_door_distances(G);
    // DRAW: move model room to world ref system to draw it
    auto const &rc = G.current_room().room_rect.center;
    auto g2w = from_grid_to_world(Eigen::Vector2f{rc.x, rc.y});
    G.current_room().draw(&viewer_robot->scene, g2w, grid_world_pose.ang);

    for (const auto &d: G.current_room().doors_ids)
        G.doors.at(d).draw(&viewer_robot->scene, grid_world_pose.pos, grid_world_pose.ang);
    local_grid_is_active = false;
    //G.draw_nodes(&viewer_graph->scene);
    //G.draw_all(&viewer_robot->scene, &viewer_graph->scene);

    return States::INIT_CHANGING_ROOM;
}
SpecificWorker::States SpecificWorker::init_changing_room()
{
    // before start
    qInfo() << __FUNCTION__ << "Entering from room " << G.current_room().id;
    // Choose an un-explored destination room
    new_room_id = -1;
    // door to unknown room
    for (const auto &d_id: G.current_room().doors_ids)
        if (G.doors.at(d_id).rooms.size() == 1) // the other room is unknown
        {
            new_door_id = d_id;
            break;
        }
            //    if( auto hit = std::ranges::find_if(G.current_room().doors_ids,[this](auto id){ auto d = G.doors.at(id); d.rooms[]
            //        return d.}); hit != G.current_room().doors.end())
            //    {
            //        new_door = (*hit);
            //        new_room_id = -1;
            //    }
        else //  door to known room
        {
            std::vector<int> selected_doors;
            auto gen = std::mt19937{std::random_device{}()};
            std::ranges::sample(G.current_room().doors_ids, std::back_inserter(selected_doors), 1, gen);
            if (not selected_doors.empty())
            {
                new_door_id = selected_doors.front();
                for (const auto &[k, v]: G.doors.at(new_door_id).rooms)
                {
                    if (k != G.current_room().id)
                        new_room_id = v.room_id;
                }
            } else
            {
                qInfo() << "WARNING, no door to choose";
            }
        }
    auto &new_door = G.doors.at(new_door_id);
    mid_point = new_door.get_external_midpoint(from_robot_to_grid(Eigen::Vector2f(0.f, 0.f)));
    return States::CHANGING_ROOM;
}
SpecificWorker::States SpecificWorker::changing_room()
{
    // move to the new room
    // pick a point 1 meter ahead of center of door position and in the other room
    auto mp = from_grid_to_world(mid_point);
    viewer_robot->scene.addEllipse(mp.x()-100, mp.y()-100, 200, 200, QPen(QColor("blue"), 20), QBrush(QColor("blue")));
    float dist = from_grid_to_robot(mid_point).norm();
    //qInfo() << __FUNCTION__ << " dist to target: " << dist << " Midpoint: "<< mid_point.x() << mid_point.y();
    if( dist > constants.final_distance_to_target) // until target is reached
    {
        auto tr = from_grid_to_robot(mid_point);
        dist = tr.norm();

        // call dynamic window
        QPolygonF laser_poly;
        for(auto &&l : ldata)
            laser_poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
        auto [_, __, adv, rot, ___] = dw.compute(tr, laser_poly, 0.f, 0.f, // current advance and rotation speed
                                                 nullptr /*&viewer_robot->scene*/);
        const float rgain = 0.8;
        float rotation = rgain*rot;
        float dist_break = std::clamp(dist / 1000.0, 0.0, 1.0);
        float advance = constants.max_advance_speed * dist_break * gaussian(rotation);
        move_robot(advance, rotation);
        return States::CHANGING_ROOM;
    }
    return States::AFTER_CHANGING_ROOM;
}
SpecificWorker::States SpecificWorker::after_changing_room()
{
    qInfo() << __FUNCTION__ << "Ended changing room";
    // if reached a room from a door to unknown room
    if(new_room_id == -1)
    {
        G.rooms.insert(std::make_pair( -1, Graph_Rooms::Room(-1)));
        G.current_room_local = new_room_id; //G.rooms.back().id; // -1 should be changed when tag is detected
    }
    else
        G.current_room_local = new_room_id;

    move_robot(0,0);
    qInfo() << __FUNCTION__ << "Robot reached target room" << G.current_room_local;
    return States::INIT_EXPLORING;
    //room_detector.minimize_door_distances(G);
    //G.project_doors_on_room_side(G.current_room(), &viewer_robot->scene);
    //G.draw_all(&viewer_robot->scene, &viewer_graph->scene);
    // if known room check if it matches the prediction. If not, set it as unknown so it is explored again
    // active = false;
}

////////////////////////////////////////////////////////////////////
std::vector<Eigen::Vector2f> SpecificWorker::detect_doors()
{
    // get peaks from former iteration and add the new ones
    std::vector<Eigen::Vector2f> peaks;
    move_robot(0, 0.5);

    // search for corners. Compute derivative wrt distance
    std::vector<float> derivatives(ldata.size());
    derivatives[0] = 0;
    for (const auto &&[k, l]: iter::sliding_window(ldata, 2) | iter::enumerate)
        derivatives[k + 1] = l[1].dist - l[0].dist;

    // filter derivatives greater than a threshold
    for (const auto &&[k, der]: iter::enumerate(derivatives))
    {
        RoboCompLaser::TData l;
        if (der > constants.door_peak_threshold)
        {
            l = ldata.at(k - 1);
            peaks.push_back(from_robot_to_grid(Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle))));
        }
        else if (der < -constants.door_peak_threshold)
        {
            l = ldata.at(k);
            peaks.push_back(from_robot_to_grid(Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle))));
        }
    }
    return peaks;
}
bool SpecificWorker::estimate_rooms()
{
    // create list of occuppied points
    RoboCompRoomDetection::ListOfPoints points;
    for (const auto &[key, val]: std::ranges::filter_view(local_grid, [](auto a) { return not a.second.free; }))
        points.emplace_back(RoboCompRoomDetection::Corner{(float)key.x, (float)key.z});

    // filter out points beyond room doors
    RoboCompRoomDetection::ListOfPoints inside_points;
    bool skip=false;
    for (const auto &c : points)
    {
        for (const auto &d_id: G.current_room().doors_ids)
        {
            const auto &d = G.doors.at(d_id);
            // all in robot's reference system
            QPointF robot_rq(0, 0);
            auto point_r = from_grid_to_robot(Eigen::Vector2f(c.x, c.y));
            auto point_rq = to_qpointf(point_r);
            auto point_r_distance = point_r.norm();
            auto mid_door_distance = from_grid_to_robot(d.get_midpoint()).norm();
            // line from point to robot
            auto l_point = QLineF(point_rq, robot_rq);
            // door line
            auto l_door = QLineF(to_qpointf(from_grid_to_robot(d.p1)), to_qpointf(from_grid_to_robot(d.p2)));
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
                local_grid.set_free(c.x, c.y);
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
    std::vector<cv::Point2f> cv_points(inside_points.size());
    for(auto &&[i, p] : inside_points | iter::enumerate)
    {
        my_points(i, 0) = p.x; my_points(i, 1) = p.y; my_points(i, 2) = 1.0;
        cv_points[i] = cv::Point2f{p.x, p.y};
    }

    // call optimizer
    cv::RotatedRect ro = cv::minAreaRect(cv_points);

//    cv::RotatedRect ro = room_detector.compute_room(my_points);
//    cv::Point2f ro_points[4];
//    ro.points(ro_points);
//    IOU::Quad max(IOU::Point(ro_points[0].x, ro_points[0].y),
//                  IOU::Point(ro_points[1].x, ro_points[1].y),
//                  IOU::Point(ro_points[2].x, ro_points[2].y),
//                  IOU::Point(ro_points[3].x, ro_points[3].y));

    // update current room
    Graph_Rooms::Room &room = G.current_room();
    //room.quad = max;
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
        robot_pose = {.ang=r_state.rz, .pos=Eigen::Vector2f(r_state.x, r_state.y)};
        robot_polygon->setRotation(r_state.rz * 180 / M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);
        robot_polygon->setZValue(55);
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
}
void SpecificWorker::read_laser()
{
    try
    {
        ldata = laser_proxy->getLaserData();
        draw_laser(ldata);
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
}
RoboCompCameraRGBDSimple::TImage SpecificWorker::read_camera()
{
    RoboCompCameraRGBDSimple::TImage img;
    try
    {
        img = camerargbdsimple_proxy->getImage("camera_top");
        if (img.image.empty()) { qWarning() << "Returning from read_camera" ; return img;}
        cv::Mat img_cv = cv::Mat(img.height, img.width, CV_8UC3, (uchar *)&img.image[0]);
        cv::Mat img_reduced;
        cv::resize(img_cv, img_reduced, cv::Size(240,320));
        cv::imshow("Eye", img_reduced);
        cv::waitKey(1);
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
    return img;
}
RoboCompAprilTags::TagsList SpecificWorker::read_apriltags(const RoboCompCameraRGBDSimple::TImage &img)
{
    RoboCompAprilTags::TagsList tags;
    try
    {
        tags = apriltags_proxy->getAprilTags(img, 0.5, "36h11");
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << " Warning. Error reading AprilTagsServer" << std::endl; }
    return tags;
}

//////////////////////////////////////////////////////////////////////////////////////////////
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
    // get the matrix to transform from robot to local_grid
    Eigen::Matrix3f r2g = from_robot_to_grid_matrix();
    auto robot_in_grid = from_world_to_grid(Eigen::Vector2f(robot_pose.pos.x(), robot_pose.pos.y()));
    for(const auto &l : ldata)
    {
        if(l.dist > constants.robot_semi_length)
        {
            // transform tip form robot's RS to local_grid RS
            Eigen::Vector2f tip = (r2g * Eigen::Vector3f(l.dist*sin(l.angle), l.dist*cos(l.angle), 1.f)).head(2);
            //Eigen::Vector2f tip_in_grid = local_grid.pointToGrid(tip);

//            int last_kx = std::numeric_limits<int>::min();
//            int last_kz = std::numeric_limits<int>::min();

//           for( const auto list = bresenham(robot_in_grid, tip); const auto pp: list)
//                grid.add_miss(pp);
//            if( l.dist < constants.max_laser_range)
//                grid.add_hit(tip_in_grid);

            int num_steps = ceil(l.dist/(constants.tile_size));
            Eigen::Vector2f p;
            for(const auto &&step : iter::range(0.0, 1.0-(1.0/num_steps), 1.0/num_steps))
            {
                p = robot_in_grid * (1-step) + tip*step;
                local_grid.add_miss(p);
            }
            if(l.dist <= constants.max_laser_range)
                local_grid.add_hit(tip);

            if((p-tip).norm() < constants.tile_size)  // in case las miss overlaps tip
                local_grid.add_hit(tip);
        }
    }
    local_grid.update_costs();
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
Eigen::Vector2f SpecificWorker::from_grid_to_world(const Eigen::Vector2f &p)
{
    // build the matrix to transform from grid to world knowing robot and grid pose in world
    Eigen::Matrix2f g2w;
    g2w <<  cos(grid_world_pose.ang), -sin(grid_world_pose.ang),
            sin(grid_world_pose.ang), cos(grid_world_pose.ang);
    return g2w * p + grid_world_pose.pos;
}
Eigen::Vector2f SpecificWorker::from_world_to_grid(const Eigen::Vector2f &p)
{
    // build the matrix to transform from world to local_grid, knowing robot and grid pose in world
    Eigen::Matrix2f w2g;
    w2g <<  cos(grid_world_pose.ang), sin(grid_world_pose.ang),
            -sin(grid_world_pose.ang), cos(grid_world_pose.ang);
    return w2g * (p - grid_world_pose.pos);
}
Eigen::Vector2f SpecificWorker::from_robot_to_grid(const Eigen::Vector2f &p)
{
    return (from_robot_to_grid_matrix() * Eigen::Vector3f(p.x(), p.y(), 1.f)).head(2);
}
Eigen::Vector2f SpecificWorker::from_grid_to_robot(const Eigen::Vector2f &p)
{
    return (from_robot_to_grid_matrix().inverse() * Eigen::Vector3f(p.x(), p.y(), 1.f)).head(2);
}
Eigen::Matrix3f SpecificWorker::from_robot_to_world_matrix()
{
    Eigen::Matrix3f matrix;
    matrix << cos(robot_pose.ang) , -sin(robot_pose.ang) , robot_pose.pos.x(),
            sin(robot_pose.ang) , cos(robot_pose.ang), robot_pose.pos.y(),
            0.f, 0.f, 1.f;
    return matrix;
}
Eigen::Matrix3f SpecificWorker::from_world_to_grid_matrix()
{
    Eigen::Matrix2f w2g_2d_matrix;
    w2g_2d_matrix <<  cos(grid_world_pose.ang), sin(grid_world_pose.ang),
            -sin(grid_world_pose.ang), cos(grid_world_pose.ang);
    auto tr = w2g_2d_matrix * grid_world_pose.pos;
    Eigen::Matrix3f matrix;
    matrix << cos(grid_world_pose.ang), sin(grid_world_pose.ang), -tr.x(),
            -sin(grid_world_pose.ang), cos(grid_world_pose.ang), -tr.y(),
            0.f, 0.f, 1.f;
    return matrix;
}
Eigen::Matrix3f SpecificWorker::from_robot_to_grid_matrix()
{
    // build the matrix to transform from robot to local_grid, knowing robot and grid pose in world
    Eigen::Matrix3f r2w = from_robot_to_world_matrix();
//    Eigen::Matrix2f w2g_2d_matrix;
//    w2g_2d_matrix <<  cos(grid_world_pose.ang), sin(grid_world_pose.ang),
//            -sin(grid_world_pose.ang), cos(grid_world_pose.ang);
//    auto tr = w2g_2d_matrix * grid_world_pose.pos;
//    Eigen::Matrix3f w2g;
//    w2g << cos(grid_world_pose.ang), sin(grid_world_pose.ang), -tr.x(),
//            -sin(grid_world_pose.ang), cos(grid_world_pose.ang), -tr.y(),
//            0.f, 0.f, 1.f;
    Eigen::Matrix3f w2g = from_world_to_grid_matrix();
    Eigen::Matrix3f r2g = w2g * r2w;  // from r to world and then from world to grid
    return r2g;
}
Eigen::Matrix3f SpecificWorker::from_grid_to_robot_matrix()
{
    return from_robot_to_grid_matrix().inverse();
}
Eigen::Matrix3f SpecificWorker::from_grid_to_world_matrix()
{
    //return from_world_to_grid_matrix().inverse();
    Eigen::Matrix3f g2w;
    g2w <<  cos(grid_world_pose.ang), -sin(grid_world_pose.ang), grid_world_pose.pos.x(),
            sin(grid_world_pose.ang), cos(grid_world_pose.ang), grid_world_pose.pos.y(),
            0.f, 0.f, 1.f;
    return g2w;
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
    //laser_polygon->setZValue(20);
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

//void SpecificWorker::compute()
//{
//    read_base();
//    read_laser();
//    static bool EXPLORE = true;
//
//        if (EXPLORE and G.current_room().is_unknown) // explore room
//        {
//            static std::future<bool> future_explore;
//            if (not future_explore.valid())
//                future_explore = std::async(std::launch::async, &SpecificWorker::explore, this);
//            else if (future_explore.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
//            {
//                auto res = future_explore.get();
//                G.current_room().is_unknown = false;
//                qInfo() << __FUNCTION__ << "Future EXPLORE ended with result:" << res;
//                // room_detector.minimize_door_distances(G);
//                // move model room to world ref system to draw it
//                auto const &rc = G.current_room().room_rect.center;
//                auto g2w = from_grid_to_world(Eigen::Vector2f{rc.x, rc.y});
//                G.current_room().draw(&viewer_robot->scene, g2w, grid_world_pose.ang);
//
//                for (const auto &d: G.current_room().doors_ids)
//                    G.doors.at(d).draw(&viewer_robot->scene, grid_world_pose.pos, grid_world_pose.ang);
//                local_grid_is_active = false;
//                EXPLORE = false;
//                //G.draw_nodes(&viewer_graph->scene);
//                //G.draw_all(&viewer_robot->scene, &viewer_graph->scene);
//            }
//        }
//        else  // change room
//        {
//            static std::future<bool> future_visit;
//            if (not future_visit.valid())
//                future_visit = std::async(std::launch::async, &SpecificWorker::change_room, this);
//            else if (future_visit.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
//            {
//                auto res = future_visit.get();
//                qInfo() << __FUNCTION__ << "Future VISIT ended with " << res;
//                G.current_room().print();
//                EXPLORE = true;
//                //room_detector.minimize_door_distances(G);
//                //G.project_doors_on_room_side(G.current_room(), &viewer_robot->scene);
//                //G.draw_all(&viewer_robot->scene, &viewer_graph->scene);
//                // if known room check if it matches the prediction. If not, set it as unknown so it is explored again
//               // active = false;
//            }
//        }
//}

//SpecificWorker::States SpecificWorker::init_explore()
//{
//    static std::vector<Eigen::Vector2f> peaks;
//    static float initial_angle;
//    static bool explore_first_time = true;
//
//    if (explore_first_time)
//    {
//        // do before
//        qInfo() << __FUNCTION__ << ": Explore first time. Starting explore with new local_grid...";
//        QRectF dim(-2000, -2000, 4000, 4000);  //compute from laser max
//        grid_world_pose = {.ang=robot_pose.ang, .pos=robot_pose.pos};
//        local_grid.initialize(dim, constants.tile_size, &viewer_robot->scene, false,
//                              std::string(), grid_world_pose.toQpointF(), grid_world_pose.ang);
//        local_grid_is_active = true;  // to synchronize with updatemap
//        move_robot(0, 0.4);
//        initial_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
//        peaks.clear();
//        explore_first_time = false;
//        qInfo() << __FUNCTION__ << "Exploring now...";
//    }
//    else if( const auto [finish, id] = exploring(initial_angle, peaks); finish == true)  // do meanwhile
//    {
//        // do after end exploring once
//        qInfo() << __FUNCTION__ << "After exploring";
//        // pairwise comparison of peaks to filter in doors. Peaks are in grid RF
//        for (auto &&c: iter::combinations_with_replacement(peaks, 2))
//            if ((c[0] - c[1]).norm() < 1100 and (c[0] - c[1]).norm() > 550)
//                G.add_door_to_current_room(c[0], c[1]);
//
//        move_robot(0, 0);
//        estimate_rooms();
//
//        qInfo() << __FUNCTION__ << "Changing key";
//        // current_room is -1 for unknown rooms
//        G.change_current_room_key_to(id);
//        G.current_room().is_unknown = false;
//        G.current_room().id = id;
//        G.current_room_local = id;
//        G.current_room().print();
//
//        // room_detector.minimize_door_distances(G);
//        // DRAW: move model room to world ref system to draw it
//        auto const &rc = G.current_room().room_rect.center;
//        auto g2w = from_grid_to_world(Eigen::Vector2f{rc.x, rc.y});
//        G.current_room().draw(&viewer_robot->scene, g2w, grid_world_pose.ang);
//
//        for (const auto &d: G.current_room().doors_ids)
//            G.doors.at(d).draw(&viewer_robot->scene, grid_world_pose.pos, grid_world_pose.ang);
//        local_grid_is_active = false;
//        //G.draw_nodes(&viewer_graph->scene);
//        //G.draw_all(&viewer_robot->scene, &viewer_graph->scene);
//
//        explore_first_time = true;
//        return States::CHANGE_ROOM;
//    }
//    return States::EXPLORE;
//}
//
//SpecificWorker::States SpecificWorker::init_change_room()
//{
//    static bool visit_first_time = true;
//    static int new_door_id, new_room_id;
//    static Eigen::Vector2f mid_point;
//    if (visit_first_time)
//    {
//        // before start
//        qInfo() << __FUNCTION__ << "Entering from room " << G.current_room().id;
//        // Choose an un-explored destination room
//        new_room_id = -1;
//        // door to unknown room
//        for(const auto &d_id : G.current_room().doors_ids)
//            if(G.doors.at(d_id).rooms.size() == 1) // the other room is unknown
//            {
//                new_door_id = d_id;
//                break;
//            }
//                //    if( auto hit = std::ranges::find_if(G.current_room().doors_ids,[this](auto id){ auto d = G.doors.at(id); d.rooms[]
//                //        return d.}); hit != G.current_room().doors.end())
//                //    {
//                //        new_door = (*hit);
//                //        new_room_id = -1;
//                //    }
//            else //  door to known room
//            {
//                std::vector<int> selected_doors;
//                auto gen = std::mt19937{std::random_device{}()};
//                std::ranges::sample(G.current_room().doors_ids, std::back_inserter(selected_doors), 1, gen);
//                if(not selected_doors.empty())
//                {
//                    new_door_id = selected_doors.front();
//                    for(const auto &[k, v] : G.doors.at(new_door_id).rooms)
//                    {
//                        if(k != G.current_room().id)
//                            new_room_id = v.room_id;
//                    }
//                }
//                else
//                {
//                    qInfo() << "WARNING, no door to choose";
//                }
//            }
//        auto &new_door = G.doors.at(new_door_id);
//        mid_point = new_door.get_external_midpoint(from_robot_to_grid(Eigen::Vector2f(0.f, 0.f)));
//        visit_first_time = false;
//    }
//    else if (changing_room(mid_point))  // do meanwhile the robot reaches the new room
//    {
//        qInfo() << __FUNCTION__ << "Ended changing room";
//        // do after end change room once
//        // if reached a room from a door to unknown room
//        if(new_room_id == -1)
//        {
//            G.rooms.insert(std::make_pair( -1, Graph_Rooms::Room(-1)));
//            G.current_room_local = new_room_id; //G.rooms.back().id; // -1 should be changed when tag is detected
//        }
//        else
//            G.current_room_local = new_room_id;
//
//        qInfo() << __FUNCTION__ << "Robot reached target room" << G.current_room_local;
//
//        move_robot(0,0);
//
//        visit_first_time = true;
//        return States::INIT_EXPLORING;
//        //room_detector.minimize_door_distances(G);
//        //G.project_doors_on_room_side(G.current_room(), &viewer_robot->scene);
//        //G.draw_all(&viewer_robot->scene, &viewer_graph->scene);
//        // if known room check if it matches the prediction. If not, set it as unknown so it is explored again
//        // active = false;
//    }
//    return States::CHANGE_ROOM;
//}