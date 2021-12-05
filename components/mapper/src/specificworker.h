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
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include "/home/robocomp/robocomp/classes/grid2d/grid.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include "iou.h"
#include <cppitertools/zip_longest.hpp>
#include "dynamic_window.h"


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);
    void new_target_slot(QPointF);

private:
	bool startup_check_flag;
    AbstractGraphicViewer *viewer_robot, *viewer_graph;

    struct Constants
    {
        const float max_advance_speed = 800;
        const float tile_size = 100;
        const float max_laser_range = 4000;
        float current_rot_speed = 0;
        float current_adv_speed = 0;
        float robot_length = 450;
        const float robot_semi_length = robot_length/2.0;
    };
    Constants constants;

    //robot
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    void draw_laser(const RoboCompLaser::TLaserData &ldata);
    RoboCompFullPoseEstimation::FullPoseEuler r_state;
    float gaussian(float x);
    void move_robot(float adv, float rot);

    // laser
    RoboCompLaser::TLaserData ldata;

    // state machine main
    struct Data_State
    {
        int current_room = -1;
        int current_door = -1;
        int last_room = -1;
        int next_room = -1;
        bool room_detected = false;
        bool at_target_room = false;
        bool no_rooms_found = false;
        void print() const
        {
            qInfo() << "data_state: ";
            qInfo() << "    current_room: " << current_room;
            qInfo() << "    current_door: " << current_door;
            qInfo() << "    next_room: " << next_room;
            qInfo() << "    room_detected: " << room_detected;
            qInfo() << "    at_target_room: " << at_target_room;
        };
    };
    Data_State data_state;
    enum class State {INIT, EXPLORING, VISITING, CHANGING_ROOM, IDLE};
    State state = State::INIT;
    Data_State exploring(const Data_State &data_state);
    void detect_doors(const Data_State &data_state);
    Data_State estimate_rooms(const Data_State &data_state);
    State visiting(State &state);
    Data_State changing_room(const Data_State &data_state);
    std::tuple<int,int> choose_exit_door(const Data_State &data_state);

    // state machine explore
    enum class ExploreState {INIT_TURN, TURN, ESTIMATE};

    // laser
    const int MAX_LASER_RANGE = 4000;

    // grid
    int TILE_SIZE = 50;
    QRectF dimensions;
    Grid grid;

    void update_map(const RoboCompLaser::TLaserData &ldata);
    Eigen::Vector2f from_robot_to_world(const Eigen::Vector2f &p);
    void fit_rectangle();

    // target
    struct Target
    {
        bool active = false;
        QPointF pos;
        Eigen::Vector2f to_eigen() const {return Eigen::Vector2f(pos.x(), pos.y());}
        QGraphicsEllipseItem *draw = nullptr;
    };
    Target target;

    void check_free_path_to_target(const RoboCompLaser::TLaserData &ldata,
                                   const Eigen::Vector2f &goal);

    Eigen::Vector2f from_world_to_robot(const Eigen::Vector2f &p);

    // doors
    struct Door
    {
        Eigen::Vector2f p1,p2;
        int id;
        std::set<int> to_rooms;
        const float diff = 400;
        float width() const {return (p1-p2).norm();}
        bool operator ==(const Door &d) { return ((d.p1-p1).norm() < diff and (d.p2-p2).norm() < diff) or
                                                 ((d.p1-p2).norm() < diff and (d.p2-p1).norm() < diff);};
        Eigen::Vector2f get_midpoint() const {return p1 + ((p2-p1)/2.0);};
        Eigen::Vector2f get_external_midpoint() const
        {
            Eigen::ParametrizedLine<float, 2> r =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (p1-p2).unitOrthogonal());
            //qInfo() << __FUNCTION__ << r.pointAt(800.0).x() << r.pointAt(800.0).y();
            return r.pointAt(1300.0);
        };
        std::optional<int> connecting_room(int inside_room) const
        {
            if( to_rooms.size() == 2 and to_rooms.find(inside_room) != to_rooms.end())
            {
                auto r = std::ranges::find_if_not(to_rooms, [inside_room](auto a) { return a == inside_room; });
                return *r;
            }
            else
                return {};
        };
        bool connects_to_room(int room) const { return to_rooms.contains(room);};
        void operator=(const Door &d){ p1 = d.p1; p2=d.p2; to_rooms=d.to_rooms;};
        float distance_to_robot(const Eigen::Vector2f &robot) const { return (get_midpoint() - robot).norm(); };
        void print()
        {
            qInfo() << "Door:" << id;
            qInfo() << "    p1:" << p1.x() << p1.y();
            qInfo() << "    p2:" << p2.x() << p2.y();
            for (const auto &r: to_rooms)
                qInfo() << "    to room -> " << r;
        }
    };
    std::vector<Door> doors;
    void draw_doors(const vector<Door> &local_doors, QGraphicsScene *scene);

    // thanks to https://github.com/CheckBoxStudio/IoU
    struct Room
    {
        IOU::Quad quad;
        int id;
        IOU::Vertexes points;
        const float diff = 300;
        bool operator == (const Room &d)
        {
            double iou = IOU::iou(quad, d.quad);
            return iou > 0.9;
        }
        Room(const IOU::Quad &quad_, int id_) : quad(quad_), id(id_)
        {
            quad.beInClockWise();
            quad.getVertList(points);
        };
        void print()
        {
            qInfo() << "Room:" << id;
        }
        void draw(QGraphicsScene *scene)
        {
            QPolygonF poly;
            for (auto p: points)
                poly << QPointF(p.x, p.y);
            auto v = scene->addPolygon(poly, QPen(QColor("Blue"), 90));
            v->setZValue(100);
        }
    };
    std::vector<Room> rooms;
    void draw_node(int id);

    // Dynanimc Window
    Dynamic_Window dw;

};

#endif
