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
#include "graph_rooms.h"
#include "room_detector_grad_stocastic.h"
#include <chrono>
#include <thread>
#include <future>

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
        const float final_distance_to_target = 150; //mm
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
    void read_base();

    // laser
    RoboCompLaser::TLaserData ldata;
    const int MAX_LASER_RANGE = 4000;
    void read_laser();

    // behaviors
    bool explore();
    bool change_room();
    void detect_doors();
    bool estimate_rooms();

    // grid
    int TILE_SIZE = 50;
    Grid grid;
    void update_map(const RoboCompLaser::TLaserData &ldata);

    // coordinates
    Eigen::Vector2f from_robot_to_world(const Eigen::Vector2f &p);
    Eigen::Vector2f from_world_to_robot(const Eigen::Vector2f &p);
    void fit_rectangle();
    void check_free_path_to_target(const RoboCompLaser::TLaserData &ldata,
                                   const Eigen::Vector2f &goal);
    inline QPointF to_qpointf(const Eigen::Vector2f &p) const { return QPointF(p.x(), p.y());};

    // target
    struct Target
    {
        bool active = false;
        QPointF pos;
        Eigen::Vector2f to_eigen() const {return Eigen::Vector2f(pos.x(), pos.y());}
        QGraphicsEllipseItem *draw = nullptr;
    };
    Target target;

    // Dynanimc Window
    Dynamic_Window dw;

    // graph
    Graph_Rooms G;

    // stocastic room detetor
    Room_Detector_Grad_Stochastic room_detector;
};

#endif
