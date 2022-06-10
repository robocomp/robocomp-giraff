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
#include "/home/robocomp/robocomp/classes/abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <QGraphicsPolygonItem>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "/home/robocomp/robocomp/classes/grid2d/grid.h"
#include "mpc.h"
//#include <pybind11/embed.h> // everything needed for embedding
//#include <pybind11/eigen.h>
//#include "tinynurbs/include/tinynurbs/tinynurbs.h"
//#include <unsupported/Eigen/Splines>
//#include <Python.h>
//#include <stdio.h>
//#include <dlib/optimization.h>
//#include <pybind11/embed.h>
//typedef dlib::matrix<float,0,1> column_vector;

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
        AbstractGraphicViewer *viewer;

        struct Constants
        {
            uint num_steps_mpc = 8;
            const float max_advance_speed = 1200;
            float tile_size = 100;
            const float max_laser_range = 4000;
            float current_rot_speed = 0;
            float current_adv_speed = 0;
            float robot_length = 500;
            const float robot_semi_length = robot_length/2.0;
            const float final_distance_to_target = 700; //mm
            const float min_dist_to_target = 100; //mm
            float lidar_noise_sigma  = 20;
            const int num_lidar_affected_rays_by_hard_noise = 2;
            double xset_gaussian = 0.5;             // gaussian break x set value
            double yset_gaussian = 0.7;             // gaussian break y set value
            const float target_noise_sigma = 50;
            const float prob_prior = 0.5;	        // Prior occupancy probability
            const float prob_occ = 0.9;	            // Probability that cell is occupied with total confidence
            const float prob_free = 0.3;            // Probability that cell is free with total confidence
        };
        Constants constants;

        //robot
        struct Pose2D  // pose X,Y + ang
        {
        public:


            void set_active(bool v) { active.store(v); };
            bool is_active() const { return active.load();};
            QGraphicsEllipseItem *draw = nullptr;
            void set_pos(const Eigen::Vector2f &p)
            {
                std::lock_guard<std::mutex> lg(mut);
                pos_ant = pos;
                pos = p;
            };
            void set_angle(float a)
            {
                std::lock_guard<std::mutex> lg(mut);
                ang_ant = ang;
                ang = a;
            };
            Eigen::Vector2f get_pos() const
            {
                std::lock_guard<std::mutex> lg(mut);
                return pos;
            };
            Eigen::Vector2f get_last_pos() const
            {
                std::lock_guard<std::mutex> lg(mut);
                return pos_ant;
            };
            float get_ang() const
            {
                std::lock_guard<std::mutex> lg(mut);
                return ang;
            };
            Eigen::Vector3d to_eigen_3() const
            {
                std::lock_guard<std::mutex> lg(mut);
                return Eigen::Vector3d(pos.x() / 1000.f, pos.y() / 1000.f, 1.f);
            }
            QPointF to_qpoint() const
            {
                std::lock_guard<std::mutex> lg(mut);
                return QPointF(pos.x(), pos.y());
            };

        private:
            Eigen::Vector2f pos, pos_ant{0.f, 0.f};
            float ang, ang_ant = 0.f;
            std::atomic_bool active = ATOMIC_VAR_INIT(false);
            mutable std::mutex mut;
        };

//        Pose2D read_robot();
        const int ROBOT_LENGTH = 400;
        QGraphicsPolygonItem *robot_polygon;
        QGraphicsRectItem *laser_in_robot_polygon;
        QPointF last_point;
        std::vector<QGraphicsLineItem *> lines;
        Eigen::Vector2f from_robot_to_world(const Eigen::Vector2f &p);
        Eigen::Vector2f from_world_to_robot(const Eigen::Vector2f &p);
        Eigen::Vector2f from_grid_to_world(const Eigen::Vector2f &p);
        Eigen::Vector2f from_world_to_grid(const Eigen::Vector2f &p);
        Eigen::Matrix3f from_grid_to_robot_matrix();
        Eigen::Matrix3f from_robot_to_grid_matrix();
        inline QPointF e2q(const Eigen::Vector2f &p) const {return QPointF(p.x(), p.y());};
        inline Eigen::Vector2f q2e(const QPointF &p) const {return Eigen::Vector2f(p.x(), p.y());};
        void goto_target_carrot(const std::vector<Eigen::Vector2f> &path_robot);
        void goto_target_mpc(const std::vector<Eigen::Vector2d> &path_robot, const RoboCompLaser::TLaserData &ldata);
        void move_robot(float adv, float rot, float side=0);

         // grid
        QRectF dimensions;
        Grid grid;
        Pose2D grid_world_pose;
        void update_map(const RoboCompLaser::TLaserData &ldata);

        // laser
        RoboCompLaser::TLaserData read_laser(bool noise=false);
        void draw_laser(const RoboCompLaser::TLaserData &ldata);

        // camera
        void read_camera();

        // target
        struct Target
        {
            bool active = false;
            QGraphicsEllipseItem *draw = nullptr;
            void set_pos(const QPointF &p) { pos_ant = pos; pos = p;};
            QPointF get_pos() const { return pos;};
            Eigen::Vector2f to_eigen() const {return Eigen::Vector2f(pos.x(), pos.y());}
            Eigen::Vector3f to_eigen_3() const {return Eigen::Vector3f(pos.x()/1000.f, pos.y()/1000.f, 1.f);}
            float dist_to_target_ant() const {return (to_eigen() - Eigen::Vector2f(pos_ant.x(), pos_ant.y())).norm();};

            private:
                    QPointF pos, pos_ant = QPoint(0.f,0.f);

        };
        Pose2D robot_pose;
        Pose2D target;
        template <typename Func, typename Obj>
        auto quick_bind(Func f, Obj* obj)
        { return [=](auto&&... args) { return (obj->*f)(std::forward<decltype(args)>(args)...); };}

        // path
        void draw_path(const std::vector<Eigen::Vector2f> &path_in_robot, bool clean = false);
        vector<Eigen::Vector2f> path;
        vector<Eigen::Vector2f> smoothed_path;
//        void path_smoother_eigen(std::vector<Eigen::Vector2f> ref_path);
        void path_smoother_library(const std::vector<Eigen::Vector2f> &ref_path);
        void draw_spline_path(const std::vector<Eigen::Vector2f> &path_in_robot, bool clean = false);

        // mpc
        mpc::MPC mpc;

        float gaussian(float x);
        void draw_solution_path(const vector<double> &path,  const mpc::MPC::Balls &balls);

        bool read_bill(const Pose2D &robot_pose);

        vector<Eigen::Vector2f> bresenham(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2);

    float cubicBezier(float p1, float c1, float c2, float p2, float t);

    Eigen::Vector2f cubicBezier(float p1x, float p1y, float c1x, float c1y, float c2x, float c2y, float p2x, float p2y, float t);

    float dist(float x1, float y1, float x2, float y2);

//    double rateCurve(const column_vector& params);

};

#endif
