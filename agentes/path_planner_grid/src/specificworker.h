/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
#include <localPerson.h>
#include <QGraphicsPolygonItem>
#include <doublebuffer/DoubleBuffer.h>
#include "/home/robocomp/robocomp/components/robocomp-giraff/etc/plan.h"
#include <custom_widget.h>
#include "/home/robocomp/robocomp/classes/grid2d/grid.h"
#include <collisions.h>

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
private:
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;
    std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
    std::unique_ptr<DSR::AgentInfoAPI> agent_info_api;

	//DSR params
	std::string agent_name;
	int agent_id;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;
	void modify_node_slot(std::uint64_t, const std::string &type);
	void modify_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};

	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from);
	bool startup_check_flag;

    std::shared_ptr<RoboCompCommonBehavior::ParameterList> conf_params;

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
    Target target;
    template <typename Func, typename Obj>
    auto quick_bind(Func f, Obj* obj)
    { return [=](auto&&... args) { return (obj->*f)(std::forward<decltype(args)>(args)...); };}

    //robot
    struct Pose2D
    {
        float ang;
        Eigen::Vector2f pos;
        QPointF toQpointF() const { return QPointF(pos.x(), pos.y());};
        Eigen::Vector3d to_vec3_meters() const { return Eigen::Vector3d(pos.x()/1000.0, pos.y()/1000.0, ang);};
    };
    inline QPointF e2q(const Eigen::Vector2f &p) const {return QPointF(p.x(), p.y());};
    Eigen::Vector2f from_robot_to_world(const Eigen::Vector2f &p);
    Eigen::Vector2f from_world_to_robot(const Eigen::Vector2f &p);
    Eigen::Vector2f from_grid_to_world(const Eigen::Vector2f &p);
    Eigen::Vector2f from_world_to_grid(const Eigen::Vector2f &p);
    Eigen::Matrix3f from_grid_to_robot_matrix();
    Eigen::Matrix3f from_robot_to_grid_matrix();
    Pose2D robot_pose;

    //local widget
    Custom_widget custom_widget;

    //drawing
    DSR::QScene2dViewer* widget_2d;

    //Plan
    Plan current_plan;
    DoubleBuffer<Plan, Plan> plan_buffer;
    void json_to_plan(const std::string &plan_string, Plan &plan);

    //Path planner
    enum class SearchState {NEW_TARGET, AT_TARGET, NO_TARGET_FOUND, NEW_FLOOR_TARGET};
    std::tuple<SearchState, Mat::Vector2d> search_a_feasible_target(const DSR::Node &target, const std::map<std::string, double> &params, const DSR::Node &robot);
    void path_planner_initialize(  DSR::QScene2dViewer *viewer_2d);
    std::optional<QPointF> search_a_feasible_target(Plan &current_plan);
    void run_current_plan();
    void update_grid();

    std::shared_ptr<Collisions> collisions;

    // grid
    QRectF dimensions;
    Grid grid;
    bool grid_updated = false;
    Pose2D grid_world_pose;



    float robotXWidth = 540;
    float robotZLong = 460;
    Mat::Vector3d robotBottomLeft, robotBottomRight, robotTopRight, robotTopLeft;
    void draw_path(const std::vector<Eigen::Vector2f> &path_in_robot);

    void update_map(const RoboCompLaser::TLaserData &ldata);

    RoboCompLaser::TLaserData read_laser(bool noise);
//    void draw_laser(const RoboCompLaser::TLaserData &ldata);

    bool generate_grid_to_point(const Pose2D &robot_pose);



    void inject_grid_in_G(const Grid &grid);

    void new_target_from_mouse(int pos_x, int pos_y, std::uint64_t id);

};

#endif
