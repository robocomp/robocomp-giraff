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
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <qcustomplot/qcustomplot.h>
#include <optional>
#include <Eigen/Dense>

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

	//DSR params
	std::string agent_name;
	int agent_id;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	// DSR graph viewer
//	std::unique_ptr<DSR::DSRViewer> graph_viewer;
//	QHBoxLayout mainLayout;
//	void modify_node_slot(std::uint64_t, const std::string &type){};
//	void modify_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
//	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
//
//	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
//	void del_node_slot(std::uint64_t from){};
	bool startup_check_flag;
    using Result = std::tuple<float, float, float, float, float>;
    struct Constants
    {
        const float max_advance_speed = 900;
        const float max_rotation_speed = 1;
        const float max_laser_range = 4000;
        const float robot_length = 500;
        const float robot_width = 500;
        const float laser_x_offset = 100.0;
        const float laser_y_offset = 0.0;
        const float robot_semi_width = robot_width/2;
        const float robot_semi_length = robot_length/2;
        const float final_distance_to_target = 1200; //mm
        const float step_along_arc = 200;      // advance step along arc
        const float time_ahead = 1.4;         // time ahead ahead
        const float initial_delta_rot = 0.1;
        const float MAX_RDP_DEVIATION_mm  =  600;       // in laser polygon simplification
        const float backward_speed = 250;               // mm/sg when going backwards after stuck
        const float A_dist_factor = 1;                  // weight for distance to target factor in optimun selection
        const float B_turn_factor = 10;                 // weight for previous turn factor in optimun selection
        const float C_advance_factor = 3;
    };
    Constants constants;

    // Structure laser

    struct ldata
    {
        std::vector<float> dists;
        std::vector<float> angles;
    };

    QRectF dimensions;

    //        QRectF dimensions;
    std::tuple<double, double> read_base();
    std::tuple<QPolygonF, ldata>  read_laser();
    using Point = std::pair<float, float>;  //only for RDP, change to QPointF
    QPolygonF ramer_douglas_peucker(ldata &ldata, double epsilon);
    void ramer_douglas_peucker_rec(const vector<Point> &pointList, double epsilon, std::vector<Point> &out);

    //robot
    AbstractGraphicViewer *viewer_robot;
    QGraphicsPolygonItem *robot_draw_polygon;
    QGraphicsEllipseItem *laser_draw_polygon;
    void draw_laser(const QPolygonF &ldata);
//    Eigen::Vector2f from_world_to_robot(const Eigen::Vector2f &p,  const RoboCompFullPoseEstimation::FullPoseEuler &r_state);
//    Eigen::Vector2f from_robot_to_world(const Eigen::Vector2f &p, const Eigen::Vector3f &robot);
    double global_advance, global_rotation;
//    RoboCompFullPoseEstimation::FullPoseEuler r_state_global;
    Eigen::Vector2f target_in_robot;
    QPolygonF laser_poly, left_polygon_robot, right_polygon_robot, polygon_robot;
    void move_robot(float adv, float rot);
    float gaussian(float x);

    // target
    struct Target
    {
        bool active = false;
        QPointF pos;
        Eigen::Vector2f to_eigen() const {return Eigen::Vector2f(pos.x(), pos.y());}
        QGraphicsEllipseItem *draw = nullptr;
    };
    Target target;

    //dwa
    std::optional<Result> control(const Eigen::Vector2f &target_r, const QPolygonF &laser_poly, double advance, double rot,
                                  QGraphicsScene *scene);
    std::vector<Result> compute_predictions(float current_adv, float current_rot, const QPolygonF &laser_poly);
    bool point_reachable_by_robot(const Result &point, const QPolygonF &laser_poly);
    std::optional<Result> compute_optimus(const std::vector<Result> &points, const Eigen::Vector2f &tr);
//    void draw_dwa(const Eigen::Vector3f &robot, const std::vector <Result> &puntos, const std::optional<Result> &best, QGraphicsScene *scene);
    inline QPointF to_qpointf(const Eigen::Vector2f &p) const {return QPointF(p.x(), p.y());}

    bool do_if_stuck(float adv, float rot, bool lhit, bool rhit);

//    void draw_timeseries(float rot, float adv, int lhit, int rhit, int stuck);
    void lateral_bumpers(float &rot, bool &lhit, bool &rhit);

    Eigen::Vector2f sub_target(Eigen::Vector2f &target, const QPolygonF &poly, const ldata &ldata);
};

#endif
