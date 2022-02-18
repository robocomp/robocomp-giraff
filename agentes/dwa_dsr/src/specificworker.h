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
#include <optional>
#include <Eigen/Dense>
#include <qcustomplot/qcustomplot.h>
#include <queue>
//#include "dynamic_window.h"
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	RoboCompMoveTowards::Command MoveTowards_move(float x, float y, float alpha);


public slots:
	void compute();
	int startup_check();
	void initialize(int period);
private:
    // Delay filter
    using Velocties = std::tuple<float, float>;
    struct delay_filter
    {
        bool first_time = true;
        std::queue<Velocties> values;
        float DELAY_PERIOD = 500;
        std::chrono::steady_clock::time_point init_time;

    public:
        std::optional<Velocties> push(const Velocties &vel)
        {
            cout << "values size: " << values.size() << endl;
            if(first_time)
            {
                init_time = std::chrono::steady_clock::now();
                first_time = false;
            }
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - init_time).count();
            cout << "init time difference: " << duration << endl;
            if (duration < DELAY_PERIOD)
            {
                values.push(vel);
            }
            else if (values.size() > 0)
            {
                auto v = values.front();
                values.pop();
                return v;
            }
            else return vel;
            return {};
        };
    };

    using Result = std::tuple<float, float, float, float, float>;
    struct Constants
    {
        const float max_advance_speed = 1000;
        const float max_rotation_speed = 1;
        const float max_laser_range = 4000;
        const float robot_length = 800;
        const float robot_width = 800   ;
        const float laser_x_offset = 0.0;
        const float laser_y_offset = 200;
        const float robot_semi_width = robot_width/2;
        const float robot_semi_length = robot_length/2;
        const float final_distance_to_target = 1000; //mm
        const float step_along_arc = 200;      // advance step along arc
        const float time_ahead = 1.4;         // time ahead ahead
        const float initial_delta_rot = 0.1;
        const float MAX_RDP_DEVIATION_mm  =  600;       // in laser polygon simplification
        const float backward_speed = 200;               // mm/sg when going backwards after stuck
        const float A_dist_factor = 1;                  // weight for distance to target factor in optimun selection
        const float B_turn_factor = 10;                 // weight for previous turn factor in optimun selection
    };
    Constants constants;

    QRectF dimensions;
    bool startup_check_flag;
    std::tuple<RoboCompFullPoseEstimation::FullPoseEuler, double, double> read_base_real();
    std::tuple<RoboCompFullPoseEstimation::FullPoseEuler, double, double> read_base_coppelia();
    QPolygonF read_laser_real();
    QPolygonF read_laser_coppelia();

    bool real_arrived, coppelia_arrived;

    using Point = std::pair<float, float>;  //only for RDP, change to QPointF
    QPolygonF ramer_douglas_peucker(RoboCompLaser::TLaserData &ldata, double epsilon);
    void ramer_douglas_peucker_rec(const vector<Point> &pointList, double epsilon, std::vector<Point> &out);
    //Dynamic_Window dwa;

    //robot
    AbstractGraphicViewer *viewer_robot;
    QGraphicsPolygonItem *robot_draw_polygon_real;
    QGraphicsPolygonItem *robot_draw_polygon_coppelia;
    QGraphicsEllipseItem *laser_draw_polygon_real;
    QGraphicsEllipseItem *laser_draw_polygon_coppelia;
    void draw_laser_real(const QPolygonF &ldata);
    void draw_laser_coppelia(const QPolygonF &ldata);
    Eigen::Vector2f from_world_to_robot(const Eigen::Vector2f &p,  const RoboCompFullPoseEstimation::FullPoseEuler &r_state);
    Eigen::Vector2f from_robot_to_world(const Eigen::Vector2f &p, const Eigen::Vector3f &robot);
    double global_advance, global_rotation;
    RoboCompFullPoseEstimation::FullPoseEuler r_state_global;
    Eigen::Vector2f target_in_robot_real, target_in_robot_coppelia;
    QPolygonF laser_poly_real, laser_poly_coppelia, left_polygon_robot_real, right_polygon_robot_real, polygon_robot_real, left_polygon_robot_coppelia, right_polygon_robot_coppelia, polygon_robot_coppelia;
    void move_robot_real(float adv, float rot);
    float gaussian(float x);
    RoboCompFullPoseEstimation::FullPoseEuler r_state_real_last;
    float delta_rot_l = constants.initial_delta_rot;
    float delta_rot_r = constants.initial_delta_rot;
    bool lhit_real = false, rhit_real= false, lhit_coppelia = false, rhit_coppelia= false,stuck_real = false, stuck_coppelia = false;
    float period_value = 0.1;

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
                                  const Eigen::Vector3f &robot, QGraphicsScene *scene);
    std::vector<Result> compute_predictions(float current_adv, float current_rot, const QPolygonF &laser_poly);
    bool point_reachable_by_robot(const Result &point, const QPolygonF &laser_poly);
    std::optional<Result> compute_optimus(const std::vector<Result> &points, const Eigen::Vector2f &tr);
    void draw_dwa(const Eigen::Vector3f &robot, const std::vector <Result> &puntos, const std::optional<Result> &best, QGraphicsScene *scene);
    inline QPointF to_qpointf(const Eigen::Vector2f &p) const {return QPointF(p.x(), p.y());}
    void removeArcs(QGraphicsScene *scene);
    bool do_if_stuck_real(float adv, float rot, const RoboCompFullPoseEstimation::FullPoseEuler &r_state, bool lhit, bool rhit);

    // QCustomPlot
    QCustomPlot custom_plot;
    QCPGraph *rot_graph, *adv_graph, *lhit_graph, *rhit_graph, * stuck_graph, *diff_graph;

    void draw_timeseries(float rot, float adv, int lhit, int rhit, int stuck, float diff);
    std::unique_ptr<DSR::RT_API> rt;
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
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;
	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};

	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};

};

#endif
