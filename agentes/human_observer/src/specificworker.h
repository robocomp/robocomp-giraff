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
#include "../../../etc/graph_names.h"
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
#include <QPolygonF>
#include <QPointF>
#include "grid.h"
#include <cppitertools/chain.hpp>
#include <cppitertools/zip.hpp>

class SpecificWorker : public GenericWorker
{
    using Myclock = std::chrono::system_clock;
    using Msec = std::chrono::duration<double, std::milli>;
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
	void del_node_slot(std::uint64_t from){};     
	bool startup_check_flag;

    //drawing
    DSR::QScene2dViewer* widget_2d;

    // Grid
    Grid grid;
    //bool grid_initialized = false;
    //bool personal_spaces_changed = false;
    std::shared_ptr<Collisions> collisions;

    std::shared_ptr<RoboCompCommonBehavior::ParameterList> conf_params;

    DoubleBuffer<std::string, std::string> grid_buffer;
    DoubleBuffer<std::vector<DSR::Node>, std::vector<DSR::Node>> space_nodes_buffer;

    using Space = std::vector<QPolygonF>;
    using Spaces = std::tuple<Space, Space, Space>;

    vector<tuple<int,int,bool>> close_people(vector<DSR::Node> person);
    Eigen::Vector2f filter_interaction(Eigen::Vector2f vector_pos);
    void create_or_delete_edges (vector<tuple<int,int,bool>>,vector<DSR::Node> person);
    void compute_velocity(vector<QPointF> &position,vector<DSR::Node> person);
    Spaces get_polylines_from_dsr(vector<DSR::Node> person);
    void update_grid(tuple<Space,Space,Space> Spaces);
    void insert_polylines_in_grid(const Spaces &spaces);

    float threshold=1500;
    int t=1;
    vector<QPointF> positions;

};

#endif
