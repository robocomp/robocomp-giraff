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
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
#include  "../..//etc/graph_names.h"
#include "/home/robocomp/robocomp/components/robocomp-giraff/etc/plan.h"

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
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;

    //Plan
    Plan current_plan;
    DoubleBuffer<Plan, Plan> plan_buffer;
    void modify_node_slot(std::uint64_t, const std::string &type);
	//void modify_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	//void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
	//void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag);
	void del_node_slot(std::uint64_t from);
	bool startup_check_flag;
    void chocachoca();
    std::tuple<float, float> send_command_to_robot(const std::tuple<float, float, float> &speeds);

    struct CONSTANTS
    {
        float stop_threshold = 800.0;
        float slow_threshold = 1400.0;
        float residue = 0.1;
        float min_speed = 0.0;
        float max_speed = 0.6;
        float trim = 3.0;

    };
    CONSTANTS consts;

};

#endif
