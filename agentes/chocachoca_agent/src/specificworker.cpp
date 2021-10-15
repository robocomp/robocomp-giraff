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
#include "cppitertools/sliding_window.hpp"
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
    QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	G->write_to_json_file("./"+agent_name+".json");
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);

	tree_view = params["tree_view"].value == "true";
	graph_view = params["graph_view"].value == "true";
	qscene_2d_view = params["2d_view"].value == "true";
	osg_3d_view = params["3d_view"].value == "true";

    consts.stop_threshold = stof(params.at("stop_threshold").value);
    consts.slow_threshold = stof(params.at("slow_threshold").value);
    consts.residue = stof(params.at("residue").value);
    consts.min_speed = stof(params.at("min_speed").value);
    consts.max_speed = stof(params.at("max_speed").value);
    consts.trim = stof(params.at("trim").value);

    return true;
}

void SpecificWorker::initialize(int period)
{
    std::srand(std::time(nullptr));
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//dsr update signals
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
//		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
//		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		    current_opts = current_opts | opts::tree;
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
		    current_opts = current_opts | opts::scene;
		if(osg_3d_view)
		    current_opts = current_opts | opts::osg;
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		this->Period = period;
		timer.start(Period);
	}

    qInfo() << __FUNCTION__ << "CONTROLLER: Params from config:" << consts.stop_threshold << consts.slow_threshold << consts.residue << consts.min_speed
            << consts.max_speed << consts.trim;

}

void SpecificWorker::compute()
{
    if (auto plan_o = plan_buffer.try_get(); plan_o.has_value())
    {
        current_plan = plan_o.value();
        qInfo() << __FUNCTION__ << " New plan arrived: ";
        std::cout << current_plan.pprint() << std::endl;
        if (current_plan.is_complete())
            current_plan.set_running();
    }
    if(current_plan.is_running())
    {
        chocachoca();
    }
}

////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
void SpecificWorker::chocachoca()
{
     float adv = 0.0;
     float rot = 0.0;

     float m = (consts.max_speed - consts.min_speed) * 1. / (consts.slow_threshold - consts.stop_threshold);
     float n = consts.min_speed - m * consts.stop_threshold + consts.residue;

    if (auto laser_node= G->get_node(laser_name);laser_node.has_value())
    {
        auto laser=laser_node.value();

        //Using only distance values
        auto distances = G->get_attrib_by_name<laser_dists_att>(laser).value().get();

        //Filter, try to fix laser measure errors
        if(distances[0] < 200)
            distances[0] = 200;

        for(auto &&window : iter::sliding_window(distances, 2))
        {
            if(window[1] < 200)
                window[1] = window[0];
        }

        //Sort and take the lower distance value
        int limit = distances.size()/consts.trim;
        std::sort(distances.begin() + limit, distances.end() - limit, [](float a, float b){return a < b;});
        float minValue = distances[limit];

        // Set the speeds depending on minValue, x1, x2, y1, y2
        if(minValue < consts.stop_threshold)
        {
            rot = 0.8;
            adv = consts.min_speed;
            sleep(std::rand() % 3 + 1);
        }
        else if(minValue < consts.slow_threshold)
            adv = m * minValue + n;
        else
            adv = consts.max_speed;

        try
        {
            cout << "adv: " << adv << "     rot: " << rot << endl;
            send_command_to_robot(std::make_tuple(adv*1000, rot, 0.0));
        }
        catch(const Ice::Exception &e)
            { std::cout << e.what() << std::endl;}
    }
}

std::tuple<float, float> SpecificWorker::send_command_to_robot(const std::tuple<float, float, float> &speeds) //adv, rot
{
    auto &[adv_, rot_, side_] = speeds;
    if(auto robot_node = G->get_node(robot_name); robot_node.has_value())
    {
        G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node.value(), (float) adv_);
        G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node.value(), (float) rot_);
        G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot_node.value(), (float) side_);

        G->update_node(robot_node.value());
    }
    else qWarning() << __FUNCTION__ << "No robot node found";
    return std::make_tuple(adv_, rot_);
}

///////////////////////////////////////////////////////////////////////////
///
/// ///////////////////////////////////////////////////////////////////////

void SpecificWorker::modify_node_slot(const std::uint64_t id, const std::string &type)
{
    if (type == intention_type_name)
    {
        if (auto intention = G->get_node(id); intention.has_value())
        {
            std::optional<std::string> plan = G->get_attrib_by_name<current_intention_att>(intention.value());
            if (plan.has_value())
            {
                qInfo() << __FUNCTION__ << QString::fromStdString(plan.value()) << " " << intention.value().id();
                Plan my_plan(plan.value());
                if (my_plan.is_action(Plan::Actions::BOUNCE))
                    plan_buffer.put(std::move(my_plan));
            }
        }
    }
}

void SpecificWorker::del_node_slot(const std::uint64_t id)
{
    if( auto node = G->get_node(current_intention_name); not node.has_value())
    {
        qInfo() << __FUNCTION__ << "Path node deleter. Aborting control";
        current_plan.reset();
        // PROTEGER
    }
}
