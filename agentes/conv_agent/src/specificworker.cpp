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

string actual_ready_person;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
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
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }





	agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);

	tree_view = params["tree_view"].value == "true";
	graph_view = params["graph_view"].value == "true";
	qscene_2d_view = params["2d_view"].value == "true";
	osg_3d_view = params["3d_view"].value == "true";

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//dsr update signals
//		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
//		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
//		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		{
		    current_opts = current_opts | opts::tree;
		}
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
		{
		    current_opts = current_opts | opts::scene;
		}
		if(osg_3d_view)
		{
		    current_opts = current_opts | opts::osg;
		}
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		this->Period = period;
		timer.start(Period);
	}
}

/////////////////////////////////// COMPUTE ///////////////////////////////////

void SpecificWorker::compute()
{
	auto robot_node = G->get_node(robot_name);
	auto people_nodes = G->get_nodes_by_type("person");
	for(int i=0;i<people_nodes.size();i++)
	{
        if(auto is_ready = G->get_attrib_by_name<is_ready_att>(people_nodes[i]); is_ready.has_value())
        {
            if(is_ready.value() == true)
            {
                ready_person = people_nodes[i];
                if(auto person_name = G->get_attrib_by_name<person_name_att>(ready_person); person_name.has_value()) name= person_name.value();
                if(auto person_role = G->get_attrib_by_name<person_role_att>(ready_person); person_role.has_value()) role= person_role.value();
                if(auto person_age = G->get_attrib_by_name<person_age_att>(ready_person); person_age.has_value()) age = person_age.value();

                this->conversation_proxy->listenToHuman();
            }
        }
        if(auto lost_edge = G->get_edge(robot_node.value().id(), people_nodes[i].id(), "lost"); lost_edge.has_value())
        {
            this->conversation_proxy->lost(name, "person");
            isLost = true;
        }
	}
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::modify_node_slot(const std::uint64_t id, const std::string &type)
{
//    if (type == intention_type_name)
//    {
//        if (auto intention = G->get_node(current_robot_intention_name); intention.has_value())
//        {
//            std::optional<std::string> plan = G->get_attrib_by_name<current_intention_att>(intention.value());
//            if (plan.has_value())
//            {
//                qInfo() << __FUNCTION__ << QString::fromStdString(plan.value()) << " " << intention.value().id();
//                Plan my_plan(plan.value());
//                if (my_plan.is_action(Plan::Actions::ASK_FOR_PERMISSION))
//				{
//                    cout << "Plan cambiado a pedir permiso" << endl;
//					this->conversation_proxy->isBlocked(true);
//					isBlocked = true;
//				}
//            }
//        }
//    }
}

// void SpecificWorker::modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type)
// {
//     if (type == intention_type_name)
//     {
//         if (auto intention = G->get_node(id); intention.has_value())
//         {
//             std::optional<std::string> plan = G->get_attrib_by_name<current_intention_att>(intention.value());
//             if (plan.has_value())
//             {
//                 qInfo() << __FUNCTION__ << QString::fromStdString(plan.value()) << " " << intention.value().id();
//                 Plan my_plan(plan.value());
//                 if (my_plan.is_action(Plan::Actions::ASK_FOR_PERMISSION))
//                     cout << "Plan cambiado a pedir permiso" << endl;
// 				else if (my_plan.is_action(Plan::Actions::ASK_FOR_TALKING))
//                     cout << "Plan cambiado a hablar" << endl;
// 				else if (my_plan.is_action(Plan::Actions::ASK_FOR_FOLLOWING))
//                     cout << "Plan cambiado a seguir" << endl;
//             }
//         }
//     }
// }

void SpecificWorker::del_node_slot(const std::uint64_t id)
{
//    if( auto node = G->get_node(current_person_intention_name); not node.has_value())
//    {
//		this->conversation_proxy->isTalking(false);
//		this->conversation_proxy->isBlocked(false);
//		this->conversation_proxy->isFollowing(false);
//
//        // current_plan.reset();
//    }
}

void SpecificWorker::remove_intention_edge(string action)
{
    if(auto robot = G->get_node(robot_name); robot.has_value())
    {
		if(auto person = G->get_node(person_name); person.has_value())
		{
			if(auto edge = G->get_edge(robot_name, person_name, action); edge.has_value())
			{
				G->delete_edge(robot.value().id(), person.value().id(),action);
			}
			else qWarning() << __FUNCTION__ << "No action edge found";       	
		}
    }
}

void SpecificWorker::remove_intention_node()
{
    // // Check if there is not 'intention' node yet in G
    // if(auto mind = G->get_node(person_mind_name); mind.has_value())
    // {
    //     if (auto intention = G->get_node(current_person_intention_name); intention.has_value())
    //     {

    //     }
    //     else // there is one intention node
    //     {
    //         std::cout << __FUNCTION__ << ": Updating existing intention node with Id: " << intention.value().id() << std::endl;
    //         // G->add_or_modify_attrib_local<current_intention_att>(intention.value(), plan.to_json());
    //         G->update_node(intention.value());
    //         std::cout << "INSERT: " << plan.to_json() << std::endl;
    //     }
    // }
    // else
    // {
    //     std::cout  << __FUNCTION__ << " No node " << robot_mind_name << " found in G" << std::endl;
    //     std::terminate();
    // }
}

void SpecificWorker::insert_intention_node(const Plan &plan)
{
//	auto person_node = G->get_node(actual_ready_person).value();
//	auto person_id = G->get_attrib_by_name<person_id_att>(person_node).value();
//	auto person_id_str = std::to_string(person_id);
//	string downbar = "_";
//	string person_mind_name_str = person_mind_name + downbar + person_id_str;
//	cout << person_mind_name_str << endl;
//
//	string person_intention = current_person_intention_name + downbar + person_id_str;
//
//    if(auto mind = G->get_node(person_mind_name_str); mind.has_value())
//    {
//
//        if (auto intention = G->get_node(person_intention); not intention.has_value())
//        {
//            DSR::Node intention_node = DSR::Node::create<intention_node_type>(person_intention);
//            G->add_or_modify_attrib_local<parent_att>(intention_node, mind.value().id());
//            // G->add_or_modify_attrib_local<level_att>(intention_node, G->get_node_level(mind.value()).value() + 1);
//            G->add_or_modify_attrib_local<pos_x_att>(intention_node, (float) 304.05);
//            G->add_or_modify_attrib_local<pos_y_att>(intention_node, (float) -130.92);
//            G->add_or_modify_attrib_local<current_intention_att>(intention_node, plan.to_json());
//            if (std::optional<int> intention_node_id = G->insert_node(intention_node); intention_node_id.has_value())
//            {
//                DSR::Edge edge = DSR::Edge::create<has_edge_type>(mind.value().id(), intention_node.id());
//                if (G->insert_or_assign_edge(edge))
//                {
//                    std::cout << __FUNCTION__ << " Edge successfully inserted: " << mind.value().id() << "->" << intention_node.id()
//                              << " type: has" << std::endl;
//                    G->add_or_modify_attrib_local<current_intention_att>(intention_node, plan.to_json());
//                    G->update_node(intention_node);
//                }
//                else
//                {
//                    std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << mind.value().id() << "->" << intention_node_id.value()
//                              << " type: has" << std::endl;
//                    std::terminate();
//                }
//            } else
//            {
//                std::cout << __FUNCTION__ << ": Fatal error inserting_new 'intention' node" << std::endl;
//                std::terminate();
//            }
//        }
//        else // there is one intention node
//        {
//            std::cout << __FUNCTION__ << ": Updating existing intention node with Id: " << intention.value().id() << std::endl;
//            G->add_or_modify_attrib_local<current_intention_att>(intention.value(), plan.to_json());
//            G->update_node(intention.value());
//            std::cout << "INSERT: " << plan.to_json() << std::endl;
//        }
//    }
//    else
//    {
//        std::cout  << __FUNCTION__ << " No node " << robot_mind_name << " found in G" << std::endl;
//        std::terminate();
//    }
}

void SpecificWorker::insert_action_edge(int action)
{
    if(auto robot = G->get_node(robot_name); robot.has_value())
    {
        switch(action)
        {
            case 0:
                DSR::Edge following_edge = DSR::Edge::create<following_edge_type>(robot.value().id(), ready_person.id());
                if (G->insert_or_assign_edge(following_edge))
                {
                    std::cout << __FUNCTION__ << " Edge successfully inserted: " << robot.value().id() << "->" << ready_person.id()
                              << " type: following" << std::endl;
                }
                break;
        }


				// case 1:
				// 	DSR::Edge talking_edge = DSR::Edge::create<talking_edge_type>(robot.value().id(), person.value().id());
				// 	if (G->insert_or_assign_edge(talking_edge))
				// 	{
				// 		std::cout << __FUNCTION__ << " Edge successfully inserted: " << robot.value().id() << "->" << person.value().id()
				// 				<< " type: talking" << std::endl;
				// 	}
				// 	break;

			// if(auto intention = G->get_node(current_person_intention_name); intention.has_value())
    		// {
			// 	std::optional<std::string> plan = G->get_attrib_by_name<current_intention_att>(intention.value());
			// 	if (plan.has_value())
			// 	{
			// 		if(auto mind = G->get_node(person_mind_name); mind.has_value())
			// 		{
			// 			G->delete_edge(robot.value().id(), person.value().id(),"ready");
			// 		}

			// 		Plan my_plan(plan.value());

			// 		if (my_plan.is_action(Plan::Actions::ASK_FOR_TALKING))
			// 		{
			// 			DSR::Edge edge = DSR::Edge::create<talking_edge_type>(robot.value().id(), person.value().id());
			// 			if (G->insert_or_assign_edge(edge))
			// 			{
			// 				std::cout << __FUNCTION__ << " Edge successfully inserted: " << robot.value().id() << "->" << person.value().id()
			// 						<< " type: talking" << std::endl;
			// 			// G->update_node(intention_node);
			// 			}
			// 		}
			// 		else if (my_plan.is_action(Plan::Actions::ASK_FOR_FOLLOWING))
			// 		{
			// 			DSR::Edge edge = DSR::Edge::create<following_edge_type>(robot.value().id(), person.value().id());	
			// 			if (G->insert_or_assign_edge(edge))
			// 			{
			// 				std::cout << __FUNCTION__ << " Edge successfully inserted: " << robot.value().id() << "->" << person.value().id()
			// 						<< " type: following" << std::endl;
			// 			}
			// 		}
			// 	}
			// }

	} 
}

void SpecificWorker::AgenteConversacional_componentState(int state)
{
	switch(state)
	{
		case 0:
			isTalking = false;
			remove_intention_edge("talking");
	}
}

void SpecificWorker::AgenteConversacional_asynchronousIntentionReceiver(int intention)
{
    if(auto robot_node = G->get_node("robot"); robot_node.has_value())
    {
        auto robot_node_value = robot_node.value();
        switch (intention)
        {
            case 0:
                isListening = false;
                isFollowing = true;
                cout << "Seguir" << endl;
                // temporary_plan.new_plan(Plan::Actions::ASK_FOR_FOLLOWING);
                // insert_intention_node(temporary_plan);
                insert_action_edge(0);
                this->conversation_proxy->talking(name, role,"seguir");
                break;

            case 1:
                cout << "Hablar" << endl;
                isTalking = true;
                isListening = false;
                // temporary_plan.new_plan(Plan::Actions::ASK_FOR_TALKING);
                // insert_intention_node(temporary_plan);
                insert_action_edge(1);
                this->conversation_proxy->talking(name, role, "hablar");
                break;
            case 2:
                cout << "Dejar de seguir" << endl;
                // insert_intention_node(temporary_plan);
                if(auto follow_edge = G->get_edge(robot_node_value.id(), ready_person.id(), "following"); follow_edge.has_value())
                {
                    G->delete_edge(robot_node.value().id(), ready_person.id(), "following");
                }
                this->conversation_proxy->talking(name, role, "no_seguir");
                break;
            case 3:
                cout << "Identificado" << endl;
                G->add_or_modify_attrib_local<robot_rotation_to_person_att>(robot_node_value, true);
                G->update_node(robot_node_value);

            case -1:
                cout << "No identifica keyword" << endl;
                isListening = false;
                this->conversation_proxy->talking(name, role, "preguntar");
                break;
            case -99:
                this->conversation_proxy->talking(name, role, "no identifica");
                isListening = false;
                break;
        }
    }

}


