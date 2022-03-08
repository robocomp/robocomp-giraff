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
	//G->write_to_json_file("./"+agent_name+".json");
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
    conf_params  = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);
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
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
		//connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
		//connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

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

        // 2D widget
        widget_2d = qobject_cast<DSR::QScene2dViewer *>(graph_viewer->get_widget(opts::scene));
        if (widget_2d != nullptr)
            widget_2d->set_draw_laser(false);

        //Inner Api
        inner_eigen = G->get_inner_eigen_api();

        //grid
        QRectF outerRegion;
        auto world_node = G->get_node(world_name).value();
        outerRegion.setLeft(G->get_attrib_by_name<OuterRegionLeft_att>(world_node).value());
        outerRegion.setRight(G->get_attrib_by_name<OuterRegionRight_att>(world_node).value());
        outerRegion.setBottom(G->get_attrib_by_name<OuterRegionBottom_att>(world_node).value());
        outerRegion.setTop(G->get_attrib_by_name<OuterRegionTop_att>(world_node).value());
        if(outerRegion.isNull())
        {
            qWarning() << __FILE__ << __FUNCTION__ << "Outer region of the scene could not be found in G. Aborting";
            std::terminate();
        }

        grid.dim.setCoords(outerRegion.left(), outerRegion.top(), outerRegion.right(), outerRegion.bottom());
        //grid.TILE_SIZE = stoi(conf_params->at("tile_size").value);
        grid.TILE_SIZE = 100;
        if( auto grid_node = G->get_node(current_grid_name); grid_node.has_value())
        {
            if (auto personal_spaces_nodes = G->get_nodes_by_type(personal_space_type_name); not personal_spaces_nodes.empty())
                space_nodes_buffer.put(std::vector(personal_spaces_nodes));
        }

        // read grid from G id it exists
        if( auto grid_node = G->get_node(current_grid_name); grid_node.has_value())
        {
            if (auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(grid_node.value()); grid_as_string.has_value())
                grid_buffer.put(std::string{grid_as_string.value().get()});
        }
        qInfo() << __FUNCTION__ << "SIZE " << grid.size();
		this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    if(auto person = G->get_nodes_by_type(person_type_name); not person.empty()) {
        auto interacting_people = close_people(person);
        create_or_delete_edges(interacting_people,person);
        compute_velocity(positions,person);
        //auto future_positions= future_position(person);
    if(auto space_nodes = space_nodes_buffer.try_get();space_nodes.has_value()){
        const auto spaces= get_polylines_from_dsr(person);
        if (auto grid_node = G->get_node(current_grid_name); grid_node.has_value()){
            if (const auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(grid_node.value());grid_as_string.has_value()){
                grid.readFromString(grid_as_string.value());
                update_grid(spaces);
                //insert_polylines_in_grid(spaces);
                //inject_grid_in_G();
            }

        }
    }
    }
    //computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	
	
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

vector<tuple<int,int,bool>> SpecificWorker::close_people(vector<DSR::Node> person)
{
    vector<tuple<int,int, bool>> close_people;
    for (int i=0 ; i<person.size()-1;i++ ){
        for (int j=i+1; j<person.size();j++ ){
            bool interacting=false;
            auto person_pose_i= inner_eigen->transform(world_name, person[i].name()).value();
            float ry_i= inner_eigen->get_euler_xyz_angles(world_name,person[i].name())->y()+osg::PI;
            auto person_pose_j= inner_eigen->transform(world_name, person[j].name()).value();
            float ry_j= inner_eigen->get_euler_xyz_angles(world_name,person[j].name())->y()+osg::PI;
            Eigen::Vector2f vector_pos (person_pose_i.x()-person_pose_j.x(), person_pose_i.y()-person_pose_j.y());
            auto vector_pol =filter_interaction(vector_pos);
            cout<< vector_pol[0] << "......"<<vector_pol[1]<<endl;

            //std::cout<< dist<<"......"<<ang<<endl;
            if (vector_pol[0]<threshold) {
                if ((abs(vector_pol[1] - ry_j) < osg::PI / 5) or (abs(vector_pol[1] - ry_j) > 9 * osg::PI / 5)) {
                    if ((abs(ry_i - ry_j) > 4 * osg::PI / 5) and (abs(ry_i - ry_j) < 6 * osg::PI / 5)) {
                        interacting= true;
                    }
                }
            }
            close_people.push_back(tuple<int, int, bool >{i, j,interacting});
        }
    }
    return close_people;
}

Eigen::Vector2f SpecificWorker::filter_interaction(Eigen::Vector2f vector_pos) {
    auto ang = atan(vector_pos[1] / vector_pos[0]);
    if (vector_pos[0] < 0){
        ang +=M_PI;

    }
    else{
        if(vector_pos[1]< 0) {
            ang = 2 * M_PI + ang;
        }
    }
    ang=abs(ang-3*M_PI_2);
    if (vector_pos[0]>0 and vector_pos[1]<0){
        ang=2*osg::PI-ang;
    }
    Eigen::Vector2f vector_pol (vector_pos.norm(),ang);
    return vector_pol;
}

void SpecificWorker::create_or_delete_edges(vector<tuple<int,int,bool>> interacting_vector,vector<DSR::Node> person){
    for (const auto &interaction : interacting_vector){
        if(get<2>(interaction)){
            DSR::Edge edge_interacting = DSR::Edge::create<interacting_edge_type>(person[get<0>(interaction)].id(),person[get<1>(interaction)].id());
            G->insert_or_assign_edge(edge_interacting);
            DSR::Edge edge_interacting1 = DSR::Edge::create<interacting_edge_type>(person[get<1>(interaction)].id(),person[get<0>(interaction)].id());;
            G->insert_or_assign_edge(edge_interacting1);
        }
        else{
            G->delete_edge(person[get<0>(interaction)].id(), person[get<1>(interaction)].id(), "interacting");
            G->delete_edge(person[get<1>(interaction)].id(), person[get<0>(interaction)].id(), "interacting");
        }
    }
}

void SpecificWorker::compute_velocity(vector<QPointF> &positions,vector<DSR::Node> person) {
    if (positions.empty()){
        for (const auto &p: person){
            auto person_pose= inner_eigen->transform(world_name, p.name()).value();
            QPointF position(person_pose[0],person_pose[1]);
            positions.push_back(position);
        }
    }
    for (int i=0 ; i<person.size();i++ ) {
        auto person_pose = inner_eigen->transform(world_name, person[i].name()).value();
        QPointF position(person_pose[0], person_pose[1]);
        QPointF position_ant= positions[i];
        auto vel_x= float((position.x()-position_ant.x())/0.1);
        auto vel_y= float((position.y()-position_ant.y())/0.1);
        float vel_arr[]={vel_x,vel_y};
        vector <float> velocity(vel_arr,vel_arr+2);
        positions[i]=position;
        G->add_or_modify_attrib_local<person_velocity_att>(person[i],velocity);
        G->update_node(person[i]);
        //cout<< "Velocity "<<velocity[0]<<"  "<<velocity[1]<< endl;
    }
}

SpecificWorker::Spaces SpecificWorker::get_polylines_from_dsr(vector<DSR::Node> person) {
    Space intimate_seq, personal_seq, social_seq;

    for (auto node: person) {
        auto velocity = G->get_attrib_by_name<person_velocity_att>(node).value();
        auto pos_x=velocity[0]*t;
        auto pos_y=velocity[1]*t;
        if (auto personal_spaces= G->get_nodes_by_type(personal_space_type_name); not personal_spaces.empty()) {
            DSR::Node correct_spaces;
            cout<<personal_spaces.size()<<endl;
            for (auto personal_space:personal_spaces){
                if(G->get_attrib_by_name<person_id_att>(node).value() == G->get_attrib_by_name<person_id_att>(personal_space).value())
                    correct_spaces=personal_space;
            }
            QPolygonF intimate_pol, personal_pol, social_pol;
            auto intimate_x = G->get_attrib_by_name<ps_intimate_x_pos_att>(correct_spaces).value().get();
            auto intimate_y = G->get_attrib_by_name<ps_intimate_y_pos_att>(correct_spaces).value().get();
            for (auto &&[point_x, point_y]: iter::zip(intimate_x, intimate_y)) {
                intimate_pol.push_back(QPointF(point_x, point_y));
            }
            auto personal_x = G->get_attrib_by_name<ps_personal_x_pos_att>(correct_spaces).value().get();
            transform(personal_x.begin(),personal_x.end(),personal_x.begin(),[&pos_x](auto &c){return c+pos_x;});
            auto personal_y = G->get_attrib_by_name<ps_personal_y_pos_att>(correct_spaces).value().get();
            transform(personal_y.begin(),personal_y.end(),personal_y.begin(),[&pos_y](auto &c){return c+pos_y;});
            for (auto &&[point_x, point_y]: iter::zip(personal_x, personal_y)) {
                personal_pol.push_back(QPointF(point_x, point_y));
            }
            auto social_x = G->get_attrib_by_name<ps_social_x_pos_att>(correct_spaces).value().get();
            transform(social_x.begin(),social_x.end(),social_x.begin(),[&pos_x](auto &c){return c+pos_x;});
            auto social_y = G->get_attrib_by_name<ps_social_y_pos_att>(correct_spaces).value().get();
            transform(social_y.begin(),social_y.end(),social_y.begin(),[&pos_y](auto &c){return c+pos_y;});
            for (auto &&[point_x, point_y]: iter::zip(social_x, social_y)) {
                social_pol.push_back(QPointF(point_x, point_y));
            }
            intimate_seq.push_back(intimate_pol);
            personal_seq.push_back(personal_pol);
            social_seq.push_back(social_pol);
        }
    }
    return std::make_tuple(intimate_seq, personal_seq, social_seq);
}

void SpecificWorker::update_grid(tuple<Space,Space,Space> spaces){
    if (auto grid_node = G->get_node(current_grid_name); grid_node.has_value()){
        if (const auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(grid_node.value());grid_as_string.has_value()){
            grid.readFromString(grid_as_string.value());
            insert_polylines_in_grid(spaces);
            string grid_modify = grid.saveToString();
            G->add_or_modify_attrib_local<grid_as_string_att>(grid_node.value(), grid_modify);
            G->update_node(grid_node.value());
        }
        else
            std::cout << __FILE__ << __FUNCTION__ << " No grid node in G. Ignoring personal spaces" << std::endl;
    }
}

void SpecificWorker::insert_polylines_in_grid(const Spaces &spaces)
{
    const auto &[intimate_seq, personal_seq, social_seq] = spaces;
    grid.resetGrid();
    //To set occupied
    for (auto &&poly_intimate : iter::chain(intimate_seq))
        grid.markAreaInGridAs(poly_intimate, false);

    for (auto &&poly_per : social_seq)
        grid.modifyCostInGrid(poly_per, 10.0);

    for (auto &&poly_soc : personal_seq)
        grid.modifyCostInGrid(poly_soc, 8.0);

    if (widget_2d != nullptr)
        grid.draw(&widget_2d->scene);
}

void SpecificWorker::modify_node_slot(std::uint64_t id, const std::string &type){
    if (type == grid_type_name)  // grid
    {
        if (auto node = G->get_node(id); node.has_value())
        {
            if (const auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(node.value()); grid_as_string.has_value())
                grid_buffer.put(std::string{grid_as_string.value().get()});
        }
    }
    if (type==personal_space_type_name)
    {
        auto personal_spaces_nodes = G->get_nodes_by_type(personal_space_type_name);
        space_nodes_buffer.put(std::vector(personal_spaces_nodes));
    }
}
