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
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
		//connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
		//connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

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

        inner_eigen = G->get_inner_eigen_api();

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
        auto future_positions= future_position(person);
        //paint_gaussian(future_positions);
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
        ang +=osg::PI;

    }
    else{
        if(vector_pos[1]< 0) {
            ang = 2 * osg::PI + ang;
        }
    }
    ang=abs(ang-3*osg::PI_2);
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
    //cout<<"Llegado"<<endl;
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
        //cout<< velocity<<endl;
    }
}

vector<QPointF> SpecificWorker::future_position(vector<DSR::Node> person) {
    vector<QPointF> future_positions;
    for (const auto &p : person){
        auto velocity= G->get_attrib_by_name<person_velocity_att>(p);
        QPointF velocity_p(velocity.value()[0],velocity.value()[1]);
        auto person_pose = inner_eigen->transform(world_name, p.name()).value();
        QPointF position(person_pose[0], person_pose[1]);
        QPointF future_pos=position+velocity_p*t;
        future_positions.push_back(future_pos);
    }
return future_positions;
}

void SpecificWorker::paint_gaussian(vector<QPointF> future_positions) {
    cv::Mat imag(500,1000,CV_8UC1);
    for (const auto &p:future_positions){
        for(auto px=int(p.x())-50;px<int(p.x())+50;px++){
            for (auto py=int(p.y())-50;px<int(p.y())+50;py++){
                auto ix=(px/10)-500;
                auto iy=(py/10)-250;
                imag.at<uchar>(iy,ix)=gauss(px,py,p.x(),p.y())*255;
            }
        }
    }
    cv::imshow("",imag);
}

float SpecificWorker::gauss(int px,int py,float cx, float cy){
    return exp(pow(px-cx,2)+pow(py-cy,2));
}