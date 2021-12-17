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

int max_lambda_value = 25;
int min_lambda_value = -25;
int hits_to_reach_top_thr = 20;
int min_insert_dist_thr = 10000;
float top_thr = 0.7;
float bot_thr = 0.3;
cv::RNG rng(12345);
cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
auto s = -hits_to_reach_top_thr/(log(1/top_thr-1));
auto integrator = [](auto x){return 1/(1 + exp(x/s));};
int person_name_idx = 0;

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
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
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
        rt_api = G->get_rt_api();
		this->Period = period;
		timer.start(Period);
	}

}

QVector3D SpecificWorker::get_person_coords(RoboCompHumanCameraBody::Person p)
{
    RoboCompHumanCameraBody::TJoints person_tjoints = p.joints;
    list<RoboCompHumanCameraBody::KeyPoint> huesitos;
    for(auto item : person_tjoints)
    {
        std::string key = item.first;

        if (key.compare("17") || key.compare("6") || key.compare("5") || key.compare("12") || key.compare("11") || key.compare("0") || key.compare("1") || key.compare("2") || key.compare("3") || key.compare("4"))
        {
            huesitos.push_back(item.second);
        }
    }

    if(huesitos.empty())
    {
        auto kp = person_tjoints.begin()->second;
        QVector3D point(kp.x,kp.y,kp.z);
        return point;
    }
    else
    {
        float avg_x = 0, avg_y = 0,avg_z=0;
        for(auto kp: huesitos)
        {
            avg_x += kp.x;
            avg_y += kp.y;
            avg_z += kp.z;
        }
        avg_x = avg_x/huesitos.size();
        avg_y = avg_y/huesitos.size();
        avg_z = avg_z/huesitos.size();
        QVector3D point(avg_x,avg_y,avg_z);
        return point;
    }
}

void SpecificWorker::compute()
{
    // Creating white image with dimension 480x640

    cv::Mat black_picture = cv::Mat::zeros(640, 480, CV_8UC3);

    // Taking people data through proxy
    RoboCompHumanCameraBody::PeopleData people_data = this->humancamerabody_proxy->newPeopleData();
    RoboCompHumanCameraBody::People people_list = people_data.peoplelist;

    // Sacamos las coords de cada persona
    for(auto p: people_list){
        QVector3D person_coords = get_person_coords(p);
        cout << "Person " << p.id << ": (" << person_coords.x() << ","<< person_coords.y()
             << "," << person_coords.z() << ")" << endl;

        auto orientation = calculate_orientation(p);
        cout << "ORIENTATION: "<<orientation << endl;
    }

    // Generating camera image
/*    RoboCompCameraRGBDSimple::TImage image = this->camerargbdsimple_proxy->getImage("123456789");
    cv::Mat frame(cv::Size(image.height, image.width), CV_8UC3, &image.image[0], cv::Mat::AUTO_STEP);

    // Vector to include people data with ne new struct (world position and orientation added)
    vector<SpecificWorker::PersonData> person_data_vector;

    // Iterating each person
    for(int i=0;i<people_list.size();i++)
    {
        RoboCompHumanCameraBody::Person person = people_list[i];
        // cout << "Person ID: " << person.id << endl;
        RoboCompHumanCameraBody::TJoints person_tjoints = person.joints;
        RoboCompHumanCameraBody::TJoints::iterator itr;
        std::vector<cv::Point> pixel_vector;
        std::vector<float> depth_vector_x, depth_vector_y, depth_vector_z;

        // Iterating TJoints
        for(auto item : person_tjoints)
        {
            // Appending joint pixels to a vector
            int screen_x_point = item.second.i;
            int screen_y_point = item.second.j;

            cv::Point pixel;
            pixel.x = screen_x_point;
            pixel.y = screen_y_point;
            pixel_vector.push_back(pixel);

            float depth_x = item.second.x; depth_vector_x.push_back(depth_x); // x position respecting the world
            float depth_y = item.second.y; depth_vector_y.push_back(depth_y); // y position respecting the world
            float depth_z = item.second.z; depth_vector_z.push_back(depth_z); // z position respecting the world
        }

        // Creating person with the new stucture
        SpecificWorker::PersonData person_data;
        person_data.id = person.id;
        person_data.joints = person.joints;


        float pos_x = std::accumulate(depth_vector_x.begin(), depth_vector_x.end(), decltype(depth_vector_x)::value_type(0)) / depth_vector_x.size();
        float pos_y = std::accumulate(depth_vector_y.begin(), depth_vector_y.end(), decltype(depth_vector_y)::value_type(0)) / depth_vector_y.size();
        float pos_z = std::accumulate(depth_vector_z.begin(), depth_vector_z.end(), decltype(depth_vector_z)::value_type(0)) / depth_vector_z.size();

        person_data.personCoords.push_back(pos_x); person_data.personCoords.push_back(pos_y); person_data.personCoords.push_back(pos_z);
        person_data.orientation = calculate_orientation(person);
        person_data_vector.push_back(person_data);

        cv::Rect person_box = cv::boundingRect(pixel_vector);
        cv::rectangle(frame, cv::Rect(person_box.x, person_box.y, person_box.width, person_box.height), color, 2);
        for(int k=0;k<pixel_vector.size();k++)
        {
            cv::circle(frame, pixel_vector[k],12,color);
        }

    }
    update_graph(person_data_vector);
    cv::imshow("RGB image", frame);
    //cv::waitKey(1);*/

    cout << "================================================================="<<endl;
}

std::int32_t SpecificWorker::increase_lambda_cont(std::int32_t lambda_cont)
// Increases lambda_cont in 1, to the maximum value and returns the new value. Returns the new lambda_cont value.
{
    std::int32_t nlc = lambda_cont + 1;
    if(nlc < max_lambda_value) {return nlc;}
    else {return max_lambda_value;}
}

std::int32_t SpecificWorker::decrease_lambda_cont(std::int32_t lambda_cont)
// Decreases lambda_cont in 1, to the minimun value and returns the new value. Returns the new lambda_cont value.
{
    std::int32_t nlc = lambda_cont - 1;
    if(nlc > min_lambda_value) {return nlc;}
    else {return min_lambda_value;}
}

float SpecificWorker::distance_3d(cv::Point3f p1, cv::Point3f p2)
{
    return cv::norm(p1-p2);
}

cv::Point3f SpecificWorker::dictionary_values_to_3d_point(auto item)
{
    cv::Point3f point;
    float x = item.x;
    float y = item.y;
    float z = item.z;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

cv::Point3f SpecificWorker::cross_product(cv::Point3f p1, cv::Point3f p2)
{
    cv::Point3f point;
    point.x = p1.y * p2.z - p1.z * p2.y;
    point.y = p1.z * p2.x - p1.x * p2.z;
    point.z = p1.x * p2.y - p1.y * p2.x;
    return point;
}

//function to calculate dot product of two vectors
float SpecificWorker::dot_product3D(cv::Point3f vector_a, cv::Point3f vector_b) {
    float product = 0;

    product = product + vector_a.x * vector_b.x;
    product = product + vector_a.y * vector_b.y;
    product = product + vector_a.z * vector_b.z;

    return product;

}

float SpecificWorker::dot_product(cv::Point2f vector_a, cv::Point2f vector_b) {
    float product = 0;

    product = product + vector_a.x * vector_b.x;
    product = product + vector_a.y * vector_b.y;

    return product;

}

float SpecificWorker::get_degrees_between_vectors(cv::Point2f vector_1, cv::Point2f vector_2, std::string format)
{
    // Returns the angle between two vectors in the 2d plane (v2 respect v1)

    if (format.compare("radians") == 0 && format.compare("degrees") == 0)
    {
        cout << "Invalid angle format. Format parameter should be \"radians\" or \"degrees\"" << endl;
        return 0.0;
    }

    // Getting unitary vectors
    cv::Point2f u_vector_1 = vector_1/cv::norm(vector_1);
    cout << "u_vector_1: ("<< u_vector_1.x << ","<< u_vector_1.y << ")" << endl;
    cv::Point2f u_vector_2 = vector_2/cv::norm(vector_2);
    cout << "vector_2: ("<< vector_2.x << ","<< vector_2.y << ")" << endl;
    cout << "NORM: "<< cv::norm(vector_2) << endl;
    cout << "u_vector_2: ("<< u_vector_2.x << ","<< u_vector_2.y << ")" << endl;

    // Extra vector: u_vector_2 rotated /90 degrees
    cv::Point2f u_vector_2_90;
    u_vector_2_90.x = cos(-M_PI / 2) * u_vector_2.x - sin(-M_PI / 2) * u_vector_2.y;
    u_vector_2_90.y = sin(-M_PI / 2) * u_vector_2.x + cos(-M_PI / 2) * u_vector_2.y;

    cout << "u_vector_2_90: ("<< u_vector_2_90.x << ","<< u_vector_2_90.y << ")" << endl;
    // Dot product of u_vector_1 with u_vector_2 and u_vector_2_90
    //float dp = u_vector_1.x * u_vector_2.x + u_vector_1.y * u_vector_2.y;
    //float dp_90 = u_vector_1.x * u_vector_2_90.x + u_vector_1.y * u_vector_2_90.y;

    // Dot product of u_vector_1 with u_vector_2 and u_vector_2_90
    float dp = dot_product(u_vector_1, u_vector_2);
    cout << "DP: " << dp << endl;
    float dp_90 = dot_product(u_vector_1, u_vector_2_90);
    cout << "DP_90: " << dp_90 << endl;

    // Comprobating if the angle is over 180 degrees and adapting
    float ret;
    if(dp_90 < 0){ret = M_PI + (M_PI-acos(dp));}
    else{ret = acos(dp);}

    cout << "RET: " << ret << endl;

    // Returning value
    if (format.compare("radians") == 0) {return ret;}
    else {return (ret*180/M_PI);}
}

float SpecificWorker::calculate_orientation(RoboCompHumanCameraBody::Person person)
{
    RoboCompHumanCameraBody::TJoints person_tjoints = person.joints;
    bool left_found, base_found, right_found = false;
    cv::Point3f base_p, right_p, left_p;

    for(auto item : person_tjoints)
    {
        std::string key = item.first;

        // Base point

        if (base_found == false && (key.compare("17") || key.compare("6") || key.compare("5") || key.compare("2") || key.compare("1")))
        {
            base_found = true;
            base_p = dictionary_values_to_3d_point(item.second);
        }

        // Right point

        if (right_found == false && (key.compare("12") || key.compare("4")))
        {
            right_found = true;
            right_p = dictionary_values_to_3d_point(item.second);
        }

        // Left point

        if (left_found == false && (key.compare("11") || key.compare("3")))
        {
            left_found = true;
            left_p = dictionary_values_to_3d_point(item.second);
        }

        if(base_found == true && right_found == true && left_found == true)
        {
            break;
        }
    }

    if(base_found == false || right_found == false || left_found == false)
    {
        cout << "Points not found. Can't calculate orientation." << endl;
        return 0.0;
    }

    // Considering "clavícula" as coordinate center. Passing leg points to "clavícula" reference system

    cv::Point3f left_v = left_p - base_p;
    cv::Point3f right_v = right_p - base_p;
    cout << "BASE_P: (" << base_p.x << ","<<base_p.y << ","<<base_p.z << ")" << endl;
    cout << "left_p: (" << left_p.x << ","<<left_p.y << ","<<left_p.z << ")" << endl;
    cout << "right_p: (" << right_p.x << ","<<right_p.y << ","<<right_p.z << ")" << endl;
    cout << "left_v: (" << left_v.x << ","<<left_v.y << ","<<left_v.z << ")" << endl;
    cout << "right_v: (" << right_v.x << ","<<right_v.y << ","<<right_v.z << ")" << endl;

    // Calculating perpendicular vector

    cv::Point3f normal = cross_product(left_v, right_v);
    cv::Point2f vector_1, vector_2;
    vector_1.x = 0;
    vector_1.y = 1;
    vector_2.x = normal.x;
    vector_2.y = normal.z;

    cout << "vector_2: (" << vector_2.x << ","<<vector_2.y << ")" << endl;

    float angle = get_degrees_between_vectors(vector_1, vector_2, "radians");
    cout << "Ángulo: " << angle << endl;
    return angle;
}

void SpecificWorker::remove_person(DSR::Node person_node, bool direct_remove)
{
    float score = 0;
    if(direct_remove == true){int score = 0;}
    else
    {
        auto person_lc = G->get_attrib_by_name<lambda_cont_att>(person_node).value();
        int nlc = decrease_lambda_cont(person_lc);
        G->add_or_modify_attrib_local<is_ready_att>(person_node, false);
        G->add_or_modify_attrib_local<lambda_cont_att>(person_node, nlc);
        G->update_node(person_node);
        auto person_lc_new = G->get_attrib_by_name<lambda_cont_att>(person_node).value();
        score = integrator(person_lc_new);
    }
    if((score <= bot_thr) or (direct_remove == true))
    {
        auto people_space_nodes = G->get_nodes_by_type("personal_space");
        auto mind_nodes = G->get_nodes_by_type("transform");
        auto person_id = G->get_attrib_by_name<person_id_att>(person_node).value();
        auto parent_id = person_node.id();
        G->delete_node(person_node.id());
        for(int i=0; i<people_space_nodes.size();i++)
        {
            auto act_space_node = people_space_nodes[i];
            auto act_space_node_person_id = G->get_attrib_by_name<person_id_att>(act_space_node).value();
            if(act_space_node_person_id == person_id);
            {
                G->delete_node(act_space_node.id());
                break;
            }
        }
        for(int i=0; i<mind_nodes.size();i++)
        {
            auto act_mind_node = mind_nodes[i];
            auto act_mind_node_parent = G->get_attrib_by_name<parent_att>(act_mind_node).value();
            if(act_mind_node_parent == parent_id);
            {
                G->delete_node(act_mind_node.id());
                break;
            }
        }
    }
}

void SpecificWorker::update_person(DSR::Node node, std::vector<float> coords, float orientation)
{
    float score = 0;
    auto world_node = G->get_node('world');
    G->add_or_modify_attrib_local<distance_to_robot_att>(node, coords[2]);
    std::vector<float> new_position_vector = {coords[0], coords[2], coords[1]};
    std::vector<float> orientation_vector = {0.0, orientation, 0.0};
    try
    {
        DSR::Edge edge = DSR::Edge::create<RT_edge_type>(world_node.value().id(), node.id());
        G->add_attrib_local<rt_rotation_euler_xyz_att>(edge, orientation_vector);
        G->add_attrib_local<rt_translation_att>(edge, new_position_vector);
        G->insert_or_assign_edge(edge);
        auto person_lc = G->get_attrib_by_name<lambda_cont_att>(node).value();
        int nlc = increase_lambda_cont(person_lc);
        auto person_lc_new = G->get_attrib_by_name<lambda_cont_att>(node).value();
        auto person_distance = G->get_attrib_by_name<distance_to_robot_att>(node).value();
        score = integrator(person_lc_new);
        if(score >= top_thr && person_distance < 2000 && person_lc_new == 25)
        {
            G->add_or_modify_attrib_local<is_ready_att>(node, true);
        }
        else
        {
            G->add_or_modify_attrib_local<is_ready_att>(node, false);
        }
        G->update_node(node);
    }
    catch(int e)
    {
        cout << "Problema" << endl;
    }
}

void SpecificWorker::insert_mind(std::uint64_t parent_id, std::int32_t person_id)
{
    auto parent_node = G->get_node(parent_id).value();

    float pos_x = rand()%(400-250 + 1) + 250;
    float pos_y = rand()%(170-(-30) + 1) + (-30);
    std::string person_id_str = std::to_string(person_id);
    std::string node_name = "person_mind_" + person_id_str;
    DSR::Node new_node = DSR::Node::create<transform_node_type>(node_name);
    G->add_or_modify_attrib_local<person_id_att>(new_node, person_id);
    G->add_or_modify_attrib_local<parent_att>(new_node, parent_id);
    G->add_or_modify_attrib_local<pos_x_att>(new_node, pos_x);
    G->add_or_modify_attrib_local<pos_y_att>(new_node, pos_y);

    try
    {
        G->insert_node(new_node);
        DSR::Edge edge = DSR::Edge::create<has_edge_type>(new_node.id(), parent_node.id());
        G->insert_or_assign_edge(edge);
    }
    catch(int e)
    {
        cout << "Problema" << endl;
    }
}

void SpecificWorker::insert_person(std::vector<float> coords, float orientation, bool direct_insert)
{
    auto world_node = G->get_node('world').value();
    float pos_x = rand()%(120-(-100) + 1) + (-100);
    float pos_y = rand()%(-100-(-370) + 1) + (-370);
    int id = person_name_idx;
    person_name_idx += 1;
    std::string person_id_str = std::to_string(id);
    std::string node_name = "person_" + person_id_str;

    DSR::Node new_node = DSR::Node::create<transform_node_type>(node_name);
    G->add_or_modify_attrib_local<person_id_att>(new_node, id);
    G->add_or_modify_attrib_local<is_ready_att>(new_node, direct_insert);

    int lc = 0;
    if(direct_insert == true){lc = hits_to_reach_top_thr;}
    else{lc = 1;}

    G->add_or_modify_attrib_local<lambda_cont_att>(new_node, lc);
    G->add_or_modify_attrib_local<distance_to_robot_att>(new_node, coords[2]);
    G->add_or_modify_attrib_local<pos_x_att>(new_node, pos_x);
    G->add_or_modify_attrib_local<pos_y_att>(new_node, pos_y);
    try
    {
        auto id_result = G->insert_node(new_node);
        DSR::Edge edge = DSR::Edge::create<has_edge_type>(world_node.id(), new_node.id());
        std::vector<float> new_position_vector = {coords[0], coords[2], coords[1]};
        std::vector<float> orientation_vector = {0.0, orientation, 0.0};
        G->add_attrib_local<rt_rotation_euler_xyz_att>(edge, orientation_vector);
        G->add_attrib_local<rt_translation_att>(edge, new_position_vector);
        G->insert_or_assign_edge(edge);
        insert_mind(id_result.value(), id);
    }
    catch(int e)
    {
        cout << "Problema" << endl;
    }
}

void SpecificWorker::update_graph(vector<SpecificWorker::PersonData> people_list)
{
    auto world_node = G->get_node('world').value();
    auto people_nodes = G->get_nodes_by_type("person");
    if(people_nodes.size() == 0)
    {
        for(auto p: people_list)
        {
            insert_person(p.personCoords, p.orientation, true);
        }
    }
    vector<DSR::Node> not_seen;
    for(auto p: people_nodes)
    {
        int best_dist = 9999999999;
        SpecificWorker::PersonData candidato;
        bool is_candidato = false;
        int idx_cand;
        for(int i=0;i<people_list.size();i++)
        {
            auto edge = G->get_edge(world_node.id(), people_list[i].id, "RT");
            auto g_coords = G->get_attrib_by_name<rt_translation_att>(edge.value()).value().get();

            cv::Point3f g_coords_point;
            g_coords_point.x = g_coords[0]; g_coords_point.y = g_coords[1]; g_coords_point.z = g_coords[2];
            auto p_c = people_list[i].personCoords;
            cv::Point3f p_coords;
            p_coords.x = p_c[0]; p_coords.y = p_c[2]; p_coords.z = p_c[1];
            auto diff_dist = distance_3d(g_coords_point, p_coords);

            if(diff_dist < best_dist)
            {
                is_candidato = true;
                best_dist = diff_dist;
                candidato = people_list[i];
                idx_cand  = i;
            }
        }
        if(is_candidato == true)
        {
            update_person(p, candidato.personCoords, candidato.orientation);
            auto it = people_list.begin();
            people_list.erase(it+idx_cand);
        }
        else{not_seen.push_back(p);}
    }
    for(auto p: people_list)
    {
        for(auto g: people_nodes)
        {
            auto edge = G->get_edge(world_node.id(), g.id(), "RT");
            auto g_coords = G->get_attrib_by_name<rt_translation_att>(edge.value()).value().get();

            cv::Point3f g_coords_point;
            g_coords_point.x = g_coords[0]; g_coords_point.y = g_coords[1]; g_coords_point.z = g_coords[2];
            auto p_c = p.personCoords;
            cv::Point3f p_coords;
            p_coords.x = p_c[0]; p_coords.y = p_c[2]; p_coords.z = p_c[1];
            auto diff_dist = distance_3d(g_coords_point, p_coords);

            if (diff_dist > min_insert_dist_thr)
            {
                insert_person(p.personCoords, p.orientation, false);
            }
        }
    }
    for(auto p: not_seen){remove_person(p, false);}
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}




/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompHumanCameraBody you can call this methods:
// this->humancamerabody_proxy->newPeopleData(...)

/**************************************/
// From the RoboCompHumanCameraBody you can use this types:
// RoboCompHumanCameraBody::TImage
// RoboCompHumanCameraBody::TGroundTruth
// RoboCompHumanCameraBody::KeyPoint
// RoboCompHumanCameraBody::Person
// RoboCompHumanCameraBody::PeopleData

