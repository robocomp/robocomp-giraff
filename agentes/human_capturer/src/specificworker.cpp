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
#include <algorithm>
#include <cmath>
#include <iostream>
#include <list>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <cppitertools/zip.hpp>
#include <cppitertools/enumerate.hpp>

// Values to determine exists in the world 
int max_lambda_value = 25;
int min_lambda_value = -25;
int hits_to_reach_top_thr = 25;
int min_insert_dist_thr = 1000;
float top_thr = 0.7;
float bot_thr = 0.3;
auto s = -hits_to_reach_top_thr/(log(1/top_thr-1));
auto integrator = [](auto x){return 1/(1 + exp(-x/s));};

cv::RNG rng(12345);
cv::Scalar color_1 = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
cv::Scalar color_2 = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
cv::Scalar color_3 = cv::Scalar( 0, 255, 0);

int person_name_idx = 0;

// Values for drawing pictures
int y_res = 640;
int x_res = 480;
float y_res_f = 640.0;
float x_res_f = 480.0;
float zero = 0;

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
    agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);
	tree_view = params["tree_view"].value == "true";
	graph_view = params["graph_view"].value == "true";
	qscene_2d_view = params["2d_view"].value == "true";
	osg_3d_view = params["3d_view"].value == "true";
    std::cout << "setParams" << std::endl;
	return true;
}
void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = 100;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);

		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
       inner_eigen = G->get_inner_eigen_api();

        // QCustomPlot
        custom_plot.setParent(custom_widget.timeseries_frame );
        custom_plot.xAxis->setLabel("time");
        custom_plot.yAxis->setLabel("rot-b adv-g lhit-m stuck-r");
        custom_plot.xAxis->setRange(0, 200);
        custom_plot.yAxis->setRange(0, 1000);
        err_img = custom_plot.addGraph();
        err_img->setPen(QColor("blue"));
        err_dist = custom_plot.addGraph();
        err_dist->setPen(QColor("red"));
        custom_plot.resize(custom_widget.timeseries_frame->size());
        custom_plot.show();

		//dsr update signals
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
//		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
//		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);
        rt = G->get_rt_api();
		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		{
		    current_opts = current_opts | opts::tree;
		}
//		if(graph_view)
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

        graph_viewer->add_custom_widget_to_dock("Elastic band", &custom_widget);

        for (int i = 0; i < 18; ++i) leader_joints.push_back(zero_pos);
	}

}
void SpecificWorker::compute()
{
    custom_plot.resize(custom_widget.timeseries_frame->size());

    // For coppelia
    auto servo_data = this->jointmotorsimple_proxy->getMotorState("eye_motor");
    servo_position = servo_data.pos;
    qInfo() << __FUNCTION__ << " Servo position" << servo_position;

    // Read skeletons through proxy
    RoboCompHumanCameraBody::PeopleData people_data;
    try
    { people_data = this->humancamerabody_proxy->newPeopleData();}
    catch(const Ice::Exception &e)
    { std::cout << e.what() << " Error connecting to HumanCameraBody" << std::endl; return;}

    // Copy new_poeple data to PersonData struct
    vector<SpecificWorker::PersonData> person_data_vector = build_local_people_data(people_data);

    if(not person_data_vector.empty())
        update_graph(person_data_vector);
    last_people_number = person_data_vector.size();
    qInfo() << __FUNCTION__ << " New_people:" << people_data.peoplelist.size() << "New people in PersonData:" << person_data_vector.size();
}
//////////////////////////////////////////////////////////////////////////////////////
// Method to include people data with the new struct (world position and orientation added)
std::vector<SpecificWorker::PersonData> SpecificWorker::build_local_people_data(const RoboCompHumanCameraBody::PeopleData &people_data_)
{
    std::vector<PersonData> new_people_vector;         // vector of local person structures
    for(const auto &img_person : people_data_.peoplelist)
    {
        PersonData new_person = {.id=img_person.id, .image=img_person.roi, .orientation=calculate_orientation(img_person)};

        //cv::Mat roi = cv::Mat(person.roi.width, person.roi.height, CV_8UC3, (uchar*)&person.roi.image[0]);
        //cv::imshow("ROI NODE", roi);

        if(auto coords = get_transformed_joint_list(img_person.joints); coords.has_value())
        {
            new_person.joints = coords.value();
            if(auto pos = position_filter(new_person.joints); pos.has_value())
            {
                new_person.personCoords_robot = get<0>(pos.value());
                new_person.personCoords_world = get<1>(pos.value());
                new_person.pixels = get<2>(pos.value());
                new_people_vector.push_back(new_person);
            }
        }
        // QUE PASA SI NO?
    }
    return new_people_vector;
}
std::optional<std::tuple<cv::Point3f, cv::Point3f, cv::Point2i>>
SpecificWorker::position_filter(const std::tuple<std::vector<cv::Point3f>, std::vector<cv::Point2i>> &person_joints)
{
    // Initial person pos and pixel pos
    cv::Point3f person_pos = zero_pos;
    cv::Point2i person_pix = zero_pix;

    float max_y_position = std::numeric_limits<float>::min(), min_y_position = std::numeric_limits<float>::max();
    float y_mean = 0.f;
    int x_pix_mean = 0;
    int y_pix_mean = 0;
    int counter_pix = 0;
    float counter_pos = 0.f;

    const auto &[robot_joints, image_joints] = person_joints;
    for (auto &&[rob_joint, img_joint] : iter::zip(robot_joints, image_joints))
    {
        if (rob_joint != zero_pos)
        {
            y_mean += rob_joint.y;
            counter_pos ++;
            if(rob_joint.y > max_y_position) max_y_position = rob_joint.y;
            if(rob_joint.y < min_y_position) min_y_position = rob_joint.y;
        }
        if(img_joint != zero_pix)
        {
            x_pix_mean += img_joint.x;
            y_pix_mean += img_joint.y;
            counter_pix ++;
        }
    }

    if(counter_pix == 0 or counter_pos == 0) { qWarning() << __FUNCTION__ << " " << __LINE__ << " No joints found"; return {};};

    y_mean /= counter_pos;
    person_pix.x = static_cast<int>(x_pix_mean / counter_pix);
    person_pix.y = static_cast<int>(y_pix_mean / counter_pix);

    float diff_division = fabs(max_y_position - min_y_position) / 100.f;
    float less_error = std::numeric_limits<float>::max();
    float best_pos_value = min_y_position;
    int i = 0;          // Counter for pixel or position mean
    for (float j = min_y_position; j < max_y_position; j += diff_division)
    {
        float act_error = 0.f;
        for (const auto &item: robot_joints)
        {
            if (item == zero_pos or item.y > (y_mean + 400))
                continue;
            else
            {
                person_pos += item;
                i++;
                act_error += abs(item.y - j);
            }
        }
        if(act_error < less_error)
        {
            less_error = act_error;
            best_pos_value = j;
        }
    }
    if(i == 0) { qWarning() << __FUNCTION__ << "Error. item == zero_pos or item.y > (y_mean + 400) always true"; return {};};

    person_pos = person_pos / i;
    person_pos.y = best_pos_value;
    person_pos.z = 1;

    Eigen::Vector3f pose_aux = {person_pos.x, person_pos.y, person_pos.z};
    auto person_pos_double = pose_aux.cast <double>();
    cv::Point3f final_point;
    if(auto person_world_pos = inner_eigen->transform(world_name, person_pos_double, robot_name); person_world_pos.has_value())
    {
        final_point.x = person_world_pos->x();
        final_point.y = person_world_pos->y();
        final_point.z = person_world_pos->z();
    }
    else { qWarning() << __FUNCTION__ << "Error. Transforming person_pos_double"; return {};};

    if(person_pos != zero_pos and person_pix != zero_pix)
        return std::make_tuple(person_pos, final_point, person_pix);
    else
    { qWarning() << __FUNCTION__ << "Error. person_pos and person_pix are zero"; return {};};
}
std::optional<cv::Point2i> SpecificWorker::get_person_pixels(RoboCompHumanCameraBody::Person p)
{
//    float avg_i = 0, avg_j = 0;
//    RoboCompHumanCameraBody::TJoints person_tjoints = p.joints;
//    list<RoboCompHumanCameraBody::KeyPoint> eye, ear, chest, hips, huesitos;
//
//    for(auto item : person_tjoints)
//    {
//        std::string key = item.first;
//        if (item.second.x != 0 && item.second.y != 0 && item.second.z != 0 && item.second.i != 0 && item.second.j != 0)
//        {
//            if (std::count(eyeList.begin(), eyeList.end(), key))
//            {
//                eye.push_back(item.second);
//                if(eye.size() == 2)
//                {
//                    huesitos = eye;
////                    cout << "EYE" << endl;
//                }
//            }
//            else if (std::count(earList.begin(), earList.end(), key))
//            {
//                ear.push_back(item.second);
//                if(ear.size() == 2)
//                {
//                    huesitos = ear;
////                    cout << "EAR" << endl;
//                }
//            }
//            else if (std::count(chestList.begin(), chestList.end(), key))
//            {
//                chest.push_back(item.second);
//                if(chest.size() == 2)
//                {
//                    huesitos = chest;
//
////                    cout << "CHEST" << endl;
//                }
//            }
//            else if (std::count(hipList.begin(), hipList.end(), key))
//            {
//                hips.push_back(item.second);
//                if(hips.size() == 2)
//                {
//                    huesitos = hips;
////                    cout << "HIP" << endl;
//                }
//            }
//        }
//    }
//
////    if(hips.size() == 2 && chest.size() == 2)
////    {
////        auto color_mean = get_body_color(list<list<RoboCompHumanCameraBody::KeyPoint>> {chest, hips});
////    }
//
//    if(eyeList.empty() && earList.empty() && chestList.empty() && hipList.empty())
//    {
//        if (auto robot_node = G->get_node("robot"); robot_node.has_value())
//        {
//            auto people_nodes = G->get_nodes_by_type("person");
//            for (auto p: people_nodes)
//            {
//                if (auto followed_node = G->get_attrib_by_name<followed_att>(p); followed_node.has_value() &&
//                                                                                 followed_node.value() == true)
//                {
//                    if (auto edge = G->get_edge(robot_node.value().id(), p.id(), "RT"); edge.has_value())
//                    {
//                        if (auto g_coords = G->get_attrib_by_name<rt_translation_att>(edge.value()); g_coords.has_value())
//                        {
//                            if (auto pix_x_coords = G->get_attrib_by_name<pixel_x_att>(p); pix_x_coords.has_value())
//                            {
//                                if (auto pix_y_coords = G->get_attrib_by_name<pixel_y_att>(p); pix_y_coords.has_value())
//                                {
//                                    avg_i = pix_x_coords.value();
//                                    avg_j = pix_y_coords.value();
//                                    cv::Point2i point(avg_i, avg_j);
//                                    return point;
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
//    else
//    {
//        avg_i = 0, avg_j = 0;
//
//        for(auto kp: huesitos)
//        {
//            avg_i += kp.i;
//            avg_j += kp.j;
//        }
//        avg_i = avg_i/huesitos.size();
//        avg_j = avg_j/huesitos.size();
//
////         cout << "MEAN i pos: " << avg_i << endl;
////         cout << "MEAN j pos: " << avg_j << endl;
//        if((0 < avg_i < 480) && (0 < avg_j < 640))
//        {
//            cv::Point2i point(avg_i, avg_j);
//            return point;
//        }
//    }
    return {};
}
void SpecificWorker::draw_timeseries(float error_dist, float error_img)
{
    static int cont = 0;
    err_dist->addData(cont, error_dist);
    err_img->addData(cont++, error_img);
    custom_plot.xAxis->setRange(cont, 200, Qt::AlignRight);
    custom_plot.replot();

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
//    cout << "p1: " << p1 << endl;
//    cout << "p2: " << p2 << endl;
//    cout << "DISTANCIA: " << cv::norm(p1-p2) << endl;
    return cv::norm(p1-p2);
}
cv::Point3f SpecificWorker::dictionary_values_to_3d_point(RoboCompHumanCameraBody::KeyPoint item)
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
std::optional<std::tuple<vector<cv::Point3f>, vector<cv::Point2i>>>
        SpecificWorker::get_transformed_joint_list(const RoboCompHumanCameraBody::TJoints &joints)
{
    vector<cv::Point3f> joint_points; joint_points.assign(18, zero_pos);
    vector<cv::Point2i> joint_pixels; joint_pixels.assign(18, zero_pix);
    Eigen::Vector3f trans_vect_1(0, -0.06, -0.12);
    Eigen::Vector3f trans_vect_2(0, -0.04, -1.55);
    Eigen::AngleAxisf z_axis_rotation_matrix (servo_position, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf x_axis_rotation_matrix (0.414, Eigen::Vector3f::UnitX());
    int joint_counter = 0;
    for(const auto &[key, item] : joints)
        if (item.x != 0 and item.y != 0 and item.z != 0 and item.i != 0 and item.j != 0 and not (std::ranges::count(avoidedJoints, key)))
        {
            joint_counter++;
            cv::Point2i point_pix(item.i,item.j);
            joint_pixels[jointPreference[std::stoi( key )]] = point_pix;
            cv::Point3f point_robot(item.x*1000, item.z*1000, item.y*1000);
            Eigen::Vector3f joint_pos(point_robot.x, point_robot.y, point_robot.z);
            joint_pos = x_axis_rotation_matrix * (z_axis_rotation_matrix * joint_pos + trans_vect_1) + trans_vect_2;
            point_robot.x = joint_pos.x();
            point_robot.y = joint_pos.y();
            point_robot.z = joint_pos.z();
            joint_points[jointPreference[std::stoi( key )]] = point_robot;
        }
    return std::make_tuple(joint_points, joint_pixels);
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
float SpecificWorker::dot_product(const cv::Point2f &vector_a, const cv::Point2f &vector_b)
{
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
    // cout << "u_vector_1: ("<< u_vector_1.x << ","<< u_vector_1.y << ")" << endl;
    cv::Point2f u_vector_2 = vector_2/cv::norm(vector_2);
    // cout << "vector_2: ("<< vector_2.x << ","<< vector_2.y << ")" << endl;
    // cout << "NORM: "<< cv::norm(vector_2) << endl;
    // cout << "u_vector_2: ("<< u_vector_2.x << ","<< u_vector_2.y << ")" << endl;

    // Extra vector: u_vector_2 rotated /90 degrees
    cv::Point2f u_vector_2_90;
    u_vector_2_90.x = cos(-M_PI / 2) * u_vector_2.x - sin(-M_PI / 2) * u_vector_2.y;
    u_vector_2_90.y = sin(-M_PI / 2) * u_vector_2.x + cos(-M_PI / 2) * u_vector_2.y;

    // cout << "u_vector_2_90: ("<< u_vector_2_90.x << ","<< u_vector_2_90.y << ")" << endl;
    // Dot product of u_vector_1 with u_vector_2 and u_vector_2_90
    //float dp = u_vector_1.x * u_vector_2.x + u_vector_1.y * u_vector_2.y;
    //float dp_90 = u_vector_1.x * u_vector_2_90.x + u_vector_1.y * u_vector_2_90.y;

    // Dot product of u_vector_1 with u_vector_2 and urobot.value_vector_2_90
    float dp = dot_product(u_vector_1, u_vector_2);
    // cout << "DP: " << dp << endl;
    float dp_90 = dot_product(u_vector_1, u_vector_2_90);
    // cout << "DP_90: " << dp_90 << endl;

    // Comprobating if the angle is over 180 degrees and adapting
    float ret;
    if(dp_90 < 0){ret = M_PI + (M_PI-acos(dp));}
    else{ret = acos(dp);}

    // cout << "RET: " << ret << endl;

    // Returning value
    if (format.compare("radians") == 0) {return ret;}
    else {return (ret*180/M_PI);}
}
float SpecificWorker::calculate_orientation(RoboCompHumanCameraBody::Person person)
{
    RoboCompHumanCameraBody::TJoints person_tjoints = person.joints;
    bool left_found= false, base_found= false, right_found = false;
    cv::Point3f base_p, right_p, left_p;

    for(auto item : person_tjoints)
    {
        std::string key = item.first;

        // Base point

        if (base_found == false && (key.compare("17")==0 || key.compare("6")==0 || key.compare("5")==0 || key.compare("2")==0 || key.compare("1")==0))
        {
            base_found = true;
            base_p = dictionary_values_to_3d_point(item.second);
            // cout << "KEYPOINT BASEP: "<< key <<endl;
        }

        // Right point

        if (right_found == false && (key.compare("12")==0 || key.compare("4")==0))
        {
            right_found = true;
            right_p = dictionary_values_to_3d_point(item.second);
            // cout << "KEYPOINT RIGHTP: "<< key <<endl;
        }

        // Left point

        if (left_found == false && (key.compare("11")==0 || key.compare("3")==0))
        {
            left_found = true;
            left_p = dictionary_values_to_3d_point(item.second);
            // cout << "KEYPOINT LEFTP: "<< key <<endl;
        }

        if(base_found == true && right_found == true && left_found == true)
        {
            //break;
        }

        // cout << "CLAVE: " << key << ", VALOR: (" << item.second.x << "," <<item.second.y << "," <<item.second.z << ")" << endl;
    }

    if(base_found == false || right_found == false || left_found == false)
    {
        // cout << "Points not found. Can't calculate orientation." << endl;
        return 0.0;
    }

    // Considering "clavícula" as coordinate center. Passing leg points to "clavícula" reference system

    cv::Point3f left_v = left_p - base_p;
    cv::Point3f right_v = right_p - base_p;

    // Calculating perpendicular vector

    cv::Point3f normal = cross_product(left_v, right_v);
    cv::Point2f vector_1, vector_2;
    vector_1.x = 0;
    vector_1.y = 1;
    vector_2.x = normal.x;
    vector_2.y = normal.z;

    // cout << "vector_2: (" << vector_2.x << ","<<vector_2.y << ")" << endl;

    float angle = get_degrees_between_vectors(vector_1, vector_2, "radians");
    // cout << "Ángulo: " << angle << endl;
    return angle;
}
void SpecificWorker::remove_person(DSR::Node person_node, bool direct_remove)
{
    std::string node_name_str = "virtual_leader";
//    std::cout << "REMOVE" << std::endl;
    float score = 0;
    int nlc;
    if(auto lost_edge = G->get_edge(G->get_node("robot").value(), person_node.id(), "lost"); not lost_edge.has_value())
    {
        if(direct_remove == true)
        { /*int score = 0; */}
        else
        {
            if(auto person_lc = G->get_attrib_by_name<lambda_cont_att>(person_node); person_lc.has_value())
                {
                    if(auto person_id = G->get_attrib_by_name<person_id_att>(person_node); person_id.has_value())
                    {
    //                cout << "calculating lambda value for person " << person_id.value() << endl;
                        nlc = decrease_lambda_cont(person_lc.value());
    //                cout << "//////////////// NLC decreasing:   " << nlc << endl;
                        G->add_or_modify_attrib_local<lambda_cont_att>(person_node, nlc);
                        G->update_node(person_node);
                        score = integrator(nlc);
                    }
                }
        }

        if((score <= bot_thr) or (direct_remove == true))
        {
            G->delete_edge(G->get_node("robot").value().id(), person_node.id(), "RT");
            G->delete_node(person_node.id());
        }
    }
    else
    {
        std::cout << "PERDIDO" << std::endl;
        auto person_nodes = G->get_nodes_by_type("virtual_person");
        for (auto node : person_nodes)
        {
            if(auto edge_robot_person = rt->get_edge_RT(G->get_node("world").value(), node.id()); edge_robot_person.has_value())
            {
                if (auto person_robot_pos = G->get_attrib_by_name<rt_translation_att>(edge_robot_person.value()); person_robot_pos.has_value())
                {
                    auto person_robot_pos_val = person_robot_pos.value().get();
                    Eigen::Vector3f person_robot_pos_point(person_robot_pos_val[0], person_robot_pos_val[1], person_robot_pos_val[2]);
                    auto person_robot_pos_cast = person_robot_pos_point.cast <double>();
                    auto person_world_pos = inner_eigen->transform("robot", person_robot_pos_cast, "world").value();
                    Eigen::Vector3f person_robot_pos_flo = person_world_pos.cast <float>();
                    std::cout << "CONVERSION A BORROT: " << person_robot_pos_flo << std::endl;
                    if(auto edge_robot_person = rt->get_edge_RT(G->get_node("robot").value(), person_node.id()); edge_robot_person.has_value())
                    {
                        std::vector<float> vector_robot_pos = {person_robot_pos_flo.x(), person_robot_pos_flo.y(), person_robot_pos_flo.z()};
                        G->add_or_modify_attrib_local<rt_translation_att>(edge_robot_person.value(), vector_robot_pos);
                        if (G->insert_or_assign_edge(edge_robot_person.value()))
                        {
//                            std::cout << __FUNCTION__ << " Edge successfully modified: " << node_value.id() << "->" << node.id()
//                                      << " type: RT" << std::endl;
                        }
                        else
                        {
//                            std::cout << __FUNCTION__ << ": Fatal error modifying new edge: " << node_value.id() << "->" << node.id()
//                                      << " type: RT" << std::endl;
                            std::terminate();
                        }
                    }
                }
            }
        }
    }

//    if (auto followed_node = G->get_attrib_by_name<followed_att>(person_node);  followed_node.has_value() && followed_node.value() == true && nlc <= 0)
////    else if(auto following_edge = G->get_edge(G->get_node("robot").value(), person_node.id(), "following"); following_edge.has_value() && nlc <= 0)
//    {
//
//        if(auto lost_edge = G->get_edge(G->get_node("robot").value(), person_node.id(), "lost"); not lost_edge.has_value())
//        {
//            std::cout << "RECIEN PERDIDO" << std::endl;
//            DSR::Edge edge = DSR::Edge::create<lost_edge_type>(G->get_node("robot").value().id(), person_node.id());
//            if (G->insert_or_assign_edge(edge))
//            {
//                std::cout << __FUNCTION__ << " Edge successfully inserted: " << G->get_node("robot").value().id() << "->" << person_node.id()
//                          << " type: lost" << std::endl;
//            }
//            else
//            {
//                std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << G->get_node("robot").value().id() << "->" << person_node.id()
//                          << " type: has" << std::endl;
//            }
//
//            DSR::Node new_virtual_node = DSR::Node::create<virtual_person_node_type>(node_name_str);
//            G->insert_node(new_virtual_node);
//            G->update_node(new_virtual_node);
//            if(auto edge_robot_person = rt->get_edge_RT(G->get_node("robot").value(), person_node.id()); edge_robot_person.has_value())
//            {
//                if(auto person_robot_pos = G->get_attrib_by_name<rt_translation_att>(edge_robot_person.value()); person_robot_pos.has_value())
//                {
//                    auto person_robot_pos_val = person_robot_pos.value().get();
//                    Eigen::Vector3f person_robot_pos_point(person_robot_pos_val[0], person_robot_pos_val[1], person_robot_pos_val[2]);
//                    auto person_robot_pos_cast = person_robot_pos_point.cast <double>();
//                    auto person_world_pos = inner_eigen->transform("world", person_robot_pos_cast, "robot").value();
//                    Eigen::Vector3f person_robot_pos_flo = person_world_pos.cast <float>();
//
//                    std::vector<float> person_pos = {person_robot_pos_flo.x(), person_robot_pos_flo.y(), person_robot_pos_flo.z()};
//                    std::vector<float> person_ori = {0.0, 0.0, 0.0};
//                    auto world_node = G->get_node("world").value();
//                    rt->insert_or_assign_edge_RT(world_node, new_virtual_node.id(), person_pos, person_ori);
//                }
//            }
//        }
//
//    }
}
void SpecificWorker::update_person(DSR::Node node, SpecificWorker::PersonData persondata)
{
    static std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();

    if(auto world_node = G->get_node(world_name); world_node.has_value())
    {
        auto node_value = world_node.value();
        if(auto robot_node = G->get_node(robot_name); robot_node.has_value())
        {
//            if(auto lost_edge = G->get_edge(robot_node.value().id(), node.id(), "lost"); lost_edge.has_value())
//            {
//                return;
//            }
            // Modify distance from human to robot
            float dist_to_robot = sqrt(pow(persondata.personCoords_robot.x,2) + pow(persondata.personCoords_robot.y,2));
            G->add_or_modify_attrib_local<distance_to_robot_att>(node, dist_to_robot);
            G->add_or_modify_attrib_local<person_pixel_x_att>(node, persondata.pixels.x);
            G->add_or_modify_attrib_local<person_pixel_y_att>(node, persondata.pixels.y);
            G->add_or_modify_attrib_local<person_image_att>(node, persondata.image.image);
            G->add_or_modify_attrib_local<person_image_width_att>(node, persondata.image.width);
            G->add_or_modify_attrib_local<person_image_height_att>(node, persondata.image.height);

            // PARA QUE SIRVE? LA INSERCION DEBE HACERSE SOLO SI SE LE HA VISTO DURANTE MAS DE MEDIO O UN SEGUNDO
            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
            leader_ROI_memory.insert(leader_ROI_memory.cbegin(), persondata.image);
            if (leader_ROI_memory.size() > memory_size and elapsed_time_ms > 1000)
            {
                t_start = std::chrono::high_resolution_clock::now();
                leader_ROI_memory.pop_back();
            }

//          std::cout << "ORIENTATION: " << persondata.orientation << std::endl;
            if(persondata.orientation > (2*M_PI - (M_PI/6)) || persondata.orientation < (M_PI/6)) G->add_or_modify_attrib_local<is_ready_att>(node, true);
            else G->add_or_modify_attrib_local<is_ready_att>(node, false);
            G->update_node(node);

            std::vector<float> new_position_vector_robot = {persondata.personCoords_world.x, persondata.personCoords_world.y, persondata.personCoords_world.z};
            std::vector<float> orientation_vector = {0.0, persondata.orientation, 0.0};
            try
            {
//                if(auto edge_robot = rt->get_edge_RT(robot_node.value(), node.id()); edge_robot.has_value())
                if(auto edge_world = rt->get_edge_RT(world_node.value(), node.id()); edge_world.has_value())
                {
                    if(auto last_pos = G->get_attrib_by_name<rt_translation_att>(edge_world.value()); last_pos.has_value())
                    {
//                        std::cout << "POS x: " << last_pos.value().get()[0] << " POS y: " << last_pos.value().get()[1] << endl;
                        auto last_pos_value = last_pos.value().get();
                        std::vector<float> new_robot_pos = {(alpha * new_position_vector_robot[0]) + (beta * last_pos_value[0]), (alpha * new_position_vector_robot[1]) + (beta * last_pos_value[1]), (alpha * new_position_vector_robot[2]) + (beta * last_pos_value[2])};
//                        std::vector<float> new_robot_pos = {(new_position_vector_robot[0]) , (new_position_vector_robot[1]) , (new_position_vector_robot[2])};
//                        std::cout << "WORLD POS FILTERED: " <<  new_robot_pos << endl;
                        G->add_or_modify_attrib_local<rt_rotation_euler_xyz_att>(edge_world.value(), orientation_vector);
                        G->add_or_modify_attrib_local<rt_translation_att>(edge_world.value(), new_robot_pos);

//                        if(auto edge_robot_world = rt->get_edge_RT(node_value, robot_node.value().id()); edge_robot_world.has_value())
//                        {
//                            if(auto robot_ang = G->get_attrib_by_name<rt_rotation_euler_xyz_att>(edge_robot_world.value()); robot_ang.has_value())
//                            {
//                                int azimut_deg = (int)(atan2(robot_coords.y, robot_coords.x)*180/M_PI) - 90;
////                                std::cout << "azimut_deg" << azimut_deg << endl;
//                                int  robot_world_angle = (int)(robot_ang.value().get()[2]*180/M_PI + 180);
////                                std::cout << "robot_world_angle" << robot_ang.value().get()[2] << endl;
//                                auto respect_to_world_angle = robot_world_angle + azimut_deg;
////                                std::cout << "respect_to_world_angle" << respect_to_world_angle << endl;
//                                if(respect_to_world_angle > 360 or respect_to_world_angle < 0 ) respect_to_world_angle = respect_to_world_angle % 360;
////                                std::cout << respect_to_world_angle << endl;
//                                G->add_or_modify_attrib_local<azimut_refered_to_robot_image_att>(node, respect_to_world_angle);
//                            }
//                        }

                        if (G->insert_or_assign_edge(edge_world.value()))
                        {
//                            std::cout << __FUNCTION__ << " Edge successfully modified: " << node_value.id() << "->" << node.id()
//                                      << " type: RT" << std::endl;
                        }
                        else
                        {
//                            std::cout << __FUNCTION__ << ": Fatal error modifying new edge: " << node_value.id() << "->" << node.id()
//                                      << " type: RT" << std::endl;
                            std::terminate();
                        }
//                        G->update_node(world_node.value());
                        G->update_node(node);

                        auto check_pos = G->get_attrib_by_name<rt_translation_att>(edge_world.value()).value().get();
//                        std::cout << "CHEK: " << check_pos << std::endl;

                        // Lambda_cont increment
                        if(auto person_lc = G->get_attrib_by_name<lambda_cont_att>(node); person_lc.has_value())
                        {
                            int nlc = increase_lambda_cont(person_lc.value());
                            G->add_or_modify_attrib_local<lambda_cont_att>(node, nlc);
                            G->update_node(node);
                        }
                    }
                }
            }
            catch(int e)
            {
                cout << e << endl;
            }
        }
    }
}
void SpecificWorker::insert_mind(std::uint64_t parent_id, std::int32_t person_id)
{
    if( auto parent_node = G->get_node(parent_id); parent_node.has_value())
    {
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
            DSR::Edge edge = DSR::Edge::create<has_edge_type>(parent_node.value().id(), new_node.id());
            if (G->insert_or_assign_edge(edge))
            {
//                std::cout << __FUNCTION__ << " Edge successfully inserted: " << parent_node.value().id() << "->" << new_node.id()
//                          << " type: has" << std::endl;
            }
            else
            {
//                std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << parent_node.value().id() << "->" << new_node.id()
//                          << " type: has" << std::endl;
                std::terminate();
            }


        }
        catch(int e)
        {
            cout << "Problema" << endl;
        }
    }
}
void SpecificWorker::insert_person(const PersonData &persondata, bool direct_insert)
{
    static std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();

    if(auto robot_node = G->get_node(robot_name); robot_node.has_value())
    {
        if(auto world_node = G->get_node(world_name); world_node.has_value())
        {
            if(auto world_level = G->get_node_level(world_node.value()); world_level.has_value())
            {
                float pos_x = rand() % (120 - (-100) + 1) + (-100);
                float pos_y = rand() % (-100 - (-370) + 1) + (-370);
                int id = person_name_idx;
                person_name_idx += 1;
                std::string person_id_str = std::to_string(id);
                std::string node_name = "person_" + person_id_str;

                DSR::Node new_node = DSR::Node::create<person_node_type>(node_name);
                G->add_or_modify_attrib_local<person_id_att>(new_node, id);
                G->add_or_modify_attrib_local<level_att>(new_node, world_level.value() + 1);
                G->add_or_modify_attrib_local<is_followed_att>(new_node, false);
                G->add_or_modify_attrib_local<is_lost_att>(new_node, false);
                G->add_or_modify_attrib_local<person_pixel_x_att>(new_node, persondata.pixels.x);
                G->add_or_modify_attrib_local<person_pixel_y_att>(new_node, persondata.pixels.y);
                G->add_or_modify_attrib_local<person_image_att>(new_node, persondata.image.image);
                G->add_or_modify_attrib_local<person_image_width_att>(new_node, persondata.image.width);
                G->add_or_modify_attrib_local<person_image_height_att>(new_node, persondata.image.height);

                // time management
                auto t_end = std::chrono::high_resolution_clock::now();
                double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
                leader_ROI_memory.insert(leader_ROI_memory.cbegin(), persondata.image);
                if (leader_ROI_memory.size() > memory_size and elapsed_time_ms > 1000)
                {
                    t_start = std::chrono::high_resolution_clock::now();
                    leader_ROI_memory.pop_back();
                }

                // para qué?
                if (persondata.orientation > (2 * M_PI - (M_PI / 6)) or persondata.orientation < (M_PI / 6))
                    G->add_or_modify_attrib_local<is_ready_att>(new_node, true);
                else
                    G->add_or_modify_attrib_local<is_ready_att>(new_node, false);

                int lc = 0;
                if (direct_insert)
                    lc = hits_to_reach_top_thr;
                else
                    lc = 1;

                G->add_or_modify_attrib_local<parent_att>(new_node, world_node.value().id());
                G->add_or_modify_attrib_local<lambda_cont_att>(new_node, lc);
                G->add_or_modify_attrib_local<distance_to_robot_att>(new_node, persondata.personCoords_robot.y);
                G->add_or_modify_attrib_local<pos_x_att>(new_node, pos_x);
                G->add_or_modify_attrib_local<pos_y_att>(new_node, pos_y);

                try
                {
                    G->insert_node(new_node);
                    std::vector<float> vector_robot_pos =
                            { persondata.personCoords_world.x, persondata.personCoords_world.y, persondata.personCoords_world.z};
                    std::vector<float> orientation_vector = {0.0, persondata.orientation, 0.0};
                    rt->insert_or_assign_edge_RT(world_node.value(), new_node.id(), vector_robot_pos, orientation_vector);
                }
                catch(const std::exception &e)
                { std::cout << e.what() << " Error inserting node" << std::endl;}
            }
            else qWarning() << "No level_att found in world node";
        }
        else qWarning() << "No world node found";
    }
    else qWarning() << "No robot node found";
}

//// ESTE DEBE TENER CUATRO PARTES BIEN DIFERNECIAS Y COMENTADAS:
/// 1: matching entre nuevos y del grafo. Los que casan, actualizarlos y quitarlos de nuevos
/// 3: los que quedan en nuevos, programalos para ser añadidos una vez pasado el tiempo requerido
/// 4: si quedan del grafo, programarlos para ser borrados

void SpecificWorker::update_graph(const std::vector<PersonData> &people_list)
{
    static HungarianAlgorithm HungAlgo;

    DSR::Node world_node;
    if (auto world_node_o = G->get_node(world_name); not world_node_o.has_value())
    { qWarning() << "No world node found. Returning"; return;}
    else world_node = world_node_o.value();

    DSR::Node robot_node;
    if (auto robot_node_o = G->get_node(robot_name); not robot_node_o.has_value())
    { qWarning() << "No robot node found. Returning"; return;}
    else robot_node = robot_node_o.value();

    auto people_in_graph = G->get_nodes_by_type(person_type_name);
    if (people_in_graph.empty())
    {
        qInfo() << __FUNCTION__ << " No person nodes in graph. Insert new ones and return";
        for (const auto &p: people_list)    // Insert the new ones
            insert_person(p, true);
        return;
    }

    // Check if some person has to be erased. Rows for nodes, columns for people
    std::vector<std::vector<double>> distance_comparisons, corr_comparisons;
    std::vector<int> matched_nodes, matched_people;
    for (const auto &p: people_in_graph)
    {
        std::vector<double> people_distance_to_nodes(people_list.size()), corr_vector(people_list.size());
        // get people from G and into a LeaderData struct
        LeaderData person_node_data;
        cv::Point2i max_point;
        if (auto person_ROI_data_att = G->get_attrib_by_name<person_image_att>(p); person_ROI_data_att.has_value())
        {
            if (auto person_ROI_width_att = G->get_attrib_by_name<person_image_width_att>(p); person_ROI_width_att.has_value())
            {
                if (auto person_ROI_height_att = G->get_attrib_by_name<person_image_height_att>(p); person_ROI_height_att.has_value())
                {
                    auto leader_ROI_data = person_ROI_data_att.value().get();
                    cv::Mat person_roi = cv::Mat(person_ROI_width_att.value(),
                                                 person_ROI_height_att.value(), CV_8UC3,
                                                 &leader_ROI_data[0]);

                    if (auto robot_person_edge = G->get_edge(G->get_node(robot_name).value().id(), p.id(), following_action_type_name);
                                                 robot_person_edge.has_value())
                    {
                        cv::imshow(p.name(), person_roi);
                        cv::waitKey(1);
                    }
                    // CAMBIAR A inner_eigen
                    if (auto pos_edge_world = rt->get_edge_RT(world_node, p.id()); pos_edge_world.has_value())
                    {
                        if (auto person_world_pos = G->get_attrib_by_name<rt_translation_att>(pos_edge_world.value()))
                        {
                            person_node_data.position.x = person_world_pos.value().get()[0];
                            person_node_data.position.y = person_world_pos.value().get()[1];
                            person_node_data.position.z = person_world_pos.value().get()[2];
                        }
                    }
                    person_node_data.ROI = person_roi;

                    // Go through detected people in image

                    for (const auto &person_in_image: people_list)
                    {
                        people_distance_to_nodes.emplace_back(people_comparison_distance(person_node_data, person_in_image));
                        corr_vector.emplace_back(people_comparison_corr(person_node_data, max_point, person_in_image));
                    }
                    corr_comparisons.push_back(corr_vector);
                    distance_comparisons.push_back(people_distance_to_nodes);
                }
            }
        }
    }
    std::vector<double> corr_values;
    for (const auto &corr : corr_comparisons)
        corr_values.push_back(std::ranges::max(corr));
    double max_corr_val = std::ranges::max(corr_values);

    for (unsigned int i = 0; i < corr_comparisons.size(); ++i)
        for (unsigned int j = 0; j < corr_comparisons[i].size(); ++j)
        {
            corr_comparisons[i][j] = 1 - (corr_comparisons[i][j] / max_corr_val);  // CHECK FOR ZERO DIVISION
            distance_comparisons[i][j] = (int) (distance_comparisons[i][j] * corr_comparisons[i][j]);
            if (distance_comparisons[i][j] == 0)
                distance_comparisons[i][j] = 1;
        }

    std::vector<int> assignment;
    double cost = HungAlgo.Solve(distance_comparisons, assignment);
    for (unsigned int i = 0; i < distance_comparisons.size(); i++)
        for (unsigned int j = 0; j < people_list.size(); j++)
            if ((int)j == assignment[i])    // Puede asignarle la posición a quien le de la gana
            {
                update_person(people_in_graph[i], people_list[j]);
                matched_nodes.push_back(i);
                matched_people.push_back(j);
                if (auto lost_edge = G->get_edge(robot_node, people_in_graph[i].id(), "lost"); lost_edge.has_value())
                {
                    auto virtual_people_nodes = G->get_nodes_by_type("virtual_person");
                    for (const auto &v_p: virtual_people_nodes)
                    {
                        G->delete_node(v_p.id());
                        G->delete_edge(robot_node.id(), people_in_graph[i].id(), "lost");
                        G->delete_edge(world_node.id(), v_p.id(), "RT");
                    }
                }
            }

    // INSERT: If list of people size is bigger than list of people nodes, new people must be inserted into the graph
    for (auto &&[i, person] : people_list | iter::enumerate)
        if (std::ranges::find(matched_people, i) == matched_people.end())
            insert_person(person, true);

    // DELETE: If list of people nodes is bigger than list of people, some nodes must be proposed to be deleted
    for (auto &&[i, person] : people_in_graph | iter::enumerate)
    {
        if (std::ranges::find(matched_nodes, i) == matched_nodes.end())
        {
            if (auto robot_person_edge = G->get_edge(robot_node.id(), person.id(), following_action_type_name);
                    robot_person_edge.has_value() and
                    G->get_attrib_by_name<lambda_cont_att>(person).value() < -23)
            {
                G->delete_edge(robot_node.id(), person.id(), following_action_type_name);
                DSR::Edge lost_edge = DSR::Edge::create<lost_edge_type>(robot_node.id(), person.id());
                if (G->insert_or_assign_edge(lost_edge))
                    std::cout << __FUNCTION__ << " Edge successfully inserted: " << std::endl;
                else
                    std::cout << __FUNCTION__ << " Fatal error inserting new edge: " << std::endl;
            }
            else if (auto lost_person_edge = G->get_edge(robot_node.id(), person.id(), "lost"); not lost_person_edge.has_value())
                remove_person(people_in_graph[i], false);
        }
    }
}
bool SpecificWorker::danger_detection(float correlation, LeaderData leader_data, const vector<PersonData> &people_list)
{
    int near_to_leader_counter = 0;
    for(auto person : people_list)
    {
        int diff_x_pix_to_leader = abs(person.pixels.x - leader_data.pix_x);
        if(diff_x_pix_to_leader < 120) near_to_leader_counter ++;
    }
    std::cout << "PEOPLE NEAR TO LEADER: " << near_to_leader_counter << std::endl;
    if(near_to_leader_counter - 1 > 0)
    {
        danger = true;
        correlation_th = 0.6;
    }

    if(danger and last_people_number > people_list.size())
    {
        occlussion = true;
        correlation_th = 0.8;
    }

    if(correlation > correlation_th)
    {
        danger = false;
        occlussion = false;
    }

    std::cout << "DANGER: " << danger << std::endl;
    std::cout << "OCCLUSSION: " << occlussion << std::endl;
    last_people_number = people_list.size();
    return danger || occlussion;
}
double SpecificWorker::people_comparison_corr(const LeaderData &node_data , const cv::Point2i &max_corr_point, const PersonData &person)
{
    // Jetson roi
    cv::Mat jetson_roi = cv::Mat( person.image.width,
                                  person.image.height, CV_8UC3,
                                  (uchar *)&person.image.image[0]);

    cv::Mat result;
    cv::resize(jetson_roi, jetson_roi, cv::Size(node_data.ROI.cols, node_data.ROI.rows), cv::INTER_CUBIC);
    cv::matchTemplate(node_data.ROI, jetson_roi, result, cv::TM_CCOEFF);
    cv::Point2i max_point_correlated, min_point_correlated;
    double max_value, min_value;
    cv::minMaxLoc(result, &min_value, &max_value, &min_point_correlated, &max_point_correlated, cv::Mat());
    return abs(max_value);
}
double SpecificWorker::people_comparison_distance(LeaderData node_data, PersonData person)
{
    auto p_c = person.personCoords_world;
    return sqrt(pow(node_data.position.x - p_c.x, 2) + pow(node_data.position.y - p_c.y, 2));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::modify_node_slot(const std::uint64_t id, const std::string &type)
{
    if (type == "intention")
    {
        if (auto intention = G->get_node(id); intention.has_value())
        {
            std::optional<std::string> plan = G->get_attrib_by_name<current_intention_att>(intention.value());
            if (plan.has_value())
            {
                Plan my_plan(plan.value());
                auto person_id = my_plan.get_attribute("person_node_id");
                uint64_t value;
                std::istringstream iss(person_id.toString().toUtf8().constData());
                iss >> value;
                if(auto followed_person_node = G->get_node(value); followed_person_node.has_value())
                {
                    DSR::Edge following_edge = DSR::Edge::create<following_edge_type>(G->get_node("robot").value().id(), followed_person_node.value().id());
                    if (G->insert_or_assign_edge(following_edge))
                    {
                        std::cout << __FUNCTION__ << " Edge successfully inserted: " << std::endl;
                    }
                    else
                    {
                        std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << std::endl;
                    }
                }
            }
        }
    }
}
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}



//    try
//    {
//        RoboCompAprilTagsServer::tagsList tags = apriltagsserver_proxy->getAprilTags();
//        if(tags.size() > 0)
//        {
//            april_pix_x = tags[0].cx;
//            existTag = true;
////            qInfo() << __FUNCTION__ << tags.size() << " TAG DATA: " << tags[0].cx << " " << tags[0].cy;
//        }
//        else existTag = false;
//    }
//    catch(int e)
//    {
//        cout << e << endl;
//    }

//    auto servo_data = this->jointmotorsimple_proxy->getMotorState("servo_joint");

//    auto servo_data = this->jointmotorsimple_proxy->getMotorState("");

// Iterating TJoints
//        for(const auto &item : person.joints)
//        {
//            // Appending joint pixels to a vector
//            cv::Point pixel;
//            pixel.x = item.second.i;
//            pixel.y = item.second.j;
//            //std::cout << "JOINT POSITION: " << item.second.x << " " << item.second.y << " " << item.second.z << std::endl;
//            pixel_vector.push_back(pixel);
//        }
// Represent image
//                cv::Rect person_box = cv::boundingRect(pixel_vector);
//                cv::rectangle(black_picture, cv::Rect(person_box.x, person_box.y, person_box.width, person_box.height), color_1, 2);
//                cv::Point square_center;
//                square_center.x = person_data.pixels.x;
//                square_center.y = person_data.pixels.y;
//                cv::circle(black_picture, square_center,12,color_2);
//                for(int k=0;k<pixel_vector.size();k++)
//                    cv::circle(black_picture, pixel_vector[k],12,color_1);


//                if(auto edge_robot_world = rt->get_edge_RT(world_node.value(), robot_node.value().id()); edge_robot_world.has_value())
//                {
//                    if(auto robot_ang = G->get_attrib_by_name<rt_rotation_euler_xyz_att>(edge_robot_world.value()); robot_ang.has_value())
//                    {
//                        int azimut_deg = (int)(atan2(robot_coords.y, robot_coords.x)*180/M_PI) - 90;
////                        std::cout << "azimut_deg" << azimut_deg << endl;
//                        int  robot_world_angle = (int)(robot_ang.value().get()[2]*180/M_PI + 180);
////                        std::cout << "robot_world_angle" << robot_ang.value().get()[2] << endl;
//                        auto respect_to_world_angle = robot_world_angle + azimut_deg;
////                        std::cout << "respect_to_world_angle" << respect_to_world_angle << endl;
//                        if(respect_to_world_angle > 360 or respect_to_world_angle < 0 ) respect_to_world_angle = respect_to_world_angle % 360;
////                        std::cout << respect_to_world_angle << endl;
//                        G->add_or_modify_attrib_local<azimut_refered_to_robot_image_att>(new_node, respect_to_world_angle);
//                    }
//                }

//                    DSR::Edge edge_world = DSR::Edge::create<RT_edge_type>(world_node.value().id(), new_node.id());

//                    G->add_or_modify_attrib_local<rt_translation_att>(edge_world, new_position_vector_world);
//                    G->add_or_modify_attrib_local<rt_rotation_euler_xyz_att>(edge_world, orientation_vector);
//
//                    if (G->insert_or_assign_edge(edge_world))
//                    {
//                        std::cout << __FUNCTION__ << " Edge successfully inserted: " << world_node.value().id() << "->" << new_node.id()
//                                  << " type: RT" << std::endl;
//                    }
//                    else
//                    {
//                        std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << world_node.value().id() << "->" << new_node.id()
//                                  << " type: RT" << std::endl;
//                        std::terminate();
//                    }
//                    G->update_node(world_node.value());
//                    G->update_node(new_node);
//                    insert_mind(id_result.value(), id);