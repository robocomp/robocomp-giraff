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
cv::RNG rng(12345);
cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
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

		this->Period = period;
		timer.start(Period);
	}

}

QVector3D SpecificWorker::get_person_coords(RoboCompHumanCameraBody::Person p)
{
    QVector3D point(3.5,3.5,3.5);
    return point;
}

void SpecificWorker::compute()
{
    try
    {
        // Creating white image with dimension 480x640

        cv::Mat black_picture = cv::Mat::zeros(640, 480, CV_8UC3);

        RoboCompHumanCameraBody::PeopleData people_data = this->humancamerabody_proxy->newPeopleData();
        // RoboCompHumanCameraBody::PeopleData people_data = test_person();
        RoboCompHumanCameraBody::People people_list = people_data.peoplelist;

        // Sacamos las coords de cada persona
        for(auto p: people_list){
            QVector3D person_coords = get_person_coords(p);
            std::cout << "Person " << p.id << ": (" << person_coords.x() << ","<< person_coords.y()
            << "," << person_coords.z() << ")";
        }

        // Reading people list

        for(int i=0;i<people_list.size();i++)
        {
            RoboCompHumanCameraBody::Person person = people_list[i];
            cout << "Person ID: " << person.id << endl;
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

                // float pos_x = std::accumulate(depth_vector_x.begin(), depth_vector_x.end(), decltype(depth_vector_x)::value_type(0)) / size(depth_vector_x);
                // float pos_y = std::accumulate(depth_vector_y.begin(), depth_vector_y.end(), decltype(depth_vector_y)::value_type(0)) / size(depth_vector_y);
                // float pos_z = std::accumulate(depth_vector_z.begin(), depth_vector_z.end(), decltype(depth_vector_z)::value_type(0)) / size(depth_vector_z);

                // person.personCoords.push_back(pos_x); person.personCoords.push_back(pos_y); person.personCoords.push_back(pos_z);
                // person.orientation = calculate_orientation(person);
            }

            cv::Rect person_box = cv::boundingRect(pixel_vector);
            cv::rectangle(black_picture, cv::Rect(person_box.x, person_box.y, person_box.width, person_box.height), color, 2);
            for(int k=0;k<pixel_vector.size();k++)
            {
                cv::circle(black_picture, pixel_vector[k],1,color);
            }
            cv::imshow("Output", black_picture);
        }
    }
    catch(const char * str)
    {
        cout << "Getting error: " << str << endl;
    }





}

int SpecificWorker::increase_lambda_cont(std::int64_t lambda_cont)
// Increases lambda_cont in 1, to the maximum value and returns the new value. Returns the new lambda_cont value.
{
    std::int64_t nlc = lambda_cont + 1;
    if(nlc < max_lambda_value) {return nlc;}
    else {return max_lambda_value;}
}

int SpecificWorker::decrease_lambda_cont(std::int64_t lambda_cont)
// Decreases lambda_cont in 1, to the minimun value and returns the new value. Returns the new lambda_cont value.
{
    std::int64_t nlc = lambda_cont - 1;
    if(nlc > min_lambda_value) {return nlc;}
    else {return min_lambda_value;}
}

double SpecificWorker::distance_3d(cv::Point3d p1, cv::Point3d p2)
{
    cv::norm(p1-p2);
}

cv::Point3d SpecificWorker::dictionary_values_to_3d_point(auto item)
{
    cv::Point3d point;
    float x = item.x;
    float y = item.y;
    float z = item.z;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

cv::Point3d SpecificWorker::cross_product(cv::Point3d p1, cv::Point3d p2)
{
    cv::Point3d point;
    point.x = p1.y * p2.z - p1.z * p2.y;
    point.y = p1.z * p2.x - p1.x * p2.z;
    point.z = p1.x * p2.y - p1.y * p2.x;
    return point;
}

float SpecificWorker::get_degrees_between_vectors(cv::Point vector_1, cv::Point vector_2, std::string format)
{
    // Returns the angle between two vectors in the 2d plane (v2 respect v1)

    if (format.compare("radians") != 0 || format.compare("degrees") != 0)
    {
        cout << "Invalid angle format" << endl;
        return 0.0;
    }

    // Getting unitary vectors
    cv::Point u_vector_1 = vector_1/cv::norm(vector_1);
    cv::Point u_vector_2 = vector_2/cv::norm(vector_2);

    // Extra vector: u_vector_2 rotated /90 degrees
    cv::Point u_vector_2_90;
    u_vector_2_90.x = cos(-M_PI / 2) * u_vector_2.x - sin(-M_PI / 2) * u_vector_2.y;
    u_vector_2_90.y = sin(-M_PI / 2) * u_vector_2.x + cos(-M_PI / 2) * u_vector_2.y;

    // Dot product of u_vector_1 with u_vector_2 and u_vector_2_90
    float dp = u_vector_1.x * u_vector_2.x + u_vector_1.y * u_vector_2.y;
    float dp_90 = u_vector_1.x * u_vector_2_90.x + u_vector_1.y * u_vector_2_90.y;

    // Comprobating if the angle is over 180 degrees and adapting
    float ret;
    if(dp_90 < 0){ret = acos(dp);}
    else{ret = M_PI + (M_PI-acos(dp));}

    // Returning value
    if (format.compare("radians") == 0) {return ret;}
    else {return (ret*180/M_PI);}
}

float SpecificWorker::calculate_orientation(RoboCompHumanCameraBody::Person person)
{
    RoboCompHumanCameraBody::TJoints person_tjoints = person.joints;
    bool left_found, base_found, right_found = false;
    cv::Point3d base_p, right_p, left_p;

    for(auto item : person_tjoints)
    {
        std::string key = item.first;

        // Base point

        if (base_found == false && (key.compare("17") == 0 || key.compare("6") == 0 || key.compare("5") || key.compare("2") || key.compare("1")))
        {
            base_found = true;
            base_p = dictionary_values_to_3d_point(item.second);
        }

        // Right point

        if (right_found == false && (key.compare("12") == 0 || key.compare("4") == 0))
        {
            right_found = true;
            right_p = dictionary_values_to_3d_point(item.second);
        }

        // Left point

        if (left_found == false && (key.compare("11") == 0 || key.compare("3") == 0))
        {
            left_found = true;
            left_p = dictionary_values_to_3d_point(item.second);
        }
    }

    if(base_found == false || right_found == false || left_found == false)
    {
        cout << "Points not found. Can't calculate orientation." << endl;
        return 0.0;
    }

    // Considering "clavícula" as coordinate center. Passing leg points to "clavícula" reference system

    cv::Point3d left_v = left_p - base_p;
    cv::Point3d right_v = right_p - base_p;

    // Calculating perpendicular vector

    cv::Point3d normal = cross_product(left_v, right_v);
    cv::Point vector_1, vector_2;
    vector_1.x = 0;
    vector_1.y = 1;
    vector_2.x = normal.x;
    vector_2.y = normal.z;
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

