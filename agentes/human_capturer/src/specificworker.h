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
#define M_PI 3.14159265358979323846
#include <genericworker.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
#include <opencv2/opencv.hpp>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
    struct PersonData
    {
        int id;
        RoboCompHumanCameraBody::TJoints joints;
        // TImage roi;
        vector<float> personCoords;
        float orientation;
    };


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
    void modify_node_slot(std::uint64_t, const std::string &type){};
    void modify_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
    void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};

    void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
    void del_node_slot(std::uint64_t from){};
    bool startup_check_flag;
    std::shared_ptr<DSR::RT_API> rt_api;




    // Functions

    RoboCompHumanCameraBody::PeopleData test_person();
    std::int32_t increase_lambda_cont(std::int32_t lambda_cont);
    std::int32_t decrease_lambda_cont(std::int32_t lambda_cont);
    cv::Point3f dictionary_values_to_3d_point(auto item);
    cv::Point3f cross_product(cv::Point3f p1, cv::Point3f p2);
    float get_degrees_between_vectors(cv::Point vector_1, cv::Point vector_2, std::string format);
    float calculate_orientation(RoboCompHumanCameraBody::Person person);
    float distance_3d(cv::Point3f p1, cv::Point3f p2);
    void remove_person(DSR::Node person_node, bool direct_remove); // direct_remove = false in python
    void update_person(DSR::Node node, std::vector<float> coords, float orientation);
    void insert_mind(std::uint64_t parent_id, std::int32_t person_id);
    void insert_person(std::vector<float> coords, float orientation, bool direct_insert);
    void update_graph(vector<SpecificWorker::PersonData> people_list);
    QVector3D get_person_coords(RoboCompHumanCameraBody::Person p);
    float dot_product3D(cv::Point3f vector_a, cv::Point3f vector_b);
    float dot_product(cv::Point vector_a, cv::Point vector_b);
};

#endif
