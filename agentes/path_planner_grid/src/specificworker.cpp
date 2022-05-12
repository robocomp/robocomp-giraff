/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/chunked.hpp>
#include <algorithm>
#include <ranges>


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
    std::cout << "Destroying SpecificWorker" << std::endl;
    auto grid_nodes = G->get_nodes_by_type("grid");
    for (auto grid : grid_nodes)
    {
        G->delete_node(grid);
    }
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    std::setlocale(LC_NUMERIC, "C");
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
		//connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_node_attrs_slot);
        //connect(G.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &SpecificWorker::modify_edge_attrs_slot);
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

        //Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // self agent api
        agent_info_api = std::make_unique<DSR::AgentInfoAPI>(G.get());

        // Ignore attributes from G
        G->set_ignored_attributes<cam_rgb_att, cam_depth_att>();

        // 2D widget
        widget_2d = qobject_cast<DSR::QScene2dViewer *>(graph_viewer->get_widget(opts::scene));
        if (widget_2d != nullptr)
        {
            widget_2d->set_draw_laser(true);
            connect(widget_2d, SIGNAL(mouse_right_click(int, int, std::uint64_t)), this, SLOT(new_target_from_mouse(int, int, std::uint64_t)));
        }

        // path planner
        path_planner_initialize(widget_2d);

        // check for existing intention node
        if (auto intentions = G->get_nodes_by_type(intention_type_name); not intentions.empty())
            this->modify_node_slot(intentions.front().id(), "intention");

		this->Period = period;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    static QGraphicsEllipseItem *target_draw = nullptr;
    float x, y;
    // robot pose
    Eigen::Vector3d robot_pose_3d = inner_eigen->transform(world_name, robot_name).value();
    auto robot_rotation_3d = inner_eigen->get_euler_xyz_angles(world_name, robot_name).value();
    robot_pose.ang = robot_rotation_3d.z();
    robot_pose.pos=Eigen::Vector2f(robot_pose_3d.x(), robot_pose_3d.y());
    RoboCompLaser::TLaserData laser_data = read_laser(false);
    std::cout << "1" << std::endl;

    std::cout << "2" << std::endl;

    if(current_plan.get_action() == QString::fromStdString("FOLLOW_PEOPLE"))
    {
        try
        {
            auto person_id = current_plan.get_attribute("person_node_id");
            uint64_t value;
            std::istringstream iss(person_id.toString().toUtf8().constData());
            iss >> value;
            if(auto followed_person_node = G->get_node(value); followed_person_node.has_value())
            {
                if(auto person_pose = inner_eigen->transform(world_name, followed_person_node.value().name()); person_pose.has_value())
                {
                    std::cout << "PERSON POSE: " << person_pose.value() << std::endl;
                    x = person_pose.value().x();
                    y = person_pose.value().y();
                    QPointF target_point(x,y);
                    target.set_pos(target_point);
                }
                else
                {
                    std::cout << "La persona ha desaparecido" << std::endl;
                    return;
                }
            }
            else
            {
                std::cout << "La persona ha desaparecido" << std::endl;
                return;
            }

            // draw target
            if(target_draw != nullptr) delete target_draw;
            target_draw = widget_2d->scene.addEllipse(target.get_pos().x()-100, target.get_pos().y()-100, 200, 200, QPen(QColor("magenta")), QBrush(QColor("magenta")));
            target_draw->setZValue(10);

            auto person_center_dist = sqrt(pow(x - robot_pose.pos.x(), 2) + pow(y - robot_pose.pos.y(), 2));
            if(person_center_dist > 1500) regenerate_grid_to_point(robot_pose);

            std::cout << "DISTANCIA AL CENTRO: " << person_center_dist << std::endl;
        }
        catch(string e)
        {
            std::cout << "Problema siguiendo el plan." << std::endl;
        }

    }

    // Check for new published intention/plan
    if (auto plan_o = plan_buffer.try_get(); plan_o.has_value())
    {
        std::cout << "5" << std::endl;
        current_plan = plan_o.value();
        qInfo() << __FUNCTION__ << "New plan arrived: ";
        current_plan.pprint();
        auto action_name = current_plan.get_action();
        auto follow_action_name = QString::fromStdString("FOLLOW_PEOPLE");
        std::cout << "6" << std::endl;
        if(action_name == follow_action_name)
        {
            auto person_id = current_plan.get_attribute("person_node_id");
            uint64_t value;
            std::istringstream iss(person_id.toString().toUtf8().constData());
            iss >> value;
            if(auto followed_person_node = G->get_node(value); followed_person_node.has_value())
            {
                if(auto person_pose = inner_eigen->transform(world_name, followed_person_node.value().name()); person_pose.has_value())
                {
                    std::cout << "PERSON POSE: " << person_pose.value() << std::endl;
                    x = person_pose.value().x();
                    y = person_pose.value().y();
                }
                else
                {
                    std::cout << "La persona ha desaparecido" << std::endl;
                    return;
                }
            }
            else
            {
                std::cout << "La persona ha desaparecido" << std::endl;
                return;
            }


        }
        else
        {
            x = current_plan.get_attribute("x").toFloat();
            y = current_plan.get_attribute("y").toFloat();
        }

        QPointF target_point(x,y);
        target.set_pos(target_point);

        // draw target
        if(target_draw != nullptr) delete target_draw;
        target_draw = widget_2d->scene.addEllipse(target.get_pos().x()-100, target.get_pos().y()-100, 200, 200, QPen(QColor("magenta")), QBrush(QColor("magenta")));
        target_draw->setZValue(10);
        std::cout << "7" << std::endl;
        regenerate_grid_to_point(robot_pose);
        std::cout << "8" << std::endl;
        update_map(laser_data);
        run_current_plan();
        std::cout << "9" << std::endl;
    }
    else if(target.active)
    {
        std::cout << "10" << std::endl;
        update_map(laser_data);
        run_current_plan();
        std::cout << "11" << std::endl;
    }
    else //do whatever you do without a plan
    {}

//    update_map(const RoboCompLaser::TLaserData &ldata)

//    QRectF dim(-5000, -2500, 10000, 5000);
//    grid.initialize(dim, constants.tile_size, &widget_2d->scene, false, std::string(),
//                    grid_world_pose.toQpointF(), grid_world_pose.ang);

//    if (widget_2d != nullptr)
//        grid.draw(&widget_2d->scene);
//    update_grid();
}

void SpecificWorker::path_planner_initialize(DSR::QScene2dViewer* widget_2d)
{
    QRectF outerRegion;
    auto world_node = G->get_node(world_name).value();
    outerRegion.setLeft(G->get_attrib_by_name<OuterRegionLeft_att>(world_node).value());
    outerRegion.setTop(G->get_attrib_by_name<OuterRegionTop_att>(world_node).value());
    outerRegion.setRight(G->get_attrib_by_name<OuterRegionRight_att>(world_node).value());
    outerRegion.setBottom(G->get_attrib_by_name<OuterRegionBottom_att>(world_node).value());
    if(outerRegion.isNull())
    {
        qWarning() << __FILE__ << __FUNCTION__ << "Outer region of the scene could not be found in G. Aborting";
        std::terminate();
    }

    robotBottomLeft     = Mat::Vector3d ( -robotXWidth / 2, robotZLong / 2, 0);
    robotBottomRight    = Mat::Vector3d ( - robotXWidth / 2,- robotZLong / 2, 0);
    robotTopRight       = Mat::Vector3d ( + robotXWidth / 2, - robotZLong / 2, 0);
    robotTopLeft        = Mat::Vector3d ( + robotXWidth / 2, + robotZLong / 2, 0);
}

RoboCompLaser::TLaserData SpecificWorker::read_laser(bool noise)
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::normal_distribution<double> normal_dist(0.0, constants.lidar_noise_sigma);
    RoboCompLaser::TLaserData ldata;
    try
    {
        if(auto laser_node = G->get_node("laser"); laser_node.has_value())
        {
            if (auto laser_dist = G->get_attrib_by_name<laser_dists_att>(laser_node.value()); laser_dist.has_value() && laser_dist.value().get().size() > 0)
            {
                auto dists = laser_dist.value().get();
                if (auto laser_angle = G->get_attrib_by_name<laser_angles_att>(laser_node.value()); laser_angle.has_value() && laser_angle.value().get().size() > 0)
                {
                    auto angles = laser_angle.value().get();
                    // Build raw polygon
                    for (int i = 0; i < dists.size(); ++i)
                    {
                        RoboCompLaser::TData act_data;
                        act_data.dist = dists[i]; act_data.angle = angles[i];
                        ldata.push_back(act_data);
                    }
                    // get n random angles to apply hard noise on them
                    static std::uniform_int_distribution<int> unif_dist(0, ldata.size());
                    static std::uniform_int_distribution<int> accept_dist(0, 10);
                    static auto generator = std::bind(unif_dist, mt);
                    std::vector<int> samples(constants.num_lidar_affected_rays_by_hard_noise, unif_dist(mt));
                    std::generate_n(samples.begin(), constants.num_lidar_affected_rays_by_hard_noise, generator);
                    for(auto &s: samples)
                        if(accept_dist(mt) < 3)
                            ldata[s].dist /= 3;
                    return ldata;
                }
            }
        }
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    return ldata;
}

void SpecificWorker::update_map(const RoboCompLaser::TLaserData &ldata)
{
    // get the matrix to transform from robot to local_grid
    Eigen::Matrix3f r2g = from_robot_to_grid_matrix();

//    auto robot_in_grid = from_world_to_grid(Eigen::Vector2f(robot_pose.pos.x(), robot_pose.pos.y()));
//    std::cout << "ROBOT POS: " << robot_in_grid << std::endl;
    for(const auto &l : ldata)
    {
        if(l.dist > constants.robot_semi_length)
        {
            // transform tip form robot's RS to local_grid RS
            Eigen::Vector2f tip = (r2g * Eigen::Vector3f(l.dist*sin(l.angle), l.dist*cos(l.angle), 1.f)).head(2);
//            Eigen::Vector2f tip = (Eigen::Vector3f(l.dist*sin(l.angle), l.dist*cos(l.angle), 1.f)).head(2);
            int num_steps = ceil(l.dist/(constants.tile_size));
            Eigen::Vector2f p;
            for(const auto &&step : iter::range(0.0, 1.0-(1.0/num_steps), 1.0/num_steps))
            {
                p = robot_pose.pos * (1-step) + tip*step;
                grid.add_miss(p);
            }
            if(l.dist <= constants.max_laser_range)
                grid.add_hit(tip);

            if((p-tip).norm() < constants.tile_size)  // in case las miss overlaps tip
                grid.add_hit(tip);
        }
    }
    grid.update_costs();
}

bool SpecificWorker::regenerate_grid_to_point(const Pose2D &robot_pose)
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::normal_distribution<double> normal_dist(0.0, constants.target_noise_sigma);
    try
    {
        QLineF r_to_target(QPointF(target.get_pos().x(), target.get_pos().y()), QPointF(robot_pose.pos.x(), robot_pose.pos.y()));
        auto t = r_to_target.pointAt(constants.final_distance_to_target / r_to_target.length());
        target.set_pos(t + QPointF(normal_dist(mt), normal_dist(mt)));              // Adding noise to target
        if(target.draw != nullptr) widget_2d->scene.removeItem(target.draw);
        target.active = true;
        target.draw = widget_2d->scene.addEllipse(target.get_pos().x()-100, target.get_pos().y()-100, 200, 200, QPen(QColor("magenta")), QBrush(QColor("magenta")));

        // create local grid for mission
        // if new target has changed enough, replace local grid
        QPointF t_in_grid = e2q(from_world_to_grid(target.to_eigen()));
            Eigen::Vector2f t_r = from_world_to_robot(target.to_eigen());
            float dist_to_robot = t_r.norm();
            //    qInfo() << __FUNCTION__ << dist_to_robot_1 << dist_to_robot << dist_to_robot_2;
            QRectF dim(-2000, -500, 4000, dist_to_robot + 2000);
            grid_world_pose = {.ang=-atan2(t_r.x(), t_r.y()) + robot_pose.ang, .pos=robot_pose.pos};
            grid.initialize(dim, constants.tile_size, &widget_2d->scene, false, std::string(),
                            grid_world_pose.toQpointF(), grid_world_pose.ang);
            inject_grid_in_G(grid);
    }
    catch(const Ice::Exception &e)
    {
        qInfo() << "Error connecting to Bill Coppelia";
        std::cout << e.what() << std::endl;
        return false;
    }
    return true;
}

void SpecificWorker::run_current_plan()
{
    Eigen::Vector3d nose_3d = inner_eigen->transform(world_name, Mat::Vector3d(0, 380, 0), robot_name).value();
//    auto valid_target = search_a_feasible_target(current_plan);
//    if( valid_target.has_value())
//    {
//        std::cout <<  "ROBOT: x: " << nose_3d.x() << " y: " << nose_3d.y() << std::endl;
//        std::cout <<  "POINT: x: " << valid_target->x() << " y: " << valid_target->y() << std::endl;

//        auto path = grid.compute_path(e2q(from_world_to_grid(robot_pose.pos)), e2q(from_world_to_grid(target.to_eigen())));
        auto path = grid.compute_path(e2q(from_world_to_grid(robot_pose.pos)), e2q(from_world_to_grid(target.to_eigen())));
        qInfo() << __FUNCTION__ << " Path computed of size: " << path.size();

        // add path to G
        if (not path.empty())
        {
            std::vector<float> x_values, y_values;
            x_values.reserve(path.size());
            y_values.reserve(path.size());
            for (auto &&p : path)
            {
                x_values.push_back(from_grid_to_world(p).x());
                y_values.push_back(from_grid_to_world(p).y());
            }
            if (widget_2d != nullptr)
                draw_path(path);

            if (auto path = G->get_node(current_path_name); path.has_value())
            {
                auto path_to_target_node = path.value();
                G->add_or_modify_attrib_local<path_x_values_att>(path_to_target_node, x_values);
                G->add_or_modify_attrib_local<path_y_values_att>(path_to_target_node, y_values);
                G->add_or_modify_attrib_local<path_target_x_att>(path_to_target_node, (float) target.get_pos().x());
                G->add_or_modify_attrib_local<path_target_y_att>(path_to_target_node, (float) target.get_pos().y());
                G->update_node(path_to_target_node);
            }
            else // create path_to_target_node with the solution path
            {
                if(auto intention = G->get_node(current_intention_name); intention.has_value())
                {
                    auto path_to_target_node = DSR::Node::create<path_to_target_node_type>(current_path_name);
                    G->add_or_modify_attrib_local<path_x_values_att>(path_to_target_node, x_values);
                    G->add_or_modify_attrib_local<path_y_values_att>(path_to_target_node, y_values);
                    G->add_or_modify_attrib_local<pos_x_att>(path_to_target_node, (float) -542);
                    G->add_or_modify_attrib_local<pos_y_att>(path_to_target_node, (float) 106);
                    G->add_or_modify_attrib_local<parent_att>(path_to_target_node, intention.value().id());
                    G->add_or_modify_attrib_local<level_att>(path_to_target_node, 3);
                    G->add_or_modify_attrib_local<path_target_x_att>(path_to_target_node, (float) target.get_pos().x());
                    G->add_or_modify_attrib_local<path_target_y_att>(path_to_target_node, (float) target.get_pos().y());
                    auto id = G->insert_node(path_to_target_node);
                    DSR::Edge edge_to_intention = DSR::Edge::create<thinks_edge_type>(id.value(), intention.value().id());
                    G->insert_or_assign_edge(edge_to_intention);
                }
                else qWarning() << __FUNCTION__ << "No intention node found. Can't create path node";
            }
        }
        else qWarning() << __FUNCTION__ << "Empty path. No path found for target despite all efforts";
//    }
//    else qWarning() << __FUNCTION__ << "No free point found close to target";
}

Eigen::Vector2f SpecificWorker::from_world_to_grid(const Eigen::Vector2f &p)
{
    // build the matrix to transform from world to local_grid, knowing robot and grid pose in world
    Eigen::Matrix2f w2g;
    w2g <<  cos(grid_world_pose.ang), sin(grid_world_pose.ang),
            -sin(grid_world_pose.ang), cos(grid_world_pose.ang);
    return w2g * (p - grid_world_pose.pos);
}

Eigen::Vector2f SpecificWorker::from_world_to_robot(const Eigen::Vector2f &p)
{
    Eigen::Matrix2f matrix;
    matrix << cos(robot_pose.ang) , -sin(robot_pose.ang) , sin(robot_pose.ang) , cos(robot_pose.ang);
    return (matrix.transpose() * (p - robot_pose.pos));
}

Eigen::Vector2f SpecificWorker::from_robot_to_world(const Eigen::Vector2f &p)
{
    Eigen::Matrix2f matrix;
    matrix << cos(robot_pose.ang) , -sin(robot_pose.ang) , sin(robot_pose.ang) , cos(robot_pose.ang);
    return (matrix * p) + robot_pose.pos;
}

Eigen::Vector2f SpecificWorker::from_grid_to_world(const Eigen::Vector2f &p)
{
    // build the matrix to transform from grid to world knowing robot and grid pose in world
    Eigen::Matrix2f g2w;
    g2w <<  cos(grid_world_pose.ang), -sin(grid_world_pose.ang),
            sin(grid_world_pose.ang), cos(grid_world_pose.ang);
    return g2w * p + grid_world_pose.pos;
}

Eigen::Matrix3f SpecificWorker::from_robot_to_grid_matrix()
{
    // build the matrix to transform from robot to local_grid, knowing robot and grid pose in world
    Eigen::Matrix3f r2w;
    r2w <<  cos(robot_pose.ang), -sin(robot_pose.ang), robot_pose.pos.x(),
            sin(robot_pose.ang) , cos(robot_pose.ang), robot_pose.pos.y(),
            0.f, 0.f, 1.f;
    Eigen::Matrix2f w2g_2d_matrix;
    w2g_2d_matrix <<  cos(grid_world_pose.ang), sin(grid_world_pose.ang),
            -sin(grid_world_pose.ang), cos(grid_world_pose.ang);
    auto tr = w2g_2d_matrix * grid_world_pose.pos;
    Eigen::Matrix3f w2g;
    w2g << cos(grid_world_pose.ang), sin(grid_world_pose.ang), -tr.x(),
            -sin(grid_world_pose.ang), cos(grid_world_pose.ang), -tr.y(),
            0.f, 0.f, 1.f;
    Eigen::Matrix3f r2g = w2g * r2w;  // from r to world and then from world to grid
    return r2g;
}
Eigen::Matrix3f SpecificWorker::from_grid_to_robot_matrix()
{
    return from_robot_to_grid_matrix().inverse();
}

void SpecificWorker::update_grid()
{
    if (grid_updated)
    {
        std::cout << __FUNCTION__ << " Updating grid ---------------------------------------" << std::endl;
        if (auto node = G->get_node(current_grid_name); node.has_value())
        {
            if (auto grid_as_string = G->get_attrib_by_name<grid_as_string_att>(node.value()); grid_as_string.has_value())
                grid.readFromString(grid_as_string.value());
        }
//        if (widget_2d != nullptr)
//        {
//            cout << "drawing grid" << endl;
//            grid.draw(&widget_2d->scene);
//        }
        grid_updated = false;
    }
}

std::optional<QPointF> SpecificWorker::search_a_feasible_target(Plan &current_plan)
{
    QPointF target(current_plan.get_attribute("x").toFloat(), current_plan.get_attribute("y").toFloat());
    auto new_target = grid.closest_free_4x4(target);
    qInfo() << __FUNCTION__ << "requested target " << target << " new target " << new_target.value();
    return new_target;
    // ad to GRID, closest free cell in the direction of the robot.
}

void SpecificWorker::inject_grid_in_G(const Grid &grid)
{
    std::string grid_as_string = grid.saveToString();
    if (auto current_grid_node_o = G->get_node(current_grid_name); current_grid_node_o.has_value())
    {
        G->add_or_modify_attrib_local<grid_as_string_att>(current_grid_node_o.value(), grid_as_string);
        G->update_node(current_grid_node_o.value());
    }
    else
    {
        if (auto mind = G->get_node(robot_mind_name); mind.has_value())
        {
            DSR::Node current_grid_node = DSR::Node::create<grid_node_type>(current_grid_name);
            G->add_or_modify_attrib_local<name_att>(current_grid_node, current_grid_name);
            G->add_or_modify_attrib_local<parent_att>(current_grid_node, mind.value().id());
            G->add_or_modify_attrib_local<level_att>(current_grid_node, G->get_node_level(mind.value()).value() + 1);
            G->add_or_modify_attrib_local<grid_as_string_att>(current_grid_node, grid_as_string);
            G->add_or_modify_attrib_local<pos_x_att>(current_grid_node, (float) -262);
            G->add_or_modify_attrib_local<pos_y_att>(current_grid_node, (float) 91);
            if (std::optional<int> current_grid_node_id = G->insert_node(current_grid_node); current_grid_node_id.has_value())
            {
                DSR::Edge edge = DSR::Edge::create<has_edge_type>(mind.value().id(), current_grid_node.id());
                if (G->insert_or_assign_edge(edge))
                    std::cout << __FUNCTION__ << "Edge successfully created between robot and grid" << std::endl;
                else
                {
                    std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting new edge: " << mind.value().id() << "->" << current_grid_node_id.value()
                              << " type: has" << std::endl;
                    std::terminate();
                }
            } else
            {
                std::cout << __FILE__ << __FUNCTION__ << " Fatal error inserting_new 'grid' node" << std::endl;
                std::terminate();
            }
        }
    }
}

void SpecificWorker::new_target_from_mouse(int pos_x, int pos_y, std::uint64_t id)
{
    qInfo() << __FUNCTION__ << "[" << pos_x << " " << pos_y << "] Id:" << id;

    if(auto node = G->get_node(id); node.has_value())
    {
        Plan plan(Plan::Actions::GOTO);
        plan.insert_attribute("x", pos_x);
        plan.insert_attribute("y", pos_y);
        plan.insert_attribute("destiny", QString::fromStdString(node.value().name()));
        if (plan.is_complete())
        {
            std::cout << plan.pprint() << " " << plan.to_json() << std::endl;
            if (auto mind = G->get_node(robot_mind_name); mind.has_value())
            {
                // Check if there is not 'intention' node yet in G
                if (auto intention = G->get_node(current_intention_name); not intention.has_value())
                {
                    DSR::Node intention_node = DSR::Node::create<intention_node_type>(current_intention_name);
                    G->add_or_modify_attrib_local<parent_att>(intention_node, mind.value().id());
                    G->add_or_modify_attrib_local<level_att>(intention_node, G->get_node_level(mind.value()).value() + 1);
                    G->add_or_modify_attrib_local<pos_x_att>(intention_node, (float) -440);
                    G->add_or_modify_attrib_local<pos_y_att>(intention_node, (float) 13);
                    G->add_or_modify_attrib_local<current_intention_att>(intention_node, plan.to_json());
                    if (std::optional<int> intention_node_id = G->insert_node(intention_node); intention_node_id.has_value())
                    {
                        DSR::Edge edge = DSR::Edge::create<has_edge_type>(mind.value().id(), intention_node.id());
                        if (G->insert_or_assign_edge(edge))
                            std::cout << __FUNCTION__ << " Edge successfully inserted: " << mind.value().id() << "->"
                                      << intention_node_id.value()
                                      << " type: has" << std::endl;
                        else
                        {
                            std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << mind.value().id() << "->"
                                      << intention_node_id.value()
                                      << " type: has" << std::endl;
                            std::terminate();
                        }
                    } else
                    {
                        std::cout << __FUNCTION__ << ": Fatal error inserting_new 'intention' node" << std::endl;
                        std::terminate();
                    }
                } else
                {
                    std::cout << __FUNCTION__ << ": Updating existing intention node with Id: " << intention.value().id()
                              << std::endl;
                    G->add_or_modify_attrib_local<current_intention_att>(intention.value(), plan.to_json());
                    G->update_node(intention.value());
                    std::cout << __FUNCTION__ << " INSERT: " << plan.to_json() << std::endl;
                }
            } else
            {
                std::cout << __FUNCTION__ << " No node " << robot_mind_name << " found in G" << std::endl;
                std::terminate();
            }
        }
    } else
        std::cout << __FUNCTION__ << " No target object node " << node.value().name() << " found in G" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////777
void SpecificWorker::modify_node_slot(const std::uint64_t id, const std::string &type)
{
    if (type == intention_type_name)
    {
        std::cout << "TIPO: " << type << std::endl;
        std::cout << "NEW PLAN" << std::endl;
        if (auto intention = G->get_node(id); intention.has_value())
        {
            std::optional<std::string> plan = G->get_attrib_by_name<current_intention_att>(intention.value());
            if (plan.has_value())
            {
                qInfo() << __FUNCTION__ << QString::fromStdString(plan.value()) << " " << intention.value().id();
                Plan my_plan(plan.value());
                if (my_plan.is_action(Plan::Actions::GOTO) || my_plan.is_action(Plan::Actions::FOLLOW_PEOPLE))
                    plan_buffer.put(std::move(my_plan));
            }
        }

    }
    else if (type == grid_type_name)
    {
        grid_updated = true;
    }
}

void SpecificWorker::del_node_slot(std::uint64_t from)
{
    qInfo() << __FUNCTION__ << "Node " << from << " deleted";
    qInfo() << __FUNCTION__ << " Waiting for a new goal";
}

///////////////////////////////////////////////////
/// GUI
//////////////////////////////////////////////////

void SpecificWorker::draw_path(const std::vector<Eigen::Vector2f> &path_in_robot)
{
    static std::vector<QGraphicsItem *> path_paint;
    static QString path_color = "Blue";

    for(auto p : path_paint)
        widget_2d->scene.removeItem(p);
    path_paint.clear();

    uint s = 100;
    for(auto &&p : path_in_robot)
    {
        auto pw = from_grid_to_world(p);  // in mm
        path_paint.push_back(widget_2d->scene.addEllipse(pw.x()-s/2, pw.y()-s/2, s , s, QPen(path_color), QBrush(QColor(path_color))));
        path_paint.back()->setZValue(30);
    }
}

/////////////////////////////////////////////////////////////////////////

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

