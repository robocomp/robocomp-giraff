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
#include <cppitertools/filter.hpp>
#include <cppitertools/reversed.hpp>
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
//    auto grid_nodes = G->get_nodes_by_type("grid");
//    for (auto grid : grid_nodes)
//    {
//        G->delete_node(grid);
//    }
//	G.reset();
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
	this->Period = 300;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);

        rt = G->get_rt_api();
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//dsr update signals
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
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

        if(auto grid_node = G->get_node(grid_type_name); grid_node.has_value())
        {
            if(auto robot_node = G->get_node(robot_name); robot_node.has_value())
            {
                if(auto robot_pos = inner_eigen->transform(grid_type_name, robot_name); robot_pos.has_value())
                {
//                    auto edge = rt->get_edge_RT(room_node.value(), robot_node->id()).value();
//                    if(auto robot_grid_pos = G->get_attrib_by_name<rt_translation_grid_att>(edge); robot_grid_pos.has_value())
//                    {
                        if(auto robot_ang = inner_eigen->get_euler_xyz_angles(grid_type_name, robot_name); robot_ang.has_value())
                        {
                            robot_pose.set_pos(Eigen::Vector2f{robot_pos.value().x(), robot_pos.value().y()});
//                            robot_pose.set_grid_pos(Eigen::Vector2f{robot_grid_pos.value().get()[0], robot_grid_pos.value().get()[1]});
                            robot_pose.set_angle(robot_ang.value().z());
                        }
//                    }
                }
            }
        }

        // check for existing intention node
        if (auto intentions = G->get_nodes_by_type(intention_type_name); not intentions.empty())
            this->modify_node_slot(intentions.front().id(), "intention");

		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::normal_distribution<double> normal_dist(0.0, constants.target_noise_sigma);

    static QGraphicsEllipseItem *target_draw = nullptr;

    // Variables for destiny pose
    float x, y;

    auto [laser_polygon, laser_data] = read_laser(false);

    // Check for new published intention/plan
    if (auto plan_o = plan_buffer.try_get(); plan_o.has_value())
    {
        current_plan = plan_o.value();
        qInfo() << __FUNCTION__ << "New plan arrived: ";
        current_plan.pprint();
        auto action_name = current_plan.get_action();
        auto follow_action_name = QString::fromStdString("FOLLOW_PEOPLE");
        qInfo() << __FUNCTION__ << "1";
        if(action_name == follow_action_name)
        {
            auto person_id = current_plan.get_attribute("person_node_id");
            qInfo() << __FUNCTION__ << "2";
            uint64_t value;
            std::istringstream iss(person_id.toString().toUtf8().constData());
            iss >> value;
            followed_person_id = value;
            qInfo() << __FUNCTION__ << "3";
            if(auto followed_person_node = G->get_node(value); followed_person_node.has_value())
            {
                qInfo() << __FUNCTION__ << "4";
                if(auto person_pose = inner_eigen->transform(robot_name, followed_person_node.value().name()); person_pose.has_value())
                {
                    qInfo() << __FUNCTION__ << "5";
                    x = person_pose.value().x();
                    y = person_pose.value().y();
                    qInfo() << __FUNCTION__ << "6";
                }
                else
                {
                    return;
                }
            }
            else
            {
                return;
            }
        }

        // If is not following...
        else
        {
            x = current_plan.get_attribute("x").toFloat();
            y = current_plan.get_attribute("y").toFloat();
        }
        qInfo() << __FUNCTION__ << "7";
        QLineF r_to_target(QPointF(x, y), robot_pose.to_qpoint());
        auto t = r_to_target.pointAt(constants.final_distance_to_target / r_to_target.length());
        t += QPointF(normal_dist(mt), normal_dist(mt));              // Adding noise to target
        target.set_pos(Eigen::Vector2f{t.x(), t.y()});
        last_relevant_person_pos = target.get_pos();
        qInfo() << __FUNCTION__ << "8";
        // draw target
        if(target_draw != nullptr) delete target_draw;
        target_draw = widget_2d->scene.addEllipse(target.get_pos().x()-100, target.get_pos().y()-100, 200, 200, QPen(QColor("magenta")), QBrush(QColor("magenta")));
        target_draw->setZValue(10);
        qInfo() << __FUNCTION__ << "9";
        target.set_active(true);
        // If grid is activated, path should be created with grid.compute_path
        if(auto grid_activated = G->get_attrib_by_name<grid_activated_att>(G->get_node(grid_type_name).value()); grid_activated.has_value() and grid_activated.value())
        {
            regenerate_grid_to_point(robot_pose, true);
            update_map(laser_data);
            run_current_plan(laser_polygon, true);
        }
        else
        {
            qInfo() << __FUNCTION__ << "10";
            run_current_plan(laser_polygon);
        }
    }

    else if(target.is_active())
    {
        if(current_plan.get_action() == QString::fromStdString("FOLLOW_PEOPLE"))
        {
            try
            {
                // draw target
                if(target_draw != nullptr) delete target_draw;
                target_draw = widget_2d->scene.addEllipse(target.get_pos().x()-100, target.get_pos().y()-100, 200, 200, QPen(QColor("magenta")), QBrush(QColor("magenta")));
                target_draw->setZValue(10);

                if(auto grid_activated = G->get_attrib_by_name<grid_activated_att>(G->get_node(grid_type_name).value()); grid_activated.value())
                {
                    auto is_person_in_polygon = person_in_grid_checker();
                    if(not is_person_in_polygon)
                    {
                        regenerate_grid_to_point(robot_pose, true);
                    }
                    update_map(laser_data);
                    run_current_plan(laser_polygon, true);
                }
                else
                {
                    qInfo() << __FUNCTION__ << "NO GRID";
                    run_current_plan(laser_polygon);
                }
            }
            catch(string e)
            {
                std::cout << "Problema siguiendo el plan." << std::endl;
            }

        }
    }
    else //do whatever you do without a plan
    {
//        qInfo() << __FUNCTION__ << "NO PLAN";
//        if(!exist_grid)
//        {
//            actual_room_id = G->get_attrib_by_name<parent_att>(G->get_node(robot_name).value()).value();
//            regenerate_grid_to_point(robot_pose);
//            exist_grid = true;
//        }
//
//        else
//        {
//            qInfo() << __FUNCTION__ << "Checking if robot in grid";
//            auto is_person_in_polygon = person_in_grid_checker(false);
//            if(not is_person_in_polygon)
//            {
//                std::cout <<"ENTRA" << std::endl;
//                regenerate_grid_to_point(robot_pose);
////        grid.set_all_to_free();
//
//            }
//        }
    }
}

Eigen::Vector2f SpecificWorker::target_before_objetive(Eigen::Vector2f robot_pose, Eigen::Vector2f target_pose)
{
    auto vector_dir = (target_pose - robot_pose) / 10;
    Eigen::Vector2f target = robot_pose + vector_dir;
    Eigen::Vector2f point;
    for (int i = 0; i < 10; ++i)
    {
        point = robot_pose + (vector_dir * i);
        if ((target_pose - point).norm() > ((target_pose - robot_pose).norm()) / 5)
            target = point;
        else
            break;
    }
    return target;
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
std::tuple<QPolygonF,RoboCompLaser::TLaserData> SpecificWorker::read_laser(bool noise)
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::normal_distribution<double> normal_dist(0.0, constants.lidar_noise_sigma);
    QPolygonF poly_robot;
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
                    for (std::size_t i = 0; i < dists.size(); ++i)
                    {
                        RoboCompLaser::TData act_data;
                        act_data.dist = dists[i]; act_data.angle = angles[i];
                        poly_robot << QPointF(act_data.dist * sin(act_data.angle), act_data.dist * cos(act_data.angle));
                        ldata.push_back(act_data);
                    }

                    poly_robot.pop_front();
                    poly_robot.push_front(QPointF(constants.robot_semi_length*1.1, -constants.robot_semi_length));
                    poly_robot.push_front(QPointF(constants.robot_semi_length*1.1, -constants.robot_length));
                    poly_robot.push_front(QPointF(0, -constants.robot_length));
                    poly_robot.pop_back();
                    poly_robot.push_back(QPointF(-constants.robot_semi_length*1.1, -constants.robot_semi_length));
                    poly_robot.push_back(QPointF(-constants.robot_semi_length*1.1, -constants.robot_length));
                    poly_robot.push_back(QPointF(0, -constants.robot_length));

                    // get n random angles to apply hard noise on them
                    static std::uniform_int_distribution<int> unif_dist(0, ldata.size());
                    static std::uniform_int_distribution<int> accept_dist(0, 10);
                    static auto generator = std::bind(unif_dist, mt);
                    std::vector<int> samples(constants.num_lidar_affected_rays_by_hard_noise, unif_dist(mt));
                    std::generate_n(samples.begin(), constants.num_lidar_affected_rays_by_hard_noise, generator);
                    for(auto &s: samples)
                        if(accept_dist(mt) < 3)
                            ldata[s].dist /= 3;
                    return std::make_tuple(poly_robot, ldata);
                }
            }
        }
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    return std::make_tuple(poly_robot, ldata);
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
                p = robot_pose.get_pos() * (1-step) + tip*step;
                grid.add_miss(p);
            }
            if(l.dist <= constants.max_laser_range)
                grid.add_hit(tip);

            if((p-tip).norm() < constants.tile_size)  // in case las miss overlaps tip
                grid.add_hit(tip);
        }
    }
    grid.update_costs();
//    grid.
}
vector<std::pair <Grid::Key, Grid::T>> SpecificWorker::get_grid_already_occupied_cells()
{
    vector<std::pair <Grid::Key, Grid::T>> occupied_cells_vector;
    // Get occupìed cells in the past grid
    for(auto &[key, val] : iter::filter( [](auto v){return not v.second.free;}, grid))
    {
        Grid::Key aux_key;
        Eigen::Vector2f occupied_pos(key.x, key.z);
        // Convert each cell to world, and from world to new grid
        occupied_pos = from_grid_to_world(occupied_pos);
        aux_key.x = occupied_pos.x(); aux_key.z = occupied_pos.y();
        auto occupied_cell = std::make_pair(aux_key, val);
        occupied_cells_vector.push_back(occupied_cell);
    }
    qInfo() << __FUNCTION__ << occupied_cells_vector.size();
    return occupied_cells_vector;
}
void SpecificWorker::insert_last_occupied_cells(const vector<std::pair <Grid::Key, Grid::T>> &last_cells)
{
    QGraphicsRectItem act_grid_dim(-2000, -500, 4000, act_grid_dist_to_robot + 2000);
    act_grid_dim.setPos(grid_world_pose.to_qpoint());
    act_grid_dim.setRotation(grid_world_pose.get_ang() *  180/ M_PI);
    int counter = 0;
    for(const auto &cell : last_cells)
    {
        Eigen::Vector2f cell_point(cell.first.x, cell.first.z);
        cell_point = from_world_to_grid(cell_point);
        QPointF cell_point_converted (cell_point.x(), cell_point.y());
        if(act_grid_dim.contains(cell_point_converted))
        {
            counter++;
//            bool found = false;
            for(auto grid_cell : grid)
            {
                if(grid_cell.first.x == cell_point.x() && grid_cell.first.z == cell_point.y())
                {
                    grid_cell.second.free = cell.second.free;
                    grid_cell.second.visited = cell.second.visited;
                    grid_cell.second.cost = cell.second.cost;
                    grid_cell.second.hits = cell.second.hits;
                    grid_cell.second.misses = cell.second.misses;
                    grid_cell.second.log_odds = cell.second.log_odds;
//                    found = true;
                    break;
                }
            }
        }
    }
//    std::cout << "COUNTER: " << counter << std::endl;
}

bool SpecificWorker::regenerate_grid_to_point(const Pose2D &robot_pose, bool with_leader)
{
    try
    {
        grid_world_pose.set_pos(robot_pose.get_pos());
        qInfo() << __FUNCTION__ << "GRID WORLD POSE:" << grid_world_pose.get_pos().x() << grid_world_pose.get_pos().y() ;
        if(with_leader)
        {
            Eigen::Vector2f t_r = from_world_to_robot(target.get_pos());
            float dist_to_robot = t_r.norm();
            act_grid_dist_to_robot = dist_to_robot;
//            qInfo() << __FUNCTION__ << (int)dist_to_robot;
            QRectF dim(-2000, -500, 4000, (int)dist_to_robot + 2000);
            act_grid = dim;
            grid_world_pose.set_angle(-atan2(t_r.x(), t_r.y()) + robot_pose.get_ang());
            grid.initialize(dim, constants.tile_size, &widget_2d->scene, false, std::string(),
                            grid_world_pose.to_qpoint(), grid_world_pose.get_ang());
//            inject_grid_data_in_G(vector<float>{-2000, -500, 4000, (int)dist_to_robot + 2000});
        }
        else
        {
            QRectF dim(-3000, -3000, 6000, 6000);
            grid_world_pose.set_angle(robot_pose.get_ang());
            grid.initialize(dim, constants.tile_size, &widget_2d->scene, false, std::string(),
                            grid_world_pose.to_qpoint(), grid_world_pose.get_ang());
//            inject_grid_data_in_G(vector<float>{-2000, -2000, 4000, 4000});
        }


//         (&widget_2d->scene);
//        auto accopied_cells = get_grid_already_occupied_cells();

//        insert_last_occupied_cells(accopied_cells);
        if(!path.empty())
        {
            path.clear();
            for(const auto &path_point : world_path)
            {
                path.push_back(from_world_to_grid(path_point));
            }
        }
    }
    catch(const Ice::Exception &e)
    {
        qInfo() << "Error connecting to Bill Coppelia";
        std::cout << e.what() << std::endl;
        return false;
    }
    return true;
}
// Adds a path that consists in the last path position to the actual person or element to follow position
std::vector<Eigen::Vector2f> SpecificWorker::add_path_section_to_person(std::vector<Eigen::Vector2f> ref_path)
{
    auto last_path_point = ref_path.back();
//    auto grid_target_pose = from_world_to_grid(target.get_pos());
    auto destination_pose = target_before_objetive(last_path_point, target.get_pos());
    auto new_path = grid.compute_path(e2q(last_path_point), e2q(destination_pose), constants.robot_length);
//    auto new_path = grid.compute_path(e2q(from_world_to_grid(robot_pose.pos)), e2q(from_world_to_grid(target.to_eigen())), constants.robot_length/2);

    if(new_path.size() > 0)
        ref_path.insert(ref_path.end(), new_path.begin()+1, new_path.end());
    return ref_path;
}
// Check if any obstacle exists near or in the actual path
bool SpecificWorker::check_path(std::vector<Eigen::Vector2f> ref_path, QPolygonF laser_poly)
{
    for(const auto &path_point : ref_path)
    {
        Grid::Key path_point_to_key(path_point.x(), path_point.y());
        QPointF point_to_q(path_point.x(), path_point.y());
        auto neighboors_from_point = grid.neighboors_16(path_point_to_key);
        if(neighboors_from_point.size() < 16)
        {
            qInfo() << "||||||||||||||||||||||||||| OBSTACLE |||||||||||||||||||||||||||";
            return false;
        }
//        if(not laser_poly.containsPoint(point_to_q, Qt::OddEvenFill))
//        {
//            qInfo() << "||||||||||||||||||||||||||| NOT IN LASER |||||||||||||||||||||||||||";
//            return false;
//        }
    }
    return true;
}

void SpecificWorker::path_smoother(std::vector<Eigen::Vector2f> ref_path)
{
    RoboCompPathSmoother::Path path_converted(ref_path.size());
    for(auto&& [i,path_point] : iter::enumerate(ref_path))
        path_converted[i] = RoboCompPathSmoother::PathPoint{.x = path_point.x(), .y = path_point.y()};
    auto smoothed_path = this->pathsmoother_proxy->smoothPath(path_converted);
    vector<Eigen::Vector2f> smoothed_path_eigen(smoothed_path.size());
    if(smoothed_path.size() > 0)
        for(auto&& [i,path_point] : iter::enumerate(smoothed_path))
            smoothed_path_eigen[i] = Eigen::Vector2f {path_point.x, path_point.y};
        path = smoothed_path_eigen;

    draw_spline_path(smoothed_path_eigen);
}

void SpecificWorker::run_current_plan(const QPolygonF &laser_poly, bool grid_exists)
{

    auto is_good_path = true;
//    auto grid_robot_pose = from_world_to_grid(robot_pose.get_pos());
//    auto grid_target_pose = from_world_to_grid(target.get_pos());
    auto destination_pose = target_before_objetive(robot_pose.get_pos(), target.get_pos());
//    auto target_to_robot = from_world_to_robot(target.get_pos());
    if(grid_exists)
    {
        if(path.empty())
        {
            path = grid.compute_path(e2q(robot_pose.get_pos()), e2q(destination_pose), constants.robot_length);
        }
        else
        {
            is_good_path = check_path(path, laser_poly);
            if(is_good_path)
            {
                if((target.get_pos() - last_relevant_person_pos).norm() * 1.5 > constants.robot_length)
                {
                    last_relevant_person_pos = target.get_pos();
                    auto aux_path = path;
                    path = add_path_section_to_person(aux_path);
                }
            }
        }

    }

    // If grid is not activated, just create a path with two points: robot position and person position
    else
    {
        std::cout << "ROBOT POSE: " << robot_pose.get_pos().x() << " " << robot_pose.get_pos().y();
        std::cout << "PERSON POSE: " << robot_pose.get_pos().x() << " " << robot_pose.get_pos().y();
        path = std::vector<Eigen::Vector2f>{robot_pose.get_pos(), target.get_pos()};
    }
//    path_smoother(path);
//    path = grid.compute_path(e2q(grid_robot_pose), e2q(destination_pose), constants.robot_length);
//    float path_length_reverse = 0.f;
//    int aux_pos = 0;
//
//    for (auto&& [i,path_points] : iter::enumerate(iter::sliding_window(iter::reversed(path), 2)))
//    {
//        path_length_reverse += (path_points[0] - path_points[1]).norm();
//        if(path_length_reverse > 1000)
//        {
//            aux_pos = i;
//            break;
//        }
//    }
//    qInfo() << "AUX POS: " << aux_pos;
//    qInfo() << "path_length_reverse: " << path_length_reverse;
//    path.erase(path.end()-aux_pos, path.end());

//    qInfo() << "IS GOOD PATH: " << is_good_path;
//    if(path.size() > 3)

    if (is_good_path && !path.empty())
    {
        world_path.clear();
        std::vector<float> x_values, y_values;
        x_values.reserve(path.size());
        y_values.reserve(path.size());
        for (auto &&p : path)
        {
            x_values.push_back(p.x());
            y_values.push_back(p.y());
//            auto point_world_reference = from_grid_to_world(p);
//            world_path.push_back(point_world_reference);
//            x_values.push_back(point_world_reference.x());
//            y_values.push_back(point_world_reference.y());

        }
        if (widget_2d != nullptr)
            draw_path(path);

        if (auto path_node = G->get_node(current_path_name); path_node.has_value())
        {
            auto path_to_target_node = path_node.value();
            G->add_or_modify_attrib_local<path_x_values_att>(path_to_target_node, x_values);
            G->add_or_modify_attrib_local<path_y_values_att>(path_to_target_node, y_values);
            G->add_or_modify_attrib_local<path_target_x_att>(path_to_target_node, (float) target.get_pos().x());
            G->add_or_modify_attrib_local<path_target_y_att>(path_to_target_node, (float) target.get_pos().y());
            G->update_node(path_to_target_node);
        }
        else // create path_to_target_node with the solution path
        {
            qInfo() << "|||||||||||||||||||||| CREATING NEW PATH NODE ||||||||||||||||||||||";
//            if(auto intention = G->get_node(current_intention_name); intention.has_value())
            if(auto robot_node = G->get_node(robot_name); robot_node.has_value())
            {
                auto path_to_target_node = DSR::Node::create<path_to_target_node_type>(current_path_name);
                G->add_or_modify_attrib_local<path_x_values_att>(path_to_target_node, x_values);
                G->add_or_modify_attrib_local<path_y_values_att>(path_to_target_node, y_values);
                G->add_or_modify_attrib_local<pos_x_att>(path_to_target_node, (float) -542);
                G->add_or_modify_attrib_local<pos_y_att>(path_to_target_node, (float) 106);
                G->add_or_modify_attrib_local<parent_att>(path_to_target_node, robot_node.value().id());
                G->add_or_modify_attrib_local<level_att>(path_to_target_node, G->get_node_level(robot_node.value()).value() + 1);
                G->add_or_modify_attrib_local<path_target_x_att>(path_to_target_node, (float) target.get_pos().x());
                G->add_or_modify_attrib_local<path_target_y_att>(path_to_target_node, (float) target.get_pos().y());
                G->insert_node(path_to_target_node);
                std::cout << "1" << std::endl;
//                rt->insert_or_assign_edge_RT(robot_node.value(), path_to_target_node.id(), std::vector<float>{0.0, 0.0, 0.0}, std::vector<float>{0.0, 0.0, 0.0});

                    DSR::Edge edge_world = DSR::Edge::create<RT_edge_type>(robot_node.value().id(), path_to_target_node.id());

                    G->add_or_modify_attrib_local<rt_translation_att>(edge_world, std::vector<float>{0.0, 0.0, 0.0});
                    G->add_or_modify_attrib_local<rt_rotation_euler_xyz_att>(edge_world, std::vector<float>{0.0, 0.0, 0.0});

                    if (G->insert_or_assign_edge(edge_world))
                    {
                        std::cout << __FUNCTION__ << " Edge successfully inserted: " << robot_node.value().id() << "->" << path_to_target_node.id()
                                  << " type: RT" << std::endl;
                    }
                    else
                    {
                        std::cout << __FUNCTION__ << ": Fatal error inserting new edge: " << robot_node.value().id() << "->" << path_to_target_node.id()
                                  << " type: RT" << std::endl;
                        std::terminate();
                    }

                std::cout << "1" << std::endl;
//                DSR::Edge edge_to_robot = DSR::Edge::create<thinks_edge_type>(id.value(), intention.value().id());
//                G->insert_or_assign_edge(edge_to_intention);
            }
            else qWarning() << __FUNCTION__ << "No intention node found. Can't create path node";
        }
    }
//    else
//    {
//        auto destination_pose = target_before_objetive(robot_pose.get_pos(), target.get_pos());
//        path = grid.compute_path(e2q(robot_pose.get_pos()), e2q(destination_pose), constants.robot_length);
////        if(laser_poly.containsPoint(e2q(target_to_robot), Qt::OddEvenFill))
////        {
////            qInfo() << "IN LASER";
////            std::vector<Eigen::Vector2f> two_points_path{path.front(),path.at(path.size()/2) , path.back()};
////            path = two_points_path;
////        }
//        qWarning() << __FUNCTION__ << "Empty path. Computing new path";
//    }
}
Eigen::Vector2f SpecificWorker::from_world_to_grid(const Eigen::Vector2f &p)
{
    // build the matrix to transform from world to local_grid, knowing robot and grid pose in world
    Eigen::Matrix2f w2g;
    w2g <<  cos(grid_world_pose.get_ang()), sin(grid_world_pose.get_ang()),
            -sin(grid_world_pose.get_ang()), cos(grid_world_pose.get_ang());
    return w2g * (p - grid_world_pose.get_pos());
}
Eigen::Vector2f SpecificWorker::from_world_to_robot(const Eigen::Vector2f &p)
{
    Eigen::Matrix2f matrix;
    matrix << cos(robot_pose.get_ang()) , -sin(robot_pose.get_ang()) , sin(robot_pose.get_ang()) , cos(robot_pose.get_ang());
    return (matrix.transpose() * (p - robot_pose.get_pos()));
}
Eigen::Vector2f SpecificWorker::from_robot_to_world(const Eigen::Vector2f &p)
{
    Eigen::Matrix2f matrix;
    matrix << cos(robot_pose.get_ang()) , -sin(robot_pose.get_ang()) , sin(robot_pose.get_ang()) , cos(robot_pose.get_ang());
    return (matrix * p) + robot_pose.get_pos();
}
Eigen::Vector2f SpecificWorker::from_grid_to_world(const Eigen::Vector2f &p)
{
    // build the matrix to transform from grid to world knowing robot and grid pose in world
    Eigen::Matrix2f g2w;
    g2w <<  cos(grid_world_pose.get_ang()), -sin(grid_world_pose.get_ang()),
            sin(grid_world_pose.get_ang()), cos(grid_world_pose.get_ang());
    return g2w * p + grid_world_pose.get_pos();
}
//Eigen::Matrix3f SpecificWorker::from_robot_to_grid_matrix()
//{
//    // build the matrix to transform from robot to local_grid, knowing robot and grid pose in world
//
//    Eigen::Matrix3f r2w;
//    r2w <<  cos(robot_pose.get_ang()), -sin(robot_pose.get_ang()), robot_pose.get_pos().x(),
//            sin(robot_pose.get_ang()) , cos(robot_pose.get_ang()), robot_pose.get_pos().y(),
//            0.f, 0.f, 1.f;
//    Eigen::Matrix2f w2g_2d_matrix;
//    w2g_2d_matrix <<  cos(grid_world_pose.get_ang()), sin(grid_world_pose.get_ang()),
//            -sin(grid_world_pose.get_ang()), cos(grid_world_pose.get_ang());
//    auto tr = w2g_2d_matrix * grid_world_pose.get_pos();
//    Eigen::Matrix3f w2g;
//    w2g << cos(grid_world_pose.get_ang()), sin(grid_world_pose.get_ang()), -tr.x(),
//            -sin(grid_world_pose.get_ang()), cos(grid_world_pose.get_ang()), -tr.y(),
//            0.f, 0.f, 1.f;
//    Eigen::Matrix3f r2g = w2g * r2w;  // from r to world and then from world to grid
////    std::cout << "ROBOT POS: " << robot_pose.get_pos() << std::endl;
////    std::cout << "ROBOT ANG: " << robot_pose.get_ang() << std::endl;
////    qInfo() << __FUNCTION__ << "GRID WORLD POSE:" << grid_world_pose.get_pos().x() << grid_world_pose.get_pos().y() ;
//    return r2g;
//}

Eigen::Matrix3f SpecificWorker::from_robot_to_grid_matrix()
{
    // build the matrix to transform from robot to local_grid, knowing robot and grid pose in world

    Eigen::Matrix3f r2w;
    r2w <<  cos(robot_pose.get_ang()), -sin(robot_pose.get_ang()), -robot_pose.get_pos().x(),
            sin(robot_pose.get_ang()) , cos(robot_pose.get_ang()), -robot_pose.get_pos().y(),
            0.f, 0.f, 1.f;
    Eigen::Matrix2f w2g_2d_matrix;
    w2g_2d_matrix <<  cos(grid_world_pose.get_ang()), sin(grid_world_pose.get_ang()),
            -sin(grid_world_pose.get_ang()), cos(grid_world_pose.get_ang());
    auto tr = w2g_2d_matrix * grid_world_pose.get_pos();
    Eigen::Matrix3f w2g;
    w2g << cos(grid_world_pose.get_ang()), sin(grid_world_pose.get_ang()), -tr.x(),
            -sin(grid_world_pose.get_ang()), cos(grid_world_pose.get_ang()), -tr.y(),
            0.f, 0.f, 1.f;
    Eigen::Matrix3f r2g = w2g * r2w;  // from r to world and then from world to grid
//    std::cout << "ROBOT POS: " << robot_pose.get_pos() << std::endl;
//    std::cout << "ROBOT ANG: " << robot_pose.get_ang() << std::endl;
//    qInfo() << __FUNCTION__ << "GRID WORLD POSE:" << grid_world_pose.get_pos().x() << grid_world_pose.get_pos().y() ;
    return r2g;
}


Eigen::Matrix3f SpecificWorker::from_grid_to_robot_matrix()
{
    return from_robot_to_grid_matrix().inverse();
}

// Check if person is in the local grid
bool SpecificWorker::person_in_grid_checker(bool is_following)
{
    if(is_following)
    {
//        qInfo() << __FUNCTION__ << "FOLLOWING";
        QGraphicsRectItem dim(-2000, -500, 4000, act_grid_dist_to_robot + 2000);
        dim.setPos(grid_world_pose.to_qpoint());
        dim.setRotation(grid_world_pose.get_ang() *  180/ M_PI);
        auto converted_person_point = from_world_to_grid(target.get_pos());
        auto converted_robot_point = from_world_to_grid(robot_pose.get_pos());
//        auto converted_person_point = target.get_pos();
//        auto converted_robot_point = robot_pose.get_pos();
        QPointF modified_target_point(converted_person_point.x(),converted_person_point.y());
        QPointF modified_robot_point(converted_robot_point.x(),converted_robot_point.y());
        return dim.contains(modified_target_point) && dim.contains(modified_robot_point);
    }
    else
    {
//        qInfo() << __FUNCTION__ << "NOT FOLLOWING";
        QGraphicsRectItem dim(-2000, -2000, 4000, 4000);
        dim.setPos(grid_world_pose.to_qpoint());
        dim.setRotation(grid_world_pose.get_ang() *  180/ M_PI);
        auto converted_robot_point = robot_pose.get_grid_pos();
        QPointF modified_robot_point(converted_robot_point.x(),converted_robot_point.y());
//        qInfo() << __FUNCTION__ << "ROBOT POS:" << converted_robot_point.x() << converted_robot_point.y();
//        qInfo() << __FUNCTION__ << "CONTAINS:" << dim.contains(modified_robot_point);
        return dim.contains(modified_robot_point);
    }


}
void SpecificWorker::update_grid()
{
    if (grid_updated)
    {
        std::cout << __FUNCTION__ << " Updating grid ---------------------------------------" << std::endl;
        if (auto node = G->get_node(grid_type_name); node.has_value())
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
    if (auto current_grid_node_o = G->get_node(grid_type_name); current_grid_node_o.has_value())
    {
        G->add_or_modify_attrib_local<grid_as_string_att>(current_grid_node_o.value(), grid_as_string);
        G->update_node(current_grid_node_o.value());
    }
    else
    {
        if (auto mind = G->get_node(robot_mind_name); mind.has_value())
        {
            DSR::Node current_grid_node = DSR::Node::create<grid_node_type>(grid_type_name);
            G->add_or_modify_attrib_local<name_att>(current_grid_node, grid_type_name);
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
void SpecificWorker::inject_grid_data_in_G(const std::vector<float> &grid_size)
{
    qInfo() << __FUNCTION__ << "ENTRA";
    if (auto current_grid_node_o = G->get_node(grid_type_name); current_grid_node_o.has_value())
    {
        G->add_or_modify_attrib_local<grid_pos_att>(current_grid_node_o.value(), std::vector<float>{grid_world_pose.get_pos().x(), grid_world_pose.get_pos().y()});
        G->add_or_modify_attrib_local<grid_size_att>(current_grid_node_o.value(), grid_size);
        G->add_or_modify_attrib_local<grid_ang_att>(current_grid_node_o.value(), grid_world_pose.get_ang());
        G->update_node(current_grid_node_o.value());
    }
    else
    {
        if (auto mind = G->get_node(robot_mind_name); mind.has_value())
        {
            qInfo() << __FUNCTION__ << "ENTRA";
            DSR::Node current_grid_node = DSR::Node::create<grid_node_type>(grid_type_name);
            G->add_or_modify_attrib_local<name_att>(current_grid_node, grid_type_name);
            G->add_or_modify_attrib_local<parent_att>(current_grid_node, mind.value().id());
            G->add_or_modify_attrib_local<level_att>(current_grid_node, G->get_node_level(mind.value()).value() + 1);
            G->add_or_modify_attrib_local<grid_pos_att>(current_grid_node, std::vector<float>{grid_world_pose.get_pos().x(), grid_world_pose.get_pos().y()});
            G->add_or_modify_attrib_local<grid_size_att>(current_grid_node, grid_size);
            G->add_or_modify_attrib_local<grid_ang_att>(current_grid_node, grid_world_pose.get_ang());
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
        plan_node_id = id;
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
//    else if (type == grid_type_name)
//    {
//        grid_updated = true;
//    }
//    else if (type == current_path_name)
//    {
//        grid_updated = true;
//    }
}
void SpecificWorker::modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type)
{
//    qInfo() << __FUNCTION__ << "MODYFIED EDGE";
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::normal_distribution<double> normal_dist(0.0, constants.target_noise_sigma);
    if(from == 200 and to == followed_person_id and type == "RT")
    {
        if(auto person = inner_eigen->transform(G->get_node(from).value().name(), G->get_node(to).value().name()); person.has_value())
        {
            target.set_pos(Eigen::Vector2f{person.value().x(), person.value().y()});
            auto r = robot_pose.get_pos();
            QLineF r_to_target(QPointF(person.value().x(), person.value().y()), robot_pose.to_qpoint());
            auto t = r_to_target.pointAt(constants.final_distance_to_target / r_to_target.length());
            t += QPointF(normal_dist(mt), normal_dist(mt));              // Adding noise to target
            target.set_pos(Eigen::Vector2f{t.x(), t.y()});
        }
    }
//    if(from == grid_id and to == 200 and type == "RT")
//    {
//        std::cout << "////////////////////////////////////////////////////" << std::endl;
//        if(auto robot_pos = inner_eigen->transform(current_grid_name, robot_name); robot_pos.has_value())
//        {
//            std::cout << "////////////////////////////////////////////////////" << std::endl;
//            if(auto robot_ang = inner_eigen->get_euler_xyz_angles(G->get_node(from).value().name(), G->get_node(to).value().name()); robot_ang.has_value())
//            {
//                qInfo() << __FUNCTION__ << "GET RT Robot EDGE";
//                robot_pose.set_pos(Eigen::Vector2f{robot_pos.value().x(), robot_pos.value().y()});
//                std::cout << robot_pose.get_pos() << std::endl;
//                robot_pose.set_grid_pos(Eigen::Vector2f{robot_grid_pos.value().get()[0], robot_grid_pos.value().get()[1]});
//                robot_pose.set_angle(robot_ang.value().z());
//            }
//        }
//    }
}
void SpecificWorker::del_node_slot(std::uint64_t from)
{
    if(from == plan_node_id)
    {
        current_plan = {};
        target.set_active(false);
        path.clear();
    }
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
//    qInfo() << "CLEAR DRAW";
    path_paint.clear();

    uint s = 100;
    for(auto &&p : path_in_robot)
    {
        auto pw = from_grid_to_world(p);  // in mm
        path_paint.push_back(widget_2d->scene.addEllipse(pw.x()-s/2, pw.y()-s/2, s , s, QPen(path_color), QBrush(QColor(path_color))));
        path_paint.back()->setZValue(30);
    }
}

void SpecificWorker::draw_spline_path(const std::vector<Eigen::Vector2f> &path_in_robot)
{
    static std::vector<QGraphicsItem *> spline_path_paint;
    static QString path_color = "Orange";

    for(auto p : spline_path_paint)
        widget_2d->scene.removeItem(p);
//    qInfo() << "CLEAR DRAW";
    spline_path_paint.clear();

    uint s = 100;
    for(auto &&p : path_in_robot)
    {
        auto pw = from_grid_to_world(p);  // in mm
        spline_path_paint.push_back(widget_2d->scene.addEllipse(pw.x()-s/2, pw.y()-s/2, s , s, QPen(path_color), QBrush(QColor(path_color))));
        spline_path_paint.back()->setZValue(30);
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

