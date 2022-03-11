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
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/filter.hpp>

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
	this->Period = 10;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
        dimensions = QRectF(-5100, -2600, 10200, 5200);
        viewer_robot = new AbstractGraphicViewer(this->graphicsView, dimensions);
        auto [rp, lp] = viewer_robot->add_robot(constants.robot_width, constants.robot_length, constants.laser_x_offset, constants.laser_y_offset);
        robot_draw_polygon = rp;
        laser_draw_polygon = lp;
        const float ext_robot_semi_width = constants.robot_width/2;
        const float ext_robot_semi_length = constants.robot_length/2;
        left_polygon_robot <<  QPointF(0, -ext_robot_semi_length) <<
                           QPointF(-ext_robot_semi_width, -ext_robot_semi_length) <<
                           QPointF(-ext_robot_semi_width, 0) <<
                           QPointF(-ext_robot_semi_width, ext_robot_semi_length/2) <<
                           QPointF(-ext_robot_semi_width, ext_robot_semi_length) <<
                           QPointF(0, constants.robot_semi_length*1.1);
        right_polygon_robot << QPointF(0,-ext_robot_semi_length) <<
                            QPointF(ext_robot_semi_width, -ext_robot_semi_length) <<
                            QPointF(ext_robot_semi_width, 0) <<
                            QPointF(ext_robot_semi_width, ext_robot_semi_length/2) <<
                            QPointF(ext_robot_semi_width, ext_robot_semi_length) <<
                            QPointF(0, ext_robot_semi_length*1.1);

        // robot polygon to verify that the candidate path is traversable
        polygon_robot <<  QPointF(-constants.robot_semi_width*1.2, constants.robot_semi_length) <<
                      QPointF(constants.robot_semi_width*1.2, constants.robot_semi_length) <<
                      QPointF(constants.robot_semi_width, -constants.robot_semi_length) <<
                      QPointF(-constants.robot_semi_width, -constants.robot_semi_length);

        // QCustomPlot
//        custom_plot.setParent(timeseries_frame);
//        custom_plot.xAxis->setLabel("time");
//        custom_plot.yAxis->setLabel("rot-b adv-g lhit-m stuck-r");
//        custom_plot.xAxis->setRange(0, 200);
//        custom_plot.yAxis->setRange(-10, 10);
//        rot_graph = custom_plot.addGraph();
//        rot_graph->setPen(QColor("blue"));
//        adv_graph = custom_plot.addGraph();
//        adv_graph->setPen(QColor("green"));
//        lhit_graph = custom_plot.addGraph();
//        lhit_graph->setPen(QColor("magenta"));
//        rhit_graph = custom_plot.addGraph();
//        rhit_graph->setPen(QColor("magenta"));
//        stuck_graph = custom_plot.addGraph();
//        stuck_graph->setPen(QColor("red"));
//        custom_plot.resize(timeseries_frame->size());
//        custom_plot.show();

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
//		using opts = DSR::DSRViewer::view;
//		int current_opts = 0;
//		opts main = opts::none;
//		if(tree_view)
//		{
//		    current_opts = current_opts | opts::tree;
//		}
//		if(graph_view)
//		{
//		    current_opts = current_opts | opts::graph;
//		    main = opts::graph;
//		}
//		if(qscene_2d_view)
//		{
//		    current_opts = current_opts | opts::scene;
//		}
//		if(osg_3d_view)
//		{
//		    current_opts = current_opts | opts::osg;
//		}
//		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
//		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    bool lhit = false, rhit= false, stuck = false;
    auto [advance, rotation] = read_base();
    global_advance = advance;
    global_rotation = rotation;

    const auto &[l_poly, ldata] = read_laser();
    laser_poly = l_poly;

    if(auto robot_node = G->get_node("robot"); robot_node.has_value())
    {

        auto person_nodes = G->get_nodes_by_type("person");
        if(person_nodes.size() > 0)
        {
            for (auto p: person_nodes)
            {
//                if(auto robot_person_edge = G->get_edge(robot_node.value().id(), p.id(), "following"); robot_person_edge.has_value())
                if(auto person_followed = G->get_attrib_by_name<followed_att>(p.id()); person_followed.has_value() && person_followed.value() == true)
                {
                    if(auto robot_person_RT = G->get_edge(robot_node.value().id(), p.id(), "RT"); robot_person_RT.has_value())
                    {
                        std::cout << "1" << std::endl;
                        auto person_pos = G->get_attrib_by_name<rt_translation_att>(robot_person_RT.value()).value().get();
                        std::cout << "2" << std::endl;
                        target_in_robot = {person_pos[0], person_pos[1]};
                        std::cout << "3" << std::endl;
                        Eigen::Vector2f target {person_pos[0], person_pos[1]};
                        std::cout << "4" << std::endl;
                        auto dist = target_in_robot.norm();
                        std::cout << "5" << std::endl;
                        if( dist > constants.final_distance_to_target)
                        {
                            std::cout << "6" << std::endl;
                            // compute the closest point to target inside laser as a provisional target

                            Eigen::Vector2f s_target = sub_target(target, l_poly, ldata );

                            for (int i = 0; i < laser_poly.size(); ++i)
                            {
                                std::cout << "VALOR LASER: " << laser_poly.value(i).x() << " " << laser_poly.value(i).y() << std::endl;
                            }
                            std::cout << "7" << std::endl;
                            Result res;
                            if( auto res_o =  control(s_target, laser_poly, advance, rotation, &viewer_robot->scene); not res_o.has_value())
                            {   // no control

                                qInfo() << __FUNCTION__ << "NO CONTROL";
                                if(stuck = do_if_stuck(0, 0,  lhit, rhit); stuck == true)
                                    return;
                                else  // no stuck. Probably an osbtacle appeared suddenly.
                                move_robot(0, 0);
                            }
                            else
                                res = res_o.value();
                            auto [_, __, adv, rot, ___] = res;
                            std::cout << "8" << std::endl;
                            rot = 0.8 * rot;
                            adv = 0.8 * adv;
                            lateral_bumpers(rot, lhit, rhit);   // check lateral collisions
                            std::cout << "9" << std::endl;
//                            adv = adv * gaussian(rot);          // slow down if rotating
                            std::cout << "10" << std::endl;
                            move_robot(adv,  rot);
                            std::cout << "11" << std::endl;
                            qInfo() << __FUNCTION__ << adv << rot;
                            //stuck = do_if_stuck(adv_n, rot, r_state, lhit, rhit);
//                            draw_timeseries(rot*5, adv/100, (int)lhit*5, (int)rhit*5, (int)stuck*5);
                        }
                        else
                        {
                            move_robot(0, 0);
                        }

                    }
                }
            }
        }
        else return;
    }
}

Eigen::Vector2f SpecificWorker::sub_target(Eigen::Vector2f &target, const QPolygonF &poly, const SpecificWorker::ldata &ldata)
{
    qInfo() << __FUNCTION__ << "fuck";
    static QGraphicsRectItem *former_draw = nullptr;
    if(former_draw != nullptr)
        viewer_robot->scene.removeItem(former_draw);

    // if target inside laser_polygon return
    if(poly.containsPoint(QPointF(target[0], target[1]), Qt::WindingFill))
        return target;

    // locate openings
    std::vector<float> derivatives(ldata.dists.size());
    derivatives[0] = 0;
    for (const auto &&[k, l]: iter::sliding_window(ldata.dists, 2) | iter::enumerate)
        derivatives[k + 1] = ldata.dists[1] - ldata.dists[0];

    std::vector<Eigen::Vector2f> peaks;
    for (const auto &&[k, der]: iter::enumerate(derivatives))
    {
        float l_angle, l_dist;
        if (der > 500)
        {
            l_angle = ldata.angles[k - 1];
            l_dist = ldata.dists[k - 1];
            peaks.push_back(Eigen::Vector2f(l_dist * sin(l_angle), l_dist * cos(l_angle)));
        }
        else if (der < -500)
        {
            l_angle = ldata.angles[k];
            l_dist = ldata.dists[k];
            peaks.push_back(Eigen::Vector2f(l_dist * sin(l_angle), l_dist * cos(l_angle)));
        }
    }
    // select closest opening to robot
    if(peaks.empty())
        return target;

    auto min = std::ranges::min_element(peaks, [target](auto a, auto b){ return (target-a).norm() < (target-b).norm();});
    auto candidate = *min;

    // if too close to subtarget ignore
    if( candidate.norm() < 200)
        return target;

    // if target closer that subtatget ignore
    if(target_in_robot.norm() < candidate.norm())
        return target;

//    Target t;
//    t.active = true;
//    auto pos = from_robot_to_world(*min, Eigen::Vector3f(r_state_global.x, r_state_global.y, r_state_global.rz));
//    t.pos = QPointF(pos.x(), pos.y());
    former_draw = viewer_robot->scene.addRect(target[0]-100, target[1]-100, 200, 200, QPen(QColor("blue")), QBrush(QColor("blue")));
    return target;
}

// lateral bumpers
void SpecificWorker::lateral_bumpers(float &rot, bool &lhit, bool &rhit)
{
    qInfo() << __FUNCTION__ << "fuck";
    static float delta_rot_l = constants.initial_delta_rot;
    static float delta_rot_r = constants.initial_delta_rot;
    auto linl = laser_draw_polygon->mapFromParent(laser_poly);
    auto lp = laser_draw_polygon->mapFromParent(left_polygon_robot);
    if (auto res = std::ranges::find_if_not(lp, [linl](const auto &p)
        { return linl.containsPoint(p, Qt::WindingFill); }); res != std::end(lp))
    {
        qInfo() << __FUNCTION__ << "---- TOCANDO POR LA IZQUIERDA-----------" << *res;
        rot += delta_rot_l;
        delta_rot_l *= 1.5;
        lhit = true;
    }
    else
    {
        delta_rot_l = constants.initial_delta_rot;
        lhit = false;
    }
    auto rp = laser_draw_polygon->mapFromParent(right_polygon_robot);
    if (auto res = std::ranges::find_if_not(rp, [linl](const auto &p)
        { return linl.containsPoint(p, Qt::WindingFill); }); res != std::end(rp))
    {
        qInfo() << __FUNCTION__ << "---- TOCANDO POR LA DERECHA-----------" << *res;
        rot -= delta_rot_r;
        delta_rot_r *= 1.5;
        rhit = true;
    }
    else
    {
        delta_rot_r = constants.initial_delta_rot;
        rhit = false;
    }
}

// checks if the robot is moving in a time interval
bool SpecificWorker::do_if_stuck(float adv, float rot, bool lhit, bool rhit)
{
    //static bool first_time = true;
    const float SMALL_FLOAT = 0.001;
    const float SMALL_INT = 1.0;
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<float> dist(0.3, 1.0);

    qInfo() << __FUNCTION__ << adv << rot;

    // TODO: REVISAR VALORES MÍNIMOS
    if( fabs(adv) < SMALL_INT  and fabs(rot) < SMALL_FLOAT )
    {
        if(lhit) move_robot(-constants.backward_speed, -dist(mt));
        else if(rhit) move_robot(-constants.backward_speed, dist(mt));
        else move_robot(-constants.backward_speed, 0);
        usleep(30000);
        qInfo() << __FUNCTION__ << "------ STUCK ----------";
        return true;
    }
    return false;
}

std::optional<SpecificWorker::Result> SpecificWorker::control(const Eigen::Vector2f &target_r, const QPolygonF &laser_poly,
                                                              double advance, double rot, QGraphicsScene *scene)
{
//    qInfo() << __FUNCTION__ << "fuck";
    // compute future positions of the robot
    std::cout << "a" << std::endl;
    auto point_list = compute_predictions(advance, rot, laser_poly);

    // compute best value
    std::cout << "b" << std::endl;
    auto best_choice = compute_optimus(point_list, target_r);

//    if(scene != nullptr)
//        draw_dwa(robot, point_list, best_choice, scene);

    if (best_choice.has_value())
    {
        std::cout << "c" << std::endl;
        auto[x, y, v, w, alpha]= best_choice.value();  // x,y coordinates of best point, v,w velocities to reach that point, alpha robot's angle at that point
        return best_choice.value();
    }
    else
        return {};
}

std::vector<SpecificWorker::Result> SpecificWorker::compute_predictions(float current_adv, float current_rot, const QPolygonF &laser_poly)
{
//    qInfo() << __FUNCTION__ << "fuck";
    std::vector<Result> list_points;
//    std::cout << "LASER POLY: " << laser_poly.size() << std::endl;
    for (float v = -constants.max_advance_speed; v <= constants.max_advance_speed; v += 50) //advance
    {
//        std::cout << "a0" << std::endl;
        float new_adv = current_adv + v;
        if (fabs(new_adv) > constants.max_advance_speed)
            continue;
        for (float w = -constants.max_rotation_speed; w <= constants.max_rotation_speed; w += 0.1) //rotation
        {
//            std::cout << "a1" << std::endl;
            float new_rot = -current_rot + w;
            if (fabs(new_rot) > constants.max_rotation_speed)
                continue;
            if (fabs(w) > 0.001)  // avoid division by zero to compute the radius
            {
//                std::cout << "a2" << std::endl;
                float r = new_adv / new_rot; // radio de giro ubicado en el eje x del robot
                float arc_length = new_rot * constants.time_ahead * r;
                for (float t = constants.step_along_arc; t < arc_length; t += constants.step_along_arc)
                {
//                    std::cout << "a3" << std::endl;
                    float x = r - r * cos(t / r);
                    float y = r * sin(t / r);  // circle parametric coordinates
                    auto point = std::make_tuple(x, y, new_adv, new_rot, t / r);
                    if (sqrt(x * x + y * y) > constants.robot_semi_width and point_reachable_by_robot(point, laser_poly)) // skip points in the robot
                        list_points.emplace_back(std::move(point));
                }
            } else // para evitar la división por cero en el cálculo de r
            {
                for (float t = constants.step_along_arc; t < new_adv * constants.time_ahead; t += constants.step_along_arc)
                {
                    std::cout << "a4" << std::endl;
                    auto point = std::make_tuple(0.f, t, new_adv, new_rot, new_rot * constants.time_ahead);
                    if (t > constants.robot_semi_width and point_reachable_by_robot(point, laser_poly))
                        list_points.emplace_back(std::make_tuple(0.f, t, new_adv, new_rot, new_rot * constants.time_ahead));
                }
            }
        }
    }
//    std::cout << "LIST POINT SIZE: " << list_points.size() << std::endl;
    return list_points;
}
bool SpecificWorker::point_reachable_by_robot(const Result &point, const QPolygonF &laser_poly)
{

    auto [x, y, adv, giro, ang] = point;
    auto goal_r = Eigen::Vector2f(x,y);
    float parts = goal_r.norm()/(constants.robot_semi_width/3);  //should be a fraction of the arc
    float ang_delta = ang / parts;
    float init_ang = 0;

//    std::cout << laser_poly.isClosed() << " " << laser_poly.size() << std::endl;

    auto linl = laser_draw_polygon->mapFromParent(laser_poly);
    auto lp = laser_draw_polygon->mapFromParent(polygon_robot);

    for(const auto &l: iter::range(0.0, 1.0, 1.0/parts))
    {
        init_ang += ang_delta;
        auto p = to_qpointf(goal_r * l);
        auto temp_robot = QTransform().rotate(init_ang).translate(p.x(), p.y()).map(lp);  // compute incremental rotation
        if (auto res = std::ranges::find_if_not(temp_robot, [linl](const auto &p)
            { return linl.containsPoint(p, Qt::OddEvenFill); }); res != std::end(temp_robot))
        {
            return false;
        }
    }
    return true;
}
std::optional<SpecificWorker::Result> SpecificWorker::compute_optimus(const std::vector<Result> &points,
                                                                      const Eigen::Vector2f &tr)
{
//    qInfo() << __FUNCTION__ << "fuck";
    static float prev_advance = 0, prev_rot = 0;
    std::vector<std::tuple<float, Result>> values(points.size());
    for(auto &&[k, point] : iter::enumerate(points))
    {
        auto [x, y, adv, giro, ang] = point;
        float dist_to_target = (Eigen::Vector2f(x, y) - tr).norm();
        float turn_variation  = fabs(giro-prev_rot);
        float advance_variation = fabs(adv-prev_advance);
        values[k] = std::make_tuple(constants.A_dist_factor*dist_to_target +
                                    constants.B_turn_factor*turn_variation +
                                    constants.C_advance_factor*advance_variation, point);
    }
    auto min = std::ranges::min_element(values, [](auto &a, auto &b){ return std::get<0>(a) < std::get<0>(b);});
    if(min != values.end())
    {
        Result r = std::get<Result>(*min);
        auto &[x, y, adv, giro, ang] = r;
        prev_advance = adv;
        prev_rot = giro;
        return r;
    }
    else
        return {};
}
//void SpecificWorker::draw_dwa(const Eigen::Vector3f &robot, const std::vector <Result> &puntos,
//                              const std::optional<Result> &best, QGraphicsScene *scene)
//{
//    static std::vector<QGraphicsEllipseItem *> arcs_vector;
//    // remove current arcs
//    for (auto arc: arcs_vector)
//        scene->removeItem(arc);
//    arcs_vector.clear();
//
//    QColor col("Blue");
//    for (auto &[x, y, vx, wx, a] : puntos)
//    {
//        //QPointF centro = robot_draw_polygon_draw->mapToScene(x, y);
//        QPointF centro = to_qpointf(from_robot_to_world(Eigen::Vector2f(x, y), robot));
//        auto arc = scene->addEllipse(centro.x(), centro.y(), 50, 50, QPen(col, 10));
//        arc->setZValue(30);
//        arcs_vector.push_back(arc);
//    }
//
//    if(best.has_value())
//    {
//        auto &[x, y, _, __, ___] = best.value();
//        QPointF selected = to_qpointf(from_robot_to_world(Eigen::Vector2f(x, y), robot));
//        auto arc = scene->addEllipse(selected.x(), selected.y(), 180, 180, QPen(Qt::black), QBrush(Qt::black));
//        arc->setZValue(30);
//        arcs_vector.push_back(arc);
//    }
//}

std::tuple<double, double> SpecificWorker::read_base()
{
//    qInfo() << __FUNCTION__ << "fuck";
    float adv_speed = 0;
    float rot_speed = 0;
    try
    {
        if(auto robot_node = G->get_node("robot"); robot_node.has_value())
        {
            if(auto advance = G->get_attrib_by_name<robot_local_advance_speed_att>(robot_node.value()); advance.has_value())
            {
                adv_speed = advance.value();
                if(auto rotation = G->get_attrib_by_name<robot_local_rotational_speed_att>(robot_node.value()); rotation.has_value())
                {
                    rot_speed = rotation.value();
                }
            }
        }
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
    return std::make_tuple(adv_speed, rot_speed);
}

std::tuple<QPolygonF, SpecificWorker::ldata> SpecificWorker::read_laser()
{
//    qInfo() << __FUNCTION__ << "fuck";
    QPolygonF poly_robot;
    SpecificWorker::ldata laser_data;
    try
    {
        if(auto laser_node = G->get_node("laser"); laser_node.has_value())
        {
            if(auto laser_dist = G->get_attrib_by_name<laser_dists_att>(laser_node.value()); laser_dist.has_value())
            {
                laser_data.dists = laser_dist.value();
                if(auto laser_angle = G->get_attrib_by_name<laser_angles_att>(laser_node.value()); laser_angle.has_value())
                {

                    laser_data.angles = laser_angle.value();
                    float dist_ant = 50;
                    for (int i = 0; i < laser_data.dists.size(); ++i)
                    {
//                        std::cout << "LASER DATA: " << laser_data.dists[i] << " " << laser_data.angles[i] << std::endl;

                        if (laser_data.dists[i] < 50)
                            laser_data.dists[i] = dist_ant;
//                        laser_data.dists[i] = 3000;
                        dist_ant = laser_data.dists[i];
                        poly_robot << QPointF(laser_data.dists[i] * sin(laser_data.angles[i]), laser_data.dists[i] * cos(laser_data.angles [i]));
//                        poly_robot << QPointF(3000 * sin(laser_data.angles[i]), 3000 * cos(laser_data.angles [i]));
                    }

                    // Simplify laser contour with Ramer-Douglas-Peucker
//                    poly_robot = ramer_douglas_peucker(ldata, constants.MAX_RDP_DEVIATION_mm);
                    // add robot contour  wrt laser_location
                    poly_robot.pop_front();
                    poly_robot.push_front(QPointF(constants.robot_semi_width*1.1, -constants.robot_semi_length));
                    poly_robot.push_front(QPointF(constants.robot_semi_width*1.1, -constants.robot_length));
                    poly_robot.push_front(QPointF(0, -constants.robot_length));
                    poly_robot.pop_back();
                    poly_robot.push_back(QPointF(-constants.robot_semi_width*1.1, -constants.robot_semi_length));
                    poly_robot.push_back(QPointF(-constants.robot_semi_width*1.1, -constants.robot_length));
                    poly_robot.push_back(QPointF(0, -constants.robot_length));

                }
            }
        }
    }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    return std::make_tuple(poly_robot, laser_data);
}




void SpecificWorker::move_robot(float adv, float rot)
{
//    qInfo() << __FUNCTION__ << "fuck";
    try
    {
        if(auto robot_node = G->get_node("robot"); robot_node.has_value())
        {
//            std::cout << "SPEEEED: " << adv << " " << rot << std::endl;
            G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot_node.value(), adv);
            G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot_node.value(), rot);
            G->update_node(robot_node.value());
        }
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
}

void SpecificWorker::draw_laser(const QPolygonF &poly) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    if (laser_polygon != nullptr)
        viewer_robot->scene.removeItem(laser_polygon);

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer_robot->scene.addPolygon(laser_draw_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}

//void SpecificWorker::draw_timeseries(float rot, float adv, int lhit, int rhit, int stuck)
//{
//    static int cont = 0;
//    rot_graph->addData(cont, rot);
//    adv_graph->addData(cont, adv);
//    lhit_graph->addData(cont, lhit);
//    rhit_graph->addData(cont, rhit);
//    stuck_graph->addData(cont++, stuck);
//    custom_plot.xAxis->setRange(cont, 200, Qt::AlignRight);
//    custom_plot.replot();
//}
QPolygonF SpecificWorker::ramer_douglas_peucker(SpecificWorker::ldata &ldata, double epsilon)
{
    if(ldata.angles.size()<2)
    {
        qWarning() << __FUNCTION__ << "Not enough points to simplify";
        return QPolygonF();
    }
    std::vector<Point> pointList(ldata.angles.size());
    float dist_ant = 50;
    for (int i = 0; i < ldata.dists.size(); ++i)
    {
        if (ldata.dists[i] < 30)
            ldata.dists[i] = dist_ant;
        else
            dist_ant = ldata.dists[i];
        pointList[i] = std::make_pair(ldata.dists[i] * sin(ldata.angles[i]), ldata.dists[i] * cos(ldata.angles[i]));
    }
    std::vector<Point> pointListOut;
    ramer_douglas_peucker_rec(pointList, epsilon, pointListOut);
    QPolygonF poly(pointListOut.size());
    for (auto &&[i, p] : pointListOut | iter::enumerate)
        poly[i] = QPointF(p.first, p.second);
    return poly;
}
void SpecificWorker::ramer_douglas_peucker_rec(const vector<Point> &pointList, double epsilon, std::vector<Point> &out)
{
//    qInfo() << __FUNCTION__ << "fuck";
    // Find the point with the maximum distance from line between start and end
    auto line = Eigen::ParametrizedLine<float, 2>::Through(Eigen::Vector2f(pointList.front().first, pointList.front().second),
                                                           Eigen::Vector2f(pointList.back().first, pointList.back().second));
    auto max = std::max_element(pointList.begin()+1, pointList.end(), [line](auto &a, auto &b)
    { return line.distance(Eigen::Vector2f(a.first, a.second)) < line.distance(Eigen::Vector2f(b.first, b.second));});

    // If max distance is greater than epsilon, recursively simplify
    float dmax =  line.distance(Eigen::Vector2f((*max).first, (*max).second));
    if(dmax > epsilon)
    {
        // Recursive call
        std::vector<Point> recResults1;
        std::vector<Point> recResults2;
        std::vector<Point> firstLine(pointList.begin(), max + 1);
        std::vector<Point> lastLine(max, pointList.end());
        ramer_douglas_peucker_rec(firstLine, epsilon, recResults1);
        ramer_douglas_peucker_rec(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end() - 1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
        if (out.size() < 2)
        {
            qWarning() << __FUNCTION__ << "Problem assembling output";
            return;
        }
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList.front());
        out.push_back(pointList.back());
    }
}

//Eigen::Vector2f SpecificWorker::from_world_to_robot(const Eigen::Vector2f &p,
//                                                    const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
//{
//    Eigen::Matrix2f matrix;
//    matrix << cos(r_state.rz) , -sin(r_state.rz) , sin(r_state.rz) , cos(r_state.rz);
//    return (matrix.transpose() * (p - Eigen::Vector2f(r_state.x, r_state.y)));
//}
//
//Eigen::Vector2f SpecificWorker::from_robot_to_world(const Eigen::Vector2f &p, const Eigen::Vector3f &robot)
//{
//    Eigen::Matrix2f matrix;
//    const float &angle = robot.z();
//    matrix << cos(angle) , -sin(angle) , sin(angle) , cos(angle);
//    return (matrix * p) + Eigen::Vector2f(robot.x(), robot.y());
//}
float SpecificWorker::gaussian(float x)
{
    const double xset = 0.5;
    const double yset = 0.4;
    const double s = -xset*xset/log(yset);
    return exp(-x*x/s);
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}