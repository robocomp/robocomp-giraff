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
        std::cout << "1" << endl;
        dimensions = QRectF(-5100, -2600, 10200, 5200);
        viewer_robot = new AbstractGraphicViewer(this->graphicsView, dimensions);
        QColor color_real;
        QColor color_coppelia;
        color_real.setBlue(255);
        color_coppelia.setRed(255);
        auto [rp_real, lp_real] = viewer_robot->add_robot(constants.robot_width, constants.robot_length, color_real, constants.laser_x_offset, constants.laser_y_offset);
        robot_draw_polygon_real = rp_real;
        laser_draw_polygon_real = lp_real;

        connect(viewer_robot, &AbstractGraphicViewer::new_mouse_coordinates, [this](QPointF t)
        {
            qInfo() << __FUNCTION__ << " Received new target at " << t;
            real_arrived = false;
            target.pos = t;
            target.active = true;
            target.draw = viewer_robot->scene.addEllipse(t.x()-50, t.y()-50, 100, 100, QPen(QColor("magenta")), QBrush(QColor("magenta")));
        });

        // left and right expanded semi-polygons to detect collisions. They have to be transformed into the laser ref. system
        const float ext_robot_semi_width = constants.robot_width/2;
        const float ext_robot_semi_length = constants.robot_length/2;
        left_polygon_robot_real <<  QPointF(0, -ext_robot_semi_length) <<
                                QPointF(-ext_robot_semi_width, -ext_robot_semi_length) <<
                                QPointF(-ext_robot_semi_width, 0) <<
                                QPointF(-ext_robot_semi_width, ext_robot_semi_length/2) <<
                                QPointF(-ext_robot_semi_width, ext_robot_semi_length) <<
                                QPointF(0, constants.robot_semi_length*1.1);
        right_polygon_robot_real << QPointF(0,-ext_robot_semi_length) <<
                                 QPointF(ext_robot_semi_width, -ext_robot_semi_length) <<
                                 QPointF(ext_robot_semi_width, 0) <<
                                 QPointF(ext_robot_semi_width, ext_robot_semi_length/2) <<
                                 QPointF(ext_robot_semi_width, ext_robot_semi_length) <<
                                 QPointF(0, ext_robot_semi_length*1.1);

        // robot polygon to verify that the candidate path is traversable
        polygon_robot_real <<  QPointF(-constants.robot_semi_width*1.2, constants.robot_semi_length) <<
                           QPointF(constants.robot_semi_width*1.2, constants.robot_semi_length) <<
                           QPointF(constants.robot_semi_width, -constants.robot_semi_length) <<
                           QPointF(-constants.robot_semi_width, -constants.robot_semi_length);

        left_polygon_robot_coppelia <<  QPointF(0, -ext_robot_semi_length) <<
                                    QPointF(-ext_robot_semi_width, -ext_robot_semi_length) <<
                                    QPointF(-ext_robot_semi_width, 0) <<
                                    QPointF(-ext_robot_semi_width, ext_robot_semi_length/2) <<
                                    QPointF(-ext_robot_semi_width, ext_robot_semi_length) <<
                                    QPointF(0, constants.robot_semi_length*1.1);
        right_polygon_robot_coppelia << QPointF(0,-ext_robot_semi_length) <<
                                     QPointF(ext_robot_semi_width, -ext_robot_semi_length) <<
                                     QPointF(ext_robot_semi_width, 0) <<
                                     QPointF(ext_robot_semi_width, ext_robot_semi_length/2) <<
                                     QPointF(ext_robot_semi_width, ext_robot_semi_length) <<
                                     QPointF(0, ext_robot_semi_length*1.1);

        // robot polygon to verify that the candidate path is traversable
        polygon_robot_coppelia <<  QPointF(-constants.robot_semi_width*1.2, constants.robot_semi_length) <<
                               QPointF(constants.robot_semi_width*1.2, constants.robot_semi_length) <<
                               QPointF(constants.robot_semi_width, -constants.robot_semi_length) <<
                               QPointF(-constants.robot_semi_width, -constants.robot_semi_length);

        // QCustomPlot
        custom_plot.setParent(timeseries_frame);
        custom_plot.xAxis->setLabel("time");
        custom_plot.yAxis->setLabel("rot-b adv-g lhit-m stuck-r diff copp-real");
        custom_plot.xAxis->setRange(0, 200);
        custom_plot.yAxis->setRange(-10, 10);
        rot_graph = custom_plot.addGraph();
        rot_graph->setPen(QColor("blue"));
        adv_graph = custom_plot.addGraph();
        adv_graph->setPen(QColor("green"));
        lhit_graph = custom_plot.addGraph();
        lhit_graph->setPen(QColor("magenta"));
        rhit_graph = custom_plot.addGraph();
        rhit_graph->setPen(QColor("magenta"));
        stuck_graph = custom_plot.addGraph();
        stuck_graph->setPen(QColor("red"));
        diff_graph = custom_plot.addGraph();
        diff_graph->setPen(QColor("orange"));
        custom_plot.resize(timeseries_frame->size());
        custom_plot.show();
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
//		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
//		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		this->Period = period;
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    auto [r_state_real, advance_real, rotation_real] = read_base_real();
    laser_poly_real = read_laser_real();

    std::vector<float> position;
    float distance;
    Eigen::Vector2f point;

    auto people_nodes = G->get_nodes_by_type("person");
    auto world_node = G->get_node("world").value();
    for(auto p: people_nodes)
    {
        auto is_followed = G->get_attrib_by_name<followed_att>(p).value();
        if(is_followed == true)
        {
            std::cout << "ENTRA" << endl;
            auto edge_world = rt->get_edge_RT(world_node, p.id());
            if( edge_world.has_value())
            {
                position = G->get_attrib_by_name<rt_translation_att>(edge_world.value()).value().get();
                std::cout << "POS x: " << position[0] << " POS y: " << position[1] << endl;
                distance = G->get_attrib_by_name<distance_to_robot_att>(p).value();
                point.x() = position[0];
                point.y() = position[1];
                viewer_robot->scene.addEllipse(point.x()-50, point.y()-50, 100, 100, QPen(QColor("magenta")), QBrush(QColor("magenta")));
                target.active = true;
            }

        }
    }
    // If a point is clicked...
    if(target.active)
    {
        auto dist_real = distance;
//        cout << "distance to point: " << dist_real << endl;
        // Comparing if target point is more far than the reference final distance (950 mm)
        if( dist_real > constants.final_distance_to_target) {
            // Creating variable that contains data about the best option to avoid an obstacle
            Result res_real;
            // Calculating best option
            auto res_o_real = control(point, laser_poly_real, advance_real, rotation_real,
                                      Eigen::Vector3f(r_state_real.x, r_state_real.y, r_state_real.rz),
                                      &viewer_robot->scene);
            if (not res_o_real.has_value()){   // no control
//                qInfo() << __FUNCTION__ << "NO CONTROL";

                stuck_real = do_if_stuck_real(advance_real, rotation_real, r_state_real, lhit_real, rhit_real);
                if(stuck_real == true){
                    move_robot_real(-constants.backward_speed, 0);
                    return;
                }
                else move_robot_real(100, 0);
            }
            else
            {
                res_real = res_o_real.value();
            }
            auto [_, __, adv, rot, ___] = res_real;
            float dist_break = std::clamp(dist_real / constants.final_distance_to_target - 1.0, -1.0, 1.0);
//            float adv_n = constants.max_advance_speed * dist_break * gaussian(rot);

            // Comprueba que no haya obstáculos por los lados
            auto linl_real = laser_draw_polygon_real->mapFromParent(laser_poly_real);
            auto lp_real = laser_draw_polygon_real->mapFromParent(left_polygon_robot_real);
            if (auto res = std::ranges::find_if_not(lp_real, [linl_real](const auto &p)
                { return linl_real.containsPoint(p, Qt::WindingFill); }); res != std::end(lp_real))
            {
                qInfo() << __FUNCTION__ << "---- TOCANDO POR LA IZQUIERDA-----------" << *res;
                rot += delta_rot_r;
                delta_rot_r *= 1.2;
                lhit_real = true;
            }
            else
            {
                std::cout << "RESTARTING DELTA ROT R" << endl;
                delta_rot_r = constants.initial_delta_rot;
                lhit_real = false;
            }
            auto rp_real = laser_draw_polygon_real->mapFromParent(right_polygon_robot_real);
            if (auto res = std::ranges::find_if_not(rp_real, [linl_real](const auto &p)
                { return linl_real.containsPoint(p, Qt::WindingFill); }); res != std::end(rp_real))
            {
                qInfo() << __FUNCTION__ << "---- TOCANDO POR LA DERECHA-----------" << *res;
                rot -= delta_rot_l;
                delta_rot_l *= 1.2;
                rhit_real = true;
            }
            else
            {
                std::cout << "RESTARTING DELTA ROT L" << endl;
                delta_rot_l = constants.initial_delta_rot;
                rhit_real = false;
            }
            rot = std::clamp(rot, -constants.max_rotation_speed, constants.max_rotation_speed);

//            if(adv < 20) adv = 20;
//            move_robot_real(adv, rot);

//            stuck_real = do_if_stuck_real(adv, rot, r_state_real, lhit_real, rhit_real);
            if (stuck_real == false)
            {
                if(adv < 50) adv = 50;
                move_robot_real(adv*exp(-pow(adv,2)/pow(10,6)), rot);
            }
        }
        else
        {
            cout << "real arrived" << endl;
            move_robot_real(0, 0);
            target.active = false;
        }
    }

}


bool SpecificWorker::do_if_stuck_real(float adv, float rot, const RoboCompFullPoseEstimation::FullPoseEuler &r_state, bool lhit, bool rhit)
{
    //static bool first_time = true;
    const float SMALL_FLOAT = 0.01;
    const float SMALL_INT = 16.0;
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<float> dist(0.3, 1.0);

    qInfo() << __FUNCTION__ << adv << rot;
    if( fabs(adv) < SMALL_INT  and fabs(rot) < SMALL_FLOAT )
    {
        if(lhit){
//            std::cout << "lhit" << endl;
            move_robot_real(-constants.backward_speed, -dist(mt));
        }
        else if(rhit) {
//            std::cout << "rhit" << endl;
            move_robot_real(-constants.backward_speed, dist(mt));
        }
        qInfo() << __FUNCTION__ << "------ STUCK ----------";
        return true;
    }
    return false;
}

std::optional<SpecificWorker::Result> SpecificWorker::control(const Eigen::Vector2f &target_r, const QPolygonF &laser_poly,
                                                              double advance, double rot, const Eigen::Vector3f &robot,
                                                              QGraphicsScene *scene)
{
    static float previous_turn = 0;
    // compute future positions of the robot
    auto point_list = compute_predictions(advance, rot, laser_poly);
    std::cout << "point list lenght: " << point_list.size()<< endl;
    // compute best value
    auto best_choice = compute_optimus(point_list, target_r);
    if(scene != nullptr)
        draw_dwa(robot, point_list, best_choice, scene);
    if (best_choice.has_value())
    {
        auto[x, y, v, w, alpha]= best_choice.value();  // x,y coordinates of best point, v,w velocities to reach that point, alpha robot's angle at that point
        previous_turn = w;

        return best_choice.value();
    }
    else
        return {};
}
std::vector<SpecificWorker::Result> SpecificWorker::compute_predictions(float current_adv, float current_rot, const QPolygonF &laser_poly)
{
    std::vector<Result> list_points;
    for (float v = 0; v <= constants.max_advance_speed; v += 10) //advance
        for (float w = -1; w <= 1; w += 0.1) //rotation
        {
            float new_adv = current_adv + v;
            float new_rot = -current_rot + w;
            if (fabs(w) > 0.001)  // avoid division by zero to compute the radius
            {
                float r = new_adv / new_rot; // radio de giro ubicado en el eje x del robot
                float arc_length = new_rot * constants.time_ahead * r;
                for (float t = constants.step_along_arc; t < arc_length; t += constants.step_along_arc)
                {
                    float x = r - r * cos(t / r); float y= r * sin(t / r);  // circle parametric coordinates
                    auto point = std::make_tuple(x, y, new_adv, new_rot, t / r);
                    bool enought_space = sqrt(x*x + y*y) > constants.robot_semi_width;
                    bool point_reachable = point_reachable_by_robot(point, laser_poly);
                    if(enought_space and point_reachable) // skip points in the robot
                        list_points.emplace_back(std::move(point));
                }
            }
            else // para evitar la división por cero en el cálculo de r
            {
                for(float t = constants.step_along_arc; t < new_adv * constants.time_ahead; t+=constants.step_along_arc)
                {
                    auto point = std::make_tuple(0.f, t, new_adv, new_rot, new_rot * constants.time_ahead);
                    if (t > constants.robot_semi_width and point_reachable_by_robot(point, laser_poly))
                        list_points.emplace_back(std::make_tuple(0.f, t, new_adv, new_rot, new_rot * constants.time_ahead));
                }
            }
        }
    return list_points;
}
bool SpecificWorker::point_reachable_by_robot(const Result &point, const QPolygonF &laser_poly)
{
    auto [x, y, adv, giro, ang] = point;
    auto goal_r = Eigen::Vector2f(x,y);

    // Divide la distancia al punto desde el robot entre la tercera parte de la mitad del ancho del robot
    float parts = goal_r.norm()/(constants.robot_semi_width/3);  //should be a fraction of the arc
    float ang_delta = ang / parts;
    float init_ang = 0;

    auto linl = laser_draw_polygon_real->mapFromParent(laser_poly_real);
    auto lp = laser_draw_polygon_real->mapFromParent(polygon_robot_real);

    for(const auto &l: iter::range(0.0, 1.0, 1.0/parts))
    {
        init_ang += ang_delta;
        // obtiene puntos del robot al punto final con distancia parts
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
    std::vector<std::tuple<float, Result>> values(points.size());
    for(auto &&[k, point] : iter::enumerate(points))
    {
        auto [x, y, adv, giro, ang] = point;
        float dist_to_target = (Eigen::Vector2f(x, y) - tr).norm();
        //float dist_to_previous_turn =  fabs(giro - previous_turn);
        float dist_to_previous_turn =  fabs(giro);
        values[k] = std::make_tuple(constants.A_dist_factor*dist_to_target + constants.B_turn_factor*dist_to_previous_turn, point);
    }
    auto min = std::ranges::min_element(values, [](auto &a, auto &b){ return std::get<0>(a) < std::get<0>(b);});
    if(min != values.end())
        return std::get<Result>(*min);
    else
        return {};
}
void SpecificWorker::draw_dwa(const Eigen::Vector3f &robot, const std::vector <Result> &puntos,
                              const std::optional<Result> &best, QGraphicsScene *scene)
{
    static std::vector<QGraphicsEllipseItem *> arcs_vector;
    for (auto arc: arcs_vector) {
        scene->removeItem(arc);
    }
    arcs_vector.clear();

    QColor col("Blue");
    for (auto &[x, y, vx, wx, a] : puntos)
    {
        //QPointF centro = robot_draw_polygon_draw->mapToScene(x, y);
        QPointF centro = to_qpointf(from_robot_to_world(Eigen::Vector2f(x, y), robot));
        auto arc = scene->addEllipse(centro.x(), centro.y(), 50, 50, QPen(col, 10));
        arc->setZValue(30);
        arcs_vector.push_back(arc);
    }

    if(best.has_value())
    {
        auto &[x, y, _, __, ___] = best.value();
        QPointF selected = to_qpointf(from_robot_to_world(Eigen::Vector2f(x, y), robot));
        auto arc = scene->addEllipse(selected.x(), selected.y(), 180, 180, QPen(Qt::black), QBrush(Qt::black));
        arc->setZValue(30);
        arcs_vector.push_back(arc);
    }
}

std::tuple<RoboCompFullPoseEstimation::FullPoseEuler, double, double> SpecificWorker::read_base_real()
{
    RoboCompFullPoseEstimation::FullPoseEuler r_state;
    float advance, rot;
    try
    {
        r_state = fullposeestimation_proxy->getFullPoseEuler();

        advance = (-sin(r_state.rz)*r_state.vx + cos(r_state.rz)*r_state.vy);
        rot = r_state.vrz;  // Rotation W
//        cout << "advance: " << advance << endl;
//        std::cout << "rot: " << rot << endl;
        robot_draw_polygon_real->setRotation(r_state.rz * 180 / M_PI);
        robot_draw_polygon_real->setPos(r_state.x, r_state.y);
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
    return std::make_tuple(r_state, advance, rot);
}

std::tuple<RoboCompFullPoseEstimation::FullPoseEuler, double, double> SpecificWorker::read_base_coppelia()
{
    RoboCompFullPoseEstimation::FullPoseEuler r_state;
    float advance, rot;

    try
    {
        r_state = fullposeestimation1_proxy->getFullPoseEuler();
        advance = -sin(r_state.rz)*r_state.vx + cos(r_state.rz)*r_state.vy;
        rot = r_state.vrz;  // Rotation W

        robot_draw_polygon_coppelia->setRotation(r_state.rz * 180 / M_PI);
        robot_draw_polygon_coppelia->setPos(r_state.x, r_state.y);
    }
    catch (const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
    return std::make_tuple(r_state, advance, rot);
}

QPolygonF SpecificWorker::read_laser_real()
{
    QPolygonF poly_robot;
    RoboCompLaser::TLaserData ldata;
    try
    {
        ldata = laser_proxy->getLaserData();
        float dist_ant = 50;
        for (auto &&l : ldata)
        {
            if (l.dist < 30)
                l.dist = dist_ant;
            else
                dist_ant = l.dist;
            poly_robot << QPointF(l.dist * sin(l.angle), l.dist * cos(l.angle));
        }
        // Simplify laser contour with Ramer-Douglas-Peucker
        //poly_robot = ramer_douglas_peucker(ldata, constants.MAX_RDP_DEVIATION_mm);
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
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    draw_laser_real(poly_robot);
    return poly_robot;
}

QPolygonF SpecificWorker::read_laser_coppelia()
{
    QPolygonF poly_robot;
    RoboCompLaser::TLaserData ldata;
    try
    {
        ldata = laser1_proxy->getLaserData();
        float dist_ant = 50;
        for (auto &&l : ldata)
        {
            if (l.dist < 30)
                l.dist = dist_ant;
            else
                dist_ant = l.dist;
            poly_robot << QPointF(l.dist * sin(l.angle), l.dist * cos(l.angle));
        }
        // Simplify laser contour with Ramer-Douglas-Peucker
        //poly_robot = ramer_douglas_peucker(ldata, constants.MAX_RDP_DEVIATION_mm);
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
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    draw_laser_coppelia(poly_robot);
    return poly_robot;
}

void SpecificWorker::move_robot_real(float adv, float rot)
{
    if( auto robot = G->get_node("robot"); robot.has_value())
    {
        G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot.value(), adv);
        G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot.value(), rot);
        G->update_node(robot.value());
        std::cout << "VELOCIDAD INTRODUCIDA" << endl;
    }

//    try
//    {
////        cout << "linear speed: " << adv << endl;
////        cout << "rotational speed: " << rot << endl;
////        differentialrobot1_proxy->setSpeedBase(0.5*adv, rot);
//
//        differentialrobot_proxy->setSpeedBase(adv, rot);
//    }
//    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
}

void SpecificWorker::draw_laser_real(const QPolygonF &poly) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    if (laser_polygon != nullptr)
        viewer_robot->scene.removeItem(laser_polygon);

    QColor color("lightRed");
    color.setAlpha(40);
    laser_polygon = viewer_robot->scene.addPolygon(laser_draw_polygon_real->mapToScene(poly), QPen(QColor("DarkRed"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}

void SpecificWorker::draw_laser_coppelia(const QPolygonF &poly) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    if (laser_polygon != nullptr)
        viewer_robot->scene.removeItem(laser_polygon);

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer_robot->scene.addPolygon(laser_draw_polygon_coppelia->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}

void SpecificWorker::draw_timeseries(float rot, float adv, int lhit, int rhit, int stuck, float diff)
{
    static int cont = 0;
    rot_graph->addData(cont, rot);
    adv_graph->addData(cont, adv);
    lhit_graph->addData(cont, lhit);
    rhit_graph->addData(cont, rhit);
    stuck_graph->addData(cont++, stuck);
    diff_graph->addData(cont, diff);
    custom_plot.xAxis->setRange(cont, 200, Qt::AlignRight);
    custom_plot.replot();
}
QPolygonF SpecificWorker::ramer_douglas_peucker(RoboCompLaser::TLaserData &ldata, double epsilon)
{
    if(ldata.size()<2)
    {
        qWarning() << __FUNCTION__ << "Not enough points to simplify";
        return QPolygonF();
    }
    std::vector<Point> pointList(ldata.size());
    float dist_ant = 50;
    for (auto &&[i, l] : ldata | iter::enumerate) {
        if (l.dist < 30)
            l.dist = dist_ant;
        else
            dist_ant = l.dist;
        pointList[i] = std::make_pair(l.dist * sin(l.angle), l.dist * cos(l.angle));
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
Eigen::Vector2f SpecificWorker::from_world_to_robot(const Eigen::Vector2f &p,
                                                    const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    Eigen::Matrix2f matrix;
    matrix << cos(r_state.rz) , -sin(r_state.rz) , sin(r_state.rz) , cos(r_state.rz);
    return (matrix.transpose() * (p - Eigen::Vector2f(r_state.x, r_state.y)));
}
Eigen::Vector2f SpecificWorker::from_robot_to_world(const Eigen::Vector2f &p, const Eigen::Vector3f &robot)
{
    Eigen::Matrix2f matrix;
    const float &angle = robot.z();
    matrix << cos(angle) , -sin(angle) , sin(angle) , cos(angle);
    return (matrix * p) + Eigen::Vector2f(robot.x(), robot.y());
}
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


RoboCompMoveTowards::Command SpecificWorker::MoveTowards_move(float x, float y, float alpha)
{
    qInfo() << __FUNCTION__ ;
    qInfo() << __FUNCTION__ << " Received new target at " << x << y << alpha;
    static float delta_rot = constants.initial_delta_rot;
    bool lhit, rhit, stuck;
    // Target in robot RS. Check if x,y,alpha are within ranges
    target.pos = QPointF(x, y);
    RoboCompMoveTowards::Command command{0.0, 0.0};
    Result res;

    if( auto res_o =  control(target.to_eigen(), laser_poly_real, global_advance, global_rotation,
                              Eigen::Vector3f(r_state_global.x, r_state_global.y, r_state_global.rz),
                              &viewer_robot->scene); not res_o.has_value())
        return command;
    else
        res = res_o.value();

    auto [_, __, adv, rot, ___] = res;
    double dist = Eigen::Vector2d(x,y).norm();
    float dist_break = std::clamp(dist / constants.final_distance_to_target - 1.0, -1.0, 1.0);
    float adv_n = constants.max_advance_speed * dist_break * gaussian(rot);
    auto linl = laser_draw_polygon_real->mapFromParent(laser_poly_real);
    auto lp = laser_draw_polygon_real->mapFromParent(left_polygon_robot_real);
    if (auto res = std::ranges::find_if_not(lp, [linl](const auto &p)
        { return linl.containsPoint(p, Qt::WindingFill); }); res != std::end(lp))
    {
        qInfo() << __FUNCTION__ << "---- TOCANDO POR LA IZQUIERDA-----------" << *res;
        rot += delta_rot;
        delta_rot *= 2;
        lhit = true;
    }
    else
    {
        delta_rot = constants.initial_delta_rot;
        lhit = false;
    }
    auto rp = laser_draw_polygon_real->mapFromParent(right_polygon_robot_real);
    if (auto res = std::ranges::find_if_not(rp, [linl](const auto &p)
        { return linl.containsPoint(p, Qt::WindingFill); }); res != std::end(rp))
    {
        qInfo() << __FUNCTION__ << "---- TOCANDO POR LA DERECHA-----------" << *res;
        rot -= delta_rot;
        delta_rot *= constants.initial_delta_rot;
        rhit = true;
    }
    else
    {
        delta_rot = rot;
        rhit = false;
    }
    rot = std::clamp(rot, -constants.max_rotation_speed, constants.max_rotation_speed);
    move_robot_real(adv_n, rot);
    stuck = do_if_stuck_real(adv_n, rot, r_state_global, lhit, rhit);
}



/**************************************/
// From the RoboCompBillCoppelia you can call this methods:
// this->billcoppelia_proxy->getPose(...)
// this->billcoppelia_proxy->setSpeed(...)
// this->billcoppelia_proxy->setTarget(...)

/**************************************/
// From the RoboCompBillCoppelia you can use this types:
// RoboCompBillCoppelia::Pose

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot1_proxy->correctOdometer(...)
// this->differentialrobot1_proxy->getBasePose(...)
// this->differentialrobot1_proxy->getBaseState(...)
// this->differentialrobot1_proxy->resetOdometer(...)
// this->differentialrobot1_proxy->setOdometer(...)
// this->differentialrobot1_proxy->setOdometerPose(...)
// this->differentialrobot1_proxy->setSpeedBase(...)
// this->differentialrobot1_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompFullPoseEstimation you can call this methods:
// this->fullposeestimation_proxy->getFullPoseEuler(...)
// this->fullposeestimation_proxy->getFullPoseMatrix(...)
// this->fullposeestimation_proxy->setInitialPose(...)

/**************************************/
// From the RoboCompFullPoseEstimation you can use this types:
// RoboCompFullPoseEstimation::FullPoseMatrix
// RoboCompFullPoseEstimation::FullPoseEuler

/**************************************/
// From the RoboCompFullPoseEstimation you can call this methods:
// this->fullposeestimation1_proxy->getFullPoseEuler(...)
// this->fullposeestimation1_proxy->getFullPoseMatrix(...)
// this->fullposeestimation1_proxy->setInitialPose(...)

/**************************************/
// From the RoboCompFullPoseEstimation you can use this types:
// RoboCompFullPoseEstimation::FullPoseMatrix
// RoboCompFullPoseEstimation::FullPoseEuler

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser1_proxy->getLaserAndBStateData(...)
// this->laser1_proxy->getLaserConfData(...)
// this->laser1_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// this->omnirobot_proxy->correctOdometer(...)
// this->omnirobot_proxy->getBasePose(...)
// this->omnirobot_proxy->getBaseState(...)
// this->omnirobot_proxy->resetOdometer(...)
// this->omnirobot_proxy->setOdometer(...)
// this->omnirobot_proxy->setOdometerPose(...)
// this->omnirobot_proxy->setSpeedBase(...)
// this->omnirobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompMoveTowards you can use this types:
// RoboCompMoveTowards::Command

