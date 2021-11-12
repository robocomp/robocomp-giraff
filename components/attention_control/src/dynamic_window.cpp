//
// Created by pbustos on 11/11/21.
//

#include "dynamic_window.h"
#include <QtCore>
#include <cppitertools/enumerate.hpp>

Dynamic_Window::Dynamic_Window()
{
    polygon_robot <<  QPointF(-SEMI_WIDTH, SEMI_WIDTH) << QPointF(SEMI_WIDTH, SEMI_WIDTH) <<
                      QPointF(SEMI_WIDTH, -SEMI_WIDTH) << QPointF(-SEMI_WIDTH, -SEMI_WIDTH);
}

Dynamic_Window::Result Dynamic_Window::compute(const Eigen::Vector2f &target_w,
                                                            const QPolygonF &laser_poly,
                                                            const Eigen::Vector3f &robot_pos,
                                                            const Eigen::Vector3f &robot_vel,
                                                            QGraphicsScene *scene)
{
    static float previous_turn = 0;
    float robot_angle = robot_pos[2];
    // advance velocity should come from robot. It is computed here from world referenced velocities
    float current_adv = -sin(robot_angle)*robot_vel[0] + cos(robot_angle)*robot_vel[1];
    float current_rot = robot_vel[2];  // Rotation W

    // compute future positions of the robot
    auto point_list = compute_predictions(current_adv, current_rot, laser_poly);

    // compute best value
    Eigen::Vector2f target_r = from_world_to_robot(target_w, robot_pos);
    auto best_choice = compute_optimus(point_list, target_r, robot_pos, previous_turn);

    if(scene != nullptr)
        draw(robot_pos, point_list, best_choice, scene);

    if (best_choice.has_value())
    {
        auto[x, y, v, w, alpha]= best_choice.value();  // x,y coordinates of best point, v,w velocities to reach that point, alpha robot's angle at that point
        previous_turn = w;
        auto va = std::min(v / 5, 1000.f);
        return best_choice.value();
    }
    else
        return Result{};
}

std::vector<Dynamic_Window::Result> Dynamic_Window::compute_predictions(float current_adv, float current_rot, const QPolygonF &laser_poly)
{
    std::vector<Result> list_points;
    const float semiwidth = 60;  // advance step along arc
    //Calculamos las posiciones futuras del robot y se insertan en un vector.
    float dt = 1.5; // 1 second ahead
    for (float v = -100; v <= 800; v += 100) //advance
        for (float w = -2; w <= 2; w += 0.2) //rotation
        {
            float new_adv = current_adv + v;
            float new_rot = -current_rot + w;
            if (fabs(w) > 0.001)  // avoid division by zero to compute the radius
            {
                // Nuevo punto posible
                float r = new_adv / new_rot; // radio de giro ubicado en el eje x del robot
                float arc_length = new_rot * dt * r;
                for (float t = semiwidth; t < arc_length; t += semiwidth)
                {
                    float x = r - r * cos(t / r); float y= r * sin(t / r);
                    auto point = std::make_tuple(x, y, new_adv, new_rot, t / r);
                    if(sqrt(x*x + y*y)> SEMI_WIDTH and not point_on_obstacle(point, laser_poly)) // skip points in the robot
                        list_points.emplace_back(std::move(point));
                }
            }
            else // para evitar la división por cero en el cálculo de r
            {
                for(float t = semiwidth; t < new_adv*dt; t+=semiwidth)
                {
                    auto point = std::make_tuple(0.f, t, new_adv, new_rot, new_rot * dt);
                    if (t > SEMI_WIDTH and not point_on_obstacle(point, laser_poly))
                        list_points.emplace_back(std::make_tuple(0.f, t, new_adv, new_rot, new_rot * dt));
                }
            }
        }
    return list_points;
}
bool Dynamic_Window::point_on_obstacle(const Result &point, const QPolygonF &laser_poly)
{
        auto [x, y, adv, giro, ang] = point;
        auto temp_robot = QTransform().rotate(ang).translate(x,y).map(polygon_robot);

        //si el poligono del laser no contiene un punto del robot, no contiene alguna esquina por tanto pasamos a otro.
        if( auto res = std::find_if_not(std::begin(temp_robot), std::end(temp_robot),
                                    [laser_poly](const auto &p){return laser_poly.containsPoint(p, Qt::OddEvenFill);}); res == std::end(temp_robot))
            return false;
        else
            return true;
}

std::optional<Dynamic_Window::Result> Dynamic_Window::compute_optimus(const std::vector<Result> &points, const Eigen::Vector2f &tr,
                                                                      const Eigen::Vector3f &robot, float previous_turn)
{
    const float A=1, B=1, C=0, D=10;
    int k=0;
    std::vector<std::tuple<float, Result>> values(points.size());
    for(auto &&[k, point] : iter::enumerate(points))
    {
        auto [x, y, adv, giro, ang] = point;
        //std::cout << __FUNCTION__ << " " << x << " " << y << " " << adv<< " " << giro << " " << ang << std::endl;
        float dist_to_target = (Eigen::Vector2f(x, y) - tr).norm();
        float dist_to_previous_turn =  fabs(-giro - previous_turn);
        //float dist_from_robot = 1/sqrt(pow(rx-x,2)+pow(ry-y,2));
        //float clearance_to_obstacle = 1/grid.dist_to_nearest_obstacle(x, y);
        values[k] = std::make_tuple(B*dist_to_target + C*dist_to_previous_turn, point);
    }
    auto min = std::ranges::min_element(values, [](auto &a, auto &b){ return std::get<0>(a) < std::get<0>(b);});
    if(min != values.end())
        return std::get<Result>(*min);
    else
        return {};
}
Eigen::Vector2f Dynamic_Window::from_robot_to_world(const Eigen::Vector2f &p, const Eigen::Vector3f &robot)
{
    Eigen::Matrix2f matrix;
    const float &angle = robot.z();
    matrix << cos(angle) , -sin(angle) , sin(angle) , cos(angle);
    return (matrix * p) + Eigen::Vector2f(robot.x(), robot.y());
}

Eigen::Vector2f Dynamic_Window::from_world_to_robot(const Eigen::Vector2f &p, const Eigen::Vector3f &robot)
{
    Eigen::Matrix2f matrix;
    const float &angle = robot.z();
    matrix << cos(angle) , -sin(angle) , sin(angle) , cos(angle);
    return (matrix.transpose() * (p - Eigen::Vector2f(robot.x(), robot.y())));
}

void Dynamic_Window::draw(const Eigen::Vector3f &robot, const std::vector <Result> &puntos,  const std::optional<Result> &best, QGraphicsScene *scene)
{
    // draw future. Draw and arch going out from the robot
    // remove existing arcspwd
    static std::vector<QGraphicsEllipseItem *> arcs_vector;
    for (auto arc: arcs_vector)
        scene->removeItem(arc);
    arcs_vector.clear();

    QColor col("Blue");
    for (auto &[x, y, vx, wx, a] : puntos)
    {
        //QPointF centro = robot_polygon_draw->mapToScene(x, y);
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
