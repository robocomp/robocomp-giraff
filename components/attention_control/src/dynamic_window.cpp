//
// Created by pbustos on 11/11/21.
//

#include "dynamic_window.h"

Dynamic_Window::Dynamic_Window()
{}

std::tuple<float, float, float> Dynamic_Window::compute(const Eigen::Vector2f &target,
                                                        const std::vector<std::tuple<float, float>> &ldata,
                                                        const Eigen::Vector3f &robot_pos,
                                                        const Eigen::Vector3f &robot_vel)
{
    // posiciones originales del robot
    float current_adv = robot_vel[1]; // Advance V
    float current_rot = robot_vel[2];  // Rotation W
    float robot_angle = robot_pos[2];
    static float previous_turn = 0;

    // calculamos las posiciones futuras del robot y se insertan en un vector.
    auto vector_arcos = calcularPuntos(current_adv, current_rot);

    // quitamos los puntos futuros que nos llevan a obstaculos
    auto vector_sin_obs = obstaculos(vector_arcos, robot_angle, ldata);

    // ordenamos el vector de puntos segun la distancia
    Eigen::Vector2f tr = from_world_to_robot(target, robot_pos);
    auto best_choice = ordenar(vector_sin_obs, tr.x(), tr.y(), bState.x, bState.z, previous_turn);

    if (best_choice.has_value())
    {
        auto[x, y, v, w, alpha] = best_choice.value();
        //std::cout << __FUNCTION__ << " " << x << " " << y << " " << v << " " << w << " " << alpha << std::endl;
        auto va = std::min(v / 5, 1000.f);
        try{  omnirobot_proxy->setSpeedBase(0, va, -w); previous_turn = -w;}  // w should come positive
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
        vector_sin_obs.insert(vector_sin_obs.begin(), best_choice.value());
        return vector_sin_obs;
    }
    else
    {
        //std::cout << "Vector vacio" << std::endl;
        return std::vector<std::tuple<float, float, float>>{};
    }
}

std::vector<std::vector<SpecificWorker::Tupla>> Dynamic_Window::calcularPuntos(float current_adv, float current_rot)
{
    std::vector <Tupla> vectorT;
    std::vector<std::vector<Tupla>> list_arcs;
    const float semiwidth = 50;
    //Calculamos las posiciones futuras del robot y se insertan en un vector.
    float dt = 1.5; // 1 second ahead
    for (float v = -100; v <= 700; v += 100) //advance
    {
        for (float w = -2; w <= 2; w += 0.1) //rotacion
        {
            std::vector<Tupla> list_points;
            float new_adv = current_adv + v;
            float new_rot = current_rot + w;
            if (fabs(w) > 0.01)
            {
                // Nuevo punto posible
                float r = new_adv / new_rot; // radio de giro ubicado en el eje x del robot
                float x = r - r * cos(new_rot * dt); //coordenada futura X
                float y = r * sin(new_rot * dt); //coordenada futura Z
                float alp = new_rot * dt; //angulo nuevo del robot
                float arc_length = new_rot * dt * r;
                for (float t = semiwidth; t < arc_length; t += semiwidth)
                    list_points.emplace_back(std::make_tuple(r - r * cos(t / r), r * sin(t / r), new_adv, new_rot, t / r));
            }
            else // para evitar la división por cero en el cálculo de r
            {
                for(float t = semiwidth; t < v*dt; t+=semiwidth)
                    list_points.emplace_back(std::make_tuple(0.f, t, new_adv, new_rot, new_rot*dt));
            }
            list_arcs.push_back(list_points);
        }
    }
    return list_arcs;
}

std::vector <SpecificWorker::Tupla> Dynamic_Window::obstaculos(std::vector<std::vector<Tupla>> vector_arcs,
                                                               float aph,
                                                               const std::vector<std::tuple<float, float> &ldata)
{
    QPolygonF polygonF_Laser;
    const float semiancho = 250; // el semiancho del robot
    std::vector<Tupla> vector_obs;

    // poligono creado con los puntos del laser
    for (auto &[dist, angle]: ldata)
        polygonF_Laser << QPointF(dist * sin(angle), dist * cos(angle));
    // extend laser to include the robot's body
    //        float size = ROBOT_LENGTH / 1.4;
    //        polygonF_Laser << QPointF(-size,size) << QPointF(-size,-size) << QPointF(size,-size) << QPointF(size,size);

    for(auto &arc_points : vector_arcs)
    {
        for (auto &point : arc_points)
        {
            auto [x, y, adv, giro, ang] = point;
            QPolygonF temp_robot;
            temp_robot << QPointF(x - semiancho, y + semiancho) << QPointF(x + semiancho, y + semiancho) <<
                       QPointF(x + semiancho, y - semiancho) << QPointF(x - semiancho, y - semiancho);
            temp_robot = QTransform().rotate(ang).map(temp_robot);

            //si el poligono del laser no contiene un punto del robot, no contiene alguna esquina por tanto pasamos a otro.

            auto res = std::find_if_not(std::begin(temp_robot), std::end(temp_robot), [polygonF_Laser](const auto &p){return polygonF_Laser.containsPoint(p,Qt::OddEvenFill);});
            if(res == std::end(temp_robot))  //all inside
                vector_obs.emplace_back(point);
            else
                break;
        }
    }
    return vector_obs;
}

/**
 * Ordenamos el vector segun distancia a las coordenadas x y z
 * @param vector
 * @param x
 * @param z
 * @return vector ordenado
 */
std::optional<SpecificWorker::Tupla> Dynamic_Window::ordenar(std::vector<Tupla> vector_points, float tx, float ty, float rx, float ry, float previous_turn)
{
    const float A=1, B=0.1, C=10, D=0;
    int k=0;
    std::vector<std::tuple<float, Tupla>> values;
    values.resize(vector_points.size());
    for(auto &point : vector_points)
    {
        auto [x, y, adv, giro, ang] = point;
        auto va = this->grid.get_value(x,y); auto vb = this->grid.get_value(x,y);
        if(va.has_value() and vb.has_value())
        {
            float nav_function = va.value().dist;
            float dist_to_target = sqrt(pow(tx-x,2)+pow(ty-y,2));
            float dist_to_previous_turn =  fabs(-giro - previous_turn);
            //float dist_from_robot = 1/sqrt(pow(rx-x,2)+pow(ry-y,2));
            //float clearance_to_obstacle = 1/grid.dist_to_nearest_obstacle(x, y);
            values[k++] = std::make_tuple(A * nav_function + B* dist_to_target + C*dist_to_previous_turn, point);
        }
    }
    auto min = std::ranges::min_element(values, [](auto &a, auto &b){ return std::get<0>(a) < std::get<0>(b);});
    if(min != values.end())
        return std::get<Tupla>(*min);
    else
        return {};

//        std::vector<tupla> vdist = vector;
//        std::sort(vdist.begin(), vdist.end(), [x, z, this](const auto &a, const auto &b)
//        {
//            const auto &[ax, ay, ca, cw, aa] = a;
//            const auto &[bx, by, ba, bw, bb] = b;
//            //return ((ax - x) * (ax - x) + (ay - z) * (ay - z)) < ((bx - x) * (bx - x) + (by - z) * (by - z));
//            auto va = this->grid.get_value(ax,ay); auto vb = this->grid.get_value(bx,by);
//            if(va.has_value() and vb.has_value())
//                return va.value().dist < vb.value().dist;
//            else
//                return false;
//        });
//
//        return vector;
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