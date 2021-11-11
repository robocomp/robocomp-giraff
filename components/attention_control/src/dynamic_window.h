//
// Created by pbustos on 11/11/21.
//

#ifndef ATTENTION_CONTROL_DYNAMIC_WINDOW_H
#define ATTENTION_CONTROL_DYNAMIC_WINDOW_H

#include <tuple>
#include <vector>
#include <optional>
#include <Eigen/Dense>

class Dynamic_Window
{
    public:
        Dynamic_Window();
        using Tupla = std::tuple<float, float, float, float, float>;
        std::tuple<float, float, float> compute(const Eigen::Vector2f &target,
                                                const std::vector<std::tuple<float, float>> &ldata,
                                                const Eigen::Vector3f &robot_pos,
                                                const Eigen::Vector3f &robot_vel);

    private:
        std::vector<std::vector<Tupla>> calcularPuntos(float current_adv, float current_rot);
        std::vector <Tupla> obstaculos(std::vector<std::vector<Tupla>> vector_arcs,
                                                   float aph,
                                                   const  std::vector<std::tuple<float, float>> &ldata);
        std::optional<Tupla> ordenar(std::vector<Tupla> vector_points, float tx, float ty, float rx, float ry, float previous_turn);
        Eigen::Vector2f from_robot_to_world(const Eigen::Vector2f &p, const Eigen::Vector3f &robot);
        Eigen::Vector2f from_world_to_robot(const Eigen::Vector2f &p, const Eigen::Vector3f &robot);
};

#endif //ATTENTION_CONTROL_DYNAMIC_WINDOW_H
