//
// Created by pbustos on 10/12/21.
//

#ifndef MAPPER_ROOM_DETECTOR_GRAD_STOCHASTIC_H
#define MAPPER_ROOM_DETECTOR_GRAD_STOCHASTIC_H

#include <QRectF>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <QtCore>
#include <iostream>
#include <chrono>
#include "graph_rooms.h"
#include <tuple>

class Room_Detector_Grad_Stochastic
{
    public:
        QRectF compute_room(Eigen::MatrixX3d &points);  // 1.0 ended points
        template <typename T> std::string type_name();
        QRectF minimize_door_distances(Graph_Rooms &G);

    private:
        std::tuple<double, Eigen::ArrayXd>  error(const std::vector<double> &params, const Eigen::MatrixX3d &points, double huber);

        std::tuple<std::vector<double>, double, size_t, Eigen::ArrayXd>
        optimize(const Eigen::MatrixX3d &points,
                 const std::vector<double> &params,
                 const  std::vector<double> &deltas,
                 unsigned int max_iter,
                 double mean_error_to_leave,
                 double  huber);


        std::tuple< std::vector<Graph_Rooms::Room>, double, size_t> optimize_door_distance( std::vector<Graph_Rooms::Room> &rooms,
                                                        const std::vector<Graph_Rooms::Door> &doors,
                                                        const  std::vector<double> &deltas,
                                                        unsigned int max_iter,
                                                        double min_error_to_leave);
    double door_distance_error(const std::vector<Graph_Rooms::Room> &local_rooms,
                               const std::vector<Graph_Rooms::Door> &doors);
};


#endif //MAPPER_ROOM_DETECTOR_GRAD_STOCHASTIC_H
