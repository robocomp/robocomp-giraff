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
#include <opencv2/core/types.hpp>

class Room_Detector_Grad_Stochastic
{
public:
    cv::RotatedRect compute_room(Eigen::MatrixX3d &points);  // 1.0 ended points
    template <typename T> std::string type_name();
    QRectF minimize_door_distances(Graph_Rooms &G);

private:
    struct Constraints
    {
        double MAX_WIDTH;
        double MIN_WIDTH;
        double MAX_HEIGHT;
        double MIN_HEIGHT;
    };
    std::tuple<double, Eigen::ArrayXd>  error(const std::vector<double> &params, const Eigen::MatrixX3d &points, double huber);
    std::tuple<std::vector<double>, double, size_t, Eigen::ArrayXd>
    optimize(const Eigen::MatrixX3d &points,
             const std::vector<double> &params,
             const  std::vector<double> &deltas,
             unsigned int max_iter,
             double mean_error_to_leave,
             double  huber);
    std::tuple< std::vector<Graph_Rooms::Room>, double, size_t> optimize_door_distance( const Graph_Rooms &G,
                                                                                        std::vector<Graph_Rooms::Room> rooms,
                                                                                        const  std::vector<double> &deltas,
                                                                                        unsigned int max_iter,
                                                                                        double min_error_to_leave,
                                                                                        const Constraints &constraints);
    double door_distance_error(const Graph_Rooms &G, const std::vector<Graph_Rooms::Room> &local_rooms);
    bool check_constraints(const Graph_Rooms::Room &r, const Constraints &room_constraints);
};


#endif //MAPPER_ROOM_DETECTOR_GRAD_STOCHASTIC_H
