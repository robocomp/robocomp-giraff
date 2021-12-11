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



class Room_Detector_Grad_Stochastic
{
    public:
        QRectF compute_room(Eigen::MatrixX3d &points);  // 1.0 ended points
        template <typename T> std::string type_name();

    private:
        std::tuple<double, Eigen::ArrayXd>  error(const std::vector<double> &params, const Eigen::MatrixX3d &points, double huber);

        std::tuple<std::vector<double>, double, size_t, Eigen::ArrayXd>
        optimize(const Eigen::MatrixX3d &points, const std::vector<double> &params,
                         const  std::vector<double> &deltas, unsigned int max_iter, double min_error_to_leave, double huber);


};


#endif //MAPPER_ROOM_DETECTOR_GRAD_STOCHASTIC_H
