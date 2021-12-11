//
// Created by pbustos on 10/12/21.
//

#include "room_detector_grad_stocastic.h"
#include <cppitertools/range.hpp>
#include <random>
#include <cppitertools/enumerate.hpp>


QRectF Room_Detector_Grad_Stochastic::compute_room(Eigen::MatrixX3d &points_raw)
{
    // initial values  points_raw: 300 x 3
    qInfo() << __FUNCTION__ << "Points:" << points_raw.rows() << points_raw.cols();
    auto center = points_raw.colwise().mean();
    float cx = center.x();  float cy = center.y();
    auto points_array = points_raw.array();
    auto std_dev = ((points_array.rowwise() - points_array.colwise().mean()).square().colwise().sum()/(points_array.rows()-1)).sqrt();
    float  sw = std_dev.x(); float sh = std_dev.y();
    qInfo() << __FUNCTION__ << " Initial values: " << cx << cy << sw << sh;
    auto params = std::vector<double>{cx, cy, sw, sh};
    double delta = sw/200.0;  // centimeters
    auto deltas = std::vector<double>{-delta, delta};
    qInfo() << __FUNCTION__ << "Optimizing...";
    int max_std_iter = 5; int i=0;
    double mean_error = std::numeric_limits<double>::max();
    std::tuple<std::vector<double>, double, size_t, Eigen::ArrayXd> res;
    while(i++<max_std_iter and mean_error>150)
    {
        res = optimize(points_raw, params, deltas, 20000, 100, std_dev.maxCoeff()*2.1);
        auto &[axis, e, iter, dists] = res;
        mean_error = dists.mean();
        double std_dev_dists = std::sqrt((dists - mean_error).square().sum()/(dists.size()-1));
        qInfo() << __FUNCTION__ << "total error: " << e;
        qInfo() << __FUNCTION__ << "mean: " << mean_error;
        qInfo() << __FUNCTION__ << "Std: " << std_dev_dists;
        qInfo() << __FUNCTION__ << "points:" << dists.rows();
        int outliers = (dists > mean_error).count();
        qInfo() << __FUNCTION__ << "Outliers: " << outliers;
        qInfo() << "--------------------------";
    }
    auto &[axis, e, iter, dists] = res;
    const float &rcx = axis[0];  const float &rcy = axis[1];
    const float &rsw = axis[2];  const float &rsh = axis[3];
    qInfo() << __FUNCTION__ << "Final error:" << e;
    return QRectF(rcx-rsw, rcy-rsh, rsw*2, rsh*2);
}

std::tuple<std::vector<double>, double, size_t, Eigen::ArrayXd>
Room_Detector_Grad_Stochastic::optimize(const Eigen::MatrixX3d &points, const std::vector<double> &params,
                                        const  std::vector<double> &deltas, unsigned int max_iter, double min_error_to_leave, double huber)
{
    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_int_distribution<int> params_selector(0, params.size()-1);
    static std::uniform_int_distribution<int> delta_selector(0, deltas.size()-1);

    std::vector<double> new_params = params;
    float e_ant = std::numeric_limits<double>::max();
    int idx = params_selector(mt);
    int new_delta = delta_selector(mt);
    std::tuple<double, Eigen::ArrayXXd> res;
    size_t loops = 0;
    for(auto &&i : iter::range(max_iter))
    {
        //qInfo() << __FUNCTION__ << idx << new_params[0] << " " << new_params[1] << " " << new_params[2] << " " << new_params[3];
        new_params.at(idx) += new_delta;
        res = error(new_params, points, huber);
        const auto [e, _] = res;
        if( e < min_error_to_leave )
            break;
        if( e >= e_ant)  // time to change param and delta
        {
            idx = params_selector(mt);
            new_delta = delta_selector(mt);
            e_ant = std::numeric_limits<double>::max();
        }
        e_ant = e;
        loops = i;
    }
    return std::make_tuple(new_params, std::get<0>(res), loops, std::get<1>(res));
}
std::tuple<double, Eigen::ArrayXd> Room_Detector_Grad_Stochastic::error(const std::vector<double> &params, const Eigen::MatrixX3d &points, double huber)
{
    const float cx = params[0];
    const float cy = params[1];
    const float sw = params[2];
    const float sh = params[3];
    Eigen::Vector2d center(cx, cy);
    Eigen::Vector2d c1(sw, sh); c1 += center;
    Eigen::Vector2d c2(sw, -sh); c2 += center;
    Eigen::Vector2d c3(-sw, -sh); c3 += center;
    Eigen::Vector2d c4(-sw, sh); c4 += center;
    // compute L matrix 4x3
    Eigen::Matrix<double, 4, 3> L;
    L << c1.y() - c2.y(), c2.x() - c1.x(), (c1.x() - c2.x()) * c1.y() + (c2.y() - c1.y()) * c1.x(),
         c2.y() - c3.y(), c3.x() - c2.x(), (c2.x() - c3.x()) * c2.y() + (c3.y() - c2.y()) * c2.x(),
         c3.y() - c4.y(), c4.x() - c3.x(), (c3.x() - c4.x()) * c3.y() + (c4.y() - c3.y()) * c3.x(),
         c4.y() - c1.y(), c1.x() - c4.x(), (c4.x() - c1.x()) * c4.y() + (c1.y() - c4.y()) * c4.x();
    // compute normalizing line coefficients M 4x1
    Eigen::Vector4d M;
    for( auto &&i : iter::range(L.rows()))
         M[i] = sqrt(L.row(i).x()*L.row(i).x() + L.row(i).y()*L.row(i).y());

    // multiply points times line coefficients: dist = 4x3 * 3*N = 4xN
    auto dist = L * points.transpose();

    // compute absolute value
    auto abs_dist = dist.cwiseAbs();

    // divide by norm coeffs  4xN / colwise
    auto abs_dist_norm = abs_dist.array().colwise() / M.array();

    // Huber robust estimator
    //Eigen::ArrayXXd abs_dist_norm_huber = (abs_dist_norm > huber).select(abs_dist_norm.colwise() -
    //                                                                 Eigen::Array4d(huber/2.0, huber/2.0, huber/2.0, huber/2.0), abs_dist_norm);
    Eigen::ArrayXXd abs_dist_norm_huber = (abs_dist_norm > huber).select(0.0, abs_dist_norm);

    // sum of minimun values minimun value along columns
    auto err = abs_dist_norm_huber.colwise().minCoeff().sum();
    //qInfo() << __FUNCTION__ << err << abs_dist_norm_huber.rows() << abs_dist_norm_huber.cols();
    //std::cout << type_name<decltype(abs_dist_norm_huber)>() << '\n';;
    return std::make_tuple(err, abs_dist_norm_huber.colwise().minCoeff()); //, abs_dist_norm_huber);
};