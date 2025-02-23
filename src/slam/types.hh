#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <nova/vec.hh>

#include <sstream>
#include <vector>

#include <cmath>


/**
 * @brief Calculate the rotation matrix of a given angle.
 *
 * @tparam T
 * @param theta
 * @return Eigen::Matrix2<T>
 */
template <typename T>
Eigen::Matrix2<T> rotation_matrix_2d(const T& theta) {
    // Eigen::Matrix2<T> result;

    const T cos_theta = ceres::cos(theta);
    const T sin_theta = ceres::sin(theta);

    // result << cos_theta, -sin_theta, sin_theta, cos_theta;

    // return result;

    return Eigen::Matrix2<T>{
        { cos_theta, -sin_theta },
        { sin_theta,  cos_theta }
    };
}

struct landmark_error_func {
    template <typename T>
    bool operator()(const T* const pose, const T* const landmark, const T* const meas, T* residual) const {
        const Eigen::Matrix2<T> rotation = rotation_matrix_2d<T>(pose[2]);

        Eigen::Vector2<T> temp{
            landmark[0] - pose[0],
            landmark[1] - pose[1]
        };

        Eigen::Map<Eigen::Vector2<T>> residual_map(residual);
        residual_map = rotation.transpose() * temp;

        residual_map(0) -= meas[0];
        residual_map(1) -= meas[1];

        return true;
    }
};

struct pose_error_func {
    template <typename T>
    bool operator()(const T* const prev, const T* const curr, const T* const meas, T* residual) const {
        T meas_x_global = meas[0] * ceres::cos(prev[2] + meas[1] / 2.0);
        T meas_y_global = meas[0] * ceres::sin(prev[2] + meas[1] / 2.0);

        residual[0] = (curr[0] - prev[0]) - meas_x_global;
        residual[1] = (curr[1] - prev[1]) - meas_y_global;
        residual[2] = (curr[2] - prev[2]) - meas[1];

        return true;
    }
};

// Generalize
struct vec2d_data_parser {
    [[nodiscard]] auto operator()(std::istream& inf) -> std::vector<nova::Vec2d> {
        std::vector<nova::Vec2d> ret;
        std::stringstream ss;
        for (std::string line; std::getline(inf, line); ) {
            ss << line << '\n';
            double x, y, dummy;
            ss >> x >> y >> dummy;
            ret.emplace_back(x, y);
            ss.clear();
        }
        return ret;
    }
};
