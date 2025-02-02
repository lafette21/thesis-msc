#pragma once

#include "logging.hh"

#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>
#include <nova/vec.hh>
#include <range/v3/view/enumerate.hpp>

#include <vector>
#include <string>
#include <filesystem>
#include <regex>


[[nodiscard]] inline auto files(const std::string& folder, const std::string& pattern = ".*")
        -> std::vector<std::string>
{
    namespace fs = std::filesystem;

    std::vector<std::string> ret;

    for (const auto & entry : fs::directory_iterator(folder)) {
        std::string full_path = entry.path().string();

        if (fs::is_regular_file(entry) && std::regex_match(entry.path().filename().string(), std::regex(pattern))) {
            ret.push_back(fs::absolute(full_path).string());
        }
    }

    return ret;
}

[[nodiscard]] inline auto pairing(
    const std::vector<nova::Vec2d>& curr,
    const std::vector<nova::Vec2d>& prev,
    double threshold,
    double input_sampling_rate,
    double velocity,
    double orientation
)
        -> std::pair<std::vector<nova::Vec2d>, std::vector<nova::Vec2d>>
{
    std::vector<nova::Vec2d> ret_a;
    std::vector<nova::Vec2d> ret_b;
    std::vector<std::vector<double>> dist_mx;

    // Predict movement based on velocity
    double delta_time = 1. / input_sampling_rate;
    double delta_x = velocity * delta_time * std::cos(orientation);
    double delta_y = velocity * delta_time * std::sin(orientation);

    for (const auto& a : curr) {
        dist_mx.emplace_back(std::vector<double>{});
        auto& vec = dist_mx.back();
        const auto& c_a = nova::Vec2d{ a.x(), a.y() };

        for (const auto& b : prev) {
            const auto predicted_pos = b + nova::Vec2d{ delta_x, delta_y };
            const auto c_b = nova::Vec2d{ predicted_pos.x(), predicted_pos.y() };
            vec.push_back((c_a - c_b).length());
        }
    }

    // for (const auto& vec : dist_mx) {
        // for (const auto& elem : vec) {
            // std::cout << elem << ", ";
        // }
        // std::cout << std::endl;
    // }

    for (const auto& [idx, vec] : ranges::views::enumerate(dist_mx)) {
        const auto& min = std::ranges::min(vec);
        const auto idx_b = static_cast<std::size_t>(std::distance(vec.begin(), std::ranges::find(vec, min)));

        if (min < threshold) {
            ret_a.push_back(curr[idx]);
            ret_b.push_back(prev[idx_b]);
            logging::debug("({}, {})\t({}, {})\tdist: {}", curr[idx].x(), curr[idx].y(), prev[idx_b].x(), prev[idx_b].y(), min);
        }
    }

    return { ret_a, ret_b };
}

[[nodiscard]] inline auto conv_pts_to_same_size_mx(const std::vector<nova::Vec2d>& points_A, const std::vector<nova::Vec2d>& points_B)
     -> std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
{
    const auto min_size = std::min(std::size(points_A), std::size(points_B));

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, static_cast<int>(min_size));
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2, static_cast<int>(min_size));

    for (std::size_t i = 0; i < min_size; ++i) {
        A(0, static_cast<int>(i)) = points_A[i].x();
        A(1, static_cast<int>(i)) = points_A[i].y();

        B(0, static_cast<int>(i)) = points_B[i].x();
        B(1, static_cast<int>(i)) = points_B[i].y();
    }

    return { A, B };
}

struct trafo_2d {
    Eigen::Matrix2d R;
    Eigen::Vector2d t;
};

/*
 * @brief   Finding optimal rotation and translation between corresponding 2D points
 *
 * https://nghiaho.com/?page_id=671
 * https://github.com/nghiaho12/rigid_transform_3D/blob/master/rigid_transform_3D.py
 *
 */
[[nodiscard]] inline auto rigid_transform_2D(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
        -> trafo_2d
{
    assert(A.rows() == B.rows() and A.cols() == B.cols());

    if (A.rows() != 2) {
        throw std::runtime_error("Matrix A is not 2xN!");
    }

    if (B.rows() != 2) {
        throw std::runtime_error("Matrix B is not 2xN!");
    }

    trafo_2d ret;

    // Compute centroids of A and B
    const Eigen::Vector2d centroid_A = A.rowwise().mean();
    const Eigen::Vector2d centroid_B = B.rowwise().mean();

    // Subtract centroids to center the points
    const Eigen::MatrixXd Am = A.colwise() - centroid_A;
    const Eigen::MatrixXd Bm = B.colwise() - centroid_B;

    // Compute the covariance matrix
    const Eigen::Matrix2d H = Am * Bm.transpose();

    // Perform SVD on the covariance matrix H
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Calculate the rotation matrix
    const Eigen::Matrix2d Ut = svd.matrixU().transpose();
    Eigen::Matrix2d V = svd.matrixV();

    ret.R = V * Ut;

    // Handle the special case for reflection
    if (ret.R.determinant() < 0) {
        V.col(1) *= -1;
        ret.R = V * Ut;
    }

    // Calculate the translation vector
    ret.t = -ret.R * centroid_A + centroid_B;

    return ret;
}

[[nodiscard]] inline auto extract_consistent_points(const boost::circular_buffer<std::vector<nova::Vec2d>>& buffer, std::size_t min_occurrence, double threshold)
        -> std::vector<nova::Vec2d>
{
    std::vector<nova::Vec2d> ret;
    const auto& last = buffer.back();

    for (const auto& p : last) {
        std::size_t count = 1;

        for (std::size_t i = 0; i < buffer.size() - 1; ++i) {
            const auto& cloud = buffer[i];

            const auto predicate = [&p, threshold](const auto& elem) {
                const auto dist = (p - elem).length();
                logging::debug("x1={}, y1={}, x2={}, y2={}, dist={}", p.x(), p.y(), elem.x(), elem.y(), dist);
                return dist <= threshold;
            };

            const auto it = std::ranges::find_if(cloud, predicate);

            if (it != std::end(cloud)) {
                count += 1;
            }
        }

        if (count >= min_occurrence) {
            ret.push_back(p);
        }
    }

    return ret;
}
