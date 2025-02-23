#pragma once

#include "catalog.hh"
#include "logging.hh"
#include "nova/vec.hh"
#include "types.hh"
#include "utils.hh"

#include <autodiff_cost_function.h>
#include <boost/circular_buffer.hpp>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <fmt/chrono.h>
#include <fmt/ranges.h>
#include <nova/io.hh>
#include <nova/yaml.hh>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <problem.h>
#include <range/v3/all.hpp>
#include <range/v3/view/enumerate.hpp>

#include <filesystem>
#include <iostream>
#include <memory>

using yaml = nova::yaml;


class slam {
public:
    slam(const yaml& config, const std::string& in_dir, const std::string& out_dir, const std::string& format)
        : m_config(config)
        , m_spat_cons_buff(m_config.lookup<std::size_t>("spatial_consistency.buff_capacity"))
        , m_out_dir(out_dir)
        , m_format(format)
    {
        logging::info("Spatial consistency buffer capacity: {}", m_spat_cons_buff.capacity());

        const auto fs_sorted = [in_dir]() {
            const auto extract_number = [](const std::string& filename) {
                std::regex pattern(".*processed-([0-9]+)\\.xyz");
                std::smatch match;
                if (std::regex_search(filename, match, pattern) && match.size() > 1) {
                    return std::stoi(match[1].str());
                }
                // If no match, return a default value or handle error as needed
                return 0; // or throw an exception
            };

            const auto fs = files(in_dir, ".*processed-([0-9]+)\\.xyz");
            auto tmp = fs
                     | ranges::views::transform([extract_number](const auto& x) { return std::make_pair(extract_number(x), x); })
                     | ranges::to<std::vector>();

            ranges::sort(tmp, [](auto& lhs, auto& rhs) { return lhs.first < rhs.first; });

            return tmp | ranges::views::values | ranges::to<std::vector>();
        }();

        for (const auto& f : fs_sorted) {
            auto lm = nova::read_file<vec2d_data_parser>(f).value();
            m_raw_landmarks.push_back(lm);
        }

        logging::info("Landmarks read: {}", m_raw_landmarks.size());
    }

    void do_() {
        const auto pairing_dist_threshold = m_config.lookup<double>("pairing.distance_threshold");
        const auto spat_cons_min_occurrence = m_config.lookup<std::size_t>("spatial_consistency.min_occurrence");
        const auto spat_cons_dist_threshold = m_config.lookup<double>("spatial_consistency.distance_threshold");
        const auto input_sampling_rate = m_config.lookup<std::size_t>("input_sampling_rate");
        const auto init_velocity = m_config.lookup<double>("initial_velocity");

        std::vector<nova::Vec2d> prev_landmarks;
        Eigen::Matrix3d trafo = Eigen::Matrix3d::Identity();
        Eigen::Vector2d pose { 0, 0 };
        std::vector<std::chrono::microseconds> processing_times;
        processing_times.reserve(m_raw_landmarks.size());
        double velocity = init_velocity;
        double angular_velocity = 0;
        double orientation = 0;

        for (const auto& [idx, landmarks] : ranges::views::enumerate(m_raw_landmarks)) {
            logging::debug("Landmarks: {}", landmarks.size());

            const auto start = nova::now();

            if (prev_landmarks.size() > 0) {
                const auto [curr_landmarks, new_prev_landmarks] = pairing(
                    landmarks,
                    prev_landmarks,
                    pairing_dist_threshold,
                    static_cast<double>(input_sampling_rate),
                    velocity,
                    angular_velocity
                );

                const auto [A, B] = conv_pts_to_same_size_mx(new_prev_landmarks, curr_landmarks);
                const auto new_trafo_tmp = rigid_transform_2D(B, A);

                Eigen::Matrix3d new_trafo = Eigen::Matrix3d::Identity();
                new_trafo.block<2, 2>(0, 0) = new_trafo_tmp.R;
                new_trafo.block<2, 1>(0, 2) = new_trafo_tmp.t;

                // std::cout << "Pose: " << pose << std::endl;
                Eigen::Vector2d prev_pose = pose;
                pose += new_trafo_tmp.t;
                pose = new_trafo_tmp.R * pose;

                // std::cout << "R: " << new_trafo_tmp.R << std::endl;

                const auto dist = (pose - prev_pose).norm();
                // std::cout << "Dist: " << dist << std::endl;
                velocity = dist / (1. / static_cast<double>(input_sampling_rate));
                std::cout << "Speed: " << velocity << std::endl;
                angular_velocity = std::acos(std::min(std::max(new_trafo_tmp.R(0, 0), -1.), 1.));
                orientation += angular_velocity;
                std::cout << "Orientation (rad): " << orientation << " Orientation (deg): " << orientation * 180. / M_PI << std::endl;

                trafo = trafo * new_trafo;
                m_poses.emplace_back(pose.x(), pose.y(), orientation);
                // m_motion.emplace_back(velocity * static_cast<double>(input_sampling_rate), angular_velocity * static_cast<double>(input_sampling_rate));
                m_motion.emplace_back(velocity, angular_velocity);

                std::vector<nova::Vec2d> registered;

                for (const auto& p : curr_landmarks) {
                    const Eigen::Vector3d pt = Eigen::Vector3d{ p.x(), p.y(), 1. };
                    const Eigen::Vector3d ptt = trafo * pt;
                    registered.emplace_back(ptt.x(), ptt.y());
                }

                m_spat_cons_buff.push_back(registered);

                if (m_spat_cons_buff.size() >= spat_cons_min_occurrence) {
                    const auto important_points = extract_consistent_points(m_spat_cons_buff, spat_cons_min_occurrence, spat_cons_dist_threshold);

                    if (m_format == "ply") {
                        pcl::PointCloud<pcl::PointXYZ> out;

                        for (const auto& point : important_points) {
                            out.emplace_back(point.x(), point.y(), 0);
                        }

                        pcl::io::savePLYFile(fmt::format("{}/registered-{}.ply", m_out_dir, idx + 1), out);
                    } else {
                        std::ofstream reg(fmt::format("{}/registered-{}.xyz", m_out_dir, idx + 1));

                        for (const auto& p : important_points) {
                            reg << p.x() << " " << p.y() << " " << 0 << "\n";
                        }
                    }

                    m_reg_landmarks.push_back(important_points);
                }
            } else {
                m_spat_cons_buff.push_back(landmarks);
            }

            prev_landmarks = landmarks;

            const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(nova::now() - start);
            processing_times.push_back(duration);
            logging::info("Processing took: {}", duration);
        }

        if (m_format == "ply") {
            pcl::PointCloud<pcl::PointXYZ> raw;

            for (const auto& landmarks : m_raw_landmarks) {
                for (const auto& lm : landmarks) {
                    raw.emplace_back(lm.x(), lm.y(), 0);
                }
            }

            pcl::io::savePLYFile(fmt::format("{}/raw.ply", m_out_dir), raw);
        } else {
            std::ofstream raw(fmt::format("{}/raw.xyz", m_out_dir));

            for (const auto& landmarks : m_raw_landmarks) {
                for (const auto& lm : landmarks) {
                    raw << lm.x() << " " << lm.y() << " " << 0 << "\n";
                }
            }
        }

        std::ofstream stats(fmt::format("{}/stats.json", m_out_dir));

        stats << "{ \"processingTimesUs\": { \"1\": " << processing_times[0].count();

        for (std::size_t i = 1; i < processing_times.size(); ++i) {
            stats << ", \"" << i + 1 << "\": " << processing_times[i].count();
        }

        stats << " } }";
    }

    bool optimize(bool report = false) {
        logging::info("Vec<Pose> size: {}", m_poses.size());
        logging::info("Vec<Motion> size: {}", m_motion.size());
        logging::info("Vec<Landmarks> size: {}", m_raw_landmarks.size());
        logging::info("Vec<Registered> size: {}", m_reg_landmarks.size());

        catalog catalog(m_config.lookup<double>("catalog.distance_threshold"));
        logging::info("Catalog created with distance threshold: {}", m_config.lookup<double>("catalog.distance_threshold"));

        m_solver_options.max_num_iterations = 500;
        m_solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        // m_solver_options.minimizer_progress_to_stdout = true;

        // m_pose_cost_function = std::make_unique<ceres::AutoDiffCostFunction<pose_error_func, 3, 3, 3, 2>>(new pose_error_func);
        // m_landmark_cost_function = std::make_unique<ceres::AutoDiffCostFunction<landmark_error_func, 2, 3, 2, 2>>(new landmark_error_func);
        m_pose_cost_function = new ceres::AutoDiffCostFunction<pose_error_func, 3, 3, 3, 2>(new pose_error_func);
        m_landmark_cost_function = new ceres::AutoDiffCostFunction<landmark_error_func, 2, 3, 2, 2>(new landmark_error_func);

        ceres::Problem problem;
        ceres::Solver::Summary summary;

        for (std::size_t i = 1; i < m_poses.size(); ++i) {
            auto& prev = m_poses[i - 1];
            auto& curr = m_poses[i];

            problem.AddResidualBlock(m_pose_cost_function, nullptr, prev.data(), curr.data(), m_motion[i - 1].data());
            problem.SetParameterBlockConstant(m_motion[i - 1].data());

            auto& measurements = m_reg_landmarks[i - 1];

            for (std::size_t j = 0; j < measurements.size(); ++j) {
                const auto id = catalog.add(measurements[j]);
                auto& lm = catalog.get(id).front();
                problem.AddResidualBlock(m_landmark_cost_function, nullptr, prev.data(), lm.data(), measurements[j].data());
                problem.SetParameterBlockConstant(measurements[j].data());
            }
        }

        // Set anchor for first pose
        problem.SetParameterBlockConstant(m_poses[0].data());

        fmt::print("{}", catalog.dump());

        Solve(m_solver_options, &problem, &summary);

        fmt::print("{}", catalog.dump());

        if (report) {
            std::cerr << summary.FullReport() << std::endl;
        }

        if (m_format == "ply") {
            pcl::PointCloud<pcl::PointXYZ> raw;

            for (const auto& [k, _] : catalog.data()) {
                const auto lm = catalog.get(k).front();
                raw.emplace_back(lm.x(), lm.y(), 0);
            }

            pcl::io::savePLYFile(fmt::format("{}/slam.ply", m_out_dir), raw);
        } else {
            std::ofstream raw(fmt::format("{}/slam.xyz", m_out_dir));

            for (const auto& [k, _] : catalog.data()) {
                const auto lm = catalog.get(k).front();
                raw << lm.x() << " " << lm.y() << " " << 0 << "\n";
            }
        }

        return summary.IsSolutionUsable();
    }

private:
    yaml m_config;
    // catalog m_catalog;
    // std::unique_ptr<ceres::CostFunction> m_pose_cost_function;
    // std::unique_ptr<ceres::CostFunction> m_landmark_cost_function;
    ceres::CostFunction* m_pose_cost_function;
    ceres::CostFunction* m_landmark_cost_function;
    ceres::Solver::Options m_solver_options;
    boost::circular_buffer<std::vector<nova::Vec2d>> m_spat_cons_buff;
    std::vector<std::vector<nova::Vec2d>> m_raw_landmarks;
    std::vector<std::vector<nova::Vec2d>> m_reg_landmarks;
    std::vector<nova::Vec3d> m_poses{ nova::Vec3d{ 0, 0, 0 } };
    std::vector<nova::Vec2d> m_motion;
    std::string m_out_dir;
    std::string m_format;
};
