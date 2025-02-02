#ifndef SLAM_HH
#define SLAM_HH

// #include "catalog.hh"
#include "logging.hh"
#include "types.hh"
#include "utils.hh"

#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>
#include <fmt/chrono.h>
#include <nova/io.hh>
#include <nova/yaml.hh>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <range/v3/all.hpp>

#include <filesystem>
#include <iostream>
#include <range/v3/view/enumerate.hpp>

using yaml = nova::yaml;


class slam {
public:
    slam(const yaml& config, const std::string& in_dir, const std::string& odometry_file, const std::string& out_dir, const std::string& format)
        : m_config(config)
        // , m_catalog(config.lookup<float>("catalog.distance_threshold"))
        , m_spat_cons_buff(m_config.lookup<std::size_t>("spatial_consistency.buff_capacity"))
        , m_out_dir(out_dir)
        , m_format(format)
    {
        logging::info("Spatial consistency buffer capacity: {}", m_spat_cons_buff.capacity());
        // m_motion = nova::read_file<motion_data_parser>(std::filesystem::path(odometry_file).string()).value();

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
            m_landmarks.push_back(lm);
        }

        logging::info("Landmarks read: {}", m_landmarks.size());
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
        processing_times.reserve(m_landmarks.size());
        double velocity = init_velocity;
        double orientation = 0;

        for (const auto& [idx, landmarks] : ranges::views::enumerate(m_landmarks)) {
            logging::debug("Landmarks: {}", landmarks.size());

            const auto start = nova::now();

            if (prev_landmarks.size() > 0) {
                const auto [curr_landmarks, new_prev_landmarks] = pairing(
                    landmarks,
                    prev_landmarks,
                    pairing_dist_threshold,
                    static_cast<double>(input_sampling_rate),
                    velocity,
                    orientation
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
                // std::cout << "Speed: " << velocity << std::endl;
                orientation = std::acos(std::min(std::max(new_trafo_tmp.R(0, 0), -1.), 1.));
                // std::cout << "Orientation: " << orientation << " Angle: " << new_trafo_tmp.R(0, 0) << std::endl;

                trafo = trafo * new_trafo;

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

            for (const auto& landmarks : m_landmarks) {
                for (const auto& lm : landmarks) {
                    raw.emplace_back(lm.x(), lm.y(), 0);
                }
            }

            pcl::io::savePLYFile(fmt::format("{}/raw.ply", m_out_dir), raw);
        } else {
            std::ofstream raw(fmt::format("{}/raw.xyz", m_out_dir));

            for (const auto& landmarks : m_landmarks) {
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

private:
    yaml m_config;
    // catalog m_catalog;
    // std::vector<motion> m_motion;
    boost::circular_buffer<std::vector<nova::Vec2d>> m_spat_cons_buff;
    std::vector<std::vector<nova::Vec2d>> m_landmarks;
    std::string m_out_dir;
    std::string m_format;
};

#endif // SLAM_HH
