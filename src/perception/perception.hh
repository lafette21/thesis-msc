#ifndef PERCEPTION_HH
#define PERCEPTION_HH

#include "logging.hh"
#include "types.hh"
#include "utils.hh"

#include <fmt/chrono.h>
#include <nova/error.hh>
#include <nova/io.hh>
#include <nova/vec.hh>
#include <nova/yaml.hh>
#include <range/v3/view.hpp>

#include <future>
#include <optional>
#include <random>
#include <string>

using yaml = nova::yaml;


class perception {
public:
    perception(const yaml& config, std::size_t start, std::size_t end, const std::string& in_dir, const std::string& out_dir, const std::string& format)
        : m_config(config)
        , m_out_dir(out_dir)
        , m_format(format)
    {
        m_clouds.reserve(end - start + 1);

        logging::info("Reading cloud(s)");

        for (std::size_t i = start; i <= end; ++i) {
            const auto cloud = nova::read_file<lidar_data_parser>(
                (std::filesystem::path(in_dir) / fmt::format("test_fn{}.xyz", i)).string()
            ).value();
            m_clouds.push_back(cloud);
        }

        logging::info("Read {} cloud(s)", m_clouds.size());
    }

    auto start() {
        std::optional<std::random_device::result_type> seed = std::nullopt;
        try {
            const auto _seed = m_config.lookup<std::random_device::result_type>("random.seed");
            seed = _seed;
        } catch (const nova::parsing_error&) {
        }

        const auto downsampling_enabled = m_config.lookup<bool>("downsampling.enabled");
        const auto leaf_size = nova::Vec3f{
            m_config.lookup<float>("downsampling.leaf_size.x"),
            m_config.lookup<float>("downsampling.leaf_size.y"),
            m_config.lookup<float>("downsampling.leaf_size.z"),
        };
        const auto fp_dist_threshold = m_config.lookup<double>("plane_filtering.distance_threshold");
        const auto fp_min_inliers = m_config.lookup<std::size_t>("plane_filtering.min_inliers");
        const auto c_k_search = m_config.lookup<int>("clustering.k_search");
        const auto c_cluster_size_max = m_config.lookup<unsigned>("clustering.cluster_size.max");
        const auto c_cluster_size_min = m_config.lookup<unsigned>("clustering.cluster_size.min");
        const auto c_num_of_neighbours = m_config.lookup<unsigned>("clustering.num_of_neighbours");
        const auto c_smoothness_threshold = m_config.lookup<float>("clustering.smoothness_threshold");
        const auto c_curvature_threshold = m_config.lookup<float>("clustering.curvature_threshold");
        const auto ce_ransac_threshold = m_config.lookup<float>("circle_extraction.ransac.distance_threshold");
        const auto ce_ransac_iter = m_config.lookup<std::size_t>("circle_extraction.ransac.iter");
        const auto ce_ransac_min_samples = m_config.lookup<std::size_t>("circle_extraction.ransac.min_samples");
        const auto ce_ransac_r_max = m_config.lookup<float>("circle_extraction.ransac.r_max");
        const auto ce_ransac_r_min = m_config.lookup<float>("circle_extraction.ransac.r_min");
        const auto pairing_dist_threshold = m_config.lookup<float>("pairing.distance_threshold");

        logging::info("Processing cloud(s)");

        pcl::PointCloud<pcl::PointXYZRGB> out;
        std::vector<nova::Vec3f> prev_circle_params;
        Eigen::Matrix3f trafo = Eigen::Matrix3f::Identity();

        for (const auto& [idx, cloud] : ranges::views::enumerate(m_clouds)) {
            logging::debug("Cloud size: {}", cloud.size());

            const auto start = nova::now();

            pcl::PointCloud<pcl::PointXYZRGB>& new_cloud = cloud;

            if (downsampling_enabled) {
                new_cloud = downsample(cloud, leaf_size);
            }
            const auto filtered = filter_planes(new_cloud, fp_dist_threshold, fp_min_inliers);
            const auto flattened = flatten(filtered);
            // const auto downsampled = downsample(flattened);
            const auto clusters = cluster(flattened, c_k_search, c_cluster_size_max, c_cluster_size_min, c_num_of_neighbours, c_smoothness_threshold, c_curvature_threshold);

            // pcl::io::savePLYFile("./flattened.ply", flattened);
            // break;

            logging::info("Clusters found: {}", clusters.size());

            for (const auto& cl : clusters) {
                logging::debug("Cluster size: {}", cl.indices.size());
            }

            const auto point_clouds = extract_clusters(filtered, clusters);

            logging::debug("Point clouds extracted: {}", point_clouds.size());

            for (const auto& elem : point_clouds) {
                logging::debug("Cloud size: {}", elem.size());
            }

            std::vector<std::future<std::tuple<nova::Vec3f, pcl::PointCloud<pcl::PointXYZRGB>, std::vector<nova::Vec2f>>>> futures;

            for (const auto& elem : point_clouds) {
                if (elem.size() >= ce_ransac_min_samples) {
                    futures.push_back(std::async(extract_circle, elem, ce_ransac_threshold, ce_ransac_iter, ce_ransac_min_samples, ce_ransac_r_max, ce_ransac_r_min, seed));
                }
            }

            std::vector<nova::Vec3f> circle_params;

            for (auto& f : futures) {
                const auto [params, cylinder, rest] = f.get();

                if (std::isnan(params.x()) or std::isnan(params.y()) or std::isnan(params.z())) {
                    continue;
                }

                circle_params.push_back(params);
            }

            // for (const auto& p : circle_params) {
                // fmt::print("{} {}\n", p.x(), p.y());
            // }

            if (prev_circle_params.size() > 0) {
                const auto [curr_circle_params, new_prev_circle_params] = pairing(circle_params, prev_circle_params, pairing_dist_threshold);

                pcl::PointCloud<pcl::PointXYZRGB> prev_points;

                for (const auto& params : new_prev_circle_params) {
                    prev_points.emplace_back(params.x(), params.y(), 0, 0, 255, 0);
                }

                pcl::PointCloud<pcl::PointXYZRGB> curr_points;

                for (const auto& params : curr_circle_params) {
                    curr_points.emplace_back(params.x(), params.y(), 0, 0, 255, 0);
                }

                out += curr_points;

                const auto [A, B] = conv_pts_to_same_size_mx(prev_points, curr_points);
                const auto new_trafo_tmp = rigid_transform_2D(B, A);

                Eigen::Matrix3f new_trafo = Eigen::Matrix3f::Identity();
                new_trafo.block<2, 2>(0, 0) = new_trafo_tmp.R;
                new_trafo.block<2, 1>(0, 2) = new_trafo_tmp.t;

                trafo = trafo * new_trafo;

                if (m_format == "ply") {
                    pcl::PointCloud<pcl::PointXYZRGB> registered;

                    for (const auto& p : curr_points) {
                        const Eigen::Vector3f pt = Eigen::Vector3f{ p.x, p.y, 1.0f };
                        const Eigen::Vector3f ptt = trafo * pt;
                        registered.emplace_back(ptt.x(), ptt.y(), 0, 255, 0, 0);
                    }

                    // for (const auto& p : prev_points) {
                        // registered.emplace_back(p.x, p.y, 0, 0, 255, 0);
                    // }

                    pcl::io::savePLYFile(fmt::format("{}/registered-{}.ply", m_out_dir, idx), registered);
                } else {
                    std::ofstream reg(fmt::format("{}/registered-{}.xyz", m_out_dir, idx));

                    for (const auto& p : curr_points) {
                        const Eigen::Vector3f pt = Eigen::Vector3f{ p.x, p.y, 1.0f };
                        const Eigen::Vector3f ptt = trafo * pt;
                        reg << ptt.x() << " " << ptt.y() << " " << 0 << "\n";
                    }
                }
            } else {
                pcl::PointCloud<pcl::PointXYZRGB> points;

                for (const auto& params : circle_params) {
                    points.emplace_back(params.x(), params.y(), 0, 0, 255, 0);
                }

                out += points;
            }

            prev_circle_params = circle_params;

            logging::info("Processing took: {}", std::chrono::duration_cast<std::chrono::milliseconds>(nova::now() - start));
        }

        if (m_format == "ply") {
            pcl::io::savePLYFile(fmt::format("{}/raw.ply", m_out_dir), out);
        } else {
            std::ofstream raw(fmt::format("{}/raw.xyz", m_out_dir));

            for (const auto& p : out) {
                raw << p.x << " " << p.y << " " << 0 << "\n";
            }
        }
    }

private:
    yaml m_config;
    std::string m_out_dir;
    std::string m_format;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> m_clouds;
};

#endif // PERCEPTION_HH
