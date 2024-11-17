#include "logging.hh"
#include "utils.hh"
#include "types.hh"

#include <nova/io.hh>

#include <boost/program_options.hpp>
#include <fmt/chrono.h>
#include <fmt/format.h>
#include <range/v3/view/enumerate.hpp>
#include <spdlog/spdlog.h>

#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <future>
#include <ranges>

namespace po = boost::program_options;


auto pairing(const std::vector<nova::Vec3f>& params_a, const std::vector<nova::Vec3f>& params_b, float threshold = 0.5f)
        -> std::pair<std::vector<nova::Vec3f>, std::vector<nova::Vec3f>>
{
    std::vector<nova::Vec3f> ret_a;
    std::vector<nova::Vec3f> ret_b;
    std::vector<std::vector<float>> dist_mx;

    for (const auto& a : params_a) {
        dist_mx.emplace_back(std::vector<float>{});
        auto& vec = dist_mx.back();
        const auto& c_a = nova::Vec2f{ a.x(), a.y() };

        for (const auto& b : params_b) {
            const auto& c_b = nova::Vec2f{ b.x(), b.y() };
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
        const auto idx_b = std::distance(vec.begin(), std::ranges::find(vec, min));

        if (min < threshold) {
            ret_a.push_back(params_a[idx]);
            ret_b.push_back(params_b[idx_b]);
            logging::debug("({}, {})\t({}, {})\tdist: {}", params_a[idx].x(), params_a[idx].y(), params_b[idx_b].x(), params_b[idx_b].y(), min);
        }
    }

    return { ret_a, ret_b };
}


auto parse_args(int argc, char* argv[])
        -> std::optional<po::variables_map>
{
    auto arg_parser = po::options_description("Perception");
    arg_parser.add_options()
        // ("config,c", po::value<std::string>()->required()->value_name("FILE"), "Config file")
        ("indir,i", po::value<std::string>()->required()->value_name("DIR"), "Input directory (file name format: `test_fn{num}.xyz`)")
        ("start,s", po::value<std::size_t>()->required()->value_name("START"), "Start of the range")
        ("end,e", po::value<std::size_t>()->required()->value_name("END"), "End of the range")
        ("outdir,o", po::value<std::string>()->required()->value_name("DIR")->default_value("out"), "Output directory")
        ("help,h", "Show this help");

    po::variables_map args;
    po::store(po::parse_command_line(argc, argv, arg_parser), args);

    if (args.contains("help")) {
        std::cerr << arg_parser << std::endl;
        return std::nullopt;
    }

    args.notify();

    return args;
}


int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[]) {
    [[maybe_unused]] auto& logger = init("logger");

    try {
        const auto args = parse_args(argc, argv);

        if (not args.has_value()) {
            return EXIT_SUCCESS;
        }

        const auto in_dir = (*args)["indir"].as<std::string>();
        const auto from = (*args)["start"].as<std::size_t>();
        const auto to = (*args)["end"].as<std::size_t>();
        const auto out_dir = (*args)["outdir"].as<std::string>();

        if (not std::filesystem::exists(out_dir)) {
            if (std::filesystem::create_directory(out_dir)) {
                logging::info("Directory created: {}", out_dir);
            } else {
                logging::error("Failed to create directory: {}", out_dir);
            }
        }

        std::vector<pcl::PointCloud<pcl::PointXYZRGB>> clouds;
        clouds.reserve(to - from);

        logging::info("Reading cloud(s)");

        for (std::size_t i = from; i < to; ++i) {
            const auto cloud = nova::read_file<lidar_data_parser>(
                (std::filesystem::path(in_dir) / fmt::format("test_fn{}.xyz", i)).string()
            ).value();
            clouds.push_back(cloud);
        }

        logging::info("Read {} cloud(s)", clouds.size());
        logging::info("Processing cloud(s)");

        pcl::PointCloud<pcl::PointXYZRGB> out;
        std::vector<nova::Vec3f> prev_circle_params;
        Eigen::Matrix3f trafo = Eigen::Matrix3f::Identity();

        for (const auto& [idx, cloud] : ranges::views::enumerate(clouds)) {
            logging::debug("Cloud size: {}", cloud.size());

            const auto start = nova::now();

            const auto downsampled = downsample(cloud);
            const auto filtered = filter_planes(downsampled);
            const auto flattened = flatten(filtered);
            // const auto downsampled = downsample(flattened);
            const auto clusters = cluster(flattened);

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
                futures.push_back(std::async(extract_circle, elem));
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
                const auto [curr_circle_params, new_prev_circle_params] = pairing(circle_params, prev_circle_params);

                pcl::PointCloud<pcl::PointXYZRGB> prev_points;

                for (const auto& params : new_prev_circle_params) {
                    prev_points.emplace_back(params.x(), params.y(), 0, 0, 255, 0);
                }

                pcl::PointCloud<pcl::PointXYZRGB> curr_points;

                for (const auto& params : curr_circle_params) {
                    curr_points.emplace_back(params.x(), params.y(), 0, 0, 255, 0);
                }

                out += curr_points;

                const auto min_size = std::min(std::size(prev_points), std::size(curr_points));

                Eigen::MatrixXf A = Eigen::MatrixXf::Zero(2, static_cast<int>(min_size));
                Eigen::MatrixXf B = Eigen::MatrixXf::Zero(2, static_cast<int>(min_size));

                for (std::size_t i = 0; i < min_size; ++i) {
                    A(0, static_cast<int>(i)) = prev_points[i].x;
                    A(1, static_cast<int>(i)) = prev_points[i].y;

                    B(0, static_cast<int>(i)) = curr_points[i].x;
                    B(1, static_cast<int>(i)) = curr_points[i].y;
                }

                const auto new_trafo_tmp = rigid_transform_2D(B, A);

                Eigen::Matrix3f new_trafo = Eigen::Matrix3f::Identity();
                new_trafo.block<2, 2>(0, 0) = new_trafo_tmp.R;
                new_trafo.block<2, 1>(0, 2) = new_trafo_tmp.t;

                trafo = trafo * new_trafo;

                pcl::PointCloud<pcl::PointXYZRGB> registered;

                for (const auto& p : curr_points) {
                    const Eigen::Vector3f pt = Eigen::Vector3f{ p.x, p.y, 1.0f };
                    const Eigen::Vector3f ptt = trafo * pt;
                    registered.emplace_back(ptt.x(), ptt.y(), 0, 255, 0, 0);
                }

                for (const auto& p : prev_points) {
                    registered.emplace_back(p.x, p.y, 0, 0, 255, 0);
                }

                pcl::io::savePLYFile(fmt::format("{}/registered-{}.ply", out_dir, idx), registered);
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

        pcl::io::savePLYFile(fmt::format("{}/raw.ply", out_dir), out);

    } catch (const std::exception& ex) {
        logging::error("Fatal error happened: {}", ex.what());
        return EXIT_FAILURE;
    }
}
