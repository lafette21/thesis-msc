#include "perception/types.hh"
#include "perception/utils.hh"

#include <nova/io.hh>
#include <nova/json.hh>
#include <nova/vec.hh>

#include <benchmark/benchmark.h>

#include <future>
#include <string>
#include <tuple>
#include <vector>

using json = nova::json;

auto create_config()
        -> json
{
    return json(R"json({
        "downsampling": {
            "leaf_size": {
                "x": 0.07,
                "y": 0.07,
                "z": 0.07,
                "_unit": "m"
            }
        },
        "plane_filtering": {
            "distance_threshold": 0.1,
            "_unit": "m",
            "min_inliers": 500
        },
        "clustering": {
            "k_search": 50,
            "cluster_size": {
                "max": 1000000,
                "min": 50
            },
            "num_of_neighbours": 30,
            "smoothness_threshold": 0.314159265,
            "_unit": "rad",
            "_comment": "18.f / 180.f * pi",
            "curvature_threshold": 1.0
        },
        "circle_extraction": {
            "ransac": {
                "distance_threshold": 0.07,
                "_unit": "m",
                "iter": 10000,
                "min_samples": 20,
                "r_max": 0.32,
                "r_min": 0.28
            }
        },
        "pairing": {
            "distance_threshold": 0.5,
            "_unit": "m"
        }
    })json");
}


static void bench_downsample_sim(benchmark::State& state) {
    const auto config = create_config();
    const auto leaf_size = nova::Vec3f{
        config.lookup<float>("downsampling.leaf_size.x"),
        config.lookup<float>("downsampling.leaf_size.y"),
        config.lookup<float>("downsampling.leaf_size.z"),
    };
    const auto cloud = nova::read_file<lidar_data_parser>("./benchmark/sim.xyz").value();

    for (auto _ : state) {
        const auto tmp = downsample(cloud, leaf_size);
    }
}

static void bench_downsample_real(benchmark::State& state) {
    const auto config = create_config();
    const auto leaf_size = nova::Vec3f{
        config.lookup<float>("downsampling.leaf_size.x"),
        config.lookup<float>("downsampling.leaf_size.y"),
        config.lookup<float>("downsampling.leaf_size.z"),
    };
    const auto cloud = nova::read_file<lidar_data_parser>("./benchmark/real.xyz").value();

    for (auto _ : state) {
        const auto tmp = downsample(cloud, leaf_size);
    }
}

static void bench_filter_planes_downsampled_sim(benchmark::State& state) {
    const auto config = create_config();
    const auto leaf_size = nova::Vec3f{
        config.lookup<float>("downsampling.leaf_size.x"),
        config.lookup<float>("downsampling.leaf_size.y"),
        config.lookup<float>("downsampling.leaf_size.z"),
    };
    const auto fp_dist_threshold = config.lookup<double>("plane_filtering.distance_threshold");
    const auto fp_min_inliers = config.lookup<std::size_t>("plane_filtering.min_inliers");
    const auto cloud = downsample(nova::read_file<lidar_data_parser>("./benchmark/sim.xyz").value(), leaf_size);

    for (auto _ : state) {
        const auto tmp = filter_planes(cloud, fp_dist_threshold, fp_min_inliers);
    }
}

static void bench_filter_planes_downsampled_real(benchmark::State& state) {
    const auto config = create_config();
    const auto leaf_size = nova::Vec3f{
        config.lookup<float>("downsampling.leaf_size.x"),
        config.lookup<float>("downsampling.leaf_size.y"),
        config.lookup<float>("downsampling.leaf_size.z"),
    };
    const auto fp_dist_threshold = config.lookup<double>("plane_filtering.distance_threshold");
    const auto fp_min_inliers = config.lookup<std::size_t>("plane_filtering.min_inliers");
    const auto cloud = downsample(nova::read_file<lidar_data_parser>("./benchmark/real.xyz").value(), leaf_size);

    for (auto _ : state) {
        const auto tmp = filter_planes(cloud, fp_dist_threshold, fp_min_inliers);
    }
}

static void bench_cluster_filtered_downsampled_sim(benchmark::State& state) {
    const auto config = create_config();
    const auto leaf_size = nova::Vec3f{
        config.lookup<float>("downsampling.leaf_size.x"),
        config.lookup<float>("downsampling.leaf_size.y"),
        config.lookup<float>("downsampling.leaf_size.z"),
    };
    const auto fp_dist_threshold = config.lookup<double>("plane_filtering.distance_threshold");
    const auto fp_min_inliers = config.lookup<std::size_t>("plane_filtering.min_inliers");
    const auto c_k_search = config.lookup<int>("clustering.k_search");
    const auto c_cluster_size_max = config.lookup<unsigned>("clustering.cluster_size.max");
    const auto c_cluster_size_min = config.lookup<unsigned>("clustering.cluster_size.min");
    const auto c_num_of_neighbours = config.lookup<unsigned>("clustering.num_of_neighbours");
    const auto c_smoothness_threshold = config.lookup<float>("clustering.smoothness_threshold");
    const auto c_curvature_threshold = config.lookup<float>("clustering.curvature_threshold");
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/sim.xyz").value(), leaf_size), fp_dist_threshold, fp_min_inliers);

    for (auto _ : state) {
        const auto tmp = cluster(cloud, c_k_search, c_cluster_size_max, c_cluster_size_min, c_num_of_neighbours, c_smoothness_threshold, c_curvature_threshold);
    }
}

static void bench_cluster_filtered_downsampled_real(benchmark::State& state) {
    const auto config = create_config();
    const auto leaf_size = nova::Vec3f{
        config.lookup<float>("downsampling.leaf_size.x"),
        config.lookup<float>("downsampling.leaf_size.y"),
        config.lookup<float>("downsampling.leaf_size.z"),
    };
    const auto fp_dist_threshold = config.lookup<double>("plane_filtering.distance_threshold");
    const auto fp_min_inliers = config.lookup<std::size_t>("plane_filtering.min_inliers");
    const auto c_k_search = config.lookup<int>("clustering.k_search");
    const auto c_cluster_size_max = config.lookup<unsigned>("clustering.cluster_size.max");
    const auto c_cluster_size_min = config.lookup<unsigned>("clustering.cluster_size.min");
    const auto c_num_of_neighbours = config.lookup<unsigned>("clustering.num_of_neighbours");
    const auto c_smoothness_threshold = config.lookup<float>("clustering.smoothness_threshold");
    const auto c_curvature_threshold = config.lookup<float>("clustering.curvature_threshold");
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/real.xyz").value(), leaf_size), fp_dist_threshold, fp_min_inliers);

    for (auto _ : state) {
        const auto tmp = cluster(cloud, c_k_search, c_cluster_size_max, c_cluster_size_min, c_num_of_neighbours, c_smoothness_threshold, c_curvature_threshold);
    }
}

static void bench_extract_clusters_filtered_downsampled_sim(benchmark::State& state) {
    const auto config = create_config();
    const auto leaf_size = nova::Vec3f{
        config.lookup<float>("downsampling.leaf_size.x"),
        config.lookup<float>("downsampling.leaf_size.y"),
        config.lookup<float>("downsampling.leaf_size.z"),
    };
    const auto fp_dist_threshold = config.lookup<double>("plane_filtering.distance_threshold");
    const auto fp_min_inliers = config.lookup<std::size_t>("plane_filtering.min_inliers");
    const auto c_k_search = config.lookup<int>("clustering.k_search");
    const auto c_cluster_size_max = config.lookup<unsigned>("clustering.cluster_size.max");
    const auto c_cluster_size_min = config.lookup<unsigned>("clustering.cluster_size.min");
    const auto c_num_of_neighbours = config.lookup<unsigned>("clustering.num_of_neighbours");
    const auto c_smoothness_threshold = config.lookup<float>("clustering.smoothness_threshold");
    const auto c_curvature_threshold = config.lookup<float>("clustering.curvature_threshold");
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/sim.xyz").value(), leaf_size), fp_dist_threshold, fp_min_inliers);
    const auto clusters = cluster(cloud, c_k_search, c_cluster_size_max, c_cluster_size_min, c_num_of_neighbours, c_smoothness_threshold, c_curvature_threshold);

    for (auto _ : state) {
        const auto tmp = extract_clusters(cloud, clusters);
    }
}

static void bench_extract_clusters_filtered_downsampled_real(benchmark::State& state) {
    const auto config = create_config();
    const auto leaf_size = nova::Vec3f{
        config.lookup<float>("downsampling.leaf_size.x"),
        config.lookup<float>("downsampling.leaf_size.y"),
        config.lookup<float>("downsampling.leaf_size.z"),
    };
    const auto fp_dist_threshold = config.lookup<double>("plane_filtering.distance_threshold");
    const auto fp_min_inliers = config.lookup<std::size_t>("plane_filtering.min_inliers");
    const auto c_k_search = config.lookup<int>("clustering.k_search");
    const auto c_cluster_size_max = config.lookup<unsigned>("clustering.cluster_size.max");
    const auto c_cluster_size_min = config.lookup<unsigned>("clustering.cluster_size.min");
    const auto c_num_of_neighbours = config.lookup<unsigned>("clustering.num_of_neighbours");
    const auto c_smoothness_threshold = config.lookup<float>("clustering.smoothness_threshold");
    const auto c_curvature_threshold = config.lookup<float>("clustering.curvature_threshold");
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/real.xyz").value(), leaf_size), fp_dist_threshold, fp_min_inliers);
    const auto clusters = cluster(cloud, c_k_search, c_cluster_size_max, c_cluster_size_min, c_num_of_neighbours, c_smoothness_threshold, c_curvature_threshold);

    for (auto _ : state) {
        const auto tmp = extract_clusters(cloud, clusters);
    }
}

static void bench_extract_1_circle_sim(benchmark::State& state) {
    const auto config = create_config();
    const auto leaf_size = nova::Vec3f{
        config.lookup<float>("downsampling.leaf_size.x"),
        config.lookup<float>("downsampling.leaf_size.y"),
        config.lookup<float>("downsampling.leaf_size.z"),
    };
    const auto fp_dist_threshold = config.lookup<double>("plane_filtering.distance_threshold");
    const auto fp_min_inliers = config.lookup<std::size_t>("plane_filtering.min_inliers");
    const auto c_k_search = config.lookup<int>("clustering.k_search");
    const auto c_cluster_size_max = config.lookup<unsigned>("clustering.cluster_size.max");
    const auto c_cluster_size_min = config.lookup<unsigned>("clustering.cluster_size.min");
    const auto c_num_of_neighbours = config.lookup<unsigned>("clustering.num_of_neighbours");
    const auto c_smoothness_threshold = config.lookup<float>("clustering.smoothness_threshold");
    const auto c_curvature_threshold = config.lookup<float>("clustering.curvature_threshold");
    const auto ce_ransac_threshold = config.lookup<float>("circle_extraction.ransac.distance_threshold");
    const auto ce_ransac_iter = config.lookup<std::size_t>("circle_extraction.ransac.iter");
    const auto ce_ransac_min_samples = config.lookup<std::size_t>("circle_extraction.ransac.min_samples");
    const auto ce_ransac_r_max = config.lookup<float>("circle_extraction.ransac.r_max");
    const auto ce_ransac_r_min = config.lookup<float>("circle_extraction.ransac.r_min");
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/sim.xyz").value(), leaf_size), fp_dist_threshold, fp_min_inliers);
    const auto clusters = cluster(cloud, c_k_search, c_cluster_size_max, c_cluster_size_min, c_num_of_neighbours, c_smoothness_threshold, c_curvature_threshold);
    const auto point_clouds = extract_clusters(cloud, clusters);

    for (auto _ : state) {
        const auto tmp = extract_circle(point_clouds[0], ce_ransac_threshold, ce_ransac_iter, ce_ransac_min_samples, ce_ransac_r_max, ce_ransac_r_min);
    }
}

static void bench_extract_1_circle_real(benchmark::State& state) {
    const auto config = create_config();
    const auto leaf_size = nova::Vec3f{
        config.lookup<float>("downsampling.leaf_size.x"),
        config.lookup<float>("downsampling.leaf_size.y"),
        config.lookup<float>("downsampling.leaf_size.z"),
    };
    const auto fp_dist_threshold = config.lookup<double>("plane_filtering.distance_threshold");
    const auto fp_min_inliers = config.lookup<std::size_t>("plane_filtering.min_inliers");
    const auto c_k_search = config.lookup<int>("clustering.k_search");
    const auto c_cluster_size_max = config.lookup<unsigned>("clustering.cluster_size.max");
    const auto c_cluster_size_min = config.lookup<unsigned>("clustering.cluster_size.min");
    const auto c_num_of_neighbours = config.lookup<unsigned>("clustering.num_of_neighbours");
    const auto c_smoothness_threshold = config.lookup<float>("clustering.smoothness_threshold");
    const auto c_curvature_threshold = config.lookup<float>("clustering.curvature_threshold");
    const auto ce_ransac_threshold = config.lookup<float>("circle_extraction.ransac.distance_threshold");
    const auto ce_ransac_iter = config.lookup<std::size_t>("circle_extraction.ransac.iter");
    const auto ce_ransac_min_samples = config.lookup<std::size_t>("circle_extraction.ransac.min_samples");
    const auto ce_ransac_r_max = config.lookup<float>("circle_extraction.ransac.r_max");
    const auto ce_ransac_r_min = config.lookup<float>("circle_extraction.ransac.r_min");
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/real.xyz").value(), leaf_size), fp_dist_threshold, fp_min_inliers);
    const auto clusters = cluster(cloud, c_k_search, c_cluster_size_max, c_cluster_size_min, c_num_of_neighbours, c_smoothness_threshold, c_curvature_threshold);
    const auto point_clouds = extract_clusters(cloud, clusters);

    for (auto _ : state) {
        const auto tmp = extract_circle(point_clouds[0], ce_ransac_threshold, ce_ransac_iter, ce_ransac_min_samples, ce_ransac_r_max, ce_ransac_r_min);
    }
}

static void bench_extract_circles_par_sim(benchmark::State& state) {
    const auto config = create_config();
    const auto leaf_size = nova::Vec3f{
        config.lookup<float>("downsampling.leaf_size.x"),
        config.lookup<float>("downsampling.leaf_size.y"),
        config.lookup<float>("downsampling.leaf_size.z"),
    };
    const auto fp_dist_threshold = config.lookup<double>("plane_filtering.distance_threshold");
    const auto fp_min_inliers = config.lookup<std::size_t>("plane_filtering.min_inliers");
    const auto c_k_search = config.lookup<int>("clustering.k_search");
    const auto c_cluster_size_max = config.lookup<unsigned>("clustering.cluster_size.max");
    const auto c_cluster_size_min = config.lookup<unsigned>("clustering.cluster_size.min");
    const auto c_num_of_neighbours = config.lookup<unsigned>("clustering.num_of_neighbours");
    const auto c_smoothness_threshold = config.lookup<float>("clustering.smoothness_threshold");
    const auto c_curvature_threshold = config.lookup<float>("clustering.curvature_threshold");
    const auto ce_ransac_threshold = config.lookup<float>("circle_extraction.ransac.distance_threshold");
    const auto ce_ransac_iter = config.lookup<std::size_t>("circle_extraction.ransac.iter");
    const auto ce_ransac_min_samples = config.lookup<std::size_t>("circle_extraction.ransac.min_samples");
    const auto ce_ransac_r_max = config.lookup<float>("circle_extraction.ransac.r_max");
    const auto ce_ransac_r_min = config.lookup<float>("circle_extraction.ransac.r_min");
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/sim.xyz").value(), leaf_size), fp_dist_threshold, fp_min_inliers);
    const auto clusters = cluster(cloud, c_k_search, c_cluster_size_max, c_cluster_size_min, c_num_of_neighbours, c_smoothness_threshold, c_curvature_threshold);
    const auto point_clouds = extract_clusters(cloud, clusters);

    for (auto _ : state) {
        std::vector<std::future<std::tuple<nova::Vec3f, pcl::PointCloud<pcl::PointXYZRGB>, std::vector<nova::Vec2f>>>> futures;

        for (const auto& elem : point_clouds) {
            futures.push_back(std::async(extract_circle, elem, ce_ransac_threshold, ce_ransac_iter, ce_ransac_min_samples, ce_ransac_r_max, ce_ransac_r_min));
        }

        for (auto& f : futures) {
            const auto tmp = f.get();
        }
    }
}

static void bench_extract_circles_par_real(benchmark::State& state) {
    const auto config = create_config();
    const auto leaf_size = nova::Vec3f{
        config.lookup<float>("downsampling.leaf_size.x"),
        config.lookup<float>("downsampling.leaf_size.y"),
        config.lookup<float>("downsampling.leaf_size.z"),
    };
    const auto fp_dist_threshold = config.lookup<double>("plane_filtering.distance_threshold");
    const auto fp_min_inliers = config.lookup<std::size_t>("plane_filtering.min_inliers");
    const auto c_k_search = config.lookup<int>("clustering.k_search");
    const auto c_cluster_size_max = config.lookup<unsigned>("clustering.cluster_size.max");
    const auto c_cluster_size_min = config.lookup<unsigned>("clustering.cluster_size.min");
    const auto c_num_of_neighbours = config.lookup<unsigned>("clustering.num_of_neighbours");
    const auto c_smoothness_threshold = config.lookup<float>("clustering.smoothness_threshold");
    const auto c_curvature_threshold = config.lookup<float>("clustering.curvature_threshold");
    const auto ce_ransac_threshold = config.lookup<float>("circle_extraction.ransac.distance_threshold");
    const auto ce_ransac_iter = config.lookup<std::size_t>("circle_extraction.ransac.iter");
    const auto ce_ransac_min_samples = config.lookup<std::size_t>("circle_extraction.ransac.min_samples");
    const auto ce_ransac_r_max = config.lookup<float>("circle_extraction.ransac.r_max");
    const auto ce_ransac_r_min = config.lookup<float>("circle_extraction.ransac.r_min");
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/real.xyz").value(), leaf_size), fp_dist_threshold, fp_min_inliers);
    const auto clusters = cluster(cloud, c_k_search, c_cluster_size_max, c_cluster_size_min, c_num_of_neighbours, c_smoothness_threshold, c_curvature_threshold);
    const auto point_clouds = extract_clusters(cloud, clusters);

    for (auto _ : state) {
        std::vector<std::future<std::tuple<nova::Vec3f, pcl::PointCloud<pcl::PointXYZRGB>, std::vector<nova::Vec2f>>>> futures;

        for (const auto& elem : point_clouds) {
            futures.push_back(std::async(extract_circle, elem, ce_ransac_threshold, ce_ransac_iter, ce_ransac_min_samples, ce_ransac_r_max, ce_ransac_r_min));
        }

        for (auto& f : futures) {
            const auto tmp = f.get();
        }
    }
}

BENCHMARK(bench_downsample_sim);
BENCHMARK(bench_downsample_real);
BENCHMARK(bench_filter_planes_downsampled_sim);
BENCHMARK(bench_filter_planes_downsampled_real);
BENCHMARK(bench_cluster_filtered_downsampled_sim);
BENCHMARK(bench_cluster_filtered_downsampled_real);
BENCHMARK(bench_extract_clusters_filtered_downsampled_sim);
BENCHMARK(bench_extract_clusters_filtered_downsampled_real);
BENCHMARK(bench_extract_1_circle_sim);
BENCHMARK(bench_extract_1_circle_real);
BENCHMARK(bench_extract_circles_par_sim);
BENCHMARK(bench_extract_circles_par_real);

BENCHMARK_MAIN();
