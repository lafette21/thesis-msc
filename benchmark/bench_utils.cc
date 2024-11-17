#include "types.hh"
#include "utils.hh"

#include <benchmark/benchmark.h>
#include <nova/io.hh>

#include <future>
#include <tuple>
#include <vector>


static void bench_downsample_sim(benchmark::State& state) {
    const auto cloud = nova::read_file<lidar_data_parser>("./benchmark/sim.xyz").value();

    for (auto _ : state) {
        const auto tmp = downsample(cloud);
    }
}

static void bench_downsample_real(benchmark::State& state) {
    const auto cloud = nova::read_file<lidar_data_parser>("./benchmark/real.xyz").value();

    for (auto _ : state) {
        const auto tmp = downsample(cloud);
    }
}

static void bench_filter_planes_downsampled_sim(benchmark::State& state) {
    const auto cloud = downsample(nova::read_file<lidar_data_parser>("./benchmark/sim.xyz").value());

    for (auto _ : state) {
        const auto tmp = filter_planes(cloud);
    }
}

static void bench_filter_planes_downsampled_real(benchmark::State& state) {
    const auto cloud = downsample(nova::read_file<lidar_data_parser>("./benchmark/real.xyz").value());

    for (auto _ : state) {
        const auto tmp = filter_planes(cloud);
    }
}

static void bench_cluster_filtered_downsampled_sim(benchmark::State& state) {
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/sim.xyz").value()));

    for (auto _ : state) {
        const auto tmp = cluster(cloud);
    }
}

static void bench_cluster_filtered_downsampled_real(benchmark::State& state) {
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/real.xyz").value()));

    for (auto _ : state) {
        const auto tmp = cluster(cloud);
    }
}

static void bench_extract_clusters_filtered_downsampled_sim(benchmark::State& state) {
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/sim.xyz").value()));
    const auto clusters = cluster(cloud);

    for (auto _ : state) {
        const auto tmp = extract_clusters(cloud, clusters);
    }
}

static void bench_extract_clusters_filtered_downsampled_real(benchmark::State& state) {
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/real.xyz").value()));
    const auto clusters = cluster(cloud);

    for (auto _ : state) {
        const auto tmp = extract_clusters(cloud, clusters);
    }
}

static void bench_extract_1_circle_sim(benchmark::State& state) {
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/sim.xyz").value()));
    const auto clusters = cluster(cloud);
    const auto point_clouds = extract_clusters(cloud, clusters);

    for (auto _ : state) {
        const auto tmp = extract_circle(point_clouds[0]);
    }
}

static void bench_extract_1_circle_real(benchmark::State& state) {
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/real.xyz").value()));
    const auto clusters = cluster(cloud);
    const auto point_clouds = extract_clusters(cloud, clusters);

    for (auto _ : state) {
        const auto tmp = extract_circle(point_clouds[0]);
    }
}

static void bench_extract_circles_par_sim(benchmark::State& state) {
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/sim.xyz").value()));
    const auto clusters = cluster(cloud);
    const auto point_clouds = extract_clusters(cloud, clusters);

    for (auto _ : state) {
        std::vector<std::future<std::tuple<nova::Vec3f, pcl::PointCloud<pcl::PointXYZRGB>, std::vector<nova::Vec2f>>>> futures;

        for (const auto& elem : point_clouds) {
            futures.push_back(std::async(extract_circle, elem));
        }

        for (auto& f : futures) {
            const auto tmp = f.get();
        }
    }
}

static void bench_extract_circles_par_real(benchmark::State& state) {
    const auto cloud = filter_planes(downsample(nova::read_file<lidar_data_parser>("./benchmark/real.xyz").value()));
    const auto clusters = cluster(cloud);
    const auto point_clouds = extract_clusters(cloud, clusters);

    for (auto _ : state) {
        std::vector<std::future<std::tuple<nova::Vec3f, pcl::PointCloud<pcl::PointXYZRGB>, std::vector<nova::Vec2f>>>> futures;

        for (const auto& elem : point_clouds) {
            futures.push_back(std::async(extract_circle, elem));
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
