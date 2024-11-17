#ifndef UTILS_HH
#define UTILS_HH

#include "circle.hh"
#include "ransac.hh"
#include "types.hh"

#include <nova/vec.hh>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/range/conversion.hpp>

#include <ranges>


[[nodiscard]] inline auto flatten(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    pcl::PointCloud<pcl::PointXYZRGB> ret;

    for (const auto& p : cloud) {
        ret.emplace_back(p.x, p.y, 0, p.r, p.g, p.b);
    }

    return ret;
}

[[nodiscard]] inline auto downsample(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    pcl::PointCloud<pcl::PointXYZRGB> ret;

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud.makeShared());
    vg.setLeafSize(0.07f, 0.07f, 0.07f); // TODO: Need a good param - Set the leaf size (adjust as needed)
    vg.filter(ret);

    return ret;
}

[[nodiscard]] inline auto filter_planes(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud = cloud.makeShared();
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    // seg.setOptimizeCoefficients(false);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);

    seg.setInputCloud(_cloud);
    seg.segment(*inliers, *coefficients);

    while (inliers->indices.size() > 500) {
        // extract inliers
        pcl::ExtractIndices<pcl::PointXYZRGB> extractor;
        extractor.setInputCloud(_cloud);
        extractor.setIndices(inliers);
        extractor.setNegative(true); // extract the inliers in consensus model (the part to be removed from point cloud)
        extractor.filter(*_cloud); // cloud_inliers contains the found plane

        seg.segment(*inliers, *coefficients);
    }

    return *_cloud;
}

[[nodiscard]] inline auto cluster(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(cloud);
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(_cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*_cloud, *indices);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);

    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(_cloud);
    reg.setIndices(indices);
    reg.setInputNormals(normals);

    reg.setSmoothnessThreshold(18.0f / 180.0f * std::numbers::pi_v<float>);
    reg.setCurvatureThreshold(1.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
    pcl::io::savePLYFile("./clusters.ply", *reg.getColoredCloud());

    return clusters;
}

[[nodiscard]] inline auto extract_clusters(
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    const std::vector<pcl::PointIndices>& clusters
) {
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>> ret;

    for (const auto& cl : clusters) {
        pcl::PointCloud<pcl::PointXYZRGB> tmp;

        for (const auto& idx : cl.indices) {
            tmp.push_back(cloud.points[idx]);
        }

        ret.push_back(tmp);
    }

    return ret;
}

/*
 * @brief   Finding optimal rotation and translation between corresponding 2D points
 *
 * https://nghiaho.com/?page_id=671
 * https://github.com/nghiaho12/rigid_transform_3D/blob/master/rigid_transform_3D.py
 *
 */
[[nodiscard]] inline auto rigid_transform_2D(const Eigen::MatrixXf& A, const Eigen::MatrixXf& B)
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
    const Eigen::Vector2f centroid_A = A.rowwise().mean();
    const Eigen::Vector2f centroid_B = B.rowwise().mean();

    // Subtract centroids to center the points
    const Eigen::MatrixXf Am = A.colwise() - centroid_A;
    const Eigen::MatrixXf Bm = B.colwise() - centroid_B;

    // Compute the covariance matrix
    const Eigen::Matrix2f H = Am * Bm.transpose();

    // Perform SVD on the covariance matrix H
    Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Calculate the rotation matrix
    const Eigen::Matrix2f Ut = svd.matrixU().transpose();
    Eigen::Matrix2f V = svd.matrixV();

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

[[nodiscard]] auto extract_circle(const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
        -> std::tuple<nova::Vec3f, pcl::PointCloud<pcl::PointXYZRGB>, std::vector<nova::Vec2f>>
{
    const auto points = cloud
                      | std::views::transform([](const auto& elem) { return nova::Vec2f { elem.x, elem.y }; })
                      | std::views::filter([](const auto& elem) { return elem != nova::Vec3f { 0, 0, 0 }; })
                      | ranges::to<std::vector>();

    const auto circle_params = estimate_circle_RANSAC(points, 0.07f, 10'000);
    const auto differences = calculate_RANSAC_diffs(points, circle_params, 0.07f);

    pcl::PointCloud<pcl::PointXYZRGB> circle;
    std::vector<nova::Vec2f> rest;

    for (std::size_t i = 0; i < points.size(); ++i) {
        if (differences.is_inliers.at(i)) {
            circle.emplace_back(points[i].x(), points[i].y(), 0, 255, 0, 0);
        } else {
            rest.push_back(points[i]);
        }
    }

    return {
        circle_params,
        circle,
        rest
    };
}

[[nodiscard]] auto pairing(const std::vector<nova::Vec3f>& params_a, const std::vector<nova::Vec3f>& params_b, float threshold = 0.5f)
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

#endif // UTILS_HH
