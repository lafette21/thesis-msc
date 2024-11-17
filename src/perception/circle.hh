#ifndef CIRCLE_HH
#define CIRCLE_HH

#include "logging.hh"
#include "ransac.hh"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <nova/random.hh>
#include <nova/vec.hh>

#include <ranges>


[[nodiscard]] auto estimate_circle_min(const auto& points)
        -> nova::Vec3f
{
    assert(points.size() == 3);

    std::array<nova::Vec2f, 3> midP;
    std::array<nova::Vec2f, 3> n;
    Eigen::MatrixXf nM(3, 2);
    Eigen::MatrixXf d(3, 1);
    Eigen::MatrixXf S0(2, 1);

    for (std::size_t i = 0; i < 3; ++i) {
        midP[i] = (points[i] + points[(i + 1) % 3]) / 2;
        n[i] = points[i] - midP[i];
        if (n[i].length() != 0) {
            n[i] /= n[i].length(); // TODO: Check
        }
        d(static_cast<int>(i) ,0) =  n[i].x() * midP[i].x() + n[i].y() * midP[i].y();
        nM(static_cast<int>(i), 0) = n[i].x();
        nM(static_cast<int>(i), 1) = n[i].y();
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(nM, Eigen::ComputeThinU | Eigen::ComputeThinV);
    S0 = svd.solve(d);

    const auto r = nova::Vec2f{ S0(0, 0) - points[0].x(), S0(1, 0) - points[0].y() };

    return {
        S0(0, 0),
        S0(1, 0),
        r.length(),
    };
}

[[nodiscard]] auto estimate_circle_lsq(const auto& points, const nova::Vec3f& params_init)
        -> nova::Vec3f
{
    constexpr auto Epsilon = 1e-6f;
    constexpr auto MaxIter = 1000;
    const auto points_size = points.size();

    const auto sum = std::accumulate(std::begin(points), std::end(points), nova::Vec2f { 0, 0 }, std::plus<nova::Vec2f>());
    const auto mean = sum / static_cast<float>(points_size);
    const auto pp = points
                  | std::views::transform([mean](const auto& elem) { return elem - mean; });
    auto S0 = nova::Vec2f{ params_init.x() - mean.x(), params_init.y() - mean.y() };
    float r = params_init.z();

    auto diff = nova::Vec2f{ 1, 1 };
    std::size_t it = 0;

    // TODO: Here I changed the equation from `diff != Point2f{0,0}`
    while (diff.length() < Epsilon && it < MaxIter) {
        float r_avg = 0;
        auto dir_avg = nova::Vec2f{ 0, 0 };
        const auto S0_tmp = S0;

        for (const auto& point : pp) {
            auto dir_i = S0 - nova::Vec2f{ point.x(), point.y() };
            float r_i = dir_i.length();
            dir_avg += dir_i / r_i;
            r_avg += r_i;
        }

        S0 = (dir_avg * r_avg) / static_cast<float>(std::pow(points_size, 2));
        r = r_avg / static_cast<float>(points_size);
        diff = S0 - S0_tmp;
        ++it;
    }

    return {
        S0.x() + mean.x(),
        S0.y() + mean.y(),
        r
    };
}

[[nodiscard]] auto estimate_circle_RANSAC(const auto& points, float threshold, std::size_t iter)
        -> nova::Vec3f
{
    constexpr auto RMax = 0.32f;
    constexpr auto RMin = 0.28f;
    const auto points_size = points.size();
    std::size_t best_sample_inlier_num = 0;
    nova::Vec3f best_circle;

    logging::debug("RANDOM ENGINE SEED: {}", nova::random().seed());

    for (std::size_t i = 0; i < iter; ++i) {
        const auto p1 = nova::random().choice(points);
        auto p2 = nova::random().choice(points);
        while (p1 == p2) { p2 = nova::random().choice(points); }
        auto p3 = nova::random().choice(points);
        while (p1 == p3) { p3 = nova::random().choice(points); }

        const auto min_sample = std::vector<nova::Vec2f>{ p1, p2, p3 };
        const auto sample_circle = estimate_circle_min(min_sample);
        const auto sample_result = calculate_RANSAC_diffs(points, sample_circle, threshold);

        if (sample_result.num_inliers > best_sample_inlier_num
            && sample_result.num_inliers > 20 // TODO: Magic number
            && sample_circle.z() < RMax
            && sample_circle.z() > RMin
        ) {
            best_sample_inlier_num = sample_result.num_inliers;
            best_circle = sample_circle;
            logging::debug("Inlier num. update: {}\t\tradius: {} m", best_sample_inlier_num, best_circle.z());
        }
    }

    // Finally, the circle is refitted from the best consensus set
    const auto best_result = calculate_RANSAC_diffs(points, best_circle, threshold);

    std::vector<nova::Vec2f> inliers;

    for (std::size_t i = 0; i < points_size; ++i) {
        if (best_result.is_inliers[i]) {
            inliers.push_back(points[i]);
        }
    }

    return estimate_circle_lsq(inliers, best_circle);
}

#endif // CIRCLE_HH
