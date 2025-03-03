#pragma once

#include <nova/vec.hh>

#include <vector>


struct RANSAC_diffs {
    std::size_t num_inliers;
    std::vector<float> distances;
    std::vector<bool> is_inliers;
};


/*
 * @brief   Calculate the circle-point differences
 *
 * TODO: generalize
 */
[[nodiscard]] auto calculate_RANSAC_diffs(const auto& points, const nova::Vec3f& circle, float threshold)
        -> RANSAC_diffs
{
    const nova::Vec2f S0{ circle.x(), circle.y() };
    const float r = circle.z();

    std::size_t num_inliers = 0;
    std::vector<float> distances;
    std::vector<bool> is_inliers;

    for (const auto& point : points) {
        const auto tmp = point - S0;
        const float dist = std::abs(tmp.length() - r);

        distances.push_back(dist);
        is_inliers.push_back(dist < threshold);
        num_inliers += dist < threshold ? 1 : 0;
    }

    return {
        .num_inliers = num_inliers,
        .distances = distances,
        .is_inliers = is_inliers,
    };
}
