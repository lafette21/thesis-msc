#include "perception/logging.hh"

// #include "perception/utils.hh"
#include <nova/vec.hh>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <range/v3/view.hpp>


[[nodiscard]] inline auto pairing(
    const std::vector<nova::Vec3f>& curr,
    const std::vector<nova::Vec3f>& prev,
    float threshold,
    float input_sampling_rate,
    float velocity,
    float orientation
)
        -> std::pair<std::vector<nova::Vec3f>, std::vector<nova::Vec3f>>
{
    std::vector<nova::Vec3f> ret_a;
    std::vector<nova::Vec3f> ret_b;
    std::vector<std::vector<float>> dist_mx;

    // Predict movement based on velocity
    float delta_time = 1.f / input_sampling_rate;
    float delta_x = velocity * delta_time * std::cos(orientation);
    float delta_y = velocity * delta_time * std::sin(orientation);

    for (const auto& a : curr) {
        dist_mx.emplace_back(std::vector<float>{});
        auto& vec = dist_mx.back();
        const auto& c_a = nova::Vec2f{ a.x(), a.y() };

        for (const auto& b : prev) {
            const auto predicted_pos = b + nova::Vec3f{ delta_x, delta_y, 0 };
            const auto c_b = nova::Vec2f{ predicted_pos.x(), predicted_pos.y() };
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
            ret_a.push_back(curr[idx]);
            ret_b.push_back(prev[idx_b]);
            logging::debug("({}, {})\t({}, {})\tdist: {}", curr[idx].x(), curr[idx].y(), prev[idx_b].x(), prev[idx_b].y(), min);
        }
    }

    return { ret_a, ret_b };
}


TEST(Utils, Pairing) {
    const auto vec1 = nova::Vec3f{ 1.5, 1, 0 };
    const auto vec2 = nova::Vec3f{ 1, 1, 0 };

    const auto threshold = 0.000001f;
    const auto input_sampling_rate = 4.f;
    const auto velocity = 2.f;
    const auto orientation = 0;

    EXPECT_EQ(
        pairing({ vec1 }, { vec2 }, threshold, input_sampling_rate, velocity, orientation),
        ( std::pair<std::vector<nova::Vec3f>, std::vector<nova::Vec3f>>{ { vec1 }, { vec2 } } )
    );
}

// Test case for when vectors are far apart
TEST(Utils, Pairing_FarApart) {
    const auto vec1 = nova::Vec3f{ 10.0, 10.0, 0 };
    const auto vec2 = nova::Vec3f{ 0.0, 0.0, 0 };

    const auto threshold = 0.1f;
    const auto input_sampling_rate = 1.f;
    const auto velocity = 0.f; // No movement expected
    const auto orientation = 0;

    auto result = pairing({ vec1 }, { vec2 }, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_TRUE(result.first.empty()); // Should not pair because distance is too large
    EXPECT_TRUE(result.second.empty());
}

// Test case for vectors with movement along X-axis
TEST(Utils, Pairing_WithMovementX) {
    const auto vec1 = nova::Vec3f{ 1.25, 1.0, 0 };
    const auto vec2 = nova::Vec3f{ 1.0, 1.0, 0 };

    const auto threshold = 0.1f;
    const auto input_sampling_rate = 4.f;
    const auto velocity = 1.0f; // Moving 1 unit per second
    const auto orientation = 0; // Moving along positive X-axis

    auto result = pairing({ vec1 }, { vec2 }, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_EQ(result.first.size(), 1);
    EXPECT_EQ(result.second.size(), 1);
    EXPECT_NEAR(result.first[0].x(), vec1.x(), threshold);
    EXPECT_NEAR(result.second[0].x(), vec2.x(), threshold);
}

// Test case for vectors with movement along Y-axis
TEST(Utils, Pairing_WithMovementY) {
    const auto vec1 = nova::Vec3f{ 1.0, 1.25, 0 };
    const auto vec2 = nova::Vec3f{ 1.0, 1.0, 0 };

    const auto threshold = 0.1f;
    const auto input_sampling_rate = 4.f;
    const auto velocity = 1.0f; // Moving 1 unit per second
    const auto orientation = M_PI / 2; // Moving along positive Y-axis

    auto result = pairing({ vec1 }, { vec2 }, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_EQ(result.first.size(), 1);
    EXPECT_EQ(result.second.size(), 1);
    EXPECT_NEAR(result.first[0].y(), vec1.y(), threshold);
    EXPECT_NEAR(result.second[0].y(), vec2.y(), threshold);
}

// Test case for vectors with movement along Y-axis
TEST(Utils, Pairing_WithMovementYNeg) {
    const auto vec1 = nova::Vec3f{ 1.0, 0.75, 0 };
    const auto vec2 = nova::Vec3f{ 1.0, 1.0, 0 };

    const auto threshold = 0.1f;
    const auto input_sampling_rate = 4.f;
    const auto velocity = 1.0f; // Moving 1 unit per second
    const auto orientation = -M_PI / 2; // Moving along positive Y-axis

    auto result = pairing({ vec1 }, { vec2 }, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_EQ(result.first.size(), 1);
    EXPECT_EQ(result.second.size(), 1);
    EXPECT_NEAR(result.first[0].y(), vec1.y(), threshold);
    EXPECT_NEAR(result.second[0].y(), vec2.y(), threshold);
}

// Test case for multiple vectors, ensuring correct pairing
TEST(Utils, Pairing_MultipleVectors) {
    const std::vector<nova::Vec3f> curr = {
        nova::Vec3f{ 1.5, 1.5, 0 },
        nova::Vec3f{ 2.5, 2.5, 0 }
    };
    const std::vector<nova::Vec3f> prev = {
        nova::Vec3f{ 1.0, 1.0, 0 },
        nova::Vec3f{ 2.0, 2.0, 0 }
    };

    const auto threshold = 0.6f; // A bit higher threshold for this test
    const auto input_sampling_rate = 4.f;
    const auto velocity = 2.0f;
    const auto orientation = 0; // Moving along X-axis

    auto result = pairing(curr, prev, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_EQ(result.first.size(), 2);
    EXPECT_EQ(result.second.size(), 2);
    // Check if each pair matches within the threshold
    for (size_t i = 0; i < result.first.size(); ++i) {
        EXPECT_NEAR(result.first[i].x(), result.second[i].x() + 0.5f, threshold); // Account for movement
        EXPECT_NEAR(result.first[i].y(), result.second[i].y() + 0.5f, threshold);
    }
}

// Test case for no velocity, ensuring no pairing due to distance
TEST(Utils, Pairing_NoVelocity) {
    const auto vec1 = nova::Vec3f{ 1.5, 1.5, 0 };
    const auto vec2 = nova::Vec3f{ 1.0, 1.0, 0 };

    const auto threshold = 0.1f;
    const auto input_sampling_rate = 4.f;
    const auto velocity = 0.0f; // No movement
    const auto orientation = 0;

    auto result = pairing({ vec1 }, { vec2 }, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_TRUE(result.first.empty()); // Should not pair because no movement occurred
    EXPECT_TRUE(result.second.empty());
}

TEST(Utils, Pairing_Real) {
    const auto curr = std::vector<nova::Vec3f>{ { 7.255823, 2.7158403, 0.2956972 }, { 0.5135908, 2.9660163, 0.30723977 }, { 5.4565997, -3.011084, 0.31414804 }, { 0.5142484, -2.964827, 0.3059463 }, { -4.5075297, 2.9669929, 0.31624302 }, { -4.507579, -2.9676514, 0.31687728 }, { -9.459756, -2.9588158, 0.3178026 }, { -4.4597526, 3.0103579, 0.29729095 } };
    const auto prev = std::vector<nova::Vec3f>{ { 0.97282803, 2.9126067, 0.29918486 }, { 7.8808265, -3.1206312, 0.3091445 }, { -13.915543, -2.9350996, 0.2873801 }, { 5.887647, 2.9723408, 0.30903324 }, { -3.9758732, -2.9981248, 0.31334537 }, { -8.970752, 2.942428, 0.30202544 }, { -3.9732058, 2.9981081, 0.31070676 }, { -3.987887, -2.972787, 0.31202015 }, { 0.9739963, -2.888132, 0.3003778 } };

    const auto threshold = 0.5f;
    const auto input_sampling_rate = 4.f;
    const auto velocity = 1.19694f;
    const auto orientation = 0.0379361;

    auto result = pairing(curr, prev, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_TRUE(result.first.empty());
    EXPECT_TRUE(result.second.empty());
}
