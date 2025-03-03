#include "perception/logging.hh"

#include <nova/vec.hh>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <range/v3/view.hpp>


[[nodiscard]] inline auto pairing(
    const std::vector<nova::Vec2d>& curr,
    const std::vector<nova::Vec2d>& prev,
    double threshold,
    double input_sampling_rate,
    double velocity,
    double orientation
)
        -> std::pair<std::vector<nova::Vec2d>, std::vector<nova::Vec2d>>
{
    std::vector<nova::Vec2d> ret_a;
    std::vector<nova::Vec2d> ret_b;
    std::vector<std::vector<double>> dist_mx;

    // Predict movement based on velocity
    double delta_time = 1. / input_sampling_rate;
    double delta_x = velocity * delta_time * std::cos(orientation);
    double delta_y = velocity * delta_time * std::sin(orientation);

    for (const auto& a : curr) {
        dist_mx.emplace_back(std::vector<double>{});
        auto& vec = dist_mx.back();
        const auto& c_a = nova::Vec2d{ a.x(), a.y() };

        for (const auto& b : prev) {
            const auto predicted_pos = b + nova::Vec2d{ delta_x, delta_y };
            const auto c_b = nova::Vec2d{ predicted_pos.x(), predicted_pos.y() };
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
        const auto idx_b = static_cast<std::size_t>(std::distance(vec.begin(), std::ranges::find(vec, min)));

        if (min < threshold) {
            ret_a.push_back(curr[idx]);
            ret_b.push_back(prev[idx_b]);
            logging::debug("({}, {})\t({}, {})\tdist: {}", curr[idx].x(), curr[idx].y(), prev[idx_b].x(), prev[idx_b].y(), min);
        }
    }

    return { ret_a, ret_b };
}


TEST(Utils, Pairing) {
    const auto vec1 = nova::Vec2d{ 1.5, 1 };
    const auto vec2 = nova::Vec2d{ 1, 1 };

    const auto threshold = 0.000001;
    const auto input_sampling_rate = 4;
    const auto velocity = 2;
    const auto orientation = 0;

    EXPECT_EQ(
        pairing({ vec1 }, { vec2 }, threshold, input_sampling_rate, velocity, orientation),
        ( std::pair<std::vector<nova::Vec2d>, std::vector<nova::Vec2d>>{ { vec1 }, { vec2 } } )
    );
}

// Test case for when vectors are far apart
TEST(Utils, Pairing_FarApart) {
    const auto vec1 = nova::Vec2d{ 10.0, 10.0 };
    const auto vec2 = nova::Vec2d{ 0.0, 0.0 };

    const auto threshold = 0.1;
    const auto input_sampling_rate = 1;
    const auto velocity = 0; // No movement expected
    const auto orientation = 0;

    auto result = pairing({ vec1 }, { vec2 }, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_TRUE(result.first.empty()); // Should not pair because distance is too large
    EXPECT_TRUE(result.second.empty());
}

// Test case for vectors with movement along X-axis
TEST(Utils, Pairing_WithMovementX) {
    const auto vec1 = nova::Vec2d{ 1.25, 1.0 };
    const auto vec2 = nova::Vec2d{ 1.0, 1.0 };

    const auto threshold = 0.1;
    const auto input_sampling_rate = 4;
    const auto velocity = 1; // Moving 1 unit per second
    const auto orientation = 0; // Moving along positive X-axis

    auto result = pairing({ vec1 }, { vec2 }, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_EQ(result.first.size(), 1);
    EXPECT_EQ(result.second.size(), 1);
    EXPECT_NEAR(result.first[0].x(), vec1.x(), threshold);
    EXPECT_NEAR(result.second[0].x(), vec2.x(), threshold);
}

// Test case for vectors with movement along Y-axis
TEST(Utils, Pairing_WithMovementY) {
    const auto vec1 = nova::Vec2d{ 1.0, 1.25 };
    const auto vec2 = nova::Vec2d{ 1.0, 1.0 };

    const auto threshold = 0.1;
    const auto input_sampling_rate = 4;
    const auto velocity = 1; // Moving 1 unit per second
    const auto orientation = M_PI / 2; // Moving along positive Y-axis

    auto result = pairing({ vec1 }, { vec2 }, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_EQ(result.first.size(), 1);
    EXPECT_EQ(result.second.size(), 1);
    EXPECT_NEAR(result.first[0].y(), vec1.y(), threshold);
    EXPECT_NEAR(result.second[0].y(), vec2.y(), threshold);
}

// Test case for vectors with movement along Y-axis
TEST(Utils, Pairing_WithMovementYNeg) {
    const auto vec1 = nova::Vec2d{ 1.0, 0.75 };
    const auto vec2 = nova::Vec2d{ 1.0, 1.0 };

    const auto threshold = 0.1;
    const auto input_sampling_rate = 4;
    const auto velocity = 1; // Moving 1 unit per second
    const auto orientation = -M_PI / 2; // Moving along positive Y-axis

    auto result = pairing({ vec1 }, { vec2 }, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_EQ(result.first.size(), 1);
    EXPECT_EQ(result.second.size(), 1);
    EXPECT_NEAR(result.first[0].y(), vec1.y(), threshold);
    EXPECT_NEAR(result.second[0].y(), vec2.y(), threshold);
}

// Test case for multiple vectors, ensuring correct pairing
TEST(Utils, Pairing_MultipleVectors) {
    const std::vector<nova::Vec2d> curr = {
        nova::Vec2d{ 1.5, 1.5 },
        nova::Vec2d{ 2.5, 2.5 }
    };
    const std::vector<nova::Vec2d> prev = {
        nova::Vec2d{ 1.0, 1.0 },
        nova::Vec2d{ 2.0, 2.0 }
    };

    const auto threshold = 0.6; // A bit higher threshold for this test
    const auto input_sampling_rate = 4;
    const auto velocity = 2;
    const auto orientation = 0; // Moving along X-axis

    auto result = pairing(curr, prev, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_EQ(result.first.size(), 2);
    EXPECT_EQ(result.second.size(), 2);
    // Check if each pair matches within the threshold
    for (size_t i = 0; i < result.first.size(); ++i) {
        EXPECT_NEAR(result.first[i].x(), result.second[i].x() + 0.5, threshold); // Account for movement
        EXPECT_NEAR(result.first[i].y(), result.second[i].y() + 0.5, threshold);
    }
}

// Test case for no velocity, ensuring no pairing due to distance
TEST(Utils, Pairing_NoVelocity) {
    const auto vec1 = nova::Vec2d{ 1.5, 1.5 };
    const auto vec2 = nova::Vec2d{ 1.0, 1.0 };

    const auto threshold = 0.1;
    const auto input_sampling_rate = 4;
    const auto velocity = 0; // No movement
    const auto orientation = 0;

    auto result = pairing({ vec1 }, { vec2 }, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_TRUE(result.first.empty()); // Should not pair because no movement occurred
    EXPECT_TRUE(result.second.empty());
}

TEST(Utils, Pairing_Real) {
    const auto curr = std::vector<nova::Vec2d>{ { 7.255823, 2.7158403 }, { 0.5135908, 2.9660163 }, { 5.4565997, -3.011084 }, { 0.5142484, -2.964827 }, { -4.5075297, 2.9669929 }, { -4.507579, -2.9676514 }, { -9.459756, -2.9588158 }, { -4.4597526, 3.0103579 } };
    const auto prev = std::vector<nova::Vec2d>{ { 0.97282803, 2.9126067 }, { 7.8808265, -3.1206312 }, { -13.915543, -2.9350996 }, { 5.887647, 2.9723408 }, { -3.9758732, -2.9981248 }, { -8.970752, 2.942428 }, { -3.9732058, 2.9981081 }, { -3.987887, -2.972787 }, { 0.9739963, -2.888132 } };

    const auto threshold = 0.5;
    const auto input_sampling_rate = 4;
    const auto velocity = 1.19694;
    const auto orientation = 0.0379361;

    auto result = pairing(curr, prev, threshold, input_sampling_rate, velocity, orientation);
    EXPECT_TRUE(result.first.empty());
    EXPECT_TRUE(result.second.empty());
}
