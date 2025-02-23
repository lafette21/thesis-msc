#include "slam/catalog.hh"

#include <nova/vec.hh>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

// TODO: exception test

TEST(Catalog, Add) {
    const auto vec1 = nova::Vec3f{ 1.5, 1, 0 };
    const auto vec2 = nova::Vec3f{ 1, 1, 0 };

    catalog cat;
    cat.bind_predicate(
        [](const nova::Vec3f& obj) -> std::optional<std::size_t> {
            // if ()
        }
    );

    cat.add(vec1);
    cat.add(vec2);

    // EXPECT_EQ();
}
