#ifndef CATALOG_HH
#define CATALOG_HH

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <nova/error.hh>
#include <nova/vec.hh>

#include <map>
#include <optional>
#include <vector>


class catalog {
static constexpr auto InitialCapacity = 20;

public:
    catalog(double distance_threshold)
        : m_distance_threshold(distance_threshold)
    {}

    std::size_t add(const nova::Vec2d& obj) {
        std::size_t id;

        if (const auto opt_id = has_similar(obj); opt_id.has_value()) {
            id = *opt_id;
        } else {
            id = m_id++;
            m_data[id].reserve(InitialCapacity);
        }
        m_data[id].push_back(obj);

        return id;
    }

    [[nodiscard]] inline std::string dump() const {
        return fmt::format("{}", m_data);
    }

    [[nodiscard]] inline auto data() const { return m_data; }
    [[nodiscard]] inline auto get(std::size_t id) const { return m_data.at(id); }

    [[nodiscard]] inline auto& get(std::size_t id) { return m_data.at(id); }

private:
    std::map<std::size_t, std::vector<nova::Vec2d>> m_data;
    double m_distance_threshold;
    std::size_t m_id = 0;

    std::optional<std::size_t> has_similar(const nova::Vec2d& obj) {
        for (const auto& [k, v] : m_data) {
            for (const auto& elem : v) {
                if ((obj - elem).length() < m_distance_threshold) {
                    return k;
                }
            }
        }
        return std::nullopt;
    }
};

#endif // CATALOG_HH
