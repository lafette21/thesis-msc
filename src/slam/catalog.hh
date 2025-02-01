#ifndef CATALOG_HH
#define CATALOG_HH

#include <nova/error.hh>
#include <nova/vec.hh>

#include <map>
#include <optional>
#include <vector>


class catalog {
static constexpr auto InitialCapacity = 20;

public:
    catalog(float distance_threshold)
        : m_distance_threshold(distance_threshold)
    {}

    void add(const nova::Vec3f& obj) {
        if (const auto id = has_similar(obj); id.has_value()) {
            m_data[*id].push_back(obj);
        } else {
            auto& vec = m_data[m_id++];
            vec.reserve(InitialCapacity);
            vec.push_back(obj);
        }
    }

    [[nodiscard]] inline auto data() const { return m_data; }
    [[nodiscard]] inline auto get(std::size_t id) const { return m_data.at(id); }

private:
    std::map<std::size_t, std::vector<nova::Vec3f>> m_data;
    float m_distance_threshold;
    std::size_t m_id = 0;

    std::optional<std::size_t> has_similar(const nova::Vec3f& obj) {
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
