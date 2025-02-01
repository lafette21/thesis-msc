#ifndef SLAM_HH
#define SLAM_HH

#include "catalog.hh"
#include "logging.hh"
#include "types.hh"
#include "utils.hh"

#include <nova/io.hh>
#include <nova/yaml.hh>
#include <range/v3/all.hpp>

#include <filesystem>
#include <iostream>

using yaml = nova::yaml;


class slam {
public:
    slam(const yaml& config, const std::string& in_dir, const std::string& odometry_file, const std::string& out_dir)
        : m_config(config)
        , m_catalog(config.lookup<float>("catalog.distance_threshold"))
        , m_out_dir(out_dir)
    {
        // m_motion = nova::read_file<motion_data_parser>(std::filesystem::path(odometry_file).string()).value();

        const auto fs_sorted = [in_dir]() {
            const auto extract_number = [](const std::string& filename) {
                std::regex pattern(".*registered-([0-9]+)\\.xyz");
                std::smatch match;
                if (std::regex_search(filename, match, pattern) && match.size() > 1) {
                    return std::stoi(match[1].str());
                }
                // If no match, return a default value or handle error as needed
                return 0; // or throw an exception
            };

            const auto fs = files(in_dir);
            auto tmp = fs
                     | ranges::views::transform([extract_number](const auto& x) { return std::make_pair(extract_number(x), x); })
                     | ranges::to<std::vector>();

            ranges::sort(tmp, [](auto& lhs, auto& rhs) { return lhs.first < rhs.first; });

            return tmp | ranges::views::values | ranges::to<std::vector>();
        }();

        for (const auto& f : fs_sorted) {
            auto lm = nova::read_file<landmark_data_parser>(f).value();
            m_landmarks.push_back(lm);
        }
    }

    void optimize() {

    }

private:
    yaml m_config;
    catalog m_catalog;
    std::vector<motion> m_motion;
    std::vector<std::vector<landmark>> m_landmarks;
    std::string m_out_dir;
};

#endif // SLAM_HH
