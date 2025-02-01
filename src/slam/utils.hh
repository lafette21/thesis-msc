#pragma once

#include <vector>
#include <string>
#include <filesystem>
#include <regex>


[[nodiscard]] inline auto files(const std::string& folder, const std::string& pattern = "registered-\\d*.xyz")
        -> std::vector<std::string>
{
    namespace fs = std::filesystem;

    std::vector<std::string> ret;

    for (const auto & entry : fs::directory_iterator(folder)) {
        std::string full_path = entry.path().string();

        if (fs::is_regular_file(entry) && std::regex_match(entry.path().filename().string(), std::regex(pattern))) {
            ret.push_back(fs::absolute(full_path).string());
        }
    }

    return ret;
}
