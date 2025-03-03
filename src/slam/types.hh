#pragma once

#include <nova/vec.hh>

#include <sstream>
#include <vector>

#include <cmath>


// TODO: Generalize
struct vec2d_data_parser {
    [[nodiscard]] auto operator()(std::istream& inf) -> std::vector<nova::Vec2d> {
        std::vector<nova::Vec2d> ret;
        std::stringstream ss;
        for (std::string line; std::getline(inf, line); ) {
            ss << line << '\n';
            double x, y, dummy;
            ss >> x >> y >> dummy;
            ret.emplace_back(x, y);
            ss.clear();
        }
        return ret;
    }
};
