#ifndef TYPES_HH
#define TYPES_HH

#include <sstream>
#include <vector>

#include <cmath>


struct motion {
    double data[2];

    motion(double vel = 0, double ang_vel = 0)
        : data{ vel, ang_vel }
    {}

    // bool operator==(const motion& obj) const = default;
};

struct motion_data_parser {
    [[nodiscard]] std::vector<motion> operator()(std::istream& iF) {
        std::vector<motion> ret;
        std::stringstream ss;
        for (std::string line; std::getline(iF, line); ) {
            ss << line << '\n';
            double vel, ang_vel;
            ss >> vel >> ang_vel;
            ret.emplace_back(vel, ang_vel);
            ss.clear();
        }
        return ret;
    }
};

struct pose {
    double data[3];

    pose(double x = 0, double y = 0, double psi = 0)
        : data{ x, y, psi }
    {}

    pose add(const motion& meas, double t) {
        return {
            data[0] + meas.data[0] * t * std::cos(data[2] + meas.data[1] * t * 0.5),
            data[1] + meas.data[0] * t * std::sin(data[2] + meas.data[1] * t * 0.5),
            data[2] + meas.data[1] * t
        };
    }
};

struct landmark {
    double data[2];

    landmark(double x = 0, double y = 0)
        : data{ x, y }
    {}

    // bool operator==(const motion& obj) const = default;
};

struct landmark_data_parser {
    [[nodiscard]] std::vector<landmark> operator()(std::istream& iF) {
        std::vector<landmark> ret;
        std::stringstream ss;
        for (std::string line; std::getline(iF, line); ) {
            ss << line << '\n';
            double x, y, dummy;
            ss >> x >> y >> dummy;
            ret.emplace_back(x, y);
            ss.clear();
        }
        return ret;
    }
};

#endif // TYPES_HH
