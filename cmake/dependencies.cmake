find_package(benchmark)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(GSL REQUIRED)
find_package(GTest REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io segmentation)
find_package(range-v3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

if(ENABLE_ROSBAG2_OUTPUT)
    find_package(rclcpp REQUIRED)
    find_package(rosbag2_cpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
endif()
