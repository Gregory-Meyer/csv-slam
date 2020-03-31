cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(csv-slam LANGUAGES CXX VERSION 0.1.0)

find_package(Boost 1.58 REQUIRED)
find_package(cartographer REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(csv-slam
    src/csv_slam.cpp
    src/cartographer_state.cpp
    src/csv_writer.cpp
    src/imu_reader.cpp
    src/util.cpp
    src/velodyne_reader.cpp
)
target_compile_features(csv-slam PRIVATE cxx_std_17)
set_target_properties(csv-slam PROPERTIES
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
target_link_libraries(csv-slam PRIVATE
    Boost::headers
    cartographer
    Eigen3::Eigen
)
