cmake_minimum_required(VERSION 3.0.2)
project(q_spline)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_msgs
        geometry_msgs
        ecl_geometry
)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(spline_node src/q_spline.cpp)

target_link_libraries(spline_node
        ${catkin_LIBRARIES}
)

