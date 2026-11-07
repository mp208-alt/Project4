cmake_minimum_required(VERSION 3.10)
project(Project4 CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    src
)

link_directories(${OMPL_LIBRARY_DIRS})

add_executable(Project4Car src/Project4Car.cpp)
target_link_libraries(Project4Car ${OMPL_LIBRARIES})

add_executable(Project4Pendulum src/Project4Pendulum.cpp)
target_link_libraries(Project4Pendulum ${OMPL_LIBRARIES})