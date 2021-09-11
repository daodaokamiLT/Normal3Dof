# CMakeLists.txt

project(motiontracker3dof)

cmake_minimum_required(VERSION 2.8)
include_directories("/usr/include/eigen3")

include_directories(${PROJECT_SOURCE_DIR}/compilefilter
                    ${PROJECT_SOURCE_DIR}/ekf)

add_subdirectory(compilefilter)
add_subdirectory(ekf)

add_executable(test_3dofalgo test_3dofalgo.cpp)

target_link_libraries(test_3dofalgo cpfilter)