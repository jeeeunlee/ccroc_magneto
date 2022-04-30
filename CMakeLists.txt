cmake_minimum_required(VERSION 3.5.1)
project(ccroc-magneto)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS off)

include_directories("eigen3")
include_directories("${PROJECT_SOURCE_DIR}")
add_subdirectory(beizier_planner)
add_subdirectory(my_geometry)
add_subdirectory(my_yaml)
add_subdirectory(my_util)

add_executable(main "main_magneto.cpp")
target_link_libraries(main beizier_planner)

SET (THIS_COM "${PROJECT_SOURCE_DIR}/")
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake
    ${PROJECT_SOURCE_DIR}/Configuration.h)
