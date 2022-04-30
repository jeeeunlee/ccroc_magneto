file(GLOB srcs "*.cpp" "*.hpp")

add_library(beizier_planner SHARED ${srcs})

target_link_libraries(beizier_planner my_geometry my_util)
