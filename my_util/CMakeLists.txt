file(GLOB srcs "*.cpp" "*.hpp")

add_library(my_util ${srcs})
target_link_libraries(my_util my_yaml)
