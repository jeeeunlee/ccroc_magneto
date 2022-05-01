add_subdirectory(goldfarb)

file(GLOB srcs "*.cpp" "*.hpp")

add_library(my_qpsolver ${srcs})
target_link_libraries(my_qpsolver goldfarb my_util)
