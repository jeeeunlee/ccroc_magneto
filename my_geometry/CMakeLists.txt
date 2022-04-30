
file(GLOB srcs "*.cpp" "*.hpp")

set(CMAKE_MODULE_PATH 
      ${CMAKE_MODULE_PATH} 
      ${PROJECT_SOURCE_DIR}/cmake)

message(${CMAKE_MODULE_PATH})

find_package(Eigen3)
find_package(Mosek)

add_subdirectory(ThirdParty/CDD)

add_library(my_geometry SHARED 
            src/Polytope/Polytope.cpp 
            include/my_geometry/Polytope/Polytope.h)
target_link_libraries(my_geometry myCdd ${EIGEN_LIBRARIES})


if(MOSEK_FOUND)
    add_library(myLownerJohnEllipsoid LownerJohnEllipsoid.cpp
                                      LownerJohnEllipsoid.hpp
                                      ${MOSEK_LIBRARIES})
    target_link_libraries(my_geometry myLownerJohnEllipsoid)
endif()

# install(TARGETS my_geometry DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES Polytope.h DESTINATION
#     "${INSTALL_INCLUDE_DIR}/my_geometry/Polytope")

