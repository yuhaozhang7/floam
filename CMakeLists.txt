cmake_minimum_required(VERSION 2.8.3)
project(floam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Boost REQUIRED COMPONENTS filesystem)

SET(EIGEN3_INCLUDE_DIR "/deps/aloam/eigen3/include/eigen3/")
SET(PCL_INCLUDE_DIR "/home/yuhao/slambench/deps/pcl/include/pcl-1.8/")
SET(CERES_INCLUDE_DIR "/deps/aloam/ceres/include/")
SET(YAML_CPP_INCLUDE_DIR "/deps/aloam/yaml-cpp/include/")

SET(PCL_LIBRARY_DIR "/home/yuhao/slambench/deps/pcl/lib/")
SET(CERES_LIBRARY_DIR "/deps/aloam/ceres/lib/")
SET(YAML_CPP_LIBRARY_DIR "/deps/aloam/yaml-cpp/lib/")

file(GLOB PCL_LIBRARIES "${PCL_LIBRARY_DIR}/libpcl_*.so*")
file(GLOB YAML_CPP_LIBRARIES "${YAML_CPP_LIBRARY_DIR}/libyaml-cpp.a")
set(CERES_LIBRARIES
    ${CERES_LIBRARY_DIR}/libceres.so
    ${CERES_LIBRARY_DIR}/libceres.so.1
    ${CERES_LIBRARY_DIR}/libceres.so.1.12.0
)

file(GLOB_RECURSE FLOAM_HEADERS ./include/*.h)
file(GLOB_RECURSE FLOAM_SOURCES ./src/*.cpp)

add_library(floam_lib SHARED ${FLOAM_SOURCES} ${FLOAM_HEADERS})

include_directories(
	include
  ${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIR}
  ${CERES_INCLUDE_DIR}
	${YAML_CPP_INCLUDE_DIR}
	${SLAMBENCH_INCLUDE_DIR}
)


add_executable(floam src/floam.cpp)
target_link_libraries(floam floam_lib ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES} pthread Boost::filesystem)
