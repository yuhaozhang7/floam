cmake_minimum_required(VERSION 2.8.3)
project(floam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

file(GLOB FLANN_LIBRARIES "${FLANN_LIBRARY_DIR}/libflann*.*")
file(GLOB PCL_LIBRARIES "${PCL_LIBRARY_DIR}/libpcl_*.so*")
file(GLOB YAML_CPP_LIBRARIES "${YAML_CPP_LIBRARY_DIR}/libyaml-cpp.a")
set(CERES_LIBRARIES
    ${CERES_LIBRARY_DIR}/libceres.so
    ${CERES_LIBRARY_DIR}/libceres.so.1
    ${CERES_LIBRARY_DIR}/libceres.so.1.12.0
)

include_directories(
	include
  ${EIGEN3_INCLUDE_DIR}
  ${FLANN_INCLUDE_DIR}
	${PCL_INCLUDE_DIR}
  ${CERES_INCLUDE_DIR}
	${YAML_CPP_INCLUDE_DIR}
	${SLAMFUSE_INCLUDE_DIR}
)

file(GLOB_RECURSE FLOAM_HEADERS ./include/*.h)
file(GLOB_RECURSE FLOAM_SOURCES ./src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${FLOAM_SOURCES} ${FLOAM_HEADERS})

add_library(${PROJECT_NAME}-original SHARED floam_library.cpp)
install(TARGETS ${PROJECT_NAME}  DESTINATION  lib/)
install(TARGETS ${PROJECT_NAME}-original  DESTINATION  lib/)

target_link_libraries(${PROJECT_NAME}-original 
  ${PROJECT_NAME}
  ${FLANN_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  pthread)

SET(SLAMBENCH_C_WRAPPER	-Wl,--whole-archive -L${WRAPPER_PATH} slambench-c-wrapper	-Wl,--no-whole-archive)
SET(NULL_FILE_PATH	${WRAPPER_PATH}/null.cpp)

function(generate_slam_library library_name )

    add_library(${library_name}-library SHARED ${NULL_FILE_PATH})
    target_link_libraries(${library_name}-library PRIVATE ${SLAMBENCH_C_WRAPPER} ${library_name})
    SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES LINK_FLAGS "${ARGN}")
    SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES INSTALL_RPATH "${LIB_PATH}")
    install(TARGETS ${library_name}-library  DESTINATION  lib/)

endfunction()

generate_slam_library(${PROJECT_NAME}-original "-Wl,--disable-new-dtags")
