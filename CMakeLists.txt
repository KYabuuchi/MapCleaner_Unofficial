cmake_minimum_required(VERSION 3.22)
project(map_cleaner)

# add_compile_options(-std=c++14 -O3)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/mod_pcl")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

include_directories(
  ${MOD_PCL_INCLUDE_DIRS}
  thirdparty
  thirdparty/patchworkpp/include
  include
)

ament_auto_add_library(patchworkpp STATIC
  thirdparty/patchworkpp/src/patchworkpp.cpp
)

target_link_libraries(patchworkpp Eigen3::Eigen)

ament_auto_add_executable(${PROJECT_NAME}
  src/map_cleaner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${MOD_PCL_LIBRARIES}
  patchworkpp
)

ament_auto_package()