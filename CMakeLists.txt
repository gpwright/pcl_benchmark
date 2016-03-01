cmake_minimum_required(VERSION 1.8)
project(pclbench)

set(CMAKE_PREFIX_PATH /opt/VTK /opt/PCL)


find_package(VTK 7.1 REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL 1.8 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

include_directories(
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

link_libraries(
  vtkFiltersGeneral
  boost_system
  pcl_common
  pcl_io
  pcl_io_ply
  pcl_octree
  pcl_visualization
)

add_executable(pclbench main.cpp)


