cmake_minimum_required(VERSION 3.8)
project(vina_slam_to_hdmapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# LASzip library
add_subdirectory(src/3rdparty/LASzip)
include_directories(src/3rdparty/LASzip/include)
include_directories(src/3rdparty/LASzip/include/laszip)
include_directories(src/3rdparty/LASzip/dll)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

# Named listener like the other benchmark converters, although it reads files,
# not a recorded bag.
add_executable(listener
  src/vina_slam_to_hdmapping.cpp
  src/laz_writer.cpp
)

target_link_libraries(listener
  Eigen3::Eigen
  laszip
  ${PCL_LIBRARIES}
)

install(TARGETS listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
