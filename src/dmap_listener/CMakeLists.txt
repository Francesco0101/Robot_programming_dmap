cmake_minimum_required(VERSION 3.0.2)
project(dmap_listener)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
)


catkin_package(

)


include_directories(/usr/include/eigen3)
find_package(OpenCV REQUIRED)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(main_node 
  src/main_node.cpp
  src/dmap.cpp
  src/dmap_localizer.cpp
  src/draw_helpers.cpp
  src/grid_map.cpp
  )


 target_link_libraries(main_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   )

