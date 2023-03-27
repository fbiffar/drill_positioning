cmake_minimum_required(VERSION 2.8.3)
project(drill_positioning)

find_package(catkin_simple REQUIRED
  roscpp)

catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

#############
# LIBRARIES #
#############
cs_add_library(drill_positioning_lib
  src/drill_positioning.cc
  )



############
# BINARIES #
############
cs_add_executable(drill_positioning
  src/drill_positioning_node.cc
)



target_link_libraries(drill_positioning drill_positioning_lib)


add_dependencies(drill_positioning ${${PROJECT_NAME}_EXPORTED_TARGETS})


##########
# EXPORT #
##########
cs_install()
cs_export()
