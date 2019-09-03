cmake_minimum_required(VERSION 2.8.3)
project(qut_tasks)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES qut_tasks
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib
  )

# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
