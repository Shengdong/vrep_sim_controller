cmake_minimum_required (VERSION 2.6)
project(controller_node)

include_directories(${PROJECT_SOURCE_DIR}
                    /usr/local/include
                    )

find_package(catkin REQUIRED COMPONENTS
             apriltags_ros_sim
             cmake_modules
             roscpp
             geometry_msgs
             std_msgs
)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_definitions("-std=c++11 -Wall")

catkin_package(
  INCLUDE_DIRS
  LIBRARIES controller
  CATKIN_DEPENDS apriltags_ros_sim roscpp geometry_msgs std_msgs message_runtime
  DEPENDS Eigen3 BOOST
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(controller
            src/controller.cpp
)

target_link_libraries(controller
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(controller_node src/controller_node.cpp)

target_link_libraries(controller_node 
                      ${catkin_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT} 
                      controller
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS controller controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/controller_node
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


