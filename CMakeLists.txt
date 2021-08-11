cmake_minimum_required(VERSION 3.0.2)
project(xyw_lidar_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
SET(CMAKE_BUILD_TYPE "Debug")
LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  std_msgs
  visualization_msgs
  dynamic_reconfigure
  eigen_conversions
  nlink_example
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/XYWLidarTest.cfg
)
find_package(PCL)

#GLOG (neccessary)
FIND_PACKAGE(GLOG REQUIRED QUIET)
IF(GLOG_FOUND)
INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIRS})
MESSAGE("GLOG [OK]:" ${GLOG_INCLUDE_DIRS}) 
ENDIF(GLOG_FOUND)

# OpenMP
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Ceres
FIND_PACKAGE(Ceres REQUIRED QUIET)
IF(Ceres_FOUND)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
MESSAGE("CERES [OK]:" ${CERES_INCLUDE_DIRS}) 
ADD_DEFINITIONS(-DCERES_ON)
ENDIF(Ceres_FOUND)


add_message_files(
  FILES
  people_item.msg
  people_list.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 DEPENDS PCL 
          dynamic_reconfigure
#  LIBRARIES xyw_lidar_test
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)



###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/xyw_lidar_test.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/main.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg nlink_example_gencpp)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
SET(DEP_LIBS ${DEP_LIBS} ${PCL_LIBRARIES})
SET(DEP_LIBS ${DEP_LIBS} ${catkin_LIBRARIES})
SET(DEP_LIBS ${DEP_LIBS} ${GLOG_LIBRARIES})
SET(DEP_LIBS ${DEP_LIBS} ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node
  ${DEP_LIBS}
)

