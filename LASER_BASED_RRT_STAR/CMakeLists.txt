cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME LASER_BASED_RRT_STAR)

project(${PROJECT_NAME})

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(LASER_BASED_RRT_STAR_SOURCE_DIR
        src/sources)

set(LASER_BASED_RRT_STAR_INCLUDE_DIR
        src/include
        )

    set(LASER_BASED_RRT_STAR_SOURCE_FILES
	${LASER_BASED_RRT_STAR_SOURCE_DIR}/mtrand.cpp
	${LASER_BASED_RRT_STAR_SOURCE_DIR}/laser_based_rrt_star.cpp
	${LASER_BASED_RRT_STAR_SOURCE_DIR}/pugixml.cpp
	${LASER_BASED_RRT_STAR_SOURCE_DIR}/rrtstar.cpp
	${LASER_BASED_RRT_STAR_SOURCE_DIR}/dubins.cpp
            )


    set(LASER_BASED_RRT_STAR_HEADER_FILES
	 ${LASER_BASED_RRT_STAR_INCLUDE_DIR}/laser_based_rrt_star.h
	 ${LASER_BASED_RRT_STAR_INCLUDE_DIR}/pugiconfig.hpp
	 ${LASER_BASED_RRT_STAR_INCLUDE_DIR}/pugixml.hpp
	 ${LASER_BASED_RRT_STAR_INCLUDE_DIR}/dubins.h
	 ${LASER_BASED_RRT_STAR_INCLUDE_DIR}/mtrand.h
	 ${LASER_BASED_RRT_STAR_INCLUDE_DIR}/rrtstar.h
            )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy  
  droneModuleROS
  droneMsgsROS
  opencv_apps
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  message_generation
  cv_bridge
  image_transport
  mav_trajectory_generation_ros
##  NLopt
)

#if ompl
	
#find_package(ompl REQUIRED)



#if OpenCV
find_package(OpenCV REQUIRED)
#set(OpenCV_DIR /home/liang/third_part_package/opencv-3.3.0/build)
#find_package(OpenCV REQUIRED PATHS /home/liang/third_part_package/opencv-3.3.0/cmake)
#if(NOT OpenCV_FOUND)
#  message(FATAL_ERROR "Failed to find OpenCV")
#endif(NOT OpenCV_FOUND)
#include_directories(${OpenCV_INCLUDE_DIR})
#MESSAGE(STATUS "OPENCVLIB IS : " ${OpenCV_INCLUDE_DIR})
#MESSAGE(STATUS "OpenCV_DIR is: " ${OpenCV_INCLUDE_DIRS})
#MESSAGE(STATUS "OpenCV_LIBS is: " ${OpenCV_LIBS})


find_package(Eigen3 REQUIRED)
##add_definitions(${EIGEN_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#add_message_files(
#  FILES

#)

#add_service_files(
#  FILES
#  MakeRRTPlan.srv
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs

)

###################################
## catkin specific configuration ##


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs 
                 droneModuleROS droneMsgsROS opencv_apps message_runtime
                 geometry_msgs nav_msgs mav_trajectory_generation_ros
                 sensor_msgs visualization_msgs cv_bridge image_transport
  

  DEPENDS OpenCV Eigen
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

##include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
    ${LASER_BASED_RRT_STAR_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)


#include_directories(${OpenCV_INCLUDE_DIRS})



#add_library(RTRRTSTAR_lib ${RTRRTSTAR_SOURCE_FILES} ${RTRRTSTAR_HEADER_FILES})
add_library(LASER_BASED_RRT_STAR_lib ${LASER_BASED_RRT_STAR_SOURCE_FILES} ${LASER_BASED_RRT_STAR_HEADER_FILES})
add_dependencies(LASER_BASED_RRT_STAR_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(LASER_BASED_RRT_STAR_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBS})



#add_executable(kd_tree_node src/sources/KDTreeTest.cpp)
#add_dependencies(kd_tree_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(kd_tree_node RTRRTSTAR_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARY})

add_executable(laser_based_rrt_star_node src/sources/laser_based_rrt_star_node.cpp)
add_dependencies(laser_based_rrt_star_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_based_rrt_star_node LASER_BASED_RRT_STAR_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARY})
#add_executable(rrt_star_node src/sources/rrt_star_test.cpp)
#add_dependencies(rrt_star_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(rrt_star_node qp_generate_lib ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${OpenCV_LIBRARY})
