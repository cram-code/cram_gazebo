cmake_minimum_required(VERSION 2.8.3)
project(gazebo_visibility_ros)

FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(PkgConfig)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()

set(MSG_DEPS
  std_msgs
  geometry_msgs
)

set(MSG_FILES
  Dummy.msg
)

set(SRV_FILES
  QueryGazeboVisibility.srv
)

set(ACT_FILES
)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  ${MSG_DEPS}
  message_generation
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

#add_action_files(DIRECTORY action FILES ${ACT_FILES})
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS 
    ${MSG_DEPS}
    message_runtime
    roscpp 
    gazebo_ros 
)
include_directories(
  ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  )
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/gazebo_visibility_ros.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})



