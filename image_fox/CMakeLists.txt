cmake_minimum_required(VERSION 2.8.3)
project(image_fox)
set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
add_compile_options(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
sensor_msgs
cv_bridge
roscpp
std_msgs
dynamixel_workbench_msgs
image_transport
OpenCV

)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_fox
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/image_fox.cpp
# )

# add the subscriber example
add_executable(image_converter src/image_converter.cpp)
add_dependencies(image_converter ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(image_mark src/image_mark.cpp)
add_dependencies(image_mark ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_mark ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(image_recorder src/video_recorder.cpp)
add_dependencies(image_recorder ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_recorder ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(depth_subscribe src/depth_subscribe.cpp)
add_dependencies(depth_subscribe ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(depth_subscribe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(depth_csv_reading src/depth_csv_reading.cpp)
add_dependencies(depth_csv_reading ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(depth_csv_reading ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(depth_processor src/depth_processor.cpp)
add_dependencies(depth_processor ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(depth_processor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(image_processor src/image_processor.cpp)
add_dependencies(image_processor ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_processor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ob3tacle_avoid src/ObstacleAvoid_3gen.cpp src/obstacle_avoid_3gen.cpp )
add_dependencies(ob3tacle_avoid ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ob3tacle_avoid ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(Stair_Det src/Stair_climbing/StairsDet.cpp src/Stair_climbing/Stair_Det.cpp)
add_dependencies(Stair_Det ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(Stair_Det ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(Up_Stair_Mode src/Stair_climbing/Up_Stair_Mode.cpp  src/Stair_climbing/UpStairMode.cpp)
add_dependencies(Up_Stair_Mode ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(Up_Stair_Mode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(Down_Stair_Mode src/Stair_climbing/Down_Stair_Mode.cpp  src/Stair_climbing/DownStairMode.cpp )
add_dependencies(Down_Stair_Mode ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(Down_Stair_Mode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(UTP_Demo src/Stair_climbing/UTP_Demo.cpp src/Stair_climbing/DownStairMode.cpp src/Stair_climbing/UpStairMode.cpp)
add_dependencies(UTP_Demo ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(UTP_Demo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(Demo_stairclimbing src/Stair_climbing/Demo_stairclimbing.cpp  src/Stair_climbing/UpStairMode.cpp src/Stair_climbing/DownStairMode.cpp src/ObstacleAvoid_3gen.cpp src/Mover.cpp)
add_dependencies(Demo_stairclimbing ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(Demo_stairclimbing ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(MOVER src/MOVER.cpp  src/Mover.cpp )
add_dependencies(MOVER ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(MOVER ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(showpath src/Stair_climbing/showpath.cpp)
add_dependencies(showpath ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(showpath ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(CrossFloorNavigation src/CrossFloorNavigation.cpp  src/Stair_climbing/UpStairMode.cpp src/Stair_climbing/DownStairMode.cpp src/ObstacleAvoid_3gen.cpp src/Mover.cpp )
add_dependencies(CrossFloorNavigation ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(CrossFloorNavigation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/image_fox_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_image_fox.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
