cmake_minimum_required(VERSION 2.8.3)
project(r_platform)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  message_generation
  tf
)


## Generate messages in the 'msg' folder
 add_message_files(
    FILES
    navi.msg
#   Message2.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pointgrey_fl3_ge_28s4_c
  CATKIN_DEPENDS roscpp std_msgs message_runtime tf
#  DEPENDS system_lib
)

find_package(OpenCV REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # /usr/include/flycapture
)


# find_library(FLYCAPTURE2 flycapture)


# add_executable(r_pointgrey_FL3_28S4 src/r_pointgrey_fl3-ge-28s4-c_driver.cpp)
#add_executable(pointgrey_FL3_28S4 src/pointgrey_fl3-ge-28s4-c_driver.cpp)

# target_link_libraries(r_pointgrey_FL3_28S4 ${catkin_LIBRARIES}
# 			     ${OpenCV_LIBS}
# 			     flycapture
#                     		)
                    		
# add_executable(r_image_process src/r_image_process.cpp)

# target_link_libraries(r_image_process ${catkin_LIBRARIES})

add_executable(decompose_vel src/decompose_vel.cpp)

target_link_libraries(decompose_vel ${catkin_LIBRARIES})  

add_executable(r_teleop src/r_teleop.cpp)

target_link_libraries(r_teleop ${catkin_LIBRARIES})  

# add_executable(r_exp_results src/r_exp_results.cpp)

# target_link_libraries(r_exp_results ${catkin_LIBRARIES})  

add_executable(r_client_node src/r_client_node.cpp)

target_link_libraries(r_client_node ${catkin_LIBRARIES}) 

add_executable(r_automatic src/r_automatic.cpp)

target_link_libraries(r_automatic ${catkin_LIBRARIES}) 

add_executable(r_hybrid src/hybrid.cpp)

target_link_libraries(r_hybrid ${catkin_LIBRARIES}) 

add_executable(test_risk src/test_risk.cpp)

target_link_libraries(test_risk ${catkin_LIBRARIES}) 
              		

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(pointgrey_fl3_ge_28s4_c ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(pointgrey_fl3_ge_28s4_c_node src/pointgrey_fl3_ge_28s4_c_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(pointgrey_fl3_ge_28s4_c_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(pointgrey_fl3_ge_28s4_c_node
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
# install(TARGETS pointgrey_fl3_ge_28s4_c pointgrey_fl3_ge_28s4_c_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pointgrey_fl3_ge_28s4_c.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


