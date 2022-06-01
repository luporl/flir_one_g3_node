cmake_minimum_required(VERSION 3.0.2)
project(flir_one_g3)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  rospy
  std_msgs
)

find_path(LIBUSB_INCLUDE_DIR
    NAMES libusb.h
    PATH_SUFFIXES "include" "libusb" "libusb-1.0")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs)

###########
## Build ##
###########

include_directories(${LIBUSB_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-Wall")
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wall")

add_executable(${PROJECT_NAME}_node flir_one_g3_node.cpp flirone.c)
target_link_libraries(${PROJECT_NAME}_node usb-1.0 ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
    palettes/15.raw
    palettes/17.raw
    palettes/7.raw
    palettes/85.raw
    palettes/92.raw
    palettes/Grayscale.raw
    palettes/Grey.raw
    palettes/Iron2.raw
    palettes/Iron_Black.raw
    palettes/Rainbow.raw
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/palettes
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_flir_one_g3.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
