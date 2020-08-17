cmake_minimum_required(VERSION 3.0.2)
project(omni_proc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
FIND_PACKAGE(OpenCV 3.4 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
dynamic_reconfigure
)
   and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/mono_param.cfg
 )

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
  INCLUDE_DIRS include
#  LIBRARIES omni_proc
  CATKIN_DEPENDS cv_bridge image_transport roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(omni_mono_proc src/omni_mono_proc.cpp  src/omni_util.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(omni_mono_proc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(omni_mono_proc
   ${catkin_LIBRARIES}
${OpenCV_LIBS}
 )
