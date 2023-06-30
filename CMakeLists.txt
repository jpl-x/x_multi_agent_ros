
cmake_minimum_required(VERSION 3.16)
project(x_vio_ros VERSION 1.2.0)

# Set build flags, depending on the architecture
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")

#################################################################################
# User build settings

option(MULTI_THREAD "Multi thread" ON) # Set ON to process image and inertial data on different
		      # threads

option(VERBOSE "Publish std out and other data" OFF) # Set false to disable all publishing and standard output
				  # stream, except pose at update rate. That will improve runtime.

option(TIMING "Publish timing information" OFF) # Set ON to enable timers

option(PROFILING "Enable profiling flags" OFF) # Set ON to disable compiler flags which are not
		      # compatible with Callgrind profiling tool.

option(PHOTOMETRIC_CALI "Enable photometric calibration for thermal images" OFF) # Set to ON to enable photometric calibration as in https://arxiv.org/abs/2012.14292

option(MULTI_UAV "Enable multi-UAV collaboration" OFF) # Set to ON to enable Multi UAV system

option(REQUEST_COMM "Enable request-response communication pipeline" OFF) # Set to ON if you want to enable the request protocol for the communication

option(GT_DEBUG "Expect landmarks position ground truth as input" OFF) # Set to ON for using the ground truth matches for multi UAV

option(UNIT_TESTS "Build unit tests" OFF) # Set ON to enable unit tests

################################################################################ 

if (CMAKE_BUILD_TYPE MATCHES Release)
  message("Release Mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

  if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64") # tested on Jetson TX2
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+crypto -mcpu=cortex-a57+crypto -flto -ffast-math -fvect-cost-model=unlimited")
  elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch32") # check for Snapdragon Flight Pro
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=softfp -flto -ffast-math -fvect-cost-model=unlimited")
  endif ()

  if (${PROFILING} MATCHES false)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -funsafe-loop-optimizations -fsee -funroll-loops -fno-math-errno -funsafe-math-optimizations -ffinite-math-only -fno-signed-zeros")
  endif ()

elseif (CMAKE_BUILD_TYPE MATCHES Debug)
  message("Debug Mode")
  add_definitions(-DDEBUG -DDEBUGMSF)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
elseif (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  message("Release with Debug Info Mode")
  # Enable asserts
  add_definitions(-UNDEBUG)
endif ()


# Set definitions
if(GT_DEBUG)
  if(PHOTOMETRIC_CALI)
    message(WARNING "PHOTOMETRIC CALIBRATION DOES NOT SUPPORT FEATURES GT.")
    message(WARNING "PHOTOMETRIC_CALI SET TO FALSE.")
    set(PHOTOMETRIC_CALI false)
  endif()
  if(REQUEST_COMM)
    message(WARNING "REQUEST COMM DOES NOT SUPPORT FEATURES GT.")
    message(WARNING "REQUEST_COMM SET TO FALSE.")
    set(REQUEST_COMM false)
  endif()
  add_definitions(-DGT_DEBUG)
  message(STATUS "GT DEBUG: ON")
endif()

if(MULTI_THREAD)
  add_definitions(-DMULTI_THREAD)
endif()

if(VERBOSE)
  add_definitions(-DVERBOSE)
endif()

if(UNIT_TESTS)
  add_definitions(-DRUN_UNIT_TESTS)
endif()

if(PHOTOMETRIC_CALI)
  add_definitions(-DPHOTOMETRIC_CALI)
  message(STATUS "Photometric calibration: ON")
endif()

if(MULTI_UAV)
  add_definitions(-DMULTI_UAV)
  message(STATUS "Multi UAV support: ON")
  if(REQUEST_COMM)
    add_definitions(-DREQUEST_COMM)
    message(STATUS "REQUEST_COMM: ON")
  endif()
endif()

add_definitions(-D_LINUX -D_REENTRANT)

#################################
#
#          OpenCV >= 3.3.1
#
#################################
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  if(OpenCV_VERSION VERSION_LESS "3.3.1")
    message(FATAL_ERROR "OpenCV >= 3.3.1 not found")
  else()
    message("OpenCV ${OpenCV_VERSION} found.")
  endif()
else()
  message(FATAL_ERROR "OpenCV >= 3.3.1 not found")
endif()


#################################
#
#             Boost
#
#################################

find_package( Boost COMPONENTS log thread system filesystem container REQUIRED )
set(NLopt_DIR /home/viciopoli/InstalledLibs/nlopt/lib/cmake/nlopt)
set(Ceres_DIR /home/viciopoli/InstalledLibs/Ceres/lib/cmake/Ceres)

find_package(x 1.2.4 REQUIRED)
if(x_FOUND)
  message("x found: ${x_VERSION}")
endif()

message("x: ${MULTI_UAV}")

find_package(catkin REQUIRED COMPONENTS 
  cv_bridge
  image_geometry
  dynamic_reconfigure
  image_transport
  roscpp
  tf
  message_generation
  sensor_msgs
  std_msgs
)

# Configure this package
generate_dynamic_reconfigure_options(
  cfg/xvio.cfg
)

add_message_files(FILES
  InertialStateWithCovarianceStamped.msg
  FeatureMsg.msg
  TrackMsg.msg
  MessageUAV.msg
  RequestUAV.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
)

message("${x_INCLUDE_DIRS}")
include_directories (
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${x_INCLUDE_DIRS}
)

# ROS node executable
#####################
set (SOURCE
  src/x_vio_ros/node.cpp
  src/x_vio_ros/main.cpp
)

if(MULTI_UAV)
  set(SOURCE ${SOURCE} src/x_vio_ros/communication.cpp)
endif()

add_executable(x_vio_ros ${SOURCE})

# Make sure configure headers are built before any node using them
add_dependencies(x_vio_ros ${PROJECT_NAME}_gencfg)

# Additional libraries to link against
target_link_libraries(x_vio_ros
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${x_LIBRARIES}
)

# Testing
#########
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(utest_node test/test_vio.test test/utest.cpp)
  target_link_libraries(utest_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${rostest_LIBRARIES}
    ${x_LIBRARIES})
endif()
