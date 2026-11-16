cmake_minimum_required(VERSION 3.10)
project(fast_livo)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Specific settings for Release build
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 64-bit ARM optimizations (e.g., RK3588 and Jetson Orin NX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations: -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    # 32-bit ARM optimizations with NEON support
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations: -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  add_definitions(-DARM_ARCH)
else()
  # x86-64 (Intel/AMD) optimizations
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops") #-flto
  message(STATUS "Using general x86 optimizations: -O3 -march=native -mtune=native -funroll-loops") 
  add_definitions(-DX86_ARCH)
endif()

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# Check for OpenMP support
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# Find catkin and required dependencies
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  gnss_comm
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_srvs
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
  vikit_common
  vikit_ros
  cv_bridge
  image_transport
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
if(EXISTS "/usr/share/cmake/geographiclib")
  list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
endif()
find_package(GeographicLib REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)

set(Sophus_LIBRARIES libSophus.so)

# Define the catkin package
add_message_files(FILES LocalizationStatus.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs gnss_comm nav_msgs roscpp rospy sensor_msgs std_msgs std_srvs tf message_runtime cv_bridge eigen_conversions vikit_common vikit_ros image_transport
  DEPENDS EIGEN3 PCL OpenCV Sophus
)

# Include directories for dependencies
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  include
)

# Add libraries
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(health_monitor src/health_monitor.cpp)
add_library(dynamic_filter src/dynamic_filter.cpp)
target_link_libraries(dynamic_filter ${catkin_LIBRARIES})
add_library(localization_safety src/localization_safety.cpp)
target_link_libraries(localization_safety ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_library(laser_mapping src/LIVMapper.cpp src/mapper_diagnostics.cpp src/mapper_dynamic.cpp src/mapper_safety.cpp src/mapper_resources.cpp)
add_library(optimization src/optimization.cpp)
add_library(online_backend src/online_backend.cpp src/online_backend_ros.cpp)
add_dependencies(online_backend ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(online_backend health_monitor gtsam ${catkin_LIBRARIES}
  ${PCL_LIBRARIES} ${GeographicLib_LIBRARIES})

# Add the main executable
add_executable(fastlivo_mapping src/main.cpp)

add_dependencies(laser_mapping ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(optimization ${catkin_EXPORTED_TARGETS})
add_dependencies(health_monitor ${catkin_EXPORTED_TARGETS})
add_dependencies(fastlivo_mapping ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Link libraries to the executable
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  optimization
  online_backend
  health_monitor
  dynamic_filter
  localization_safety
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  gtsam
)

# Link mimalloc if found
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

# Finite-value checks must not be removed by the ARM release fast-math flags.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(health_monitor PRIVATE -fno-fast-math)
  target_compile_options(laser_mapping PRIVATE -fno-fast-math)
  target_compile_options(optimization PRIVATE -fno-fast-math)
  target_compile_options(online_backend PRIVATE -fno-fast-math)
  target_compile_options(dynamic_filter PRIVATE -fno-fast-math)
  target_compile_options(localization_safety PRIVATE -fno-fast-math)
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(resource_limits_test test/resource_limits_test.cpp)
  if(TARGET resource_limits_test)
    target_link_libraries(resource_limits_test lio vio ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES})
    if(OpenMP_CXX_FOUND)
      target_link_libraries(resource_limits_test OpenMP::OpenMP_CXX)
    endif()
  endif()
  catkin_add_gtest(localization_safety_test test/localization_safety_test.cpp)
  if(TARGET localization_safety_test)
    target_link_libraries(localization_safety_test localization_safety ${catkin_LIBRARIES} ${PCL_LIBRARIES})
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(localization_safety_test PRIVATE -fno-fast-math)
    endif()
  endif()
  catkin_add_gtest(dynamic_filter_test test/dynamic_filter_test.cpp)
  if(TARGET dynamic_filter_test)
    add_dependencies(dynamic_filter_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(dynamic_filter_test dynamic_filter vio lio
      ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES})
    if(OpenMP_CXX_FOUND)
      target_link_libraries(dynamic_filter_test OpenMP::OpenMP_CXX)
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(dynamic_filter_test PRIVATE -fno-fast-math)
    endif()
  endif()
  catkin_add_gtest(backend_events_test test/backend_events_test.cpp)
  if(TARGET backend_events_test)
    add_dependencies(backend_events_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(backend_events_test
      lio optimization health_monitor gtsam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GeographicLib_LIBRARIES})
    if(OpenMP_CXX_FOUND)
      target_link_libraries(backend_events_test OpenMP::OpenMP_CXX)
    endif()
  endif()
  catkin_add_gtest(health_monitor_test test/health_monitor_test.cpp)
  if(TARGET health_monitor_test)
    add_dependencies(health_monitor_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(health_monitor_test health_monitor ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
  endif()
  catkin_add_gtest(vio_reference_test test/vio_reference_test.cpp)
  if(TARGET vio_reference_test)
    add_dependencies(vio_reference_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(vio_reference_test vio lio health_monitor
      ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES})
    if(OpenMP_CXX_FOUND)
      target_link_libraries(vio_reference_test OpenMP::OpenMP_CXX)
    endif()
  endif()
  catkin_add_gtest(online_backend_test test/online_backend_test.cpp)
  if(TARGET online_backend_test)
    add_dependencies(online_backend_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(online_backend_test online_backend)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(online_backend_test PRIVATE -fno-fast-math)
    endif()
  endif()
endif()
