# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.20.2)
project(robot_body_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG TRUE)

set(THIS_PACKAGE_DEPS
  cras_cpp_common
  fcl
  filters
  geometric_shapes
  laser_geometry
  moveit_core
  moveit_ros_perception
  pcl_conversions
  pluginlib
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros
  tf2_sensor_msgs
  Eigen3
  GLUT
  Threads
)
set(MESSAGE_DEPS
  geometry_msgs
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(PCL REQUIRED COMPONENTS common filters)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/OrientedBoundingBox.msg"
  "msg/OrientedBoundingBoxStamped.msg"
  "msg/SphereStamped.msg"
  "msg/Sphere.msg"
  DEPENDENCIES ${MESSAGE_DEPS}
  LIBRARY_NAME ${PROJECT_NAME}
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}_msgs" "rosidl_typesupport_cpp")

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(UTILS_SRCS
  src/utils/bodies.cpp
  src/utils/shapes.cpp
  src/utils/tf2_eigen.cpp
)

add_library(${PROJECT_NAME}_utils ${UTILS_SRCS})
target_include_directories(${PROJECT_NAME}_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_utils ${THIS_PACKAGE_DEPS})
target_link_libraries(${PROJECT_NAME}_utils ${PCL_LIBRARIES})

add_library(RayCastingShapeMask src/RayCastingShapeMask.cpp)
target_link_libraries(RayCastingShapeMask ${PROJECT_NAME}_utils)

add_library(TFFramesWatchdog src/TFFramesWatchdog.cpp)
target_link_libraries(TFFramesWatchdog ${PROJECT_NAME}_utils)

add_library(${PROJECT_NAME} src/RobotBodyFilter.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  Threads::Threads
  ${PROJECT_NAME}_utils
  TFFramesWatchdog
  RayCastingShapeMask
  "${cpp_typesupport_target}"
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})

add_library(${PROJECT_NAME}_version INTERFACE)
ament_generate_version_header(${PROJECT_NAME}_version)

ament_export_include_directories(include)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(TARGETS ${PROJECT_NAME} TFFramesWatchdog RayCastingShapeMask ${PROJECT_NAME}_utils ${PROJECT_NAME}_version
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}/examples
)
install(FILES rviz/debug.rviz
  DESTINATION share/${PROJECT_NAME}/rviz
)

pluginlib_export_plugin_description_file(filters laser_filters.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(cras_lint REQUIRED)
  find_package(ros_testing REQUIRED)

  cras_lint_common()
  cras_lint_cpp(SKIP_UNCRUSTIFY)
  cras_lint_py()

  ament_add_gtest(test_shapes_rbf test/test_shapes.cpp)
  target_link_libraries(test_shapes_rbf ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_shapes_rbf ${THIS_PACKAGE_DEPS})
  target_compile_definitions(test_shapes_rbf PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")

  ament_add_gtest(test_tf2_eigen test/test_tf2_eigen.cpp)
  target_link_libraries(test_tf2_eigen ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_tf2_eigen ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_bodies test/test_bodies.cpp)
  target_link_libraries(test_bodies ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_bodies ${THIS_PACKAGE_DEPS})
  target_compile_definitions(test_bodies PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")

  ament_add_gtest_executable(test_filter_utils test/test_filter_utils.cpp)
  target_include_directories(test_filter_utils PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_filter_utils ${THIS_PACKAGE_DEPS})
  add_ros_test(test/test_filter_utils.test.py TIMEOUT 30 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest_executable(test_tf_frames_watchdog test/test_tf_frames_watchdog.cpp)
  target_link_libraries(test_tf_frames_watchdog TFFramesWatchdog)
  ament_target_dependencies(test_tf_frames_watchdog ${THIS_PACKAGE_DEPS})
  add_ros_test(test/test_with_ros.test.py TIMEOUT 30 TARGET test_tf_frames_watchdog_rostest ARGS
    "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}" "binary:=test_tf_frames_watchdog" "name:=test_tf_frames_watchdog")

  ament_add_gtest_executable(test_ray_casting_shape_mask test/test_ray_casting_shape_mask.cpp)
  target_link_libraries(test_ray_casting_shape_mask RayCastingShapeMask)
  ament_target_dependencies(test_ray_casting_shape_mask ${THIS_PACKAGE_DEPS})
  if("${CMAKE_BUILD_TYPE}" STREQUAL "Release" OR "${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    target_compile_definitions(test_ray_casting_shape_mask PRIVATE RELEASE_BUILD=1)
  else()
    target_compile_definitions(test_ray_casting_shape_mask PRIVATE RELEASE_BUILD=0)
  endif()
  target_compile_definitions(test_ray_casting_shape_mask PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
  add_ros_test(test/test_with_ros.test.py TIMEOUT 30 TARGET test_ray_casting_shape_mask_rostest ARGS
    "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}" "binary:=test_ray_casting_shape_mask" "name:=test_ray_casting_mask")

  ament_add_gtest_executable(test_robot_body_filter test/test_robot_body_filter.cpp)
  target_link_libraries(test_robot_body_filter ${PROJECT_NAME})
  ament_target_dependencies(test_robot_body_filter ${THIS_PACKAGE_DEPS})
  target_compile_options(test_robot_body_filter PRIVATE -fno-var-tracking-assignments)  # speed up build
  add_ros_test(test/test_robot_body_filter.test.py TIMEOUT 30 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  install(
    FILES test/box.dae test/triangle.dae
    DESTINATION share/robot_body_filter/test)
endif()

ament_package()
