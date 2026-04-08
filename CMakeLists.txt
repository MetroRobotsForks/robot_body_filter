cmake_minimum_required(VERSION 3.8)
project(robot_body_filter)

set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(DetectOptional)

if(${ROBOT_BODY_FILTER_HAVE_CXX_OPTIONAL})
  # add_compile_definitions would be nicer, but isn't available in Stretch
  add_compile_options(-DROBOT_BODY_FILTER_USE_CXX_OPTIONAL=1)
else()
  add_compile_options(-DROBOT_BODY_FILTER_USE_CXX_OPTIONAL=0)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPS cras_cpp_common fcl filters geometric_shapes laser_geometry moveit_core moveit_ros_perception pcl_conversions pluginlib
    rclcpp sensor_msgs std_srvs tf2_ros tf2_sensor_msgs Eigen3 GLUT
)
set(MESSAGE_DEPS geometry_msgs std_msgs)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPS})
    find_package(${Dependency} REQUIRED)
endforeach()

find_package(PCL REQUIRED COMPONENTS common filters)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(
  ${PROJECT_NAME}_msgs
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
  src/utils/cloud.cpp
  src/utils/shapes.cpp
  src/utils/string_utils.cpp
  src/utils/tf2_eigen.cpp
  src/utils/tf2_sensor_msgs.cpp
  src/utils/time_utils.cpp)

add_library(${PROJECT_NAME}_utils ${UTILS_SRCS})
target_include_directories(${PROJECT_NAME}_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_utils ${THIS_PACKAGE_DEPS})
target_link_libraries(${PROJECT_NAME}_utils ${PCL_LIBRARIES})

add_library(tf2_sensor_msgs_rbf src/utils/tf2_sensor_msgs.cpp)
target_link_libraries(tf2_sensor_msgs_rbf ${PROJECT_NAME}_utils)
ament_export_libraries(tf2_sensor_msgs_rbf)

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
  ${PROJECT_NAME}_utils
  tf2_sensor_msgs_rbf
  TFFramesWatchdog
  RayCastingShapeMask
  "${cpp_typesupport_target}"
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPS})
ament_export_targets(export_${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include
)
install(TARGETS ${PROJECT_NAME} TFFramesWatchdog RayCastingShapeMask ${PROJECT_NAME}_utils tf2_sensor_msgs_rbf
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(FILES laser_filters.xml
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY examples/
        DESTINATION share/${PROJECT_NAME}/examples
)
install(FILES rviz/debug.rviz
        DESTINATION share/${PROJECT_NAME}/rviz
)
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(filters laser_filters.xml)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_set_utils test/test_set_utils.cpp)
  target_link_libraries(test_set_utils ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_set_utils ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_shapes_rbf test/test_shapes.cpp)
  target_link_libraries(test_shapes_rbf ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_shapes_rbf ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_time_utils test/test_time_utils.cpp)
  target_link_libraries(test_time_utils ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_time_utils ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_tf2_eigen test/test_tf2_eigen.cpp)
  target_link_libraries(test_tf2_eigen ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_tf2_eigen ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_string_utils test/test_string_utils.cpp)
  target_link_libraries(test_string_utils ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_string_utils ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_bodies test/test_bodies.cpp)
  target_link_libraries(test_bodies ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_bodies ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_cloud test/test_cloud.cpp)
  target_link_libraries(test_cloud ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_cloud ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_tf2_sensor_msgs test/test_tf2_sensor_msgs.cpp)
  target_link_libraries(test_tf2_sensor_msgs ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_tf2_sensor_msgs ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_filter_utils test/test_filter_utils.test test/test_filter_utils.cpp)
  target_link_libraries(test_filter_utils ${PROJECT_NAME}_utils)
  ament_target_dependencies(test_filter_utils ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_tf_frames_watchdog test/test_tf_frames_watchdog.cpp)
  target_link_libraries(test_tf_frames_watchdog TFFramesWatchdog)
  ament_target_dependencies(test_tf_frames_watchdog ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_ray_casting_shape_mask test/test_ray_casting_shape_mask.cpp)
  target_link_libraries(test_ray_casting_shape_mask RayCastingShapeMask)
  ament_target_dependencies(test_ray_casting_shape_mask ${THIS_PACKAGE_DEPS})
  if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release" OR "${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    target_compile_definitions(test_ray_casting_shape_mask PRIVATE RELEASE_BUILD=1)
  else()
    target_compile_definitions(test_ray_casting_shape_mask PRIVATE RELEASE_BUILD=0)
  endif()

  ament_add_gtest(test_robot_body_filter test/test_robot_body_filter.test test/test_robot_body_filter.cpp)
  target_link_libraries(test_robot_body_filter ${PROJECT_NAME})
  ament_target_dependencies(test_robot_body_filter ${THIS_PACKAGE_DEPS})
  target_compile_options(test_robot_body_filter PRIVATE -fno-var-tracking-assignments)  # speed up build

  install(FILES test/box.dae test/triangle.dae
    DESTINATION share/robot_body_filter/test)
endif()

ament_package()
