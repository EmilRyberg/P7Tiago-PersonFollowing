cmake_minimum_required(VERSION 3.5)
project(behavior_tree_plugins)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)

find_package(nav2_behavior_tree REQUIRED)
find_package(person_follower_interfaces REQUIRED)
#find_package(tiago_person_following REQUIRED)

nav2_package()

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  nav2_util
  nav2_behavior_tree
  person_follower_interfaces
  #tiago_person_following
)

set(library_name ${PROJECT_NAME})

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include_directories(
  include/behavior_tree_plugins
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
#find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#add_library(${library_name} SHARED
#  src/behavior_tree_engine.cpp
#)

#ament_target_dependencies(${library_name}
##  ${dependencies}
#)

add_library(find_human_action SHARED src/find_human_action.cpp)
list(APPEND plugin_libs find_human_action)

add_library(find_same_human_action SHARED src/find_same_human_action.cpp)
list(APPEND plugin_libs find_same_human_action)

add_library(has_moved_condition SHARED src/has_moved_condition.cpp)
list(APPEND plugin_libs has_moved_condition)

add_library(is_human_found_condition SHARED src/is_human_found_condition.cpp)
list(APPEND plugin_libs is_human_found_condition)

add_library(is_first_run_condition SHARED src/is_first_run_condition.cpp)
list(APPEND plugin_libs is_first_run_condition)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

target_include_directories(find_human_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(find_same_human_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(has_moved_condition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(is_human_found_condition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(is_first_run_condition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES UpdatedBehaviourTreePalette.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
