cmake_minimum_required(VERSION 3.0.2)
project(exotica_core_task_maps)

find_package(catkin REQUIRED COMPONENTS
  exotica_core
  exotica_python
  eigen_conversions
  geometry_msgs
)

AddInitializer(
  avoid_look_at_sphere
  center_of_mass
  continuous_joint_pose
  control_regularization
  distance_to_line_2d
  interaction_mesh
  eff_position
  eff_position_xy
  eff_orientation
  eff_frame
  frame_with_axis_and_direction
  frame_with_box_limits
  eff_axis_alignment
  eff_box
  eff_velocity
  gaze_at_constraint
  joint_limit
  joint_torque_minimization_proxy
  joint_velocity_limit
  distance
  point_to_line
  joint_pose
  sphere_collision
  sphere
  collision_check
  point_to_plane
  quasi_static
  joint_velocity_limit_constraint
  joint_velocity_backward_difference
  joint_acceleration_backward_difference
  joint_jerk_backward_difference
  look_at
  manipulability

  collision_distance
  variable_size_collision_distance
  smooth_collision_distance
  sum_of_penetrations
)
GenInitializers()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica_core geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES
    src/avoid_look_at_sphere.cpp  
    src/center_of_mass.cpp
    src/continuous_joint_pose.cpp
    src/control_regularization.cpp
    src/distance_to_line_2d.cpp
    src/interaction_mesh.cpp
    src/eff_position.cpp
    src/eff_position_xy.cpp
    src/eff_orientation.cpp
    src/eff_frame.cpp
    src/eff_axis_alignment.cpp
    src/eff_box.cpp
    src/eff_velocity.cpp
    src/gaze_at_constraint.cpp
    src/joint_limit.cpp
    src/joint_torque_minimization_proxy.cpp
    src/joint_velocity_limit.cpp
    src/distance.cpp
    src/point_to_line.cpp
    src/point_to_plane.cpp
    src/joint_pose.cpp
    src/sphere_collision.cpp
    src/collision_check.cpp
    src/collision_distance.cpp
    src/variable_size_collision_distance.cpp
    src/smooth_collision_distance.cpp
    src/sum_of_penetrations.cpp
    src/quasi_static.cpp
    src/joint_velocity_limit_constraint.cpp
    src/joint_velocity_backward_difference.cpp
    src/joint_acceleration_backward_difference.cpp
    src/joint_jerk_backward_difference.cpp
    src/look_at.cpp
    src/manipulability.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})

pybind11_add_module(${PROJECT_NAME}_py MODULE src/task_map_py.cpp)
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_py ${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})

## Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS ${PROJECT_NAME}_py LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_maps test/test_maps.cpp)
  target_link_libraries(test_maps ${catkin_LIBRARIES})
  add_dependencies(test_maps ${catkin_EXPORTED_TARGETS})
endif()
