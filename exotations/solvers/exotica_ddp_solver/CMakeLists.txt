cmake_minimum_required(VERSION 3.0.2)
project(exotica_ddp_solver)

find_package(catkin REQUIRED COMPONENTS
  exotica_core
  exotica_python
)

AddInitializer(
  abstract_ddp_solver
  analytic_ddp_solver
  control_limited_ddp_solver
  feasibility_driven_ddp_solver
  control_limited_feasibility_driven_ddp_solver
)
GenInitializers()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS exotica_core
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Ignore Eigen::Tensor warnings
add_compile_options(-Wno-ignored-attributes)

set(SOURCES
  src/abstract_ddp_solver.cpp
  src/analytic_ddp_solver.cpp
  src/control_limited_ddp_solver.cpp
  src/feasibility_driven_ddp_solver.cpp
  src/control_limited_feasibility_driven_ddp_solver.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})

pybind11_add_module(${PROJECT_NAME}_py MODULE src/ddp_solver_py.cpp)
target_link_libraries(${PROJECT_NAME}_py PRIVATE ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_py ${PROJECT_NAME} ${PROJECT_NAME}_initializers ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES exotica_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS ${PROJECT_NAME}_py LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
