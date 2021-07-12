#
# This project is a convenience wrapper to build Exotica using pure CMake
# Dependencies must be manually installed and on the CMAKE_PREFIX_PATH before continuing.
#
cmake_minimum_required(VERSION 3.0.2)

# From https://stackoverflow.com/a/48555098
# MUST be done before call to 'project'
get_cmake_property(vars CACHE_VARIABLES)
foreach(var ${vars})
  get_property(currentHelpString CACHE "${var}" PROPERTY HELPSTRING)
  if("${currentHelpString}" MATCHES "No help, variable specified on the command line." OR "${currentHelpString}" STREQUAL "")
    # message("${var} = [${${var}}]  --  ${currentHelpString}") # uncomment to see the variables being processed
    list(APPEND CL_ARGS "-D${var}=${${var}}")
  endif()
endforeach()

project(exotica_superbuild)

# Add all Exotica packages as individual external projects
include(ExternalProject)

# Core & Python
ExternalProject_Add(exotica_core
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotica_core
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
ExternalProject_Add(exotica_python
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotica_python
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core)

# Task Maps
ExternalProject_Add(exotica_core_task_maps
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/exotica_core_task_maps
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core exotica_python)

# Collision Scenes
ExternalProject_Add(exotica_collision_scene_fcl_latest
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/exotica_collision_scene_fcl_latest
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core)

# Motion Solvers
ExternalProject_Add(exotica_aico_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/solvers/exotica_aico_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core)
ExternalProject_Add(exotica_ddp_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/solvers/exotica_ddp_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core exotica_python)
ExternalProject_Add(exotica_ik_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/solvers/exotica_ik_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core)
ExternalProject_Add(exotica_ilqg_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/solvers/exotica_ilqg_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core exotica_python)
ExternalProject_Add(exotica_ilqr_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/solvers/exotica_ilqr_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core exotica_python)
ExternalProject_Add(exotica_levenberg_marquardt_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/solvers/exotica_levenberg_marquardt_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core)
ExternalProject_Add(exotica_ompl_control_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/solvers/exotica_ompl_control_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core exotica_python)
ExternalProject_Add(exotica_ompl_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/solvers/exotica_ompl_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core exotica_python)
ExternalProject_Add(exotica_scipy_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/solvers/exotica_scipy_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core exotica_python)
ExternalProject_Add(exotica_time_indexed_rrt_connect_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/solvers/exotica_time_indexed_rrt_connect_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core exotica_python)

# Dynamics Solvers
ExternalProject_Add(exotica_cartpole_dynamics_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/dynamics_solvers/exotica_cartpole_dynamics_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core)
ExternalProject_Add(exotica_double_integrator_dynamics_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/dynamics_solvers/exotica_double_integrator_dynamics_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core)
ExternalProject_Add(exotica_pendulum_dynamics_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/dynamics_solvers/exotica_pendulum_dynamics_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core)
ExternalProject_Add(exotica_pinocchio_dynamics_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/dynamics_solvers/exotica_pinocchio_dynamics_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core)
ExternalProject_Add(exotica_quadrotor_dynamics_solver
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotations/dynamics_solvers/exotica_quadrotor_dynamics_solver
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS exotica_core)

# Examples
ExternalProject_Add(exotica_examples
  URL ${CMAKE_CURRENT_SOURCE_DIR}/../exotica_examples
  CMAKE_ARGS ${CL_ARGS}
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  DEPENDS
    exotica_aico_solver
    exotica_core_task_maps
    exotica_core
    exotica_ik_solver
)
