cmake_minimum_required(VERSION 3.0.2)

# Set desired Python version
if(${PYTHON_VERSION})
  set(PYBIND11_PYTHON_VERSION ${PYTHON_VERSION})
endif()

set(pybind11_DEPENDENCY "")
set(pybind11_catkin_DEPENDENCY "")
find_package(pybind11_catkin CONFIG)
if(NOT ${pybind11_catkin_FOUND})
  find_package(pybind11 REQUIRED)
  set(pybind11_DEPENDENCY "pybind11")
  message(STATUS "Using vanilla pybind11")
else()
  message(STATUS "Using pybind11_catkin")
  set(pybind11_catkin_DEPENDENCY "pybind11_catkin")
endif()
