cmake_minimum_required(VERSION 3.0.2)

find_package(PythonInterp REQUIRED)

set(_InitializerInputFiles)
set(_InitializerOutputFiles)
set(_InitializerCopyFiles)
set(_InitializerScriptDir ${CMAKE_CURRENT_LIST_DIR})
set(_InitializerSearchPaths ${CMAKE_INSTALL_PREFIX}:${CATKIN_DEVEL_PREFIX})
foreach(path ${CMAKE_PREFIX_PATH})
    set(_InitializerSearchPaths "${_InitializerSearchPaths}:${path}")
endforeach()

macro(AddInitializer)
  catkin_destinations()
  file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

  foreach(arg ${ARGN})
    list(APPEND _InitializerInputFiles ${CMAKE_CURRENT_SOURCE_DIR}/init/${arg}.in)
    list(APPEND _InitializerOutputFiles ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${arg}_initializer.h)
    list(APPEND _InitializerCopyFiles ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/init/${arg}.in)
    message(STATUS "Adding initializer: '${arg}'")
  endforeach()
endmacro(AddInitializer)

macro(GenInitializers)
  # Note: use optional arguments to specify dependencies
  set(_InitializerProjectFile ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}_initializers_numerator.h)
  include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
  add_custom_command(
    OUTPUT ${_InitializerOutputFiles} ${_InitializerProjectFile}
    COMMAND ${PYTHON_EXECUTABLE} ${_InitializerScriptDir}/generate_initializers.py ${PROJECT_NAME} "${_InitializerSearchPaths}" ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION} ${_InitializerProjectFile} ${_InitializerInputFiles} ${_InitializerOutputFiles}
    DEPENDS ${_InitializerInputFiles} ${_InitializerScriptDir}/generate_initializers.py
  )
  list(APPEND _InitializerOutputFiles)
  add_custom_target(${PROJECT_NAME}_initializers DEPENDS ${_InitializerOutputFiles} ${ARGN} ${exotica_core_EXPORTED_TARGETS})

  install(FILES ${_InitializerOutputFiles} ${_InitializerProjectFile}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  install(FILES ${_InitializerInputFiles}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/init)

  GenInitializers_append_include_dirs()
endmacro(GenInitializers)

# akin to https://github.com/ros/gencpp/blob/indigo-devel/cmake/gencpp-extras.cmake.em#L54
macro(GenInitializers_append_include_dirs)
  if(NOT GenInitializers_APPENDED_INCLUDE_DIRS)
    # make sure we can find generated messages and that they overlay all other includes
    include_directories(BEFORE ${CATKIN_DEVEL_PREFIX}/include)
    # pass the include directory to catkin_package()
    list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)
    set(GenInitializers_APPENDED_INCLUDE_DIRS TRUE)
  endif()
endmacro()
