cmake_minimum_required(VERSION 3.10)

project(SAI-GRAPHICS)
set(PROJECT_VERSION 0.1.0)

option(BUILD_EXAMPLES "Build examples" ON)

set(CMAKE_CXX_FLAGS "-std=c++17 -I/usr/include -I/usr/local/include -fPIC")
if(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/homebrew/include")
endif()

# set default build to release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# include Chai3d
find_package(CHAI3D REQUIRED)
add_definitions(${CHAI3D_DEFINITIONS})
link_directories(${CHAI3D_LIBRARY_DIRS})

# include Sai-Model
find_package(SAI-MODEL REQUIRED)

# include Eigen
find_package(Eigen3 REQUIRED)

# include sai-URDF
find_package(SAI-URDF REQUIRED)

# include Parser
set(PARSER_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/parser)
set(PARSER_SOURCE ${PROJECT_SOURCE_DIR}/src/parser/UrdfToSaiGraphics.cpp)

# glfw3
find_package(glfw3 QUIET)
find_library(GLFW_LIBRARY glfw)

# include Widgets
set(WIDGETS_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/widgets)
set(WIDGETS_SOURCE ${PROJECT_SOURCE_DIR}/src/widgets/UIForceWidget.cpp
                   ${PROJECT_SOURCE_DIR}/src/widgets/ForceSensorDisplay.cpp)

# include Graphics
set(SAI-GRAPHICS_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)
set(GRAPHICS_SOURCE
    ${PROJECT_SOURCE_DIR}/src/SaiGraphics.cpp
    ${PROJECT_SOURCE_DIR}/src/chai_extension/Capsule.cpp
    ${PROJECT_SOURCE_DIR}/src/chai_extension/CapsuleMesh.cpp
    ${PROJECT_SOURCE_DIR}/src/chai_extension/Pyramid.cpp
    ${PROJECT_SOURCE_DIR}/src/chai_extension/PyramidMesh.cpp)

# Add the include directory to the include paths
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${SAI-GRAPHICS_INCLUDE_DIRS}
  ${SAI-MODEL_INCLUDE_DIRS}
  ${SAI-URDF_INCLUDE_DIRS}
  ${CHAI3D_INCLUDE_DIRS}
  ${PARSER_INCLUDE_DIR}
  ${WIDGETS_INCLUDE_DIR})

# Create the library
add_library(sai-graphics STATIC ${GRAPHICS_SOURCE} ${PARSER_SOURCE}
                                 ${WIDGETS_SOURCE})

set(SAI-GRAPHICS_LIBRARIES sai-graphics ${GLFW_LIBRARY})

#
# export package
#

# export package for use from the build tree
export(TARGETS sai-graphics
       FILE ${PROJECT_BINARY_DIR}/SAI-GRAPHICSTargets.cmake)
export(PACKAGE SAI-GRAPHICS)

# configure package
set(CONF_INCLUDE_DIRS ${SAI-GRAPHICS_INCLUDE_DIRS})
set(CONF_LIBRARIES ${SAI-GRAPHICS_LIBRARIES})
set(CONF_DEFINITIONS ${PROJECT_DEFINITIONS})

# create package configuration and version files
configure_file(SAI-GRAPHICSConfig.cmake.in
               "${PROJECT_BINARY_DIR}/SAI-GRAPHICSConfig.cmake" @ONLY)
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SAI-GRAPHICSConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

# add examples
if(BUILD_EXAMPLES)
  add_subdirectory(${PROJECT_SOURCE_DIR}/examples)
endif()
