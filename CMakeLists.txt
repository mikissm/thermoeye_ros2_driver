cmake_minimum_required(VERSION 3.8)
project(thermal_cam_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================================
# ROS2
# =====================================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED)

# =====================================
# TmSDK
# =====================================
set(TMSDK_ROOT "$ENV{TMSDK_ROOT}" CACHE PATH "TmSDK extraction root containing usr/include and usr/lib/TmSDK (empty uses system installation)")

# Clear discoveries when an explicitly selected root changes. An environment
# change needs --cmake-clean-cache because TMSDK_ROOT itself is a cache entry.
if(NOT "${TMSDK_DISCOVERY_ROOT}" STREQUAL "${TMSDK_ROOT}")
  unset(TMSDK_INCLUDE_DIR CACHE)
  unset(TMSDK_CORE_LIBRARY CACHE)
endif()
set(TMSDK_DISCOVERY_ROOT "${TMSDK_ROOT}" CACHE INTERNAL "Last SDK search root" FORCE)
if(TMSDK_ROOT)
  find_path(TMSDK_INCLUDE_DIR TmSDK/TmLocalCamera.hxx
      PATHS "${TMSDK_ROOT}/usr/include" NO_DEFAULT_PATH)
  find_library(TMSDK_CORE_LIBRARY
      NAMES TmCore.2.1.0 TmCore.2.0.1 TmCore
      PATHS "${TMSDK_ROOT}/usr/lib/TmSDK" NO_DEFAULT_PATH)
else()
  find_path(TMSDK_INCLUDE_DIR TmSDK/TmLocalCamera.hxx)
  find_library(TMSDK_CORE_LIBRARY
      NAMES TmCore.2.1.0 TmCore.2.0.1 TmCore
      PATHS /usr/lib/TmSDK)
endif()
if(NOT TMSDK_INCLUDE_DIR OR NOT TMSDK_CORE_LIBRARY)
  message(FATAL_ERROR "TmSDK headers/library not found. Run set.sh and source .tmsdk/env.sh, or set -DTMSDK_ROOT=/path/to/extracted/sdk")
endif()
get_filename_component(TMSDK_LIB_DIR "${TMSDK_CORE_LIBRARY}" DIRECTORY)

message(STATUS "TMSDK_ROOT      = ${TMSDK_ROOT}")
message(STATUS "TMSDK_LIB_DIR   = ${TMSDK_LIB_DIR}")

# =====================================
# Include
# =====================================
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)
include_directories(SYSTEM ${TMSDK_INCLUDE_DIR})

link_directories(
    ${TMSDK_LIB_DIR}
)

# =====================================
# Executable
# =====================================
add_executable(thermal_node
    src/thermal_node.cpp
    src/thermal_camera.cpp
)
target_compile_features(thermal_node PRIVATE cxx_std_17)

ament_target_dependencies(
    thermal_node
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    cv_bridge
    image_transport
)

# =====================================
# Libraries
# =====================================
target_link_libraries(
    thermal_node
    ${TMSDK_CORE_LIBRARY}
    ${TMSDK_LIB_DIR}/libopencv_world.so
    ${OpenCV_LIBS}
)

# =====================================
# RPATH
# =====================================
set_target_properties(thermal_node PROPERTIES
    BUILD_RPATH "${TMSDK_LIB_DIR}"
    INSTALL_RPATH "${TMSDK_LIB_DIR}"
)
# Encode a Python literal instead of interpolating an unescaped filesystem path.
set(TMSDK_LIB_DIR_PY "${TMSDK_LIB_DIR}")
string(REPLACE "\\" "\\\\" TMSDK_LIB_DIR_PY "${TMSDK_LIB_DIR_PY}")
string(REPLACE "\"" "\\\"" TMSDK_LIB_DIR_PY "${TMSDK_LIB_DIR_PY}")
string(REPLACE "\n" "\\n" TMSDK_LIB_DIR_PY "${TMSDK_LIB_DIR_PY}")
string(REPLACE "\r" "\\r" TMSDK_LIB_DIR_PY "${TMSDK_LIB_DIR_PY}")
string(REPLACE "\t" "\\t" TMSDK_LIB_DIR_PY "${TMSDK_LIB_DIR_PY}")
set(TMSDK_LIB_DIR_PY "\"${TMSDK_LIB_DIR_PY}\"")
configure_file(launch/thermal_camera.launch.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/thermal_camera.launch.py @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/thermal_camera.launch.py
    DESTINATION share/${PROJECT_NAME}/launch)
configure_file(launch/thermal_camera_manager.launch.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/thermal_camera_manager.launch.py @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/thermal_camera_manager.launch.py
    DESTINATION share/${PROJECT_NAME}/launch)

# =====================================
# Install
# =====================================
install(
    TARGETS thermal_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_package()
install(PROGRAMS scripts/thermal_calibration_gui.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/thermal_camera_manager.py DESTINATION lib/${PROJECT_NAME})
