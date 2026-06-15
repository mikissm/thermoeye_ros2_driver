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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED)

# =====================================
# TmSDK
# =====================================
set(TMSDK_ROOT "/home/ssm/vision2601_ws/third_party/tmsdk")

set(TMSDK_INCLUDE_DIR
    ${TMSDK_ROOT}/usr/include
)

set(TMSDK_LIB_DIR
    ${TMSDK_ROOT}/usr/lib/TmSDK
)

message(STATUS "TMSDK_ROOT      = ${TMSDK_ROOT}")
message(STATUS "TMSDK_LIB_DIR   = ${TMSDK_LIB_DIR}")

# =====================================
# Include
# =====================================
include_directories(
    include
    ${TMSDK_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

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

ament_target_dependencies(
    thermal_node
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
)

# =====================================
# Libraries
# =====================================
target_link_libraries(
    thermal_node
    ${TMSDK_LIB_DIR}/libTmCore.2.0.1.so
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