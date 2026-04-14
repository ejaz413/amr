cmake_minimum_required(VERSION 3.8)
project(lvs_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =========================================================
# Fixed OxApi paths for simple "colcon build"
# =========================================================
set(OXAPI_ROOT "/home/pavetech/OxApi/usr")
set(OXAPI_INCLUDE_DIR "${OXAPI_ROOT}/include")
set(OXAPI_LIBRARY_DIR "${OXAPI_ROOT}/lib/amd64")
set(OXAPI_LIBRARY_FILE "${OXAPI_LIBRARY_DIR}/libOXApi.so")

# =========================================================
# Find packages
# =========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# =========================================================
# Validate OxApi paths
# =========================================================
if(NOT EXISTS "${OXAPI_INCLUDE_DIR}")
  message(FATAL_ERROR "OXAPI_INCLUDE_DIR does not exist: ${OXAPI_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${OXAPI_INCLUDE_DIR}/OXApi/Ox.h")
  message(FATAL_ERROR "OxApi header not found: ${OXAPI_INCLUDE_DIR}/OXApi/Ox.h")
endif()

if(NOT EXISTS "${OXAPI_LIBRARY_FILE}")
  message(FATAL_ERROR "OxApi library not found: ${OXAPI_LIBRARY_FILE}")
endif()

# =========================================================
# Message generation
# =========================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LvsProfile.msg"
  DEPENDENCIES builtin_interfaces
)

# =========================================================
# Include directories
# =========================================================
include_directories(
  include
  ${OXAPI_INCLUDE_DIR}
)

# =========================================================
# Executable
# =========================================================
add_executable(lvs_node
  src/main.cpp
  src/lvs_node.cpp
  src/oxapi_backend.cpp
)

target_compile_definitions(lvs_node PRIVATE LVS_USE_OXAPI=1)

ament_target_dependencies(lvs_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

target_include_directories(lvs_node PRIVATE
  include
  ${OXAPI_INCLUDE_DIR}
)

# Link generated ROS message typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lvs_node "${cpp_typesupport_target}")

# Link OxApi
target_link_directories(lvs_node PRIVATE "${OXAPI_LIBRARY_DIR}")
target_link_libraries(lvs_node "${OXAPI_LIBRARY_FILE}")

# Add runtime search path so lvs_node can find libOXApi.so without extra setup
set_target_properties(lvs_node PROPERTIES
  BUILD_RPATH "${OXAPI_LIBRARY_DIR}"
  INSTALL_RPATH "${OXAPI_LIBRARY_DIR}"
)

# =========================================================
# Install
# =========================================================
install(TARGETS
  lvs_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/lvs_profile_viewer.py
  scripts/lvs_pipeline_filter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# =========================================================
# Export / package
# =========================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()