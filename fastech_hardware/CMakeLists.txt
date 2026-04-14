cmake_minimum_required(VERSION 3.8)
project(fastech_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ------------------------------------------------------------
# FASTECH Plus-E SDK paths (your case: EVERYTHING is in Include/)
# ------------------------------------------------------------
set(FASTECH_SDK_INCLUDE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/include/fastech_hardware/Include"
  CACHE PATH "Path to FASTECH Plus-E SDK headers & .so (contains FAS_EziMOTIONPlusE.h and libEziMOTIONPlusE.so)"
)

# In your setup, the .so is also inside Include.
set(FASTECH_SDK_LIB_DIR
  "${FASTECH_SDK_INCLUDE_DIR}"
  CACHE PATH "Path to FASTECH Plus-E SDK .so (same as include dir in this layout)"
)

set(_FASTECH_LIB_SEARCH_DIRS
  "${FASTECH_SDK_LIB_DIR}"
  "${FASTECH_SDK_INCLUDE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib"
  "${CMAKE_CURRENT_SOURCE_DIR}/Lib"
)

find_library(EZIMOTIONPLUSE_LIB
  NAMES EziMOTIONPlusE
  PATHS ${_FASTECH_LIB_SEARCH_DIRS}
)

if(NOT EZIMOTIONPLUSE_LIB)
  message(FATAL_ERROR
    "Could not find libEziMOTIONPlusE.so. Searched: ${_FASTECH_LIB_SEARCH_DIRS}\n"
    "Fix: ensure libEziMOTIONPlusE.so is located in include/fastech_hardware/Include/"
  )
endif()

add_library(${PROJECT_NAME} SHARED
  src/fastech_plus_e_system.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Add vendor headers as SYSTEM to suppress warnings from them
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${FASTECH_SDK_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  ${EZIMOTIONPLUSE_LIB}
)

# Runtime lookup for vendor .so
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${FASTECH_SDK_LIB_DIR}"
  INSTALL_RPATH "\$ORIGIN;\$ORIGIN/../lib;${FASTECH_SDK_LIB_DIR}"
)

pluginlib_export_plugin_description_file(hardware_interface fastech_hardware_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# Install vendor .so into install/lib as well (recommended)
install(FILES ${EZIMOTIONPLUSE_LIB}
  DESTINATION lib
)

install(FILES fastech_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fastech_teleop_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()