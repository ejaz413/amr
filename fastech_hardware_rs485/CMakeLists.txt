cmake_minimum_required(VERSION 3.8)
project(fastech_hardware_rs485)

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
# FASTECH SDK paths (DEFAULT to the copies inside this package)
# ------------------------------------------------------------
set(FASTECH_SDK_INCLUDE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/include/fastech_hardware_rs485/Include"
  CACHE PATH "Path to FASTECH SDK headers (folder that contains FAS_EziMOTIONPlusR.h)"
)

# Try a few common lib locations (you can override with -DFASTECH_SDK_LIB_DIR=...)
set(FASTECH_SDK_LIB_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/include/fastech_hardware_rs485/Lib"
  CACHE PATH "Path to FASTECH SDK library directory (folder that contains libEziMOTIONPlusR.so)"
)

# If user copied the .so into Include folder (common in some zips), search there too.
set(_FASTECH_LIB_SEARCH_DIRS
  "${FASTECH_SDK_LIB_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/fastech_hardware_rs485/Include"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib"
  "${CMAKE_CURRENT_SOURCE_DIR}/Lib"
)

find_library(EZIMOTIONPLUSR_LIB
  NAMES EziMOTIONPlusR
  PATHS ${_FASTECH_LIB_SEARCH_DIRS}
)

if(NOT EZIMOTIONPLUSR_LIB)
  message(FATAL_ERROR
    "Could not find libEziMOTIONPlusR.so. Searched: ${_FASTECH_LIB_SEARCH_DIRS}\n"
    "Fix: put libEziMOTIONPlusR.so into one of those folders OR build with:\n"
    "  -DFASTECH_SDK_LIB_DIR=/path/to/fastech/Lib"
  )
endif()

add_library(${PROJECT_NAME} SHARED
  src/fastech_system.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${FASTECH_SDK_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# IMPORTANT: use keyword signature (PRIVATE) to avoid the plain/keyword conflict
target_link_libraries(${PROJECT_NAME} ${EZIMOTIONPLUSR_LIB}
)

# Make runtime find vendor .so (either in its original dir, or next to our plugin)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${FASTECH_SDK_LIB_DIR}"
  INSTALL_RPATH "\$ORIGIN;\$ORIGIN/../lib;${FASTECH_SDK_LIB_DIR}"
)

pluginlib_export_plugin_description_file(hardware_interface fastech_hardware_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# OPTIONAL but recommended: also install the vendor .so into install/lib
# so you don't need LD_LIBRARY_PATH later.
install(FILES ${EZIMOTIONPLUSR_LIB}
  DESTINATION lib
)

install(FILES fastech_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fastech_teleop_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
