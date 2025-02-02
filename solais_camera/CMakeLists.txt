cmake_minimum_required(VERSION 3.8)
project(solais_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# Build server and client
add_library(${PROJECT_NAME} SHARED
  src/solais_camera_server.cpp
  src/solais_camera_client.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  camera_info_manager
  cv_bridge
  OpenCV
  image_transport
  rclcpp
  rclcpp_components
  sensor_msgs
  solais_interfaces)

# Build usb_camera
add_library(usb_camera_component SHARED
  src/usb_camera/solais_usb_camera.cpp
  src/usb_camera/solais_usb_camera_node.cpp)
target_link_libraries(usb_camera_component ${PROJECT_NAME})
target_include_directories(usb_camera_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(usb_camera_component
  PLUGIN "solais_camera::USBCameraNode"
  EXECUTABLE usb_camera_node)

# Build ip_camera
add_library(ip_camera_component SHARED
  src/ip_camera/solais_ip_camera.cpp
  src/ip_camera/solais_ip_camera_node.cpp)
target_link_libraries(ip_camera_component ${PROJECT_NAME})
target_include_directories(ip_camera_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(ip_camera_component
  PLUGIN "solais_camera::IPCameraNode"
  EXECUTABLE ip_camera_node)

add_library(mindvision_camera_component SHARED
  src/mindvision_camera/solais_mindvision_camera.cpp
  src/mindvision_camera/solais_mindvision_camera_node.cpp)

message(STATUS "System architecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(MVSDK_LIB_DIR "mvsdk/lib/x64")
    target_link_directories(mindvision_camera_component PUBLIC "mvsdk/lib/x64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(MVSDK_LIB_DIR "mvsdk/lib/arm64")
    target_link_directories(mindvision_camera_component PUBLIC "mvsdk/lib/arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(MVSDK_LIB_DIR "mvsdk/lib/arm64")
    target_link_directories(mindvision_camera_component PUBLIC "mvsdk/lib/arm64")
else()
    message(FATAL_ERROR "Unkown host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()
set(MVSDK_LIB_SO "${PROJECT_SOURCE_DIR}/${MVSDK_LIB_DIR}/libMVSDK.so")
target_link_libraries(mindvision_camera_component PRIVATE ${PROJECT_NAME} MVSDK)
target_include_directories(mindvision_camera_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/mvsdk/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(mindvision_camera_component
  PLUGIN "solais_camera::MindVisionCameraNode"
  EXECUTABLE mindvision_camera_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  usb_camera_component
  ip_camera_component
  mindvision_camera_component
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
    FILES ${MVSDK_LIB_SO}
    DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  camera_info_manager
  cv_bridge
  OpenCV
  image_transport
  rclcpp
  rclcpp_components
  sensor_msgs
  solais_interfaces
)

ament_package()
