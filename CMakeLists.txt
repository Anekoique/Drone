cmake_minimum_required(VERSION 3.14)
project(drone VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# --- Utility library (math, control, trajectory) ---
add_library(drone_utils
  src/control/fuzzy_pid.cpp
  src/control/autotune.cpp
  src/math/utils.cpp
  src/math/pid.cpp
  src/math/scurve.cpp
  src/trajectory/trajectory_generator.cpp
)

target_include_directories(drone_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(drone_utils PUBLIC yaml-cpp)
ament_target_dependencies(drone_utils PUBLIC Eigen3 ament_index_cpp)
target_include_directories(drone_utils SYSTEM PUBLIC ${Eigen3_INCLUDE_DIRS})

# --- Perception library (camera, filter, clustering, adapter) ---
add_library(drone_perception
  src/perception/camera_driver.cpp
  src/perception/camera_model.cpp
  src/perception/detection_filter.cpp
  src/perception/clustering.cpp
  src/perception/detection_adapter.cpp
  src/perception/detector/postprocess.cpp
)

target_include_directories(drone_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(drone_perception PUBLIC drone_utils)
ament_target_dependencies(drone_perception PUBLIC Eigen3 OpenCV vision_msgs rclcpp)
target_include_directories(drone_perception SYSTEM PUBLIC ${Eigen3_INCLUDE_DIRS})

# --- Conditional CUDA/TensorRT detector ---
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  find_package(CUDAToolkit)
  find_library(NVINFER_LIB nvinfer PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

  if(CUDAToolkit_FOUND AND NVINFER_LIB)
    message(STATUS "CUDA + TensorRT found — building drone_perception_cuda")
    add_definitions(-DDRONE_HAS_CUDA)

    add_library(drone_perception_cuda
      src/perception/detector/detector.cpp
      src/perception/detector/preprocess.cu
      src/perception/detector/plugin/yolo_layer.cu
    )

    target_include_directories(drone_perception_cuda PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(drone_perception_cuda PUBLIC
      drone_perception
      CUDA::cudart
      ${NVINFER_LIB}
    )
    set_target_properties(drone_perception_cuda PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
  else()
    message(STATUS "CUDA found but TensorRT missing — detector excluded")
  endif()
else()
  message(STATUS "No CUDA compiler — detector excluded")
endif()

# --- Control library (position control, no ROS) ---
add_library(drone_control
  src/control/yaw_utils.cpp
  src/control/velocity_controller.cpp
  src/control/cascade_controller.cpp
  src/control/trajectory_controller.cpp
)

target_include_directories(drone_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(drone_control PUBLIC drone_utils)
ament_target_dependencies(drone_control PUBLIC Eigen3)
target_include_directories(drone_control SYSTEM PUBLIC ${Eigen3_INCLUDE_DIRS})

# --- Drivers library (MAVROS interface + ROS-coupled control) ---
add_library(drone_drivers
  src/drivers/node_base.cpp
  src/drivers/motors.cpp
  src/drivers/inertial_nav.cpp
  src/drivers/servo.cpp
  src/drivers/gimbal.cpp
  src/control/mavros_commander.cpp
  src/control/pos_control.cpp
)

target_include_directories(drone_drivers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(drone_drivers PUBLIC drone_control drone_utils)
ament_target_dependencies(drone_drivers PUBLIC
  rclcpp mavros_msgs geometry_msgs nav_msgs sensor_msgs std_msgs Eigen3)
target_include_directories(drone_drivers SYSTEM PUBLIC ${Eigen3_INCLUDE_DIRS})

# --- Mission library (state machine + handlers) ---
add_library(drone_mission
  src/mission/mission_types.cpp
  src/mission/frame_transforms.cpp
  src/mission/waypoint_nav.cpp
  src/mission/takeoff_handler.cpp
  src/mission/airdrop_handler.cpp
  src/mission/recon_handler.cpp
  src/mission/landing_handler.cpp
  src/mission/drone_node.cpp
)

target_include_directories(drone_mission PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(drone_mission PUBLIC drone_drivers drone_control drone_utils)
ament_target_dependencies(drone_mission PUBLIC
  rclcpp std_msgs mavros_msgs geometry_msgs Eigen3)
target_include_directories(drone_mission SYSTEM PUBLIC ${Eigen3_INCLUDE_DIRS})

# --- Install ---
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(TARGETS drone_utils drone_control drone_perception drone_drivers drone_mission
  EXPORT export_drone
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(TARGET drone_perception_cuda)
  install(TARGETS drone_perception_cuda
    EXPORT export_drone
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

ament_export_targets(export_drone HAS_LIBRARY_TARGET)
ament_export_dependencies(
  eigen3_cmake_module Eigen3 ament_index_cpp yaml-cpp OpenCV
  vision_msgs rclcpp mavros_msgs geometry_msgs nav_msgs sensor_msgs std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Phase 1 — utils
  ament_add_gtest(test_timer test/test_timer.cpp)
  target_link_libraries(test_timer drone_utils)

  ament_add_gtest(test_rotate test/test_rotate.cpp)
  target_link_libraries(test_rotate drone_utils)

  ament_add_gtest(test_fuzzy_pid test/test_fuzzy_pid.cpp)
  target_link_libraries(test_fuzzy_pid drone_utils)

  # Phase 2 — math
  ament_add_gtest(test_math_utils test/test_math_utils.cpp)
  target_link_libraries(test_math_utils drone_utils)

  ament_add_gtest(test_pid test/test_pid.cpp)
  target_link_libraries(test_pid drone_utils)

  # Phase 3 — perception
  ament_add_gtest(test_detection_filter test/test_detection_filter.cpp)
  target_link_libraries(test_detection_filter drone_perception)

  ament_add_gtest(test_clustering test/test_clustering.cpp)
  target_link_libraries(test_clustering drone_perception)

  ament_add_gtest(test_camera_model test/test_camera_model.cpp)
  target_link_libraries(test_camera_model drone_perception)

  # Phase 5 — control
  ament_add_gtest(test_yaw_utils test/test_yaw_utils.cpp)
  target_link_libraries(test_yaw_utils drone_control)

  ament_add_gtest(test_velocity_controller test/test_velocity_controller.cpp)
  target_link_libraries(test_velocity_controller drone_control)

  ament_add_gtest(test_cascade_controller test/test_cascade_controller.cpp)
  target_link_libraries(test_cascade_controller drone_control)

  ament_add_gtest(test_trajectory_controller test/test_trajectory_controller.cpp)
  target_link_libraries(test_trajectory_controller drone_control)

  ament_add_gtest(test_limits test/test_limits.cpp)
  target_link_libraries(test_limits drone_control)

  # Phase 6 — mission
  ament_add_gtest(test_frame_transforms test/test_frame_transforms.cpp)
  target_link_libraries(test_frame_transforms drone_mission)

  ament_add_gtest(test_mission_config test/test_mission_config.cpp)
  target_link_libraries(test_mission_config drone_mission)

  ament_add_gtest(test_fly_state test/test_fly_state.cpp)
  target_link_libraries(test_fly_state drone_mission)
endif()

ament_package()
