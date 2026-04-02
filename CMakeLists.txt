cmake_minimum_required(VERSION 3.14)
project(drone VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Utility library
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

# Suppress warnings from Eigen system headers
target_include_directories(drone_utils SYSTEM PUBLIC ${Eigen3_INCLUDE_DIRS})

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install config
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# Install library
install(TARGETS drone_utils
  EXPORT export_drone_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_drone_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 ament_index_cpp yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linters that conflict with our clang-format / license setup
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
