cmake_minimum_required(VERSION 3.8)
project(ddynamic_reconfigure2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
    src/ddynamic_reconfigure2.cpp
)

target_compile_features(
  ${PROJECT_NAME} PUBLIC
    c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(
  ${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  ${PROJECT_NAME}
    rclcpp
)

add_executable(
  testnode
    src/testnode.cpp
)

target_compile_features(
  testnode PUBLIC
    c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(
  testnode PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  testnode
    rclcpp
)

target_link_libraries(
  testnode
  ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} testnode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_package()
