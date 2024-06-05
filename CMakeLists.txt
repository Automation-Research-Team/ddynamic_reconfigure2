cmake_minimum_required(VERSION 3.8)
project(ddynamic_reconfigure2)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nlohmann_json REQUIRED)

#################################################
#  shared library: libddynamic_reconfigure2.so  #
#################################################
add_library(
  ${PROJECT_NAME} SHARED
    src/ddynamic_reconfigure2.cpp
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(
  ${PROJECT_NAME} PRIVATE
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
    rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS  ${PROJECT_NAME}
  EXPORT   export_${PROJECT_NAME}
  LIBRARY  DESTINATION lib
  ARCHIVE  DESTINATION lib
  RUNTIME  DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

##############################
#  cpp executable: testnode  #
##############################
add_executable(
  testnode
    src/testnode.cpp
)

target_include_directories(
  testnode
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  testnode
    rclcpp
)

install(
  TARGETS testnode
  DESTINATION lib/${PROJECT_NAME}
)

###################################
#  python executable: pytestnode  #
###################################
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/pytestnode.py
  DESTINATION lib/${PROJECT_NAME}
)

##################
#  launch files  #
##################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

##########
#  test  #
##########
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
