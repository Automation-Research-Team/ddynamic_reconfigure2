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

##################
#  dependencies  #
##################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

####################################################
#  header only library: ddynamic_reconfigure2.hpp  #
####################################################
install(
  DIRECTORY   include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

# The following is allowed only after jazzy.
#ament_auto_add_library(
#  ${PROJECT_NAME} INTERFACE
#)

##############################
#  cpp executable: testnode  #
##############################
ament_auto_add_executable(
  testnode
    src/testnode.cpp
)

###############################################
#  python library and executable: pytestnode  #
###############################################
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/pytestnode.py
  DESTINATION lib/${PROJECT_NAME}
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

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE launch config
)
