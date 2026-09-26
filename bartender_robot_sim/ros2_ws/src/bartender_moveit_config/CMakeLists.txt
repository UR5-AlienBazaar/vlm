cmake_minimum_required(VERSION 3.8)
project(bartender_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY srdf config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # pep257 runs as a pytest test rather than through ament_lint_auto, only
  # to get at its --ignore list: ament's defaults leave D213 ON while
  # ignoring D212, which between them require a docstring's summary to start
  # on the SECOND line -- the opposite of the convention every file in this
  # repo follows. Same arrangement as bartender_description, same file.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(pep257_repo_style test/test_pep257.py)
  set(ament_cmake_pep257_FOUND TRUE)

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
