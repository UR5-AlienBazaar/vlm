cmake_minimum_required(VERSION 3.8)
project(bartender_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(
  DIRECTORY urdf config scripts meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Renders the description and checks the result. It shells out to xacro and
  # so needs this package's own dependencies resolvable, which they are by
  # the time tests run.
  ament_add_pytest_test(render test/test_render.py)
  # The arm controllers' goal tolerances: load-bearing, and their
  # absence was silent. See the test's own docstring.
  ament_add_pytest_test(controller_limits test/test_controller_limits.py)
  # The one-armed workcell the real robot is brought up from.
  ament_add_pytest_test(workcell test/test_workcell.py)

  # pep257 runs as a pytest test instead of through ament_lint_auto, only to
  # get at its --ignore list: ament's default set leaves D213 ON while
  # ignoring D212, and between them those require a docstring's summary to
  # start on the SECOND line -- the opposite of the convention every file in
  # this repo follows. The Python packages here exclude it the same way, in
  # test/test_pep257.py, which is why this borrows their file rather than
  # inventing a second way of saying it. Everything else pep257 checks still
  # applies.
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
