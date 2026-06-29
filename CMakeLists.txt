cmake_minimum_required(VERSION 3.8)
project(robot_base_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/global_localization_trigger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME global_localization_trigger
)


install(PROGRAMS
  scripts/isaac_restamper.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME isaac_restamper
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
