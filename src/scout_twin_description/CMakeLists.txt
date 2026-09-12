cmake_minimum_required(VERSION 3.8)
project(scout_twin_description)
find_package(ament_cmake REQUIRED)

install(FILES README.md SOURCES.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch rviz urdf meshes
  DESTINATION share/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/materials")
  install(DIRECTORY materials DESTINATION share/${PROJECT_NAME})
endif()
install(PROGRAMS scripts/check_streams.py scripts/pivot_test.py scripts/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
