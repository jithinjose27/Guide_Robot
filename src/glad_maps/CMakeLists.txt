cmake_minimum_required(VERSION 3.8)
project(glad_maps)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  robotics_floor
  jithin_home
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  package.xml
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
