cmake_minimum_required(VERSION 3.0.2)
project(b3m_hw_interface)

add_compile_options(-std=c++17)

find_package(
  catkin REQUIRED
  COMPONENTS b3m_driver
             combined_robot_hw
             controller_manager
             hardware_interface
             pluginlib
             roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    b3m_driver
    combined_robot_hw
    controller_manager
    hardware_interface
    pluginlib
    roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/b3m_hw_interface.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/b3m_hw_interface_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES b3m_hw_interface_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
