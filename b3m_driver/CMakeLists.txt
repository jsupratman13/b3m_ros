cmake_minimum_required(VERSION 3.0.2)
project(b3m_driver)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS roscpp serial)
find_package(async_comm REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp serial
  DEPENDS async_comm
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${async_comm_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/b3m_interface.cpp src/b3m_async_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${async_comm_LIBRARIES})

add_executable(sample_${PROJECT_NAME}_node src/sample_b3m_driver_node.cpp)
add_dependencies(sample_${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  sample_${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS sample_${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
