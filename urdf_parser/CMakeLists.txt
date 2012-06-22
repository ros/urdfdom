include_directories(
  ${PROJECT_SOURCE_DIR}/urdf_model/include
  ${PROJECT_SOURCE_DIR}/urdf_parser/include
  ${PROJECT_SOURCE_DIR}/urdf_sensor/include
  ${PROJECT_SOURCE_DIR}/urdf_model_state/include
)

add_library(urdfdom_model SHARED src/console.cpp src/pose.cpp src/model.cpp src/link.cpp src/joint.cpp src/urdf_model.cpp)
target_link_libraries(urdfdom_model tinyxml)

add_executable(check_urdf src/check_urdf.cpp)
target_link_libraries(check_urdf urdfdom_model)

add_executable(urdf_to_graphiz src/urdf_to_graphiz.cpp)
target_link_libraries(urdf_to_graphiz urdfdom_model)

add_executable(mem_test test/memtest.cpp)
target_link_libraries(mem_test urdfdom_model)

add_library(urdfdom_sensor SHARED src/urdf_sensor.cpp)
target_link_libraries(urdfdom_sensor tinyxml)

add_library(urdfdom_model_state SHARED src/urdf_model_state.cpp src/twist.cpp)
target_link_libraries(urdfdom_model_state tinyxml)

INSTALL(TARGETS urdfdom_model DESTINATION lib/)
INSTALL(TARGETS check_urdf urdf_to_graphiz mem_test DESTINATION share/urdfdom_model)
INSTALL(TARGETS urdfdom_sensor DESTINATION lib/)
INSTALL(TARGETS urdfdom_model_state DESTINATION lib/)
INSTALL(DIRECTORY include/urdf_parser DESTINATION include)
