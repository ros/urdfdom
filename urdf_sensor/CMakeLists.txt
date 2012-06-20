include_directories(
  ${PROJECT_SOURCE_DIR}/urdf_model/include
  ${PROJECT_SOURCE_DIR}/urdf_sensor/include
#  ${PROJECT_SOURCE_DIR}/urdf_state/include
  ${PROJECT_SOURCE_DIR}/urdf_parser/include
)

add_library(urdfdom_sensor SHARED src/urdf_sensor.cpp)
target_link_libraries(urdfdom_sensor tinyxml)

INSTALL(TARGETS urdfdom_sensor DESTINATION lib/)
INSTALL(DIRECTORY include/urdf_sensor DESTINATION include)
