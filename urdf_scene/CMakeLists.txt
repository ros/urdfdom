include_directories(
  ${PROJECT_SOURCE_DIR}/urdf_model/include
  ${PROJECT_SOURCE_DIR}/urdf_sensor/include
  ${PROJECT_SOURCE_DIR}/urdf_scene/include
  ${PROJECT_SOURCE_DIR}/urdf_parser/include
)

add_library(urdfdom_scene SHARED src/urdf_scene.cpp)
target_link_libraries(urdfdom_scene tinyxml)

INSTALL(TARGETS urdfdom_scene DESTINATION lib/)
INSTALL(DIRECTORY include/urdf_scene DESTINATION include)
