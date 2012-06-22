
include_directories(
  ${PROJECT_SOURCE_DIR}/urdf_model/include
  ${PROJECT_SOURCE_DIR}/urdf_parser/include
  ${PROJECT_SOURCE_DIR}/urdf_model_state/include
  ${PROJECT_SOURCE_DIR}/urdf_scene_state/include
)

add_library(urdfdom_scene_state SHARED src/urdf_scene_state.cpp)
target_link_libraries(urdfdom_scene_state tinyxml)

INSTALL(TARGETS urdfdom_scene_state DESTINATION lib/)
INSTALL(DIRECTORY include/urdf_scene_state DESTINATION include)
