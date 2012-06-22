
include_directories(
  ${PROJECT_SOURCE_DIR}/urdf_model/include
  ${PROJECT_SOURCE_DIR}/urdf_parser/include
  ${PROJECT_SOURCE_DIR}/urdf_model_state/include
)

add_library(urdfdom_model_state SHARED src/urdf_model_state.cpp src/twist.cpp)
target_link_libraries(urdfdom_model_state tinyxml)

INSTALL(TARGETS urdfdom_model_state DESTINATION lib/)
INSTALL(DIRECTORY include/urdf_model_state DESTINATION include)
