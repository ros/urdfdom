
include_directories(
  ${PROJECT_SOURCE_DIR}/urdf_model/include
  ${PROJECT_SOURCE_DIR}/urdf_state/include
)

add_library(urdfdom_state SHARED src/urdf_state.cpp src/twist.cpp)
target_link_libraries(urdfdom_state tinyxml)

INSTALL(TARGETS urdfdom_state DESTINATION lib/)
INSTALL(DIRECTORY include/urdf_state DESTINATION include)
