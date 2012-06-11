include_directories(
  ${PROJECT_SOURCE_DIR}/urdf_model/include
  ${PROJECT_SOURCE_DIR}/urdf_parser/include
)

set (headers include/urdf_parser/urdf_parser.h)

add_library(urdfdom SHARED src/pose.cpp src/link.cpp src/joint.cpp src/urdf_parser.cpp)
target_link_libraries(urdfdom tinyxml)

add_executable(check_urdf src/check_urdf.cpp)
target_link_libraries(check_urdf urdfdom)

add_executable(urdf_to_graphiz src/urdf_to_graphiz.cpp)
target_link_libraries(urdf_to_graphiz urdfdom)

add_executable(mem_test test/memtest.cpp)
target_link_libraries(mem_test urdfdom)

INSTALL(TARGETS urdfdom DESTINATION lib/)
INSTALL(TARGETS check_urdf urdf_to_graphiz mem_test DESTINATION share/urdfdom)
INSTALL(DIRECTORY include/urdf_parser DESTINATION include)
