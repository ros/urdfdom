include_directories(
  ${PROJECT_SOURCE_DIR}/urdf_interface/include
  ${PROJECT_SOURCE_DIR}/urdf_parser/include
)

add_library(urdf_parser src/link.cpp src/joint.cpp src/urdf_parser.cpp)
target_link_libraries(urdf_parser tinyxml)

add_executable(check_urdf src/check_urdf.cpp)
target_link_libraries(check_urdf urdf_parser)

add_executable(urdf_to_graphiz src/urdf_to_graphiz.cpp)
target_link_libraries(urdf_to_graphiz urdf_parser)

add_executable(mem_test test/memtest.cpp)
target_link_libraries(mem_test urdf_parser)

