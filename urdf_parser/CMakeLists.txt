include_directories(
  ${PROJECT_SOURCE_DIR}/urdf_interface/include
  ${PROJECT_SOURCE_DIR}/urdf_parser/include
)

add_library(${PROJECT_NAME} src/link.cpp src/joint.cpp src/urdf_parser.cpp)
target_link_libraries(${PROJECT_NAME} tinyxml)

add_executable(check_urdf src/check_urdf.cpp)
target_link_libraries(check_urdf ${PROJECT_NAME})

add_executable(urdf_to_graphiz src/urdf_to_graphiz.cpp)
target_link_libraries(urdf_to_graphiz ${PROJECT_NAME})

add_executable(mem_test test/memtest.cpp)
target_link_libraries(mem_test ${PROJECT_NAME})

