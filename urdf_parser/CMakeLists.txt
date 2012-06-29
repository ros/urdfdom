include_directories(include)

add_library(urdfdom_model SHARED src/console.cpp src/pose.cpp src/model.cpp src/link.cpp src/joint.cpp)
target_link_libraries(urdfdom_model ${tinyxml_libraries} ${Boost_LIBRARIES})

add_library(urdfdom_sensor SHARED src/urdf_sensor.cpp)
target_link_libraries(urdfdom_sensor ${tinyxml_libraries} ${Boost_LIBRARIES})

add_library(urdfdom_model_state SHARED src/urdf_model_state.cpp src/twist.cpp)
target_link_libraries(urdfdom_model_state ${tinyxml_libraries} ${Boost_LIBRARIES})

# --------------------------------

add_executable(check_urdf src/check_urdf.cpp)
target_link_libraries(check_urdf urdfdom_model)

add_executable(urdf_to_graphiz src/urdf_to_graphiz.cpp)
target_link_libraries(urdf_to_graphiz urdfdom_model)

add_executable(mem_test test/memtest.cpp)
target_link_libraries(mem_test urdfdom_model)


INSTALL(TARGETS urdfdom_model DESTINATION lib/)
INSTALL(TARGETS check_urdf urdf_to_graphiz mem_test DESTINATION share/urdfdom_model)
INSTALL(TARGETS urdfdom_sensor DESTINATION lib/)
INSTALL(TARGETS urdfdom_model_state DESTINATION lib/)
INSTALL(DIRECTORY include/urdf_parser DESTINATION include)
