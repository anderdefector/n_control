cmake_minimum_required(VERSION 2.8.3)
project(n_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  image_transport
  sensor_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

generate_messages(
   DEPENDENCIES
#   bebop_msgs#   geometry_msgs#   nav_msgs#   std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}	
)

#add_executable(C_Vi src/C_Vi.cpp)
# target_link_libraries(C_Vi
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

add_executable(C_Master src/c_master.cpp)
target_link_libraries(C_Master ${catkin_LIBRARIES})
add_dependencies(C_Master C_Master_generate_messages_cpp)

#add_executable(i_data src/i_data.cpp)
#target_link_libraries(i_data ${catkin_LIBRARIES})
#add_dependencies(i_data i_data_generate_messages_cpp)

add_executable(c_con src/c_con.cpp)
target_link_libraries(c_con ${catkin_LIBRARIES})
add_dependencies(c_con c_con_generate_messages_cpp)

add_executable(C_Vx src/c_vx.cpp)
target_link_libraries(C_Vx ${catkin_LIBRARIES})
add_dependencies(C_Vx C_Vx_generate_messages_cpp)

add_executable(C_Vy src/c_vy.cpp)
target_link_libraries(C_Vy ${catkin_LIBRARIES})
add_dependencies(C_Vy C_Vy_generate_messages_cpp)

add_executable(C_Vz src/c_vz.cpp)
target_link_libraries(C_Vz ${catkin_LIBRARIES})
#add_dependencies(C_Vz C_Vz_generate_messages_cpp)

add_executable(C_Vaz src/c_vaz.cpp)
target_link_libraries(C_Vaz ${catkin_LIBRARIES})
add_dependencies(C_Vaz C_Vaz_generate_messages_cpp)

