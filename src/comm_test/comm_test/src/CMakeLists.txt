# play_volleyball node
add_executable(play_volleyball
   volleyball_node.cpp
)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   sensor_msgs
   message_generation
)

target_include_directories(play_volleyball PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${fmt_SOURCE_DIR}/include
    ${spdlog_INCLUDE_DIR}
)

# Build ROS nodes
set(play_volleyball_LIBS
   ${catkin_LIBRARIES}
   fmt::fmt-header-only
   spdlog::spdlog_header_only
)

target_link_libraries(play_volleyball PUBLIC
   ${play_volleyball_LIBS}
)
