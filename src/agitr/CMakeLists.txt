# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# Name of this package.
project(agitr)

# Find the name of the catkin build system, and any other pacakages
# on which we depend
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs turtlesim)

# Declare our catkin package.
catkin_package()

# Specify locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
# If there are multiple executables, use multiple copies of
# this line.
add_executable(hello hello.cpp)
add_executable(pubvel pubvel.cpp)
add_executable(subpose subpose.cpp)
add_executable(count count.cpp)
add_executable(anon anon.cpp)
add_executable(reverse_cmd_vel reverse_cmd_vel.cpp)
add_executable(set_bg_color set_bg_color.cpp)
add_executable(pubvel_with_max pubvel_with_max.cpp)
add_executable(spawn_turtle spawn_turtle.cpp)
add_executable(pubvel_toggle pubvel_toggle.cpp)

# Specify libraries agains which to link. Again,
# this line should be copied for each distinct executable interface
# the package.
target_link_libraries(hello ${catkin_LIBRARIES})
target_link_libraries(pubvel ${catkin_LIBRARIES})
target_link_libraries(subpose ${catkin_LIBRARIES})
target_link_libraries(count ${catkin_LIBRARIES})
target_link_libraries(anon ${catkin_LIBRARIES})
target_link_libraries(reverse_cmd_vel ${catkin_LIBRARIES})
target_link_libraries(set_bg_color ${catkin_LIBRARIES})
target_link_libraries(pubvel_with_max ${catkin_LIBRARIES})
target_link_libraries(spawn_turtle ${catkin_LIBRARIES})
target_link_libraries(pubvel_toggle ${catkin_LIBRARIES})
