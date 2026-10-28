cmake_minimum_required(VERSION 3.16)
project(joint_velocity_controller CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(joint_sim src/motor_simulation.cpp src/velocity_controller.cpp)
target_include_directories(joint_sim PUBLIC include)
target_compile_options(joint_sim PUBLIC
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wconversion>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>)

add_executable(sim_main app/sim_main.cpp)
target_link_libraries(sim_main PRIVATE joint_sim)

enable_testing()
add_executable(test_motor_sim tests/test_motor_sim.cpp)
target_link_libraries(test_motor_sim PRIVATE joint_sim)
add_test(NAME test_motor_sim COMMAND test_motor_sim)

add_executable(test_velocity_controller tests/test_velocity_controller.cpp)
target_link_libraries(test_velocity_controller PRIVATE joint_sim)
add_test(NAME test_velocity_controller COMMAND test_velocity_controller)
