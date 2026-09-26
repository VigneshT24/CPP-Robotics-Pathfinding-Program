cmake_minimum_required(VERSION 3.20)

project(RobotPathfinder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
    ftxui
    GIT_REPOSITORY https://github.com/ArthurSonzogni/FTXUI
    GIT_TAG v7.0.3
)

FetchContent_MakeAvailable(ftxui)

add_executable(pathfinder_main pathfinder_main.cpp)

target_link_libraries(pathfinder_main PRIVATE ftxui::ftxui)