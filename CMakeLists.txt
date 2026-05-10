cmake_minimum_required(VERSION 3.22)
project(teseo)

set(CMAKE_CXX_STANDARD 20)

find_package(raylib CONFIG REQUIRED)
find_package(box2d CONFIG REQUIRED)

set(SOURCES
    src/main.cpp
    src/sim/agent.cpp
    src/sim/maze.cpp
    src/sim/sim.cpp
    src/ui/ui.cpp
    src/starter_mouse/starter_mouse.cpp
    src/keyboard_mouse/keyboard_mouse.cpp
)

add_executable(teseo ${SOURCES})

target_link_libraries(teseo PRIVATE raylib box2d::box2d)

target_include_directories(teseo PRIVATE src)
