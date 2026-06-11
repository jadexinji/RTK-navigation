cmake_minimum_required(VERSION 3.16)

project(RTK-Navigation-System VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/sensor/RTKReader.cpp
    src/sensor/CassDatReader.cpp
    src/coordinate/CoordinateTransformer.cpp
    src/map/GridMap.cpp
    src/map/MapBuilder.cpp
    src/map/CassMapBuilder.cpp
    src/planner/AStar.cpp
    src/navigation/Navigator.cpp
    src/visualization/Viewer.cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        src
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Eigen3::Eigen
        ${OpenCV_LIBS}
)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()
