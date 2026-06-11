cmake_minimum_required(VERSION 3.16)

project(RTK-Navigation-System VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/config/LayerConfig.cpp
    src/dxf/DxfReader.cpp
    src/sensor/RTKReader.cpp
    src/sensor/CassDatReader.cpp
    src/coordinate/CoordinateTransformer.cpp
    src/map/GridMap.cpp
    src/map/MapBuilder.cpp
    src/map/CassMapBuilder.cpp
    src/map/FeatureMapBuilder.cpp
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

include(CTest)

if(BUILD_TESTING)
    add_executable(feature_map_pipeline_test
        tests/FeatureMapPipelineTest.cpp
        src/config/LayerConfig.cpp
        src/map/GridMap.cpp
        src/map/FeatureMapBuilder.cpp
        src/planner/AStar.cpp
    )

    target_include_directories(feature_map_pipeline_test
        PRIVATE
            src
            ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(feature_map_pipeline_test
        PRIVATE
            ${OpenCV_LIBS}
    )

    target_compile_definitions(feature_map_pipeline_test
        PRIVATE
            RTK_NAV_TEST_LAYER_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/dxf_layers.yaml"
    )

    if(MSVC)
        target_compile_options(feature_map_pipeline_test PRIVATE /W4)
    else()
        target_compile_options(feature_map_pipeline_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME feature_map_pipeline COMMAND feature_map_pipeline_test)

    add_executable(dxf_reader_test
        tests/DxfReaderTest.cpp
        src/dxf/DxfReader.cpp
    )

    target_include_directories(dxf_reader_test PRIVATE src)

    target_compile_definitions(dxf_reader_test
        PRIVATE
            RTK_NAV_TEST_DXF="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/navigation_map_ascii.dxf"
    )

    if(MSVC)
        target_compile_options(dxf_reader_test PRIVATE /W4)
    else()
        target_compile_options(dxf_reader_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME dxf_reader COMMAND dxf_reader_test)

    add_executable(astar_clearance_test
        tests/AStarClearanceTest.cpp
        src/map/GridMap.cpp
        src/planner/AStar.cpp
    )

    target_include_directories(astar_clearance_test PRIVATE src)

    if(MSVC)
        target_compile_options(astar_clearance_test PRIVATE /W4)
    else()
        target_compile_options(astar_clearance_test PRIVATE -Wall -Wextra -Wpedantic)
    endif()

    add_test(NAME astar_clearance COMMAND astar_clearance_test)
endif()
