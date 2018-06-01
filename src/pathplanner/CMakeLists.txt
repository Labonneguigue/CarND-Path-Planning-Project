# Path Planner library

add_library(pathplanner
        pathplanner.cpp
        behaviorplanner.cpp
        trajectorygenerator.cpp
        predictor.cpp
        sensorfusion.cpp
    )

target_compile_features(pathplanner
    PUBLIC
        cxx_constexpr
    )

target_include_directories(pathplanner
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ./
    )

# Path Planner Test Library

add_executable(pathplanner_tests
        tests/main_tests.cpp
        tests/behaviorplanner_tests.cpp
        tests/highway_tests.cpp
        tests/sensorfusion_tests.cpp
        tests/utl_tests.cpp
    )

target_link_libraries(pathplanner_tests
    PRIVATE
        pathplanner
        gtest
    )

add_test(pathplanner_tests pathplanner_tests)


