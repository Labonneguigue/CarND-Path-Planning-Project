# Path Planner library

add_library(pathplanner
        pathplanner.cpp
        behaviorplanner.cpp
        trajectorygenerator.cpp
        predictor.cpp
        sensorfusion.cpp
        # headers
        include/behaviorplanner.h
        include/controllerfeedback.h
        include/drivingpolicy.h
        include/mapdata.h
        include/pathplanner.h
        include/PID.h
        include/predictor.h
        include/roadtypes.h
        include/sensorfusion.h
        include/spline.h
        include/trajectorygenerator.h
        include/utl.h
        include/vehicledata.h
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
        tests/behaviorplanner_tests.cpp
        tests/highway_tests.cpp
        tests/main_tests.cpp
        tests/predictor_tests.cpp
        tests/sensorfusion_tests.cpp
        tests/utl_tests.cpp
    )

target_link_libraries(pathplanner_tests
    PRIVATE
        pathplanner
        gtest
    )

add_test(pathplanner_tests pathplanner_tests)


