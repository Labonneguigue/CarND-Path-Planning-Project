#project(Path_Planning)

add_subdirectory(pathplanner)
add_subdirectory(profiler)
#add_subdirectory(third-party-libraries)

set(MAIN_SOURCES
    main.cpp)


set(GOOGLETEST_ROOT googletest/googletest CACHE STRING "Google Test source root")

include_directories(
    ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}
    ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/include
    )

set(GOOGLETEST_SOURCES
    ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/src/gtest-all.cc
    ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/src/gtest_main.cc
    )

foreach(_source ${GOOGLETEST_SOURCES})
    set_source_files_properties(${_source} PROPERTIES GENERATED 1)
endforeach()

add_library(googletest ${GOOGLETEST_SOURCES})

add_executable(
    unit_tests
    tests/main_tests.cpp
    tests/profiler_tests.cpp
    tests/utl_tests.cpp
    tests/sensorfusion_tests.cpp
    tests/behaviorplanner_tests.cpp
    )

add_dependencies(unit_tests googletest)

target_link_libraries(
    unit_tests
    googletest
    pathplanner
    profiler
    )

target_include_directories(unit_tests
	PUBLIC
	${PROJECT_SOURCE_DIR}/src/profiler/include
	${PROJECT_SOURCE_DIR}/src/pathplanner/include
	)

enable_testing()

add_test(unit ${PROJECT_BINARY_DIR}/unit_tests)


find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${EIGEN3_INCLUDE_DIR} ${pathplanner_SOURCE_DIR})


