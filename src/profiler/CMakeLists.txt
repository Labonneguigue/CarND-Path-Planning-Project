# Profiler library

add_library(profiler
        profiler.cpp
        entry.cpp
    )

target_compile_features(profiler
    PRIVATE
        cxx_defaulted_move_initializers
    )

target_include_directories(profiler
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ./
    )
    
add_executable(profiler_tests
        tests/main_tests.cpp
        tests/profiler_tests.cpp
    )

target_link_libraries(profiler_tests
    PRIVATE
        profiler
    PUBLIC
        gtest
    )

add_test(profiler_tests profiler_tests)