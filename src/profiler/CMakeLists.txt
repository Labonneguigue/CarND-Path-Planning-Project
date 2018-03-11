
# Bring my headers in the build environment
include_directories(include)

set(SOURCES
	# Main Component
    profiler.cpp
    include/profiler.h
	# Structure for individual profiling 
    entry.cpp
    include/entry.h
    )

add_library(profiler STATIC ${SOURCES})

target_include_directories(profiler
	PUBLIC
	${PROJECT_SOURCE_DIR}/profiler/include
	)
