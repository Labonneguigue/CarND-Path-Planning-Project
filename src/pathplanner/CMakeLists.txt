# Set the project name
#project(pathplanner)

MESSAGE("Library : " ${PROJECT_NAME})

# Bring my headers in the build environment
include_directories(include)

# Source variable with a link to all cpp files to compile
set(SOURCES
    pathplanner.cpp
    include/mapdata.h
    include/pathplanner.h
    include/utl.h
    include/vehicledata.h
    include/controllerfeedback.h
    include/spline.h
)

set(HEADERS

    )

# Add a library with the above sources
add_library(pathplanner SHARED ${SOURCES})

MESSAGE(${PROJECT_SOURCE_DIR}/pathplanner/include)
target_include_directories(pathplanner
    PUBLIC
    ${PROJECT_SOURCE_DIR}/pathplanner/include
    )

install(TARGETS pathplanner DESTINATION /usr/lib)
