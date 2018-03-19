# Set the project name
#project(pathplanner)

# Bring my headers in the build environment
include_directories(include)

# Source variable with a link to all cpp files to compile
set(SOURCES
	# Main Component
    pathplanner.cpp
    include/pathplanner.h
	# Behavior Planning sub-component
    behaviorplanner.cpp
    include/behaviorplanner.h
	# Trajectory Generation sub-component
    trajectorygenerator.cpp
    include/trajectorygenerator.h
	# PID used for speed regulation
    include/PID.h
	# Prediction sub-component
    predictor.cpp
    include/predictor.h
	# Sensor Fusion sub-component
    sensorfusion.cpp
    include/sensorfusion.h
	# Road types
    include/roadtypes.h
	# Drinving Policy
    include/drivingpolicy.h
	# Data Structures 
    include/mapdata.h
    include/vehicledata.h
    include/controllerfeedback.h
	# Helper functions/classes
    include/spline.h
    include/utl.h
)

# Add a library with the above sources
add_library(pathplanner STATIC ${SOURCES})

target_include_directories(pathplanner
    PUBLIC
    ${PROJECT_SOURCE_DIR}/pathplanner/include
    )

install(TARGETS pathplanner DESTINATION /usr/lib)
