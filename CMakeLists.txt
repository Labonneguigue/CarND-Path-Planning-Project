project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

add_subdirectory(src)

set(MAIN_SOURCES
    src/main.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(path_planning ${MAIN_SOURCES})

target_link_libraries(path_planning pathplanner profiler z ssl uv uWS)

target_include_directories(path_planning
	PUBLIC
	${PROJECT_SOURCE_DIR}/src/pathplanner/include
	${PROJECT_SOURCE_DIR}/src/profiler/include
	)