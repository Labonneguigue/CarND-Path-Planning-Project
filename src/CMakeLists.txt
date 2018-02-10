project(Path_Planning)

add_subdirectory(pathplanner)
#add_subdirectory(third-party-libraries)

set(MAIN_SOURCES
    main.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(path_planning ${MAIN_SOURCES})

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${EIGEN3_INCLUDE_DIR} ${pathplanner_SOURCE_DIR})

target_link_libraries(path_planning pathplanner z ssl uv uWS Eigen3::Eigen)


#MESSAGE(${PROJECT_NAME})
#target_include_directories(${PROJECT_NAME}
#    PUBLIC
#    ${PROJECT_SOURCE_DIR}/pathplanner/include
#    )
