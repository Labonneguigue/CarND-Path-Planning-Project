cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(Path_Planning VERSION 1.0.0 LANGUAGES CXX)

if(MSVC)
	#add_compile_options(/W3 /WX)
else()
	add_compile_options(-W -Wall)# -Werror
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	include_directories(/usr/local/include)
	include_directories(/usr/local/opt/openssl/include)
	link_directories(/usr/local/lib)
	link_directories(/usr/local/opt/openssl/lib)
	link_directories(/usr/local/Cellar/libuv/1.20.3/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(path_planning
	src/main.cpp
	)

message("Generator is set to ${CMAKE_GENERATOR}.")
if (${CMAKE_GENERATOR} STREQUAL "Xcode")
	target_compile_definitions(path_planning PRIVATE XCODE)
	message("Preprocessor macro XCODE set.")
endif()

target_link_libraries(path_planning
	PRIVATE
		pathplanner
		profiler
		json
		z
		ssl
		uv
		uWS
	)

enable_testing()

add_subdirectory(src)