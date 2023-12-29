cmake_minimum_required(VERSION 3.10)

set(LIB_NAME serial_connection)
project(${LIB_NAME}
		LANGUAGES CXX)

option(BUILD_EXAMPLE "Build examples" ON)

set(RootDir  ${CMAKE_CURRENT_SOURCE_DIR})
set(API_PATH ${RootDir}/include)

file (GLOB SOURCES
	src/*.cpp
	include/*.h
)

add_library(${PROJECT_NAME} STATIC ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${API_PATH}/)

if(${BUILD_EXAMPLE})
    add_subdirectory(example)
endif(${BUILD_EXAMPLE})
