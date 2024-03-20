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

target_compile_options(${PROJECT_NAME} PRIVATE "-Werror")
target_compile_options(${PROJECT_NAME} PRIVATE "-pedantic-errors")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wall")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wextra")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wpedantic")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wcast-align")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wcast-qual")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wconversion")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wctor-dtor-privacy")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wfloat-equal")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wlogical-op")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wnon-virtual-dtor")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wold-style-cast")
target_compile_options(${PROJECT_NAME} PRIVATE "-Woverloaded-virtual")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wredundant-decls")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wsign-promo")
