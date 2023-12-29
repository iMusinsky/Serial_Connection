cmake_minimum_required(VERSION 3.10)

get_filename_component(CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)

project(${CURRENT_DIR}
		LANGUAGES CXX)

file (GLOB SOURCES
	*.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${API_PATH})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIB_NAME})
