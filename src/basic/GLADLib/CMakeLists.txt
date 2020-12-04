project(GLAD)

file(GLOB_RECURSE SRC "*.c" "*.h" "*.hpp" "*.cpp")

add_library(${PROJECT_NAME} ${SRC})

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME} PUBLIC ${CMAKE_DL_LIBS})