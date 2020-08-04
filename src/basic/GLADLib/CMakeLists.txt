project(GLAD)

file(GLOB_RECURSE SRC "*.c" "*.h" "*.hpp" "*.cpp")

add_library(${PROJECT_NAME} SHARED ${SRC})

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})