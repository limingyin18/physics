project(IMGUI)

file(GLOB_RECURSE SRC "*.c" "*.h" "*.hpp" "*.cpp")

add_library(${PROJECT_NAME} SHARED ${SRC})

target_link_libraries(${PROJECT_NAME} PRIVATE GLAD)