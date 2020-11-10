project(IMGUI)

file(GLOB_RECURSE SRC "*.c" "*.h" "*.hpp" "*.cpp")

add_library(${PROJECT_NAME} ${SRC})

target_link_libraries(${PROJECT_NAME} PRIVATE glad::glad glfw)