project(basic)

file(GLOB_RECURSE SRC "*.hpp" "*.cpp")

add_library(${PROJECT_NAME} ${SRC})

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen imgui::imgui glfw glad::glad)