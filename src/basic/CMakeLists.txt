add_subdirectory(GLADLib)
add_subdirectory(imguiGlfwOpenGL)

project(basic)

file(GLOB SRC "*.hpp" "*.cpp")

add_library(${PROJECT_NAME} ${SRC})

target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen glfw GLAD IMGUI)