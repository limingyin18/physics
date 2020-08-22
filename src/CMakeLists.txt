add_subdirectory(basic)
add_subdirectory(PiratePhysics)

file(GLOB SRC "*.hpp" "*.cpp")

add_executable(${CMAKE_PROJECT_NAME} ${SRC})

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    Eigen3::Eigen
    glfw
    opencv_core 
    opencv_imgcodecs
    basic
    PiratePhysics)

add_custom_command(
  TARGET ${CMAKE_PROJECT_NAME}
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/src/shaders  $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/data  $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>
  )