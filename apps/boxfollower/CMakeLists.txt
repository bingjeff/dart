###############################################
# apps/activity2

project(boxfollower)
file(GLOB boxfollower_srcs "*.cpp")
file(GLOB boxfollower_hdrs "*.h")
add_executable(boxfollower ${boxfollower_srcs} ${boxfollower_hdrs})
target_link_libraries(boxfollower dart ${DARTExt_LIBRARIES})
set_target_properties(boxfollower PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
