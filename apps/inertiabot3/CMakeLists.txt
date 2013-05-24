###############################################
# apps/inertiabot3

project(inertiabot3)
file(GLOB inertiabot3_srcs "*.cpp")
file(GLOB inertiabot3_hdrs "*.h")
add_executable(inertiabot3 ${inertiabot3_srcs} ${inertiabot3_hdrs})
target_link_libraries(inertiabot3 dart ${DARTExt_LIBRARIES})
set_target_properties(inertiabot3 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
