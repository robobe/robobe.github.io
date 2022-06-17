cmake_minimum_required(VERSION 3.11)
project(xacro)

find_package(xacro REQUIRED)

file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/*.xacro)
foreach(it ${xacro_files})
    string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
    set(output_filename ${CMAKE_MATCH_1})
    xacro_add_xacro_file(${it} ${output_filename})

    list(APPEND xacro_files ${output_filename})
endforeach(it)

# add an abstract target to actually trigger the builds
add_custom_target(media_files ALL DEPENDS ${xacro_files})