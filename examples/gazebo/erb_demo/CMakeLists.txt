cmake_minimum_required(VERSION 3.10)
project(HELLO_ERB)

find_program(ERB_EXE_PATH erb)
if(NOT ERB_EXE_PATH)
  message(FATAL_ERROR "Could not find the `erb` tool.  Try `sudo apt-get install ruby`")
endif()

set(erb_files
  variable_and_math.sdf.erb
  loops_and_func.sdf.erb
)

foreach(_erb ${erb_files})
  string(REGEX REPLACE ".sdf.erb" ".sdf" _model ${_erb})
  set(_model ${CMAKE_CURRENT_SOURCE_DIR}/${_model})
  add_custom_command(OUTPUT ${_model}
                     COMMAND ${ERB_EXE_PATH} ${_erb} > ${_model}
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  list(APPEND _files ${_model})
endforeach()
add_custom_target(erb_generation ALL DEPENDS ${_files})