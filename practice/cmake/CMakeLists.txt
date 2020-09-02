project(practice_cmake)
cmake_minimum_required(VERSION 3.0)

set(simplevar "check" "check2" "check3")
message("This is the CMAKE_PREFIX_PATH: ${simplevar}")
message("Now I append something.")
list(APPEND simplevar "test")
message("Now it is: ${simplevar}")