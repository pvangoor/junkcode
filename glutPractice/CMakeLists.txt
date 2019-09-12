project(glutPractice)
cmake_minimum_required(VERSION 3.0)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(Threads REQUIRED)

set(ADDITIONAL_INCLUDE_DIRS
${EIGEN_INCLUDE_DIRS}
${GLUT_INCLUDE_DIRS}
${OPENGL_INCLUDE_DIR}
# ${EIGEN_INCLUDE_DIR}/unsupported
)

# Find project files
####################

set(glutPractice_SOURCE_FILES
    src/Plotter.cpp
)

set(glutPractice_HEADER_FILES
    include/Plotter.h
    include/PlotDataTypes.h
)

add_library(glutPracticeLib
    ${glutPractice_HEADER_FILES}
    ${glutPractice_SOURCE_FILES}
)


add_executable(glutPractice src/main.cpp)

include_directories(
    include
    ${ADDITIONAL_INCLUDE_DIRS}
)

target_link_libraries(glutPracticeLib
    ${GLUT_LIBRARY}
    ${OPENGL_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

target_link_libraries(glutPractice
    glutPracticeLib
)